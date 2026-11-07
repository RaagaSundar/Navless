#!/usr/bin/env python3
"""Isolated optical oracle bench. Never connects to an existing world or ROS/PX4."""
import argparse,copy,hashlib,json,math,os,subprocess,time,threading
from pathlib import Path
import xml.etree.ElementTree as ET

def prepare(a):
    a.output.mkdir(parents=True,exist_ok=False)
    world=ET.parse(a.world);w=world.getroot().find('world');w.set('name','texture_ab_static')
    # Keep scene/light/geometry unchanged; remove non-optical systems and GUI.
    for e in list(w):
        if e.tag=='gui' or (e.tag=='plugin' and 'Sensors' not in e.get('name','')):w.remove(e)
    p=json.loads(a.pose.read_text())['nearest_physical_sample']
    # FRD->FLU changes signs of pitch and yaw, preserves roll.
    att=json.loads(a.attitude.read_text())['roll_pitch_yaw_ENU_FLU_rad']
    if len(att) != 3 or not all(math.isfinite(v) for v in att):
        raise ValueError('finite ENU/FLU attitude required')
    if not all(math.isfinite(p[k]) for k in ('x','y','z')):
        raise ValueError('finite physical position required')
    r,pitch=att[:2]
    p['yaw']=att[2]
    rig=ET.SubElement(w,'model',name='passive_camera_rig');ET.SubElement(rig,'static').text='true'
    ET.SubElement(rig,'pose').text=' '.join(map(str,[p['x'],p['y'],p['z'],r,pitch,p['yaw']]))
    link=ET.SubElement(rig,'link',name='base_link');quad=ET.parse(a.camera)
    for s in quad.findall('.//sensor'):
        if s.get('name') in ('oak_left','oak_right'):link.append(copy.deepcopy(s))
    if len(link.findall('sensor')) != 2:
        raise ValueError('both existing stereo camera sensors are required')
    world.write(a.output/'world.sdf',encoding='utf-8',xml_declaration=True)
    (a.output/'provenance.json').write_text(json.dumps({'position_and_yaw':p,'rpy_enu_rad':[r,pitch,p['yaw']],
      'attitude_caveat':'archived vehicle_attitude_groundtruth; nearest physical reference may differ from image timestamp; physical pose used only for passive optical matching',
      'synthetic_appearance':bool(w.find("./model[@name='ground']/link/visual/material/pbr") is not None),
      'oracle_use':'offline static optical only; no navigation/control',
      'input_sha256':{k:hashlib.sha256(x.read_bytes()).hexdigest() for k,x in [('camera',a.camera),('world',a.world),('pose',a.pose),('attitude',a.attitude)]},
      'qualification':'single fixed pose optical check, no temporal stereo/VO/flight/hardware qualification'},indent=2)+'\n')

def capture(a):
    if not os.environ.get('GZ_PARTITION','').startswith('appearance_ab_') or os.environ.get('ROS_DOMAIN_ID')!='223':raise RuntimeError('isolation required')
    from gz.transport13 import Node
    from gz.msgs10.image_pb2 import Image
    import cv2,numpy as np
    node=Node();frames={};lock=threading.Lock();pair=None
    def cb(side,msg):
        if msg.width!=640 or msg.height!=400 or len(msg.data)!=640*400:return
        if msg.step != 640:return
        with lock:
            frames[side]=(msg.header.stamp.sec,msg.header.stamp.nsec,bytes(msg.data))
    for side in ['left','right']:node.subscribe(Image,'/oak/'+side+'/image_raw',lambda m,s=side:cb(s,m))
    env=os.environ.copy();env['GZ_SIM_RESOURCE_PATH']=str(a.assets/'models')
    log=open(a.output/'gazebo.log','w')
    proc=subprocess.Popen(['gz','sim','-s','-r',str(a.output/'world.sdf')],env=env,stdout=log,stderr=log)
    try:
        until=time.monotonic()+90
        while time.monotonic()<until:
            with lock:
                if len(frames)==2 and frames['left'][:2]==frames['right'][:2] and frames['left'][0]>=1:
                    pair=dict(frames)
            if pair is not None:break
            if proc.poll() is not None:raise RuntimeError('Gazebo exited')
            time.sleep(.02)
        else:raise RuntimeError('synchronized stereo timeout')
        for side,(_,_,data) in pair.items():cv2.imwrite(str(a.output/(side+'.png')),np.frombuffer(data,np.uint8).reshape(400,640))
        (a.output/'capture.json').write_text(json.dumps({'stamp':pair['left'][:2],'dimensions':[640,400],'format':'mono8','static':True,'synchronized':True})+'\n')
    finally:
        proc.terminate()
        try:proc.wait(timeout=8)
        except subprocess.TimeoutExpired:proc.kill();proc.wait()
        log.close()

if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('mode',choices=['prepare','capture']);p.add_argument('--output',required=True,type=Path);p.add_argument('--world',type=Path);p.add_argument('--camera',type=Path);p.add_argument('--pose',type=Path);p.add_argument('--attitude',type=Path);p.add_argument('--assets',type=Path);a=p.parse_args()
    required=('world','camera','pose','attitude') if a.mode=='prepare' else ('assets',)
    if any(getattr(a,k) is None for k in required):p.error('missing arguments for '+a.mode+': '+', '.join(required))
    (prepare if a.mode=='prepare' else capture)(a)
