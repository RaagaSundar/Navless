#!/usr/bin/env python3
"""Bounded native Gazebo camera observation, without ROS or truth inputs."""
import json
import threading
import time
from pathlib import Path
from gz.transport13 import Node
from gz.msgs10.image_pb2 import Image

samples=[]
metadata={}
lock=threading.Lock()

def image(msg):
    now=time.monotonic()
    with lock:
        samples.append((now,msg.header.stamp.sec+msg.header.stamp.nsec*1e-9))
        if not metadata:
            metadata.update(width=msg.width,height=msg.height,step=msg.step,
                            pixel_format_type=msg.pixel_format_type,bytes=len(msg.data))
            try:
                import cv2
                import numpy as np
                frame=np.frombuffer(msg.data,dtype=np.uint8).reshape(msg.height,msg.step)[:,:msg.width]
                metadata['mean']=float(frame.mean())
                metadata['std']=float(frame.std())
                cv2.imwrite('/root/gazebo_state/probe/first.png',frame)
            except Exception as error:
                metadata['image_save_error']=str(error)

node=Node()
node.subscribe(Image,'/oak/left/image_raw',image)
started=time.monotonic()
while time.monotonic()-started<12:
    time.sleep(.05)
with lock:
    result={'observation_wall_s':time.monotonic()-started,'frames':len(samples),'metadata':metadata}
    if len(samples)>1:
        wall=samples[-1][0]-samples[0][0]
        sim=samples[-1][1]-samples[0][1]
        result.update(rate_wall_hz=(len(samples)-1)/wall,rate_sim_hz=(len(samples)-1)/sim,
                      sim_to_wall_ratio=sim/wall,first_stamp=samples[0][1],last_stamp=samples[-1][1])
Path('/root/gazebo_state/probe/images.json').write_text(json.dumps(result,indent=2))
print(json.dumps(result,indent=2),flush=True)
if not samples:
    raise SystemExit(4)
