#!/usr/bin/env python3
"""Independent finite-capture outcome. Truth/actual collision used ONLY here.

This never replaces the ordinary full-course grade and never runs on aircraft.
"""
import argparse,bisect,hashlib,json,math,sqlite3,sys,zlib
from pathlib import Path
import numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parents[2]/'gazebo_sim'))
from dh.geom import Obstacles


def sha(p):return hashlib.sha256(Path(p).read_bytes()).hexdigest()

def read_rows(p):return [json.loads(x)for x in p.read_text().splitlines()if x.strip()]

def brake_measurement(events):
 start=next((x for x in events if x.get('event')=='commanded_open_corridor_stop'),None)
 finish=next((x for x in events if x.get('event')=='open_corridor_stop_window_complete'),None)
 if not start:return {'available':False,'reason':'no actual commanded-stop event'}
 rows=[r for r in events if r.get('event')=='actual_navigation_sample' and r.get('source_stamp_s',-1)>=start['stamp_s'] and (finish is None or r['source_stamp_s']<=finish['stamp_s'])]
 rows.sort(key=lambda r:r['source_stamp_s']);seen=set();rows=[r for r in rows if not(r['source_stamp_s']in seen or seen.add(r['source_stamp_s']))]
 if len(rows)<3:return {'available':False,'reason':'insufficient actual stop samples'}
 speed=[math.dist(r['velocity_odom_mps'],[0,0,0])for r in rows];t=[r['source_stamp_s']for r in rows];p=np.asarray([r['position_odom_m']for r in rows]);v=np.asarray(rows[0]['velocity_odom_mps']);initial=float(np.linalg.norm(v));direction=v/initial if initial>1e-8 else np.zeros(3)
 settled=None;low=None
 for i,s in enumerate(speed):
  if s<.03:
   low=i if low is None else low
   if t[i]-t[low]>=.5:settled=i;break
  else:low=None
 stop_idx=len(rows)-1 if settled is None else settled
 return {'available':True,'source':'actual post-nav-guard fused PX4 samples; not truth','source_samples':len(rows),'initial_measured_speed_mps':initial,'max_source_gap_s':max(np.diff(t)),'settled_below_03_for_half_second':settled is not None,'measured_time_to_settled_s':None if settled is None else t[settled]-t[0],'measured_max_forward_excursion_until_settled_m':float(np.max((p[:stop_idx+1]-p[0])@direction)),'measured_max_3d_excursion_until_settled_m':float(np.max(np.linalg.norm(p[:stop_idx+1]-p[0],axis=1))),'informative_nonstationary_stop':initial>.03,'model_qualification':'single observed transient only; not a guaranteed braking/deceleration bound'}

def main(a):
 if a.output.exists():raise ValueError('immutable result output exists')
 contract=json.loads(a.contract.read_text());route=json.loads(a.route.read_text());target=np.asarray(route['waypoints_enu'][-1]);truth=json.loads(a.truth.read_text());events=read_rows(a.events);telemetry=read_rows(a.telemetry);reasons=[]
 if sha(a.route)!=contract['route_sha256']:raise ValueError('route differs from declared contract')
 statuses=sorted([r for r in telemetry if r.get('kind')=='status'],key=lambda r:r['timestamp']);lands=sorted([r for r in telemetry if r.get('kind')=='land'],key=lambda r:r['timestamp']);arm=next((r['timestamp']/1e6 for r in statuses if r['armed']),None)
 if arm is None:reasons.append('no actual armed interval')
 auto_land=[r for r in statuses if r.get('nav_state')==18 and(arm is None or r['timestamp']/1e6>=arm)]
 last_airborne=max((r['timestamp']/1e6 for r in lands if not r['landed'] and arm is not None and r['timestamp']/1e6>=arm),default=math.inf)
 disarmed=[r for r in statuses if not r['armed'] and arm is not None and r['timestamp']/1e6>last_airborne]
 actual_landed=[r for r in lands if r['landed'] and arm is not None and r['timestamp']/1e6>last_airborne]
 normal_land=bool(auto_land)and any(r.get('event')=='normal_autoland_intent'for r in events)
 if not normal_land:reasons.append('no declared normal land intent plus actual PX4 AUTO_LAND state')
 if not disarmed or not actual_landed:reasons.append('missing actual final landed+disarmed state')
 finish=max((r['timestamp']/1e6 for r in actual_landed),default=None)
 samples=[r for r in truth.get('samples',[])if arm is not None and r['t']>=arm and(finish is None or r['t']<=finish)]
 if not samples:reasons.append('no complete actual moving truth track')
 p=np.array([[r['x'],r['y'],r['z']]for r in samples]);min_clearance=None
 if len(p):
  obs=Obstacles(a.actual_course_sdf);minimum=math.inf
  for start in range(0,len(p),200):minimum=min(minimum,float(np.min(obs.sdf(p[start:start+200])))-.30)
  min_clearance=minimum
  if minimum<0:reasons.append('actual full .30m sphere intersects course obstacle')
  moved=float(np.max(np.linalg.norm(p[:,:2]-p[0,:2],axis=1)))
  reached=float(np.min(np.linalg.norm(p[:,:2]-target[:2],axis=1)))
  final_distance=float(np.linalg.norm(p[-1,:2]-target[:2]))
  if moved<.5:reasons.append('no meaningful actual moving prefix')
  if reached>.15:reasons.append('did not reach declared bounded M3 capture region')
  if final_distance>.15:reasons.append('actual touchdown outside declared capture endpoint tolerance')
 else:moved=reached=final_distance=None
 if not truth.get('contact_topics_verified'):reasons.append('independent body/prop contact topics not verified')
 if truth.get('contacts'):reasons.append('actual non-ground contact recorded')
 source_db={'available':False}
 if a.sensors and a.sensors.exists():
  db=sqlite3.connect(f'file:{a.sensors.resolve()}?mode=ro',uri=True);integrity=db.execute('PRAGMA integrity_check').fetchone()[0];pairs=db.execute("SELECT count(*),min(l.stamp),max(l.stamp)FROM samples l JOIN samples r ON l.stamp=r.stamp WHERE l.kind='left' AND r.kind='right'").fetchone();infos=db.execute("SELECT kind,count(*) FROM samples WHERE kind IN('left_info','right_info') GROUP BY kind").fetchall();source_db={'available':True,'integrity':integrity,'matched_pairs':pairs[0],'first_stamp_ns':pairs[1],'last_stamp_ns':pairs[2],'camera_info_counts':infos,'sha256':sha(a.sensors)}
  if integrity!='ok'or pairs[0]<20 or len(infos)!=2:reasons.append('actual stereo archive missing/invalid/too short')
  db.close()
 else:reasons.append('genuine timestamped stereo archive unavailable')
 shadow=[r for r in events if r.get('event')=='shadow_obstacle_prediction'];ordinary=json.loads(a.ordinary_grade.read_text())if a.ordinary_grade and a.ordinary_grade.exists()else None
 output={'name':a.name,'experiment_id':a.experiment_id,'baseline_source_id':a.baseline_source_id,'goal':'actual bounded moving M3 sensor capture and observed stopping/normal-land outcome; NOT avoidance/fullcourse','capture_met_declared_goals':not reasons,'reasons':reasons,'what_changed':contract['changes']+[a.world_difference],'source_hashes':{name:sha(path)for name,path in zip(('contract','route','events','telemetry','truth','actual_course_sdf'),(a.contract,a.route,a.events,a.telemetry,a.truth,a.actual_course_sdf))},'actual_course_clearance_sphere_m':min_clearance,'actual_nonground_contacts':truth.get('contacts'),'actual_horizontal_travel_extent_m':moved,'actual_nearest_declared_capture_endpoint_xy_m':reached,'actual_final_endpoint_xy_error_m':final_distance,'actual_auto_land_seen':bool(auto_land),'actual_landed_disarmed_seen':bool(disarmed and actual_landed),'genuine_stereo_archive':source_db,'shadow_detector':{'state':'received'if shadow else'unavailable','receipt_events':len(shadow),'used_for_control':False},'actual_braking_observation':brake_measurement(events),'ordinary_full_course_grade_unchanged':ordinary,'scope':'Independent simulation-only report; truth never supplied to control; original full-course grade remains unchanged/expected incomplete; no hardware qualification'}
 a.output.write_text(json.dumps(output,indent=2));print(json.dumps({k:output[k]for k in('capture_met_declared_goals','reasons','shadow_detector','actual_braking_observation')},indent=2))
 return 0 if not reasons else 1
if __name__=='__main__':
 p=argparse.ArgumentParser()
 for key in('contract','route','events','telemetry','truth','actual-course-sdf','output'):p.add_argument('--'+key,type=Path,required=True)
 p.add_argument('--sensors',type=Path);p.add_argument('--ordinary-grade',type=Path);p.add_argument('--name',required=True);p.add_argument('--experiment-id',required=True);p.add_argument('--baseline-source-id',required=True);p.add_argument('--world-difference',required=True);sys.exit(main(p.parse_args()))
