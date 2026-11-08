"""Pure validation and measured-frame contracts for staged RETURN flight."""
import hashlib,json,math
from pathlib import Path

def sha(path):return hashlib.sha256(Path(path).read_bytes()).hexdigest()

def load_contract(path,expected=None,runtime=None):
 p=Path(path).resolve()
 if expected is not None and sha(p)!=expected:raise ValueError('return contract SHA mismatch')
 d=json.loads(p.read_text())
 if d.get('schema_version')!=1 or d.get('leg')!='return':raise ValueError('not a RETURN contract')
 if d.get('mode') not in ('standalone','combined'):raise ValueError('unknown return frame mode')
 def artifact(key):
  item=d[key];q=(p.parent/item['path']).resolve()
  if sha(q)!=item['sha256']:raise ValueError(key+' SHA mismatch')
  return q,json.loads(q.read_text())
 route_path,route=artifact('route');_,screen=artifact('offline_screen')
 for key in ('adapter','contract_validator'):
  if key in d:
   item=d[key];q=(p.parent/item['path']).resolve()
   if sha(q)!=item['sha256']:raise ValueError(key+' source SHA mismatch')
 if screen.get('route_sha256')!=d['route']['sha256'] or not screen.get('passed'):
  raise ValueError('offline screen does not bind passing RETURN route')
 if screen.get('controller')!=d['controller'] or any(not r.get('return_geometry_passed') for r in screen['runs']):
  raise ValueError('return controller/dynamics evidence mismatch')
 if not all(r['total_with_25s_takeoff_landing_reserve_s']<=d['duration_sim_s'] for r in screen['runs']):raise ValueError('return exceeds bounded duration')
 if route['origin_mode']!=d['mode']:raise ValueError('route/frame mode mismatch')
 target=[-2.5146,.9144]
 if math.dist(route['goal_course_enu'][:2],target)>1e-8 or math.dist(route['waypoints_enu'][-1][:2],target)>1e-8:raise ValueError('return target is not the actual original launch')
 if route.get('landing_center_tolerance_m')!=.20:raise ValueError('return target criterion differs')
 for entry in d.get('support_sources',[]):
  if sha((p.parent/entry['path']).resolve())!=entry['sha256']:raise ValueError('support source SHA mismatch')
 if runtime is not None and sha(Path(runtime)/'mission.py')!=d['base_mission_sha256']:raise ValueError('base mission changed after offline review')
 if runtime is not None and 'base_sources_sha256' in d and sha(Path(runtime)/'sources.sha256')!=d['base_sources_sha256']:raise ValueError('base source manifest changed')
 d['route_path']=str(route_path);d['route_data']=route;return d

def course_position(odom,origin,bearing_deg,start_height=.07):
 b=math.radians(bearing_deg);s,c=math.sin(b),math.cos(b)
 return (origin[0]+odom[0]*s-odom[1]*c,origin[1]+odom[0]*c+odom[1]*s,odom[2]+start_height)

def pad_start_ok(odom,origin,bearing_deg,start_height=.07):
 # This uses measured retained odom/PX4 navigation, never a simulator truth fix.
 p=course_position(odom,origin,bearing_deg,start_height)
 return math.dist(p[:2],(-2.04216,-.79248))<=.20 and -.03<=p[2]<=.18
