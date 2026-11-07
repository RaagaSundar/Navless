"""Independent simulation safety screen ONLY. Never an onboard control input."""
import argparse,json,math,sys
from pathlib import Path
import numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parents[2]/'gazebo_sim'))
from dh.geom import Obstacles,DRONE_PARTS

def main(a):
 data=json.loads(a.route.read_text());p=np.asarray(data['waypoints_enu'],float);samples=[]
 for x,y in zip(p,p[1:]):
  for f in np.linspace(0,1,max(2,math.ceil(np.linalg.norm(y-x)/.005)+1)):samples.append(x+(y-x)*f)
 # Actual endpoint hold and vertical autonomous landing column; floor touchdown
 # is expected and not part of course-model primitive set.
 for z in np.arange(p[-1,2],.07,-.005):samples.append(np.r_[p[-1,:2],z])
 q=np.asarray(samples);out={'scope':'independent collision/geometry screen, evaluator-only; never onboard input','spacing_m':.005,'body_sphere_m':.30,'expected_floor_touchdown':True,'max_part_extent_m':float(np.max(np.linalg.norm(DRONE_PARTS[:,:3],axis=1)+DRONE_PARTS[:,3])),'variants':{}}
 for name,file in [('clean',a.clean),('changed',a.changed)]:
  obs=Obstacles(file);best=math.inf;where=None
  for start in range(0,len(q),200):
   dist=obs.sdf(q[start:start+200]).min(axis=1)-.30;i=int(np.argmin(dist))
   if dist[i]<best:best=float(dist[i]);where=q[start+i].tolist()
  out['variants'][name]={'min_sphere_clearance_m':best,'min_point':where,'screened_route_and_landing_column':True,'positive_geometry_only':best>0}
 out['passed']=all(v['positive_geometry_only']for v in out['variants'].values())and out['max_part_extent_m']<=.30
 a.output.write_text(json.dumps(out,indent=2));print(json.dumps(out,indent=2))
if __name__=='__main__':
 p=argparse.ArgumentParser()
 for key in('route','clean','changed','output'):p.add_argument('--'+key,required=True,type=Path)
 main(p.parse_args())
