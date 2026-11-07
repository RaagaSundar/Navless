#!/usr/bin/env python3
"""Independent actual-world clearance; explicit archived SDF, no default/prior map."""
import argparse,hashlib,json,math,sys
from pathlib import Path
import numpy as np

def evaluate(track,comparison,repo):
    meta=json.loads(comparison.read_text());record=meta['actual']['model'];sdf=comparison.parent/record['path']
    assert hashlib.sha256(sdf.read_bytes()).hexdigest()==record['sha256'],'actual scene hash mismatch'
    sys.path.insert(0,str(repo/'gazebo_sim'));from dh.geom import Obstacles,DRONE_PARTS
    obs=Obstacles(str(sdf));data=json.loads(track.read_text());rows=data.get('samples',[]);out=[]
    for s in rows:
        if s['z']<=.18:continue
        q=s.get('orientation_xyzw');assert q is not None,'full physical quaternion required; no yaw substitute'
        x,y,z,w=map(float,q);assert all(math.isfinite(v) for v in q) and abs(sum(v*v for v in q)-1)<.02
        rot=np.array([[1-2*(y*y+z*z),2*(x*y-z*w),2*(x*z+y*w)],[2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w)],[2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y)]])
        xyz=np.array([s['x'],s['y'],s['z']]);assert np.isfinite(xyz).all();parts=DRONE_PARTS[:,:3]@rot.T+xyz
        matrix=obs.sdf(parts)-DRONE_PARTS[:,3,None];ind=np.unravel_index(np.argmin(matrix),matrix.shape);out.append({'t':s['t'],'clearance_m':float(matrix[ind]),'nearest_obstacle':obs.names[ind[1]]})
    assert out,'no genuine airborne full-pose evidence'
    contacts=data.get('contacts',[]);verified=data.get('contact_topics_verified',False)
    return {'test_kind':'actual simulated flight independent physical check','actual_model_sha256':record['sha256'],'map_comparison_sha256':hashlib.sha256(comparison.read_bytes()).hexdigest(),'track_sha256':hashlib.sha256(track.read_bytes()).hexdigest(),'min_full_attitude_parts_clearance_m':min(s['clearance_m'] for s in out),'contact_topics_verified':verified,'recorded_contacts':contacts,'clearance_and_contact_passed':verified and not contacts and min(s['clearance_m'] for s in out)>0,'samples':out,'scope':'case-dependent actual physical geometry only; original course grader unchanged. Bounded stop/landing is not full-course completion; execution/guard evidence required separately.'}
if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('track',type=Path);p.add_argument('--map-comparison',type=Path,required=True);p.add_argument('--repo',type=Path,required=True);p.add_argument('--output',type=Path,required=True);a=p.parse_args();result=evaluate(a.track,a.map_comparison,a.repo);a.output.write_text(json.dumps(result,indent=2)+'\n');print(json.dumps({k:v for k,v in result.items() if k!='samples'},indent=2))
