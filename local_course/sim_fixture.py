"""Static launch fixture only; no live truth interface or vehicle commands."""
import json
import math
import os
from pathlib import Path


def fixture(bearing, metadata):
    bearing=float(bearing)
    if not math.isfinite(bearing):raise ValueError('nonfinite launch bearing')
    bearing=bearing%360
    if bearing not in (0.,90.):raise ValueError('reviewed fixtures are north0 or east90 degrees')
    yaw=math.radians(90-bearing)
    return {'start_bearing_deg':bearing,'yaw_enu_rad':yaw,'spawn_enu_m':metadata['spawn_enu_m'],
            'orientation_xyzw':[0.,0.,math.sin(yaw/2),math.cos(yaw/2)],
            'note':'Immutable static physical launch fixture, never runtime truth. Bridge and mission must read this exact recorded bearing.'}


def main():
    run=Path(os.environ['RUN']); data=fixture(os.environ.get('COURSE_START_BEARING','0'),json.load(open('/local_course/sim-frames.json')))
    path=run/'fixture.json'
    with path.open('x') as f:json.dump(data,f,indent=2)
    os.chmod(path,0o444)
    x,y,z=data['spawn_enu_m'];qx,qy,qz,qw=data['orientation_xyzw']
    req=f'sdf_filename: "/course-state/assets/dh_quad_stereo.sdf", name: "dh_quad_0", allow_renaming: false, pose: {{position: {{x: {x}, y: {y}, z: {z}}}, orientation: {{x: {qx}, y: {qy}, z: {qz}, w: {qw}}}}}'
    (run/'spawn-request.txt').write_text(req)
if __name__=='__main__':main()
