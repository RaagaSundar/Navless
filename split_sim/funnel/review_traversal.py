"""Independent post-flight check against the two collision tips in the archived world."""
import json
import math
import sys
from pathlib import Path
from xml.etree import ElementTree as ET


def wall_tips(world):
    tips = []
    for name in ('left','right'):
        model = world.find(f"model[@name='{name}']")
        x,y,_,_,_,yaw = map(float,model.findtext('pose').split())
        length,width,_ = map(float,model.findtext('link/collision/geometry/box/size').split())
        corners = [(x+sx*math.cos(yaw)-sy*math.sin(yaw),
                    y+sx*math.sin(yaw)+sy*math.cos(yaw))
                   for sx in (-length/2,length/2) for sy in (-width/2,width/2)]
        tips.append(min(corners,key=lambda point: abs(point[1])))
    if abs(tips[0][0]-tips[1][0]) > 1e-5:
        raise ValueError('expected a transverse exit plane')
    return tips


def review(run):
    track = json.loads((run/'flight/track.json').read_text())
    tips = wall_tips(ET.parse(run/'funnel.sdf').find('world'))
    plane = tips[0][0]
    middle = (tips[0][1]+tips[1][1])/2
    samples = track['samples']
    air = [s for s in samples if s['armed'] and not s['landed'] and s['status_age'] <= 1.5]
    cross = None
    for a,b in zip(air,air[1:]):
        if a['x'] < plane <= b['x'] and 0 < b['t']-a['t'] < .10:
            f = (plane-a['x'])/(b['x']-a['x'])
            cross = {k:a[k]+f*(b[k]-a[k]) for k in ('t','x','y','z')}
            break
    complete = bool(samples and samples[-1]['status_age'] <= 1.5 and samples[-1]['landed'] and not samples[-1]['armed'])
    # First fresh landed and disarmed sample following the airborne interval.
    landing = next((s for s in samples if air and s['t']>air[-1]['t'] and s['landed']
                    and not s['armed'] and s['status_age'] <= 1.5),None)
    offset = cross['y']-middle if cross else None
    approach = [abs(s['y']-middle) for s in air if plane-1 <= s['x'] <= plane]
    phases = [json.loads(line) for line in (run/'flight/mission.log').read_text().splitlines()
              if line.startswith('{"event": "passage"')]
    far_side = landing['x']-plane if landing else None
    verified = track['contact_topics_verified'] and not track['contacts']
    successful = bool(cross and abs(offset) < math.dist(*tips)/2 and 0 < cross['z'] < 1.73
                      and far_side is not None and far_side > .25 and complete and verified)
    return dict(run_id=run.name,crossing_verified=successful,airborne_samples=len(air),
                opening_width_m=math.dist(*tips), center_at_crossing=cross, lateral_error_m=offset,
                max_centerline_error_last_meter_m=max(approach,default=None),
                landing_distance_beyond_plane_m=far_side, final_landed_disarmed=complete,
                contacts_verified=track['contact_topics_verified'],wall_contacts=len(track['contacts']),
                phases=phases)


if __name__ == '__main__':
    print(json.dumps(review(Path(sys.argv[1])),indent=2))
