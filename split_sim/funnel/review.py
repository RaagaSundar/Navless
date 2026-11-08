"""Summarize a funnel hover from independently recorded motion and onboard estimates."""
import argparse
import json
import math
from pathlib import Path
from statistics import median


def review(run):
    track = json.loads((run/'flight/track.json').read_text())
    samples = track['samples']
    airborne = [s for s in samples if s['armed'] and not s['landed'] and s['status_age'] < 1.5]
    ground = [s for s in samples if s['landed'] and not s['armed'] and s['t'] < airborne[0]['t']]
    origin = [median([s[k] for s in ground]) for k in ('x','y','z')]
    obs = []
    request = None
    for line in (run/'flight/mission.log').read_text().splitlines():
        if line.startswith('{"stamp_ns"'):
            obs.append(json.loads(line))
        elif 'requesting PX4 autonomous landing' in line and obs:
            request = obs[-1]['stamp_ns'] / 1e9
    def errors(t0,t1):
        return [s['aim']['bearing_right_deg'] for s in obs
                if t0 <= s['stamp_ns']/1e9 <= t1 and s['aim']]
    first = airborne[0]['t']
    return dict(run_id=run.name, airborne_samples=len(airborne), peak_height_above_launch_m=max(s['z'] for s in airborne)-origin[2],
                max_horizontal_offset_m=max(math.hypot(s['x']-origin[0],s['y']-origin[1]) for s in airborne),
                initial_median_bearing_right_deg=median(errors(first-1,first)),
                pre_land_median_bearing_right_deg=median(errors(request-1,request)),
                pre_land_frame_count=len(errors(request-1,request)),
                pre_land_approx_sim_s=request,contact_topics_verified=track['contact_topics_verified'],
                recorded_non_ground_contacts=len(track['contacts']),
                final_landed=samples[-1]['landed'],final_armed=samples[-1]['armed'])


if __name__ == '__main__':
    parser=argparse.ArgumentParser()
    parser.add_argument('run',type=Path)
    args=parser.parse_args()
    print(json.dumps(review(args.run),indent=2))
