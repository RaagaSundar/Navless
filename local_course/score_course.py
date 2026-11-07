#!/usr/bin/env python3
"""Independent offline course evidence evaluator. Never publishes commands.
Input JSON: samples=[{t,x,y,z,yaw,armed,landed,status_age}], contacts=[...].
Ground-truth samples are exclusively grading evidence, never a navigation input.
"""
import argparse
import json
import math
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[1]


ENVELOPE = .30  # m: conservative sphere enclosing every stand-in part at any attitude


def crossing(samples, axis, value, low, high, other, after, direction=1):
    for a, b in zip(samples, samples[1:]):
        if b['t'] <= after:
            continue
        u, v = a[axis] - value, b[axis] - value
        if (u == 0 or u * v < 0) and (v-u)*direction > 0:
            f = u / (u - v) if u != v else 0
            t = a['t'] + f * (b['t'] - a['t'])
            o = a[other] + f * (b[other] - a[other])
            z = a['z'] + f * (b['z'] - a['z'])
            if t > after and low <= o <= high and z > .18:
                return t, z
    return None


def grade(data, repo=None, clearance_fn=None):
    repo = Path(repo) if repo else ROOT
    course = json.loads((repo / 'gazebo_sim/course/course_v3.json').read_text())
    samples = data.get('samples', [])
    reasons, checkpoints = [], {}
    if len(samples) < 3:
        return {'passed': False, 'reasons': ['insufficient samples']}
    if any(b['t'] <= a['t'] or b['t'] - a['t'] > .5 for a, b in zip(samples, samples[1:])):
        reasons.append('nonmonotonic timestamps or ground-truth gap >0.5 simulated seconds')
    if any(not all(math.isfinite(float(s[k])) for k in ('t','x','y','z','yaw')) for s in samples):
        return {'passed': False, 'reasons': ['nonfinite samples']}
    if not data.get('contact_topics_verified', False):
        reasons.append('contact sensors not verified')
    if data.get('contacts'):
        reasons.append('obstacle contacts recorded')
    if any(s.get('status_age', math.inf) > 2.5 for s in samples):
        reasons.append('stale or missing PX4 status/land telemetry')
    if samples[0]['armed'] or not samples[0]['landed']:
        reasons.append('recording did not start landed and disarmed')
    if not any(s['armed'] and not s['landed'] and s['z'] > .25 for s in samples):
        reasons.append('no observed armed flight')
    W = lambda v: v * .0254 - 3.048
    ordered = [('corridor_done','y',W(203.5),W(0)+ENVELOPE,W(42)-ENVELOPE,'x',1),
               ('maze_m2','x',W(88),W(140)+ENVELOPE,W(174.5)-ENVELOPE,'y',1),
               ('gate_a','x',W(132),W(203.5)+ENVELOPE,W(240)-ENVELOPE,'y',1),
               ('tube_in','y',W(168),W(182)+ENVELOPE,W(240)-ENVELOPE,'x',-1),
               ('tube_out','y',W(108),W(182)+ENVELOPE,W(240)-ENVELOPE,'x',-1)]
    after = samples[0]['t']
    for name, axis, v, low, high, other, direction in ordered:
        found = crossing(samples, axis, v + direction*ENVELOPE, low, high, other, after,direction)
        if found is None:
            reasons.append('missing ordered checkpoint: ' + name)
            break
        checkpoints[name] = after = found[0]
    wall = next(w for w in course['walls'] if 'window' in w)
    a = [W(v) for v in wall['start_in']]; b = [W(v) for v in wall['end_in']]
    c = [W(v) for v in wall['window']['center_in']]
    length = math.hypot(b[0]-a[0], b[1]-a[1]); ux,uy = (b[0]-a[0])/length,(b[1]-a[1])/length
    rotated = [dict(s, along=(s['x']-c[0])*ux+(s['y']-c[1])*uy,
                    across=-(s['x']-c[0])*uy+(s['y']-c[1])*ux) for s in samples]
    width = wall['window']['width_in']*.0254
    window = crossing(rotated,'across',ENVELOPE,-width/2+ENVELOPE,width/2-ENVELOPE,'along',after,1)
    if window is None:
        reasons.append('missing ordered window crossing')
    else:
        checkpoints['window'] = window[0]
        # Full prop-envelope clearance below independently checks opening boundaries.
        if not .3048+ENVELOPE <= window[1] <= 1.2192-ENVELOPE:
            reasons.append('window crossing outside vertical opening')
    final = samples[-1]
    landed_tail = [s for s in samples if s['t'] >= final['t']-2]
    if (final['t']-landed_tail[0]['t'] < 1.8 or
            any(s['armed'] or not s['landed'] or s['z'] > .18 for s in landed_tail)):
        reasons.append('no stable 2-second landed and disarmed finish')
    pad = [W(v) for v in course['landing_pad_in']['center']]
    pad_distance = math.hypot(final['x']-pad[0],final['y']-pad[1])
    pad_radius = course['landing_pad_in']['outer_ring_radius']*.0254
    if pad_distance+ENVELOPE > pad_radius:
        reasons.append('full aircraft not contained inside outer landing pad ring')
    air_indices=[i for i,s in enumerate(samples) if s['armed'] and not s['landed'] and s['z']>.25]
    if air_indices:
        first,last=air_indices[0],air_indices[-1]
        if any(s['landed'] or not s['armed'] for s in samples[first:last+1]):
            reasons.append('intermediate landing or disarm during course')
    wall_height=min(w['height_in'] for w in course['walls'])*.0254
    if any(s['z']+ENVELOPE > wall_height for s in samples):
        reasons.append('aircraft envelope exceeds wall height; overflight prohibited')
    if clearance_fn is None:
        import numpy as np
        sys.path.insert(0, str(repo/'gazebo_sim'))
        from dh.geom import Obstacles, clearance
        obs = Obstacles()
        clearance_fn = lambda rows: clearance(obs, np.array([[s['x'],s['y'],s['z']] for s in rows]),
                                               math.pi/2-np.array([s['yaw'] for s in rows]))[0].tolist()
    flying = [s for s in samples if s['z'] > .18]
    minimum = min(clearance_fn(flying)) if flying else None
    if minimum is None or minimum <= 0:
        reasons.append('nonpositive prop-envelope clearance or no flight')
    return {'passed': not reasons, 'reasons': reasons, 'checkpoints_sim_s':checkpoints,
            'pad_distance_cm':round(pad_distance*100,2), 'min_clearance_cm':None if minimum is None else round(minimum*100,2),
            'samples':len(samples), 'contacts':data.get('contacts',[]),
            'aircraft_envelope_radius_m':ENVELOPE, 'scope':'conservative full-aircraft simulation grading; truth solely for independent scoring'}


if __name__ == '__main__':
    ap=argparse.ArgumentParser(description=__doc__); ap.add_argument('evidence'); ap.add_argument('--repo'); ap.add_argument('--output')
    args=ap.parse_args(); result=grade(json.loads(Path(args.evidence).read_text()),args.repo)
    text=json.dumps(result,indent=2); print(text)
    if args.output: Path(args.output).write_text(text+'\n')
    raise SystemExit(0 if result['passed'] else 1)
