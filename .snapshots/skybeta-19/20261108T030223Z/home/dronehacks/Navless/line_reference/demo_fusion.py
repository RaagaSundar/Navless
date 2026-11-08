"""Seeded synthetic sensor replay, not a flight/simulator or real recording.

Run: uv run --with numpy python line_reference/demo_fusion.py
The map line and truth here are TEST FIXTURES ONLY, never mission inputs.
"""
import json
import math

import numpy as np

from fusion import FusionSession, LineSample, MapSegment, SlamSample
from reference import fit_reference


def run():
    rng = np.random.default_rng(7)
    session = FusionSession('odom', 'synthetic-1')
    segment = MapSegment('synthetic-line', (0, 0), (3, 0), 'odom', 'synthetic-1')
    raw_y, fused_y, raw_yaw, fused_yaw = [], [], [], []
    modes = {}
    for i in range(100):
        stamp = 1 + i*.05
        # Deliberately imposed SLAM errors; no claim of real cuVSLAM statistics.
        y = .10 * math.sin(i/30) + rng.normal(0, .015)
        yaw = .06 + rng.normal(0, .01)
        slam = SlamSample(stamp, stamp, (1+i*.01, y, yaw),
                          ((.01, 0, 0), (0, .01, 0), (0, 0, .01)),
                          'odom', 'synthetic-1', True, True)
        # True vehicle y=yaw=0; only measurement noise enters the floor line.
        offset, angle = rng.normal(0, .01), rng.normal(0, .02)
        points = [(x*math.cos(angle)-offset*math.sin(angle),
                   x*math.sin(angle)+offset*math.cos(angle))
                  for x in np.linspace(.5, 1.5, 20)]
        ref = fit_reference(points, stamp, stamp)
        line = LineSample(ref, ((.0004, 0), (0, .0009)), 'synthetic-line', True, True)
        # Deliberate image dropout. SLAM remains healthy.
        result = session.fuse(slam, None if 40 <= i < 50 else line, segment, stamp+.01)
        modes[result.mode] = modes.get(result.mode, 0)+1
        raw_y.append(y)
        fused_y.append(result.state[1])
        raw_yaw.append(yaw)
        fused_yaw.append(result.state[2])
    rms = lambda a: float(np.sqrt(np.mean(np.square(a))))
    return {'scenario': 'synthetic only; independent noise assumed for this demo',
            'seed': 7, 'samples': 100, 'modes': modes,
            'raw_lateral_rmse_m': rms(raw_y), 'fused_lateral_rmse_m': rms(fused_y),
            'raw_yaw_rmse_rad': rms(raw_yaw), 'fused_yaw_rmse_rad': rms(fused_yaw)}


if __name__ == '__main__':
    print(json.dumps(run(), indent=2))
