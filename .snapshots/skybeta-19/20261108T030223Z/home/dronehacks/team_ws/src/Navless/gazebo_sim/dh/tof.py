"""Simulated side time-of-flight sensors: what the drone's two side VL53L1X sensors would read at a pose.

  from dh.tof import side_ranges, side_ranges_q
  r = side_ranges(pos_world, yaw_enu, roll=0.0, pitch=0.0)     # or side_ranges_q(pos_world, (qx, qy, qz, qw))
  r["left"]  -> (range_m, hit_xyz or None)       r["right"] -> (...)

Frames: world x = east, y = north, z = up (Gazebo course frame). Body FLU (x forward, y left, z up).
yaw_enu = heading of body +x, radians CCW from east (nose north = pi/2). roll / pitch about body x / y, radians,
REP-103 sense (positive pitch = nose down); attitude = Rz(yaw) Ry(pitch) Rx(roll).

Sensors (as mounted in tube/tof_driver.py): body (-0.019, +0.075, -0.060) left, (-0.019, -0.075, -0.060) right,
beams along body +y / -y, 15 deg field of view. Each beam is 5 rays (centre + 4 at the cone edge); the reading is
the nearest return, like the sensor's closest-target ranging. Valid 0.04-4.0 m (long mode): math.inf when nothing
is within 4 m, 0.04 when closer than 0.04 m. Casts against the course collision model, net walls and floor included.
"""
import math

import numpy as np

from .geom import RayCaster

MOUNTS = {"left": (np.array([-0.019, 0.075, -0.060]), np.array([0.0, 1.0, 0.0])),
          "right": (np.array([-0.019, -0.075, -0.060]), np.array([0.0, -1.0, 0.0]))}
FOV_DEG = 15.0
RANGE_MIN, RANGE_MAX = 0.04, 4.0

_caster = None


def _rays(direction, half_angle):
    """Centre ray plus 4 rays on the cone edge (two perpendicular axes)."""
    a = np.array([1.0, 0, 0]) if abs(direction[0]) < 0.9 else np.array([0, 0, 1.0])
    u = np.cross(direction, a)
    u /= np.linalg.norm(u)
    v = np.cross(direction, u)
    c, s = math.cos(half_angle), math.sin(half_angle)
    return np.array([direction] + [c * direction + s * w for w in (u, -u, v, -v)])


_BODY_RAYS = {k: _rays(d, math.radians(FOV_DEG / 2)) for k, (_, d) in MOUNTS.items()}


def _attitude(yaw, pitch, roll):
    cy, sy, cp, sp, cr, sr = math.cos(yaw), math.sin(yaw), math.cos(pitch), math.sin(pitch), math.cos(roll), math.sin(roll)
    return np.array([[cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
                     [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
                     [-sp, cp * sr, cp * cr]])


def side_ranges(pos_world, yaw_enu, roll=0.0, pitch=0.0):
    return _ranges(np.asarray(pos_world, float), _attitude(yaw_enu, pitch, roll))


def side_ranges_q(pos_world, q_xyzw):
    """Same, with the attitude as a quaternion (x, y, z, w): body FLU -> world ENU (Gazebo pose convention)."""
    x, y, z, w = q_xyzw
    n = math.sqrt(x * x + y * y + z * z + w * w) or 1.0
    x, y, z, w = x / n, y / n, z / n, w / n
    R = np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                  [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                  [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])
    return _ranges(np.asarray(pos_world, float), R)


def _ranges(p, R):
    global _caster
    if _caster is None:
        _caster = RayCaster()
    names = list(MOUNTS)
    origins, dirs, owner = [], [], []
    for k in names:
        o = p + R @ MOUNTS[k][0]
        for d in _BODY_RAYS[k]:
            origins.append(o)
            dirs.append(R @ d)
            owner.append(k)
    t = _caster.cast_many(origins, dirs)
    out = {}
    for k in names:
        idx = [i for i, w in enumerate(owner) if w == k]
        j = idx[int(np.argmin(t[idx]))]
        r = float(t[j])
        if r > RANGE_MAX:
            out[k] = (math.inf, None)
        else:
            hit = origins[j] + dirs[j] * r
            out[k] = (max(r, RANGE_MIN), tuple(round(float(v), 4) for v in hit))
    return out


if __name__ == "__main__":
    import time
    launch = (-2.5146, 0.9144, 0.85)
    cases = [
        ("launch mark, nose north (corridor 1: net west, M1 east)", launch, math.pi / 2, 0, 0),
        ("same, rolled 10 deg right", launch, math.pi / 2, math.radians(10), 0),
        ("maze corridor x=-1.37 (M1 west, M2 east) nose north", (-1.3716, 2.0, 0.94), math.pi / 2, 0, 0),
        ("tube centre, nose south", (2.3114, 0.4572, 0.94), -math.pi / 2, 0, 0),
        ("open floor near the pad, nose north", (-2.04, -0.79, 0.9), math.pi / 2, 0, 0),
    ]
    for name, p, yaw, roll, pitch in cases:
        r = side_ranges(p, yaw, roll, pitch)
        print(f"{name}: left {r['left'][0]:.3f} m at {r['left'][1]}, right {r['right'][0]:.3f} m at {r['right'][1]}")
    t0 = time.perf_counter()
    for i in range(300):
        side_ranges((-2.5146 + 0.01 * i, 0.9144, 0.85), math.pi / 2)
    print(f"{(time.perf_counter() - t0) / 300 * 1e3:.2f} ms per call (both sensors)")
