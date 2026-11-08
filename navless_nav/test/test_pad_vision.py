"""pad_vision in course_mission.py, on test_fly_home.py's stub ROS and a point-mass drone.

The pad is found by camera wherever it is (the course can change). The stub camera sees the
pad only where the real forward one can: 0.9-2.2 m ahead of the nose, within ~30 degrees of it,
not from above. The stub drone turns to a pose setpoint's yaw (as flight_interface makes PX4 do).
"""
import json
import math
import os
import sys
import types

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import test_fly_home as h    # noqa: E402  (stubs ROS, imports course_mission)
from navless_nav.pad_tracker import PadTracker    # noqa: E402

cm = h.course_mission


def pad_msg(x, y, stamp=None, frame='odom'):
    t = h.T[0] if stamp is None else stamp
    sec = int(t)
    return types.SimpleNamespace(
        header=types.SimpleNamespace(
            stamp=types.SimpleNamespace(sec=sec, nanosec=int((t - sec) * 1e9)), frame_id=frame),
        point=types.SimpleNamespace(x=x, y=y, z=1.0))


def visible(pos, yaw, pad):
    dx, dy = pad[0] - pos[0], pad[1] - pos[1]
    ahead = dx * math.cos(yaw) + dy * math.sin(yaw)
    left = -dx * math.sin(yaw) + dy * math.cos(yaw)
    return pos[2] > 0.2 and 0.9 <= ahead <= 2.2 and abs(left) <= 0.58 * ahead


def fly(node, px4, drone, until, pad, false_fixes=(), max_s=900.0, dt=0.1):
    """test_fly_home.fly with a heading and the stub camera. drone = [x, y, z, yaw]."""
    end = h.T[0] + max_s
    states = set()
    done = False
    while not done:
        done = until()
        assert h.T[0] < end, f'timed out in {node.state} {node.pad.get("phase")} at {drone}'
        x, y, z, yaw = drone
        h.send_odom(node, x, y, z, yaw=yaw)
        h.vslam(node)
        h.px4_heading(node, yaw)
        if pad is not None and visible(drone[:3], yaw, pad):
            node.pad_center_cb(pad_msg(*pad))
        for t, fx, fy in false_fixes:
            if abs(h.T[0] - t) < dt / 2:
                node.pad_center_cb(pad_msg(fx, fy))
        px4['local_enu'] = [x, y, z]
        node.state_cb(types.SimpleNamespace(data=json.dumps(px4)))
        h.SENT.clear()
        node.tick()
        states.add(node.state)
        if px4['armed'] and h.SENT:
            topic, m = h.SENT[-1]
            if topic == cm.POSE_TOPIC:
                goal = (m.pose.position.x, m.pose.position.y, m.pose.position.z)
                d = math.dist(drone[:3], goal)
                f = min(1.0, 1.5 * dt / d) if d > 0 else 0.0
                drone[:3] = [a + (b - a) * f for a, b in zip(drone[:3], goal)]
                o = m.pose.orientation                  # unset stub fields are Msg, i.e. 0
                qx, qy, qz, qw = (v if isinstance(v, (int, float)) else 0.0
                                  for v in (o.x, o.y, o.z, o.w))
                if any((qx, qy, qz, qw)):               # turn at up to 60 deg/s
                    want = math.atan2(2 * (qw * qz + qx * qy), 1 - 2 * (qy ** 2 + qz ** 2))
                    err = math.remainder(want - drone[3], 2 * math.pi)
                    drone[3] += max(-math.radians(6), min(math.radians(6), err))
            else:
                v = m.twist.linear
                drone[:3] = [drone[0] + v.x * dt, drone[1] + v.y * dt, drone[2] + v.z * dt]
            drone[2] = max(0.0, drone[2])
            px4['landed'] = drone[2] == 0.0
            if px4['landed'] and node.state in ('LAND', 'REFUSED'):
                px4.update(armed=False, offboard_active=False)
        h.T[0] += dt
    return states


def run(pad_offset=(0.0, 0.0), see_pad=True, false_fixes=(), **overrides):
    """Fly route_gate_b.json; the real pad sits pad_offset (odom) from the route's end."""
    h.T[0] = 0.0
    h.OVERRIDES.clear()
    h.OVERRIDES.update(route_file='route_gate_b.json', cruise_speed_mps=1.0, **overrides)
    node = cm.CourseMission()
    px4 = {'armed': False, 'offboard_active': False, 'landed': True}
    drone = [0.0, 0.0, 0.0, 0.0]
    planned = node.route[-1]
    pad = (planned[0] + pad_offset[0], planned[1] + pad_offset[1]) if see_pad else None
    fly(node, px4, drone, lambda: h.T[0] >= 2.5, pad, false_fixes)
    px4.update(armed=True, offboard_active=True)
    states = fly(node, px4, drone, lambda: not px4['armed'], pad, false_fixes)
    return node, drone, planned, pad, states


def test_off_by_default_lands_on_the_route_end():
    node, drone, planned, pad, states = run(pad_offset=(0.3, -0.2))
    assert node.state == 'DONE' and 'PAD_FIND' not in states
    assert '/team/pad/center' not in h.SUBS
    assert math.dist(drone[:2], planned[:2]) < 0.1


def test_drifted_pad_lands_where_the_camera_saw_it():
    node, drone, planned, pad, states = run(pad_offset=(0.3, -0.2), pad_vision=True)
    assert node.state == 'DONE' and 'PAD_FIND' in states and drone[2] == 0.0
    assert math.dist(drone[:2], pad) < 0.05


def test_moved_pad_is_found_and_landed_on():
    # The course changed: the pad is 1.1 m from where the map says (open floor on the map).
    node, drone, planned, pad, states = run(pad_offset=(-1.0, -0.5), pad_vision=True)
    assert node.state == 'DONE' and drone[2] == 0.0
    assert math.dist(drone[:2], pad) < 0.05
    assert math.dist(drone[:2], planned[:2]) > 1.0


def test_scattered_false_fixes_never_steer():
    # No pad in view at all; three one-off detections in different places.
    fakes = [(20.0, 1.0, 1.0), (30.0, -1.0, -2.0), (40.0, -2.5, 0.5)]
    node, drone, planned, pad, states = run(see_pad=False, false_fixes=fakes, pad_vision=True)
    assert node.state == 'DONE' and 'PAD_FIND' in states
    assert node.pad_tracker.best() is None
    assert math.dist(drone[:2], planned[:2]) < 0.1        # full turn, nothing: the route's end


def test_tracker_confirms_agreement_not_one_offs():
    tr = PadTracker(confirm=3)
    tr.add(0.0, 5.0, 5.0)
    tr.add(1.0, 1.0, 2.0)
    tr.add(2.0, 1.05, 2.0)
    assert tr.best() is None
    tr.add(3.0, 1.02, 2.02)
    x, y, n, t = tr.best()
    assert n == 3 and math.dist((x, y), (1.023, 2.007)) < 0.01 and t == 3.0
