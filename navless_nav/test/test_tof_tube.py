"""Side ToF tube centring in course_mission.py (tof_tube_centering), on test_tube_blind.py's stub ROS
and point-mass drone.

tube/tube_node.py publishes /tube/offset (how far the tube centreline is to the drone's left) only
while both side readings span the tube. In the tube, course_mission pulls its target toward that
centre: at most tof_tube_max_shift_m from the route, at tof_tube_rate_mps, and straight back to the
route without fresh readings. The closed-loop test reads the sensors off the course model
(gazebo_sim/dh/tof.py) through tube_node's own gates, with the real tube placed off the route.
"""
import math
import os
import sys
import types

import pytest

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import test_tube_blind as tb    # noqa: E402  (stub ROS, point-mass drone, fly_to_tube)
from navless_nav.route import REPO_ROOT, odom_to_course    # noqa: E402

h, cm = tb.h, tb.cm
RATE = 0.10                     # tof_tube_rate_mps default
TICK = 0.1


def offset_msg(offset, stamp=None, frame='base_link'):
    """tube_node's /tube/offset: the centreline is `offset` m to the drone's left."""
    t = h.T[0] if stamp is None else stamp
    sec = int(t)
    return types.SimpleNamespace(
        header=types.SimpleNamespace(
            stamp=types.SimpleNamespace(sec=sec, nanosec=int((t - sec) * 1e9)), frame_id=frame),
        point=types.SimpleNamespace(x=0.0, y=offset, z=1.45))


def course_x(node, p):
    return odom_to_course(p, node.frame[0], node.frame[1])[0]


def run(node, px4, pos, centre_x, ticks, yaw=0.0, **kw):
    """`ticks` ticks with tube_node reporting a tube centred on course x `centre_x`, as the sensors
    see it from the drone's real position. Nose north, so the drone's left is west; a heading `yaw`
    off the tube stretches both readings by 1/cos(yaw). Per tick: (drone x, route x, target x)."""
    out = []
    for _ in range(ticks):
        node.tof_offset_cb(offset_msg((course_x(node, pos) - centre_x) / math.cos(yaw)))
        tb.step(node, px4, pos, yaw=yaw, **kw)
        out.append((course_x(node, pos), course_x(node, node.carrot.point),
                    course_x(node, tb.last_pose(node))))
    return out


def in_tube_now(node, pos):
    return node.in_tube(pos, node.carrot.point)


def enter_tube(node, px4, pos):
    """On from fly_to_tube (carrot in the tube) until the drone is in it too."""
    for _ in range(100):
        if in_tube_now(node, pos):
            return
        tb.step(node, px4, pos)
    raise AssertionError('drone never entered the tube')


def shifts(rows):
    return [target - route for _, route, target in rows]


def test_off_by_default():
    h.SUBS.clear()
    node, px4, pos = tb.fly_to_tube()
    assert '/tube/offset' not in h.SUBS
    rows = run(node, px4, pos, course_x(node, node.carrot.point) - 0.10, 15)
    assert all(abs(s) < 1e-9 for s in shifts(rows))


def test_pulls_toward_the_measured_centre_at_the_rate_limit():
    h.SUBS.clear()
    node, px4, pos = tb.fly_to_tube(tof_tube_centering=True)
    assert '/tube/offset' in h.SUBS
    enter_tube(node, px4, pos)
    centre = course_x(node, node.carrot.point) - 0.10      # real tube 10 cm west of the route
    rows = run(node, px4, pos, centre, 15)
    s = shifts(rows)
    assert all(abs(b - a) <= RATE * TICK + 1e-9 for a, b in zip([0.0] + s, s))
    assert s[-1] == pytest.approx(-0.10, abs=0.01)          # target on the real centre
    assert rows[-1][0] == pytest.approx(centre, abs=0.02)   # and the drone with it


def test_stale_readings_drop_the_correction_at_once():
    node, px4, pos = tb.fly_to_tube(tof_tube_centering=True)
    enter_tube(node, px4, pos)
    run(node, px4, pos, course_x(node, node.carrot.point) - 0.10, 10)
    assert node.tof_guidance.shift > 0.05           # along the tube's left normal: west here
    for _ in range(4):                                      # tube_node silent for 0.4 s > 0.3 s
        tb.step(node, px4, pos)
    assert course_x(node, tb.last_pose(node)) == pytest.approx(course_x(node, node.carrot.point))
    assert node.tof_guidance.shift == 0.0


def test_never_outside_the_tube():
    h.T[0] = 0.0
    h.OVERRIDES.clear()
    h.OVERRIDES.update(route_file='gazebo_sim/course/route_centreline_window.json',
                       cruise_speed_mps=0.5, tof_tube_centering=True)
    node = cm.CourseMission()
    px4 = {'armed': False, 'offboard_active': False, 'landed': True}
    pos = [0.0, 0.0, 0.0]
    for _ in range(30):
        tb.step(node, px4, pos)
    px4.update(armed=True, offboard_active=True)
    for _ in range(100):                                    # into the first corridor
        tb.step(node, px4, pos)
    assert node.state == 'ROUTE' and not in_tube_now(node, pos)
    rows = run(node, px4, pos, course_x(node, pos) - 0.10, 20)
    assert all(abs(s) < 1e-9 for s in shifts(rows))


def test_correction_never_exceeds_the_limit():
    node, px4, pos = tb.fly_to_tube(tof_tube_centering=True)
    enter_tube(node, px4, pos)
    rows = run(node, px4, pos, course_x(node, node.carrot.point) - 0.40, 25)
    s = shifts(rows)
    assert min(s) == pytest.approx(-0.15, abs=1e-6) and min(s) >= -0.15 - 1e-9


def test_guidance_clamp_is_opt_in():
    """Stereo (default) still drops a centre beyond max_shift; ToF (clamp) pulls max_shift toward it."""
    from navless_nav.tube_guidance import TubeGuidance
    for clamp, expected in ((False, 0.0), (True, 0.15)):
        g = TubeGuidance(max_shift=0.15, rate=10.0, timeout=0.3, clamp=clamp)
        for t in (0.0, 0.1):
            g.update(t, [0, 0.40, 0.85], 0, t)
            out = g.apply([0.4, 0, 0.85], [0, 0, 0.85], 0, t, enabled=True, max_lead=0.6)
        assert out[1] == pytest.approx(expected)


def test_ignores_wrong_frame_and_nan():
    node, px4, pos = tb.fly_to_tube(tof_tube_centering=True)
    node.tof_offset_cb(offset_msg(0.1, frame='map'))
    assert node.tof_offset is None
    node.tof_offset_cb(offset_msg(math.nan))
    assert node.tof_offset is None
    node.tof_offset_cb(offset_msg(0.1))
    assert node.tof_offset is not None


def test_with_stereo_on_steers_on_stereo_only():
    h.SUBS.clear()
    h.OVERRIDES.clear()
    h.OVERRIDES.update(stereo_tube_centering=True, tof_tube_centering=True)
    node = cm.CourseMission()
    assert node.stereo_tube and not node.tof_tube
    assert '/tube/offset' not in h.SUBS


def test_keeps_correcting_on_px4_estimate_during_a_blind_carry():
    node, px4, pos = tb.fly_to_tube(tof_tube_centering=True, blind_carry=True)
    enter_tube(node, px4, pos)
    centre = course_x(node, node.carrot.point) - 0.10
    run(node, px4, pos, centre, 12)
    rows = run(node, px4, pos, centre, 8, vo_state=2)       # VIO lost: no odometry
    assert node.blind_carrying
    assert shifts(rows)[-1] == pytest.approx(-0.10, abs=0.01)


def test_frozen_carrot_drops_the_correction():
    node, px4, pos = tb.fly_to_tube(tof_tube_centering=True)  # blind_carry off
    enter_tube(node, px4, pos)
    centre = course_x(node, node.carrot.point) - 0.10
    run(node, px4, pos, centre, 12)
    rows = run(node, px4, pos, centre, 3, vo_state=2)
    assert shifts(rows)[-1] == 0.0                         # holds the frozen carrot, as before


def test_heading_off_the_tube_still_finds_the_centre():
    node, px4, pos = tb.fly_to_tube(tof_tube_centering=True)
    enter_tube(node, px4, pos)
    centre = course_x(node, node.carrot.point) - 0.10
    rows = run(node, px4, pos, centre, 15, yaw=math.radians(12))
    assert shifts(rows)[-1] == pytest.approx(-0.10, abs=0.01)


@pytest.mark.parametrize('off_centre', [0.10, -0.12])
def test_closed_loop_on_the_course_model(off_centre):
    """The drone powered up off the launch mark, so the whole route runs `off_centre` m east of the
    real tube's centreline. Sensors simulated off the course model, through tube_node's gates with
    the launch file's settings: only the run with ToF centring ends up on the centreline."""
    sys.path[:0] = [os.path.join(REPO_ROOT, 'gazebo_sim'), os.path.join(REPO_ROOT, 'tube')]
    from dh.tof import side_ranges
    from tube_node import tube_offset
    tube_centre = sum(cm.TUBE_X) / 2
    node, _, _ = tb.fly_to_tube()
    misplaced = off_centre - (course_x(node, node.carrot.point) - tube_centre)
    ends = {}
    for on in (False, True):
        node, px4, pos = tb.fly_to_tube(tof_tube_centering=on)
        offs, heard = [], 0
        for _ in range(60):
            c = odom_to_course(pos, node.frame[0], node.frame[1], node.start_height)
            real = (c[0] + misplaced, c[1], c[2])
            r = side_ranges(real, math.pi / 2)              # nose north
            res = tube_offset(r['left'][0], r['right'][0], 0.15, 1.3, 0.075, 0.075, 1.45, 0.15)
            if res is not None:
                heard += 1
                node.tof_offset_cb(offset_msg(res[0]))
            tb.step(node, px4, pos)
            if node.state == 'ROUTE' and in_tube_now(node, pos):
                offs.append(real[0] - tube_centre)
        assert heard >= 15 and len(offs) >= 15
        ends[on] = offs[-5:]
    assert all(abs(o - off_centre) < 0.02 for o in ends[False])    # without: as far off as the route
    assert all(abs(o) < 0.02 for o in ends[True])                  # with: on the centreline


def test_launch_is_opt_in_and_forwarded():
    import test_course_launch as launch
    assert launch._arguments(launch.BASE)['tof_tube_centering'] == 'false'
    assert launch._arguments(launch.COURSE)['tof_tube_centering'] == 'false'
    assert 'tof_tube_centering' in launch._forwarded(launch.COURSE)
    params = {e.value for e in launch._assigned(launch._tree(launch.BASE), 'MISSION_PARAMS').elts}
    assert {'tof_tube_centering', 'tof_tube_max_shift_m', 'tof_tube_rate_mps',
            'tof_tube_timeout_s'} <= params
    defaults = dict(cm.PARAMETERS)
    for name in ('tof_tube_max_shift_m', 'tof_tube_rate_mps', 'tof_tube_timeout_s'):
        assert float(launch._arguments(launch.BASE)[name]) == defaults[name]
