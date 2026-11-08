"""Closed loop of course_mission.py on stub ROS and a point-mass drone (no ROS needed).

Flies the shipped route_gate_b.json onto the pad; with fly_home on, a re-arm
must then fly back over the wall to the launch mark (odom origin) and land.
"""
import json
import math
import os
import sys
import types

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

T = [0.0]           # sim time, s
OVERRIDES = {}      # parameter overrides for the next node
SENT = []           # (topic, msg) the node published
SUBS = []           # topics the node subscribed to
Z_BIAS = [0.0]      # PX4's height error vs odom, m (pre-arm check tests)
# Heading of the point-mass drone (odom ENU yaw, rad). In the air it turns
# toward a commanded heading like PX4 (2/s, capped at 1.5 rad/s); with a
# zero quaternion (yaw NaN) PX4 does not steer it, so only YAW_DRIFT (a
# steady yaw disturbance, rad/s) and a twist's yaw rate move it.
YAW = [0.0]
YAW_DRIFT = [0.0]
PX4_HOOK = [None]   # called as PX4_HOOK[0](node) each tick (PX4 heading tests)
# Where the drone really is, off where it was flown to: ODOM_OFFSET[0](t)
# -> (dx, dy, dz) added to the odometry (a liftoff slide PX4 is slow to undo).
ODOM_OFFSET = [None]
# PX4 flies its own height: it holds (true z + Z_BIAS) at a pose setpoint's z,
# so the drone ends up Z_BIAS below it (as on skybeta-19). Off: the drone flies
# to the setpoint's z directly, as every older test assumes.
PX4_OWN_Z = [False]


def reset_heading():
    YAW[0] = 0.0
    YAW_DRIFT[0] = 0.0
    PX4_HOOK[0] = None
    ODOM_OFFSET[0] = None
    PX4_OWN_Z[0] = False


def wrap(a):
    return (a + math.pi) % (2.0 * math.pi) - math.pi


def pose_yaw(m):
    """Commanded yaw of a pose setpoint, or None for the zero quaternion."""
    o = m.pose.orientation
    if o.z == 0.0 and o.w == 0.0:
        return None
    return 2.0 * math.atan2(o.z, o.w)


class Msg:
    """Any message: attributes spring into existence when touched."""

    def __getattr__(self, name):
        if name.startswith('__'):
            raise AttributeError(name)
        value = Msg()
        setattr(self, name, value)
        return value


class Node:
    def __init__(self, name, **kwargs):
        self._params = {}

    def declare_parameter(self, name, value, descriptor=None):
        self._params[name] = OVERRIDES.get(name, value)

    def get_parameter(self, name):
        return types.SimpleNamespace(value=self._params[name])

    def create_publisher(self, msg_type, topic, qos):
        return types.SimpleNamespace(publish=lambda m: SENT.append((topic, m)))

    def create_subscription(self, msg_type, topic, *args):
        SUBS.append(topic)

    def create_timer(self, *args):
        pass

    def get_clock(self):
        return types.SimpleNamespace(now=lambda: types.SimpleNamespace(
            nanoseconds=int(T[0] * 1e9), to_msg=lambda: None))

    def get_logger(self):
        say = lambda m: print(f'[{T[0]:7.1f}] {m}')   # noqa: E731
        return types.SimpleNamespace(info=say, warn=say, error=say)


def _import_course_mission():
    stubs = {
        'rclpy': types.ModuleType('rclpy'),
        'rclpy.node': types.SimpleNamespace(Node=Node),
        'rclpy.qos': types.SimpleNamespace(qos_profile_sensor_data=None),
        'geometry_msgs.msg': types.SimpleNamespace(PointStamped=Msg, PoseStamped=Msg,
                                                   TwistStamped=Msg),
        'nav_msgs.msg': types.SimpleNamespace(Odometry=Msg),
        'std_msgs.msg': types.SimpleNamespace(String=Msg),
        'rcl_interfaces.msg': types.SimpleNamespace(ParameterDescriptor=lambda **kw: kw),
    }
    for pkg in ('geometry_msgs', 'nav_msgs', 'std_msgs', 'rcl_interfaces'):
        stubs[pkg] = types.ModuleType(pkg)
    saved = {k: sys.modules.get(k) for k in stubs}
    sys.modules.update(stubs)
    try:
        from navless_nav import course_mission
        return course_mission
    finally:     # leave real ROS (if any) alone for other tests
        for k, v in saved.items():
            if v is None:
                sys.modules.pop(k, None)
            else:
                sys.modules[k] = v


course_mission = _import_course_mission()


def fly(node, px4, pos, until, max_s=600.0, dt=0.1):
    """Tick node + point-mass drone until `until()` after a tick, then one more
    tick so the node sees the result. PX4 disarms on touchdown in LAND."""
    end = T[0] + max_s
    top = pos[2]
    done = False
    while not done:
        done = until()
        assert T[0] < end, f'timed out in {node.state} at {pos}'
        off = ODOM_OFFSET[0](T[0]) if ODOM_OFFSET[0] is not None else (0.0, 0.0, 0.0)
        seen = [a + b for a, b in zip(pos, off)]
        node.odometry_cb(types.SimpleNamespace(pose=types.SimpleNamespace(
            pose=types.SimpleNamespace(position=types.SimpleNamespace(
                x=seen[0], y=seen[1], z=seen[2]), orientation=types.SimpleNamespace(
                    x=0.0, y=0.0, z=math.sin(YAW[0] / 2), w=math.cos(YAW[0] / 2))))))
        if PX4_HOOK[0] is not None:
            PX4_HOOK[0](node)
        px4['local_enu'] = [pos[0], pos[1], pos[2] + Z_BIAS[0]]
        node.state_cb(types.SimpleNamespace(data=json.dumps(px4)))
        SENT.clear()
        node.tick()
        if px4['armed'] and SENT:
            topic, m = SENT[-1]
            rate = 0.0
            if topic == course_mission.POSE_TOPIC:
                goal = (m.pose.position.x, m.pose.position.y,
                        m.pose.position.z - (Z_BIAS[0] if PX4_OWN_Z[0] else 0.0))
                d = math.dist(pos, goal)
                f = min(1.0, 1.5 * dt / d) if d > 0 else 0.0     # 1.5 m/s cap
                pos[:] = [a + (b - a) * f for a, b in zip(pos, goal)]
                cmd = pose_yaw(m)
                if cmd is not None:
                    rate = max(-1.5, min(1.5, 2.0 * wrap(cmd - YAW[0])))
            else:
                v = m.twist.linear
                pos[:] = [pos[0] + v.x * dt, pos[1] + v.y * dt, pos[2] + v.z * dt]
                rate = m.twist.angular.z
            if pos[2] > 0.0:            # PX4 does not turn it on the floor
                YAW[0] = wrap(YAW[0] + (rate + YAW_DRIFT[0]) * dt)
            pos[2] = max(0.0, pos[2])
            px4['landed'] = pos[2] == 0.0
            if px4['landed'] and node.state in ('LAND', 'REFUSED'):   # COM_DISARM_LAND
                px4.update(armed=False, offboard_active=False)
        top = max(top, pos[2])
        T[0] += dt
    return top


def arm(px4):
    px4.update(armed=True, offboard_active=True)


def run(**overrides):
    T[0] = 0.0
    Z_BIAS[0] = 0.0
    reset_heading()
    OVERRIDES.clear()
    OVERRIDES.update(route_file='route_gate_b.json', cruise_speed_mps=1.0, **overrides)
    node = course_mission.CourseMission()
    px4 = {'armed': False, 'offboard_active': False, 'landed': True}
    pos = [0.0, 0.0, 0.0]
    # disarmed on the floor past the pre-arm settle: streams position
    fly(node, px4, pos, lambda: T[0] >= 2.5)
    arm(px4)
    fly(node, px4, pos, lambda: not px4['armed'])
    return node, px4, pos


def test_default_stops_on_the_pad():
    node, px4, pos = run()
    pad = node.route[-1]
    assert node.state == 'DONE'
    assert math.dist(pos[:2], pad[:2]) < 0.1 and pos[2] == 0.0


def test_fly_home_rearm_flies_back_to_the_launch_mark():
    node, px4, pad = run(fly_home=True)
    assert node.state == 'WAIT_FOR_OFFBOARD' and node.going_home
    assert math.dist(pad[:2], (0.0, 0.0)) > 1.0     # landed on the pad, not home
    t0 = T[0]                                       # disarmed: nothing moves,
    fly(node, px4, pad, lambda: T[0] - t0 >= 2.5, max_s=5.0)   # heights re-checked
    assert node.state == 'WAIT_FOR_OFFBOARD'
    pos = list(pad)
    arm(px4)
    top = fly(node, px4, pos, lambda: not px4['armed'])
    assert node.state == 'DONE'
    assert math.dist(pos[:2], (0.0, 0.0)) < 0.1 and pos[2] == 0.0
    assert abs(top - node.home_z) < 0.05             # hopped the 1.727 m wall


def test_fly_home_hop_leaves_only_at_hop_height():
    # The leg over Barrier 1 must not start at arrival_radius_m (0.25 m)
    # below the hop height: SITL crossed the wall at 1.91-1.99 m that way.
    node, px4, pad = run(fly_home=True)
    t0 = T[0]
    fly(node, px4, pad, lambda: T[0] - t0 >= 2.5, max_s=5.0)
    pos = list(pad)
    arm(px4)
    fly(node, px4, pos, lambda: node.state == 'TAKEOFF')
    seen = []
    while node.state == 'TAKEOFF':
        seen.append(pos[2])
        fly(node, px4, pos, lambda: True, max_s=1.0)
    assert node.state == 'ROUTE'
    assert seen[-1] >= node.home_z - node.home_climb_tol - 1e-9
    assert node.home_z == 2.2 - 0.07


def test_fly_home_hop_leaves_when_px4_says_it_is_at_hop_height():
    # SITL: PX4 settled 7 cm under the 2.2 m hop by truth/odom, with its own
    # estimate on the setpoint. That counts as climbed; it would never get
    # closer by odom.
    node, px4, pad = run(fly_home=True, takeoff_timeout_s=8.0)
    t0 = T[0]
    fly(node, px4, pad, lambda: T[0] - t0 >= 2.5, max_s=5.0)
    pos = list(pad)
    arm(px4)
    cap = node.home_z - 0.07
    t0 = T[0]
    while node.state in ('WAIT_FOR_OFFBOARD', 'TAKEOFF') and T[0] - t0 < 10.0:
        Z_BIAS[0] = 0.07 if pos[2] >= cap - 1e-9 else 0.0
        fly(node, px4, pos, lambda: True, max_s=1.0)
        pos[2] = min(pos[2], cap)
    Z_BIAS[0] = 0.0
    assert node.state == 'ROUTE'


def test_fly_home_lands_over_the_mark_even_with_px4_holding_low():
    # The 0.10 m landing radius is horizontal: 0.12 m below the hop by odom
    # (PX4 on its setpoint) must still land, not stall over the mark.
    node, px4, pad = run(fly_home=True, takeoff_timeout_s=8.0)
    t0 = T[0]
    fly(node, px4, pad, lambda: T[0] - t0 >= 2.5, max_s=5.0)
    pos = list(pad)
    arm(px4)
    cap = node.home_z - 0.12
    t0 = T[0]
    while node.state in ('WAIT_FOR_OFFBOARD', 'TAKEOFF', 'ROUTE') and T[0] - t0 < 60.0:
        Z_BIAS[0] = 0.12 if pos[2] >= cap - 1e-9 else 0.0
        fly(node, px4, pos, lambda: True, max_s=1.0)
        if node.state != 'LAND':
            pos[2] = min(pos[2], cap)
    Z_BIAS[0] = 0.0
    assert node.state == 'LAND'
    assert math.hypot(*pos[:2]) <= node.home_arrival_radius + 0.2


def test_fly_home_hop_that_never_reaches_height_holds_over_the_pad():
    # e.g. a ceiling clamp below the hop: takeoff_timeout_s -> HOLD, and the
    # leg over the wall never starts.
    node, px4, pad = run(fly_home=True, takeoff_timeout_s=3.0)
    t0 = T[0]
    fly(node, px4, pad, lambda: T[0] - t0 >= 2.5, max_s=5.0)
    pos = list(pad)
    arm(px4)
    cap = node.home_z - 0.2
    t0 = T[0]
    while node.state in ('WAIT_FOR_OFFBOARD', 'TAKEOFF') and T[0] - t0 < 10.0:
        fly(node, px4, pos, lambda: True, max_s=1.0)
        pos[2] = min(pos[2], cap)
    assert node.state == 'HOLD'
    assert math.dist(pos[:2], pad[:2]) < 0.05


def test_fly_home_lands_only_close_to_the_launch_mark():
    node, px4, pad = run(fly_home=True)
    t0 = T[0]
    fly(node, px4, pad, lambda: T[0] - t0 >= 2.5, max_s=5.0)
    pos = list(pad)
    arm(px4)
    fly(node, px4, pos, lambda: node.state == 'ROUTE')
    # A slower drone (0.3 m/s) lags the carrot, so it is still closing on the
    # mark when the carrot gets there: the landing must wait for 0.10 m.
    dt = 0.1
    while node.state == 'ROUTE' and T[0] - t0 < 120.0:
        node.odometry_cb(types.SimpleNamespace(pose=types.SimpleNamespace(
            pose=types.SimpleNamespace(position=types.SimpleNamespace(
                x=pos[0], y=pos[1], z=pos[2])))))
        px4['local_enu'] = list(pos)
        node.state_cb(types.SimpleNamespace(data=json.dumps(px4)))
        SENT.clear()
        node.tick()
        seen = math.hypot(*pos[:2])
        if SENT and SENT[-1][0] == course_mission.POSE_TOPIC:
            m = SENT[-1][1]
            goal = (m.pose.position.x, m.pose.position.y, m.pose.position.z)
            d = math.dist(pos, goal)
            f = min(1.0, 0.3 * dt / d) if d > 0 else 0.0
            pos[:] = [a + (b - a) * f for a, b in zip(pos, goal)]
        T[0] += dt
    assert node.state == 'LAND'
    assert seen <= node.home_arrival_radius + 1e-9


if __name__ == '__main__':
    test_default_stops_on_the_pad()
    test_fly_home_rearm_flies_back_to_the_launch_mark()
    test_fly_home_hop_leaves_only_at_hop_height()
    test_fly_home_hop_that_never_reaches_height_holds_over_the_pad()
    test_fly_home_lands_only_close_to_the_launch_mark()
    print('ok')
