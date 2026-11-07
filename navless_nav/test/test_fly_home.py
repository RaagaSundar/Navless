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

    def create_subscription(self, *args):
        pass

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
        'geometry_msgs.msg': types.SimpleNamespace(PoseStamped=Msg, TwistStamped=Msg),
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
        node.odometry_cb(types.SimpleNamespace(pose=types.SimpleNamespace(
            pose=types.SimpleNamespace(position=types.SimpleNamespace(
                x=pos[0], y=pos[1], z=pos[2])))))
        node.state_cb(types.SimpleNamespace(data=json.dumps(px4)))
        SENT.clear()
        node.tick()
        if px4['armed'] and SENT:
            topic, m = SENT[-1]
            if topic == course_mission.POSE_TOPIC:
                goal = (m.pose.position.x, m.pose.position.y, m.pose.position.z)
                d = math.dist(pos, goal)
                f = min(1.0, 1.5 * dt / d) if d > 0 else 0.0     # 1.5 m/s cap
                pos[:] = [a + (b - a) * f for a, b in zip(pos, goal)]
            else:
                v = m.twist.linear
                pos[:] = [pos[0] + v.x * dt, pos[1] + v.y * dt, pos[2] + v.z * dt]
            pos[2] = max(0.0, pos[2])
            px4['landed'] = pos[2] == 0.0
            if px4['landed'] and node.state == 'LAND':       # COM_DISARM_LAND
                px4.update(armed=False, offboard_active=False)
        top = max(top, pos[2])
        T[0] += dt
    return top


def arm(px4):
    px4.update(armed=True, offboard_active=True)


def run(**overrides):
    T[0] = 0.0
    OVERRIDES.clear()
    OVERRIDES.update(route_file='route_gate_b.json', cruise_speed_mps=1.0, **overrides)
    node = course_mission.CourseMission()
    px4 = {'armed': False, 'offboard_active': False, 'landed': True}
    pos = [0.0, 0.0, 0.0]
    fly(node, px4, pos, lambda: True)          # one disarmed tick: streams position
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
    fly(node, px4, pad, lambda: True, max_s=5.0)    # disarmed: nothing moves
    assert node.state == 'WAIT_FOR_OFFBOARD'
    pos = list(pad)
    arm(px4)
    top = fly(node, px4, pos, lambda: not px4['armed'])
    assert node.state == 'DONE'
    assert math.dist(pos[:2], (0.0, 0.0)) < 0.1 and pos[2] == 0.0
    assert abs(top - (2.03 - 0.07)) < 0.3            # hopped the 1.727 m wall


if __name__ == '__main__':
    test_default_stops_on_the_pad()
    test_fly_home_rearm_flies_back_to_the_launch_mark()
    print('ok')
