"""Planner route files -> odom-frame waypoints, and a carrot that walks them.

No ROS in here, so it is unit-tested off the drone (test/test_route.py).

Route files (route_gate_b.json, route_window.json, ... at the repo root, made
by planner/) carry `waypoints_enu` in the course frame: origin at the floor
centre of the 20x20 ft net, x east, y north, z up from the floor, metres.
`launch_origin_enu` is the launch mark the drone powers up on.

cuVSLAM's odom frame starts at the power-up pose: origin on the launch mark,
+x along the nose, +y to its left, z from the start height. So course -> odom
is a shift to the launch mark plus a rotation by the nose's bearing at power-up
(degrees clockwise from course north, the convention the course file uses).
"""
import json
import math
import os

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.dirname(
    os.path.abspath(__file__))))

NET_HALF_M = 3.048      # the 20 x 20 ft net; course origin at its floor centre


def course_to_odom(p, launch_xy, start_bearing_deg, start_height_m=0.0):
    e, n = p[0] - launch_xy[0], p[1] - launch_xy[1]
    b = math.radians(start_bearing_deg)
    s, c = math.sin(b), math.cos(b)
    return (e * s + n * c, -e * c + n * s, p[2] - start_height_m)


def odom_to_course(q, launch_xy, start_bearing_deg, start_height_m=0.0):
    b = math.radians(start_bearing_deg)
    s, c = math.sin(b), math.cos(b)
    return (launch_xy[0] + q[0] * s - q[1] * c,
            launch_xy[1] + q[0] * c + q[1] * s,
            q[2] + start_height_m)


def clamp_to_net(q, launch_xy, start_bearing_deg, margin_m,
                 half_m=NET_HALF_M):
    """(odom point pulled inside the net by `margin_m`, whether it moved).

    flight_interface has no horizontal geofence (removed in skybeta 0.3.0),
    so this is the only thing between a bad setpoint and the netting.
    """
    c = odom_to_course(q, launch_xy, start_bearing_deg)
    lim = half_m - margin_m
    x, y = min(max(c[0], -lim), lim), min(max(c[1], -lim), lim)
    if (x, y) == (c[0], c[1]):
        return tuple(q), False
    return course_to_odom((x, y, c[2]), launch_xy, start_bearing_deg), True


def resolve_route_path(name):
    """An absolute path as is; otherwise look next to the repo, then in cwd."""
    if os.path.isabs(name):
        return name
    for base in (REPO_ROOT, os.getcwd()):
        candidate = os.path.join(base, name)
        if os.path.isfile(candidate):
            return candidate
    raise FileNotFoundError(
        f'route file {name!r} not found in {REPO_ROOT} or {os.getcwd()}')


def load_route(path):
    """(course-frame waypoints, launch mark xy, route type) from a route file."""
    with open(path) as f:
        data = json.load(f)
    waypoints = [tuple(float(v) for v in p) for p in data['waypoints_enu']]
    if not waypoints:
        raise ValueError(f'{path}: no waypoints')
    for p in waypoints:
        if len(p) != 3 or not all(math.isfinite(v) for v in p):
            raise ValueError(f'{path}: bad waypoint {p}')
    launch = data.get('launch_origin_enu') or waypoints[0][:2]
    return waypoints, (float(launch[0]), float(launch[1])), data.get(
        'route_type', '?')


class Carrot:
    """A setpoint that slides along the path at `speed` m/s.

    It never runs more than `max_lead` metres ahead of the drone: if the drone
    lags (wind, a clamp, a bad estimate) the carrot waits for it. That keeps
    every pose setpoint a short step away, because PX4 flies a far-away pose
    at MPC_XY_VEL_MAX (12 m/s on these airframes).
    """

    def __init__(self, path, speed, max_lead):
        if len(path) < 1:
            raise ValueError('empty path')
        self.path = [tuple(p) for p in path]
        self.speed, self.max_lead = float(speed), float(max_lead)
        self.index = 0                  # last path vertex the carrot passed
        self.point = self.path[0]

    def step(self, drone, dt):
        if math.dist(drone, self.point) > self.max_lead:
            return self.point
        budget = self.speed * max(0.0, dt)
        while budget > 0.0 and self.index < len(self.path) - 1:
            nxt = self.path[self.index + 1]
            d = math.dist(self.point, nxt)
            if d <= budget:
                self.point, self.index, budget = nxt, self.index + 1, budget - d
            else:
                f = budget / d
                self.point = tuple(a + (b - a) * f
                                   for a, b in zip(self.point, nxt))
                budget = 0.0
        return self.point

    @property
    def at_end(self):
        return self.index == len(self.path) - 1

    def done(self, drone, radius):
        return self.at_end and math.dist(drone, self.point) <= radius
