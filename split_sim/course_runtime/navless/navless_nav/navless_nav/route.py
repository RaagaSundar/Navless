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
import bisect
import json
import math
import os
import sys

# realpath, not abspath: on the drone, team_build.sh runs `colcon build
# --symlink-install`, which imports this package from team_ws/build/navless_nav/,
# a symlink into the source. abspath keeps that link, so the repo would be
# team_ws/build, no route file is found, and the node takes off and hovers.
REPO_ROOT = os.path.dirname(os.path.dirname(os.path.dirname(
    os.path.realpath(__file__))))
if REPO_ROOT not in sys.path:
    sys.path.insert(0, REPO_ROOT)

NET_HALF_M = 3.048      # the 20 x 20 ft net; course origin at its floor centre

_BASE_PLANNER_GRID = None
_CLEAN_DYNAMIC_ROUTE_CACHE = {}


def create_live_course_planner():
    """Create a live (PlannerConfig, OccupancyGrid3D, GlobalCourseRouter) backed by the 3D course geometry."""
    global _BASE_PLANNER_GRID
    from planner.config import PlannerConfig
    from planner.fsm_router import GlobalCourseRouter
    from planner.grid_map import OccupancyGrid3D

    cfg_path = os.path.join(REPO_ROOT, 'planner', 'config', 'course_config.json')
    cfg = PlannerConfig.from_json(cfg_path) if os.path.isfile(cfg_path) else PlannerConfig()
    if _BASE_PLANNER_GRID is None:
        base = OccupancyGrid3D(
            bounds_min_m=cfg.global_map_bounds_min_m,
            bounds_max_m=cfg.global_map_bounds_max_m,
            resolution_m=cfg.voxel_resolution_m,
            inflation_radius_m=cfg.obstacle_inflation_m,
            local_window_m=cfg.local_window_size_m,
        )
        obj_candidates = [
            os.path.join(REPO_ROOT, 'dronehacks-course-asbuilt.obj'),
            os.path.join(os.path.dirname(REPO_ROOT), 'dronehacks-course-asbuilt.obj'),
            os.path.join(REPO_ROOT, 'dronehacks-course-v3.obj'),
            os.path.join(os.path.dirname(REPO_ROOT), 'dronehacks-course-v3.obj'),
        ]
        json_candidates = [
            os.path.join(REPO_ROOT, 'planner', 'config', 'prior_course_map.json'),
            os.path.join(REPO_ROOT, 'planner', 'config', 'course_occupancy.json'),
            os.path.join(os.path.dirname(REPO_ROOT), 'planner', 'config', 'prior_course_map.json'),
        ]
        loaded_prior = False
        for json_path in json_candidates:
            if os.path.isfile(json_path):
                base.load_prior_from_json(json_path)
                loaded_prior = True
                break
        if not loaded_prior:
            for obj_path in obj_candidates:
                if os.path.isfile(obj_path):
                    base.load_prior_from_obj(obj_path)
                    loaded_prior = True
                    break
        if not loaded_prior:
            base._update_prior_snapshot()
        base.build_soft_clearance_cost()
        base.build_soft_clearance_cost(0.55)
        _BASE_PLANNER_GRID = base

    grid = OccupancyGrid3D(
        bounds_min_m=cfg.global_map_bounds_min_m,
        bounds_max_m=cfg.global_map_bounds_max_m,
        resolution_m=cfg.voxel_resolution_m,
        inflation_radius_m=cfg.obstacle_inflation_m,
        local_window_m=cfg.local_window_size_m,
    )
    grid.raw_grid[:] = _BASE_PLANNER_GRID.raw_grid
    grid.prior_grid[:] = _BASE_PLANNER_GRID.prior_grid
    grid.prior_near_mask[:] = _BASE_PLANNER_GRID.prior_near_mask
    grid.inflated_grid[:] = _BASE_PLANNER_GRID.inflated_grid
    grid.detected_concentric_pad = _BASE_PLANNER_GRID.detected_concentric_pad
    grid.detected_launch_mark_xy = _BASE_PLANNER_GRID.detected_launch_mark_xy
    grid._soft_cost_cache = {k: v.copy() for k, v in _BASE_PLANNER_GRID._soft_cost_cache.items()}
    router = GlobalCourseRouter(cfg, grid)
    return cfg, grid, router


def plan_dynamic_course_route(branch='window', start_enu=None, router=None):
    """Plan a collision-free 3D A* + EGO B-spline route dynamically from the 3D occupancy grid."""
    use_win = str(branch).lower() != 'gate_b'
    cache_key = ('window' if use_win else 'gate_b')
    if start_enu is None and router is None and cache_key in _CLEAN_DYNAMIC_ROUTE_CACHE:
        wps, launch, kind = _CLEAN_DYNAMIC_ROUTE_CACHE[cache_key]
        return list(wps), launch, kind

    if router is None:
        _, _, router = create_live_course_planner()

    if start_enu is None and not router.grid.get_discovered_discrepancy_points().size:
        if cache_key in _CLEAN_DYNAMIC_ROUTE_CACHE:
            wps, launch, kind = _CLEAN_DYNAMIC_ROUTE_CACHE[cache_key]
            return list(wps), launch, kind
        data = router.plan_full_course_route(use_window=use_win, include_fly_home=False)
        raw_wps = [tuple(float(v) for v in p) for p in data['waypoints_enu']]
        launch_raw = data.get('launch_origin_enu') or raw_wps[0][:2]
        launch = (float(launch_raw[0]), float(launch_raw[1]))
        z0 = float(raw_wps[0][2])
        # Anchor initial 0.25 m climb-lean segment straight North along Corridor 1
        aligned_wps = [
            (launch[0], launch[1], z0),
            (launch[0], launch[1] + 0.25, z0),
        ] + [p for p in raw_wps if math.hypot(p[0] - launch[0], p[1] - launch[1]) > 0.28]
        kind = f"DYNAMIC_3D_ASTAR_BSPLINE_{data.get('route_type', cache_key.upper())}"
        _CLEAN_DYNAMIC_ROUTE_CACHE[cache_key] = (aligned_wps, launch, kind)
        return list(aligned_wps), launch, kind

    # Online mid-flight replanning from current drone pose around newly discovered map discrepancies
    from planner.fsm_router import CourseStage
    cp = router.config.checkpoints
    launch = (float(cp.start_pad[0]), float(cp.start_pad[1]))
    s_pt = tuple(float(v) for v in (start_enu or cp.start_pad))
    pad_goal, pad_r = router.landing_target_enu()
    prof = router.profile_for_stage(CourseStage.STAGE_4_CHICANE)
    features = router.grid.discover_course_features(standoff_distance_m=router.config.window_inspect_distance_m)
    win_center = features.window_center_m if features.window_center_m is not None else cp.window_aperture

    # Try Window first if requested and clear; automatically generalize to Gate B if blocked!
    spline = None
    chosen = 'WINDOW'
    inspect_res = router.grid.inspect_window_aperture(
        win_center,
        confidence_threshold=router.config.window_confidence_threshold,
    )
    if use_win and inspect_res.is_passable:
        spline, _ = router.optimizer.plan_spline(
            s_pt,
            pad_goal,
            prof,
            use_clearance_cost=True,
            goal_radius_m=pad_r,
        )
    if spline is None:
        chosen = 'GATE_B'
        spline, _ = router.optimizer.plan_spline(
            s_pt,
            pad_goal,
            prof,
            use_clearance_cost=True,
            avoid_spheres=[(win_center, 0.55)],
            goal_radius_m=pad_r,
        )
    if spline is None:
        raise ValueError('3D A* + B-spline could not find a collision-free path from current pose')
    pts = spline.sample_points(80)
    wps = [tuple(round(float(v), 4) for v in pt) for pt in pts]
    return wps, launch, f'DYNAMIC_REPLAN_{chosen}'


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

    flight_interface clamps altitude only, never x/y, so this is the only
    thing between a bad setpoint and the netting.
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


def _point_along(path, cum, s):
    s = min(max(s, 0.0), cum[-1])
    i = min(bisect.bisect_right(cum, s) - 1, len(path) - 2)
    seg = cum[i + 1] - cum[i]
    f = (s - cum[i]) / seg if seg > 0.0 else 0.0
    return tuple(a + (b - a) * f for a, b in zip(path[i], path[i + 1]))


def turn_radii(path, h=0.25):
    """Horizontal radius of the path at each vertex, from points h m either side."""
    if len(path) < 3:
        return [math.inf] * len(path)
    cum = [0.0]
    for a, b in zip(path, path[1:]):
        cum.append(cum[-1] + math.dist(a, b))
    radii = []
    for i, b in enumerate(path):
        a, c = _point_along(path, cum, cum[i] - h), _point_along(path, cum, cum[i] + h)
        cross = abs((b[0] - a[0]) * (c[1] - a[1]) - (b[1] - a[1]) * (c[0] - a[0]))
        sides = math.dist(a[:2], b[:2]) * math.dist(b[:2], c[:2]) * math.dist(a[:2], c[:2])
        radii.append(sides / (2.0 * cross) if cross > 1e-9 else math.inf)
    return radii


def speed_profile(path, cruise, max_lat_accel=0.0, max_accel=0.5):
    """Per-vertex speed: cruise, capped at sqrt(max_lat_accel * radius) in
    bends, then ramped at max_accel so the carrot slows BEFORE a bend.

    Why: with stock PX4 gains and position-only setpoints the drone overshoots
    to the outside of bends. In Gazebo, 0.5 m/s gave 11-14 cm of overshoot at
    the 0.5 m maze and chicane bends; 0.35 m/s gave under 5 cm. The planned
    routes leave about 6 cm to the walls.
    """
    v = [float(cruise)] * len(path)
    if max_lat_accel > 0.0:
        v = [min(x, math.sqrt(max_lat_accel * r)) for x, r in zip(v, turn_radii(path))]
    if max_accel > 0.0:
        d = [math.dist(a, b) for a, b in zip(path, path[1:])]
        for i in range(len(v) - 2, -1, -1):
            v[i] = min(v[i], math.sqrt(v[i + 1] ** 2 + 2.0 * max_accel * d[i]))
        for i in range(1, len(v)):
            v[i] = min(v[i], math.sqrt(v[i - 1] ** 2 + 2.0 * max_accel * d[i - 1]))
    return v


class Carrot:
    """A setpoint that slides along the path at up to `speed` m/s.

    It never runs more than `max_lead` metres ahead of the drone: if the drone
    lags (wind, a clamp, a bad estimate) the carrot waits for it. That keeps
    every pose setpoint a short step away, because PX4 flies a far-away pose
    at MPC_XY_VEL_MAX (12 m/s on these airframes). With max_lat_accel > 0 it
    also slows through bends (speed_profile).

    With max_sagitta > 0 the lead also shrinks in tight turns. A drone chasing
    a carrot L ahead round an arc of radius R settles on a circle of radius
    sqrt(R^2 - L^2), i.e. R - sqrt(R^2 - L^2) ~ L^2 / (2 R) inside the path,
    so the lead is capped at sqrt(2 R s - s^2) for s = max_sagitta (never
    below MIN_LEAD_M), with R the tightest radius between drone and carrot.
    """

    MIN_LEAD_M = 0.15

    def __init__(self, path, speed, max_lead, max_lat_accel=0.0,
                 max_sagitta=0.0):
        if len(path) < 1:
            raise ValueError('empty path')
        self.path = [tuple(p) for p in path]
        self.speed, self.max_lead = float(speed), float(max_lead)
        self.limits = speed_profile(self.path, self.speed, max_lat_accel)
        self.max_sagitta = max(0.0, float(max_sagitta))
        self.radii = turn_radii(self.path) if self.max_sagitta > 0.0 else None
        self.cum = [0.0]
        for a, b in zip(self.path, self.path[1:]):
            self.cum.append(self.cum[-1] + math.dist(a, b))
        self.index = 0                  # last path vertex the carrot passed
        self.point = self.path[0]

    def lead_limit(self):
        """How far ahead of the drone the carrot may be, m."""
        if self.radii is None or not math.isfinite(self.max_lead):
            return self.max_lead
        hi = min(self.index + 1, len(self.path) - 1)
        lo = self.index
        while lo > 0 and self.cum[self.index] - self.cum[lo - 1] <= self.max_lead:
            lo -= 1
        r = min(self.radii[lo:hi + 1])
        if not math.isfinite(r):
            return self.max_lead
        s = min(self.max_sagitta, r)
        return min(self.max_lead,
                   max(self.MIN_LEAD_M, math.sqrt(2.0 * r * s - s * s)))

    def speed_here(self):
        nxt = min(self.index + 1, len(self.path) - 1)
        return min(self.limits[self.index], self.limits[nxt])

    def step(self, drone, dt, vel=None):
        if math.dist(drone, self.point) > self.max_lead:
            pt = self.point
        else:
            limit = self.lead_limit()
            # The turn cap is about the horizontal cut, so measure it in x/y: a
            # height offset (PX4 may hold height up to ~0.2 m off odom) must not
            # exceed a small cap and stall the carrot in a tight turn.
            if limit < self.max_lead and math.dist(drone[:2], self.point[:2]) > limit:
                pt = self.point
            else:
                pt = self.advance(self.speed_here() * max(0.0, dt))
        if vel is None or self.index >= len(self.path) - 1:
            return pt
        a_pt = self.path[self.index]
        b_pt = self.path[min(len(self.path) - 1, self.index + 1)]
        seg_len = math.dist(a_pt, b_pt)
        if seg_len < 1e-4:
            return pt
        tx, ty = (b_pt[0] - a_pt[0]) / seg_len, (b_pt[1] - a_pt[1]) / seg_len
        ex, ey = pt[0] - drone[0], pt[1] - drone[1]
        e_lag = ex * tx + ey * ty
        ecx, ecy = ex - e_lag * tx, ey - e_lag * ty
        v_lag = vel[0] * tx + vel[1] * ty
        vcx, vcy = vel[0] - v_lag * tx, vel[1] - v_lag * ty
        target = (pt[0] + 0.35 * ecx - 0.18 * vcx, pt[1] + 0.35 * ecy - 0.18 * vcy, pt[2])
        lim = min(self.max_lead, self.lead_limit())
        d_xy = math.hypot(target[0] - drone[0], target[1] - drone[1])
        if d_xy > lim and d_xy > 1e-6:
            s = lim / d_xy
            target = (drone[0] + (target[0] - drone[0]) * s, drone[1] + (target[1] - drone[1]) * s, target[2])
        return target

    def advance(self, distance):
        """Slide the carrot `distance` metres along the path, ignoring speed."""
        budget = max(0.0, float(distance))
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

    def catch_up(self, drone, window=2.5, max_off=0.5):
        """Slide the carrot forward to the drone's own place on the path,
        when the drone is ahead of it (e.g. it flew on after the carrot froze,
        or on a drifting estimate) and within max_off of the path. Looks at
        most `window` metres ahead; never moves the carrot back. Returns how
        far it moved, m."""
        best_s, best_d = 0.0, float(max_off)
        s, a, i = 0.0, self.point, self.index
        while i < len(self.path) - 1 and s < window:
            b = self.path[i + 1]
            seg = math.dist(a, b)
            if seg > 0.0:
                t = sum((d - x) * (y - x) for d, x, y in zip(drone, a, b)) / seg ** 2
                t = max(0.0, min(1.0, t))
                q = tuple(x + (y - x) * t for x, y in zip(a, b))
                dq = math.dist(drone, q)
                if dq <= best_d:
                    best_d, best_s = dq, s + t * seg
            s += seg
            a, i = b, i + 1
        if best_s > 0.0:
            self.advance(best_s)
        return best_s

    @property
    def at_end(self):
        return self.index == len(self.path) - 1

    def done(self, drone, radius):
        return self.at_end and math.dist(drone, self.point) <= radius
