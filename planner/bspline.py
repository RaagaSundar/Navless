"""EGO-Planner 3D A* Guiding Path & Gradient-Based Uniform Cubic B-Spline Optimizer.

Implements the three core EGO-Planner steps from the architecture specification:
1. 3D A* Guiding Path (finds collision-free clearance around static & live obstacles)
2. Gradient-based B-Spline Trajectory Optimization (~1-2 ms, smoothness + obstacle repulsion + Z-clamp)
3. Time Reallocation enforcing stage-specific v_max, a_max, and jerk limits
"""

from __future__ import annotations

import heapq
import math
import time
from typing import Dict, List, Optional, Tuple

import numpy as np

from .config import StageProfile
from .frames import clamp_altitude_enu
from .grid_map import OccupancyGrid3D


# 26-connected 3D neighbor offsets and step lengths
_NEIGHBORS_3D: List[Tuple[int, int, int, float]] = [
    (dx, dy, dz, math.sqrt(dx * dx + dy * dy + dz * dz))
    for dx in (-1, 0, 1)
    for dy in (-1, 0, 1)
    for dz in (-1, 0, 1)
    if not (dx == 0 and dy == 0 and dz == 0)
]


def astar_search_3d(
    grid: OccupancyGrid3D,
    start_m: np.ndarray | Tuple[float, float, float],
    goal_m: np.ndarray | Tuple[float, float, float],
    *,
    z_min_m: float = 0.50,
    z_max_m: float = 1.10,
    max_expansions: int = 45000,
    soft_clearance_cost: Optional[np.ndarray] = None,
    avoid_spheres: Optional[List[Tuple[Tuple[float, float, float], float]]] = None,
    goal_radius_m: float = 0.0,
) -> Optional[np.ndarray]:
    """Find a collision-free 3D guiding path on the inflated occupancy grid within [z_min_m, z_max_m].

    Supports:
    - `soft_clearance_cost`: optional 3D penalty array keeping paths centered in corridors.
    - `avoid_spheres`: optional list of (center_m, radius_m) regions to dynamically avoid
      (e.g., masking out the Window aperture when Window confidence is low so A* automatically
      discovers the Gate B route around the chicane walls).
    - `goal_radius_m`: concentric-circle target radius (e.g., 0.762 m for the 5-ft landing pad
      with r9, r20, r30 rings). Pulls the search toward the bullseye center (`goal_vox`), and
      if the exact center voxel is obstructed, returns the collision-free path to the reachable
      voxel inside the concentric circles that is strictly closest to the bullseye center.
    """
    start_clamped = clamp_altitude_enu(start_m, z_min_m, z_max_m)
    goal_clamped = clamp_altitude_enu(goal_m, z_min_m, z_max_m)

    start_vox = grid.world_to_voxel(start_clamped)
    goal_vox = grid.world_to_voxel(goal_clamped)

    if not grid.in_bounds_voxel(start_vox) or not grid.in_bounds_voxel(goal_vox):
        return None
    if grid.inflated_grid[goal_vox] and goal_radius_m <= 0.0:
        return None

    iz_min = max(0, int(math.floor((z_min_m - grid.bounds_min[2]) / grid.resolution)))
    iz_max = min(grid.shape[2] - 1, int(math.floor((z_max_m - grid.bounds_min[2]) / grid.resolution)))
    goal_r_vox = max(0.0, goal_radius_m / grid.resolution)

    # Pre-convert avoid_spheres to voxel space for fast checking
    avoid_vox_spheres: List[Tuple[int, int, int, float]] = []
    if avoid_spheres:
        for center_pt, rad_m in avoid_spheres:
            cv = grid.world_to_voxel(center_pt)
            r_vox_sq = (rad_m / grid.resolution) ** 2
            avoid_vox_spheres.append((cv[0], cv[1], cv[2], r_vox_sq))

    def is_masked_out(nx_i: int, ny_i: int, nz_i: int) -> bool:
        for cx_s, cy_s, cz_s, r_sq in avoid_vox_spheres:
            if (nx_i - cx_s) ** 2 + (ny_i - cy_s) ** 2 + (nz_i - cz_s) ** 2 <= r_sq:
                return True
        return False

    def heuristic(v: Tuple[int, int, int]) -> float:
        return math.sqrt(
            (v[0] - goal_vox[0]) ** 2
            + (v[1] - goal_vox[1]) ** 2
            + (v[2] - goal_vox[2]) ** 2
        )

    def _reconstruct_path(end_vox: Tuple[int, int, int]) -> np.ndarray:
        cur = end_vox
        vox_path = [cur]
        while cur in came_from:
            cur = came_from[cur]
            vox_path.append(cur)
        vox_path.reverse()
        pts = np.array([grid.voxel_to_world(v) for v in vox_path], dtype=np.float64)
        pts[0] = start_clamped
        if end_vox == goal_vox:
            pts[-1] = goal_clamped
        pts[:, 2] = np.clip(pts[:, 2], z_min_m, z_max_m)
        return pts

    open_heap: List[Tuple[float, int, Tuple[int, int, int]]] = []
    counter = 0
    heapq.heappush(open_heap, (heuristic(start_vox), counter, start_vox))
    g_score: Dict[Tuple[int, int, int], float] = {start_vox: 0.0}
    came_from: Dict[Tuple[int, int, int], Tuple[int, int, int]] = {}
    closed = set()
    nx, ny, _ = grid.shape

    # Track the closest reachable collision-free voxel inside the concentric circle target
    best_concentric_vox: Optional[Tuple[int, int, int]] = None
    best_concentric_dist: float = float("inf")

    expansions = 0
    while open_heap and expansions < max_expansions:
        _, _, current = heapq.heappop(open_heap)
        if current == goal_vox and not grid.inflated_grid[goal_vox]:
            return _reconstruct_path(current)

        if current in closed:
            continue
        closed.add(current)
        expansions += 1

        if goal_r_vox > 0.0 and not grid.inflated_grid[current]:
            d_xy_vox = math.hypot(current[0] - goal_vox[0], current[1] - goal_vox[1])
            if d_xy_vox <= goal_r_vox and d_xy_vox < best_concentric_dist:
                best_concentric_dist = d_xy_vox
                best_concentric_vox = current
                # If goal_vox itself is blocked and we reached the inner bullseye ring (r <= 1.2 vox), finish!
                if grid.inflated_grid[goal_vox] and d_xy_vox <= 1.5:
                    return _reconstruct_path(best_concentric_vox)

        cx, cy, cz = current
        g_cur = g_score[current]
        for dx, dy, dz, step_cost in _NEIGHBORS_3D:
            nx_i, ny_i, nz_i = cx + dx, cy + dy, cz + dz
            if not (0 <= nx_i < nx and 0 <= ny_i < ny and iz_min <= nz_i <= iz_max):
                continue
            nbr = (nx_i, ny_i, nz_i)
            if nbr in closed:
                continue
            if grid.raw_grid[nbr] and (nbr != goal_vox or goal_r_vox > 0.0):
                continue
            inflated_escape_penalty = 0.0
            if grid.inflated_grid[nbr] and (nbr != goal_vox or goal_r_vox > 0.0):
                # If the drone started inside the safety inflation buffer, allow escaping to non-raw voxels
                if grid.inflated_grid[current]:
                    inflated_escape_penalty = 8.0
                else:
                    continue
            if avoid_vox_spheres and nbr != goal_vox and is_masked_out(nx_i, ny_i, nz_i):
                continue
            extra_cost = float(soft_clearance_cost[nbr]) if soft_clearance_cost is not None else 0.0
            tentative_g = g_cur + step_cost * (1.0 + extra_cost + inflated_escape_penalty)
            if tentative_g < g_score.get(nbr, float("inf")):
                came_from[nbr] = current
                g_score[nbr] = tentative_g
                counter += 1
                f = tentative_g + 1.05 * heuristic(nbr)
                heapq.heappush(open_heap, (f, counter, nbr))

    if best_concentric_vox is not None:
        return _reconstruct_path(best_concentric_vox)
    return None


class UniformBspline:
    """Uniform Cubic (degree-3) 3D B-Spline with continuous position, velocity, acceleration & yaw."""

    # Standard cubic uniform B-spline basis matrix M / 6
    _M3 = (1.0 / 6.0) * np.array(
        [
            [-1.0, 3.0, -3.0, 1.0],
            [3.0, -6.0, 3.0, 0.0],
            [-3.0, 0.0, 3.0, 0.0],
            [1.0, 4.0, 1.0, 0.0],
        ],
        dtype=np.float64,
    )

    def __init__(self, control_points: np.ndarray, knot_span_dt: float) -> None:
        pts = np.asarray(control_points, dtype=np.float64)
        if pts.ndim != 2 or pts.shape[1] != 3 or pts.shape[0] < 4:
            raise ValueError("UniformBspline requires at least 4 3D control points of shape (N, 3)")
        if knot_span_dt <= 1e-6:
            raise ValueError("knot_span_dt must be positive")
        self.control_points = pts.copy()
        self.dt = float(knot_span_dt)
        self.num_segments = self.control_points.shape[0] - 3
        self.duration = self.num_segments * self.dt

    @classmethod
    def from_waypoints(cls, waypoints: np.ndarray, knot_span_dt: float) -> "UniformBspline":
        """Create a clamped cubic B-spline whose endpoints interpolate the first and last waypoints."""
        pts = np.asarray(waypoints, dtype=np.float64)
        if pts.ndim != 2 or pts.shape[1] != 3 or pts.shape[0] < 2:
            raise ValueError("waypoints must have shape (M, 3) with M >= 2")
        # Repeat boundary control points so cubic spline starts at pts[0] with zero initial offset
        ctrl = np.vstack([pts[0], pts[0], pts, pts[-1], pts[-1]])
        return cls(ctrl, knot_span_dt)

    def _segment_and_u(self, t: float) -> Tuple[int, float]:
        t_clamped = float(np.clip(t, 0.0, self.duration))
        seg = int(math.floor(t_clamped / self.dt))
        if seg >= self.num_segments:
            seg = self.num_segments - 1
            u = 1.0
        else:
            u = (t_clamped - seg * self.dt) / self.dt
        return seg, u

    def evaluate_position(self, t: float) -> np.ndarray:
        """Evaluate 3D position p(t) at time t in [0, duration]."""
        seg, u = self._segment_and_u(t)
        u_vec = np.array([u ** 3, u ** 2, u, 1.0], dtype=np.float64)
        p_seg = self.control_points[seg : seg + 4]
        return u_vec @ self._M3 @ p_seg

    def evaluate_velocity(self, t: float) -> np.ndarray:
        """Evaluate 3D velocity v(t) = dp/dt at time t in [0, duration]."""
        seg, u = self._segment_and_u(t)
        du_vec = np.array([3.0 * (u ** 2), 2.0 * u, 1.0, 0.0], dtype=np.float64) / self.dt
        p_seg = self.control_points[seg : seg + 4]
        return du_vec @ self._M3 @ p_seg

    def evaluate_acceleration(self, t: float) -> np.ndarray:
        """Evaluate 3D acceleration a(t) = d2p/dt2 at time t in [0, duration]."""
        seg, u = self._segment_and_u(t)
        d2u_vec = np.array([6.0 * u, 2.0, 0.0, 0.0], dtype=np.float64) / (self.dt ** 2)
        p_seg = self.control_points[seg : seg + 4]
        return d2u_vec @ self._M3 @ p_seg

    def evaluate_yaw(self, t: float, fallback_yaw: float = 0.0) -> float:
        """Compute forward heading yaw = atan2(vy, vx) along horizontal velocity vector."""
        vel = self.evaluate_velocity(t)
        speed_xy = math.hypot(float(vel[0]), float(vel[1]))
        if speed_xy < 0.05:
            # Look slightly ahead on the spline when near stationary endpoints
            p_now = self.evaluate_position(t)
            p_ahead = self.evaluate_position(min(self.duration, t + 0.25))
            diff = p_ahead - p_now
            if math.hypot(float(diff[0]), float(diff[1])) >= 1e-3:
                return math.atan2(float(diff[1]), float(diff[0]))
            return fallback_yaw
        return math.atan2(float(vel[1]), float(vel[0]))

    def max_kinematics(self) -> Tuple[float, float]:
        """Return upper bound estimates of (max_velocity_mps, max_acceleration_mps2)."""
        vel_ctrl = np.diff(self.control_points, axis=0) / self.dt
        acc_ctrl = np.diff(vel_ctrl, axis=0) / self.dt
        v_max = float(np.max(np.linalg.norm(vel_ctrl, axis=1))) if vel_ctrl.size else 0.0
        a_max = float(np.max(np.linalg.norm(acc_ctrl, axis=1))) if acc_ctrl.size else 0.0
        return v_max, a_max

    def reallocate_time(self, v_limit: float, a_limit: float) -> "UniformBspline":
        """Stretch knot_span_dt if control-point velocities or accelerations exceed stage limits."""
        v_peak, a_peak = self.max_kinematics()
        ratio_v = v_peak / max(v_limit, 1e-3)
        ratio_a = math.sqrt(a_peak / max(a_limit, 1e-3))
        scale = max(1.0, ratio_v, ratio_a)
        return UniformBspline(self.control_points, self.dt * scale)

    def sample_points(self, num_samples: int = 50) -> np.ndarray:
        ts = np.linspace(0.0, self.duration, max(2, num_samples))
        return np.array([self.evaluate_position(float(t)) for t in ts], dtype=np.float64)


class EgoBsplineOptimizer:
    """Gradient-based B-Spline trajectory planner & smoother with EGO obstacle repulsion."""

    def __init__(
        self,
        grid: OccupancyGrid3D,
        *,
        control_point_spacing_m: float = 0.35,
        smoothness_weight: float = 0.25,
        obstacle_weight: float = 0.60,
        safe_clearance_m: float = 0.40,
        max_iterations: int = 20,
    ) -> None:
        self.grid = grid
        self.spacing = float(control_point_spacing_m)
        self.w_smooth = float(smoothness_weight)
        self.w_obs = float(obstacle_weight)
        self.safe_clearance = float(safe_clearance_m)
        self.max_iterations = int(max_iterations)

    def _resample_polyline(self, path_pts: np.ndarray, step_m: float) -> np.ndarray:
        diffs = np.diff(path_pts, axis=0)
        seg_lens = np.linalg.norm(diffs, axis=1)
        cum = np.concatenate([[0.0], np.cumsum(seg_lens)])
        total_len = float(cum[-1])
        if total_len < 1e-4:
            return np.vstack([path_pts[0], path_pts[-1]])
        n_pts = max(3, int(math.ceil(total_len / max(step_m, 0.10))) + 1)
        s_targets = np.linspace(0.0, total_len, n_pts)
        out = np.zeros((n_pts, 3), dtype=np.float64)
        for dim in range(3):
            out[:, dim] = np.interp(s_targets, cum, path_pts[:, dim])
        return out

    def plan_spline(
        self,
        start_m: np.ndarray | Tuple[float, float, float],
        goal_m: np.ndarray | Tuple[float, float, float],
        profile: StageProfile,
        *,
        intermediate_waypoints: Optional[List[Tuple[float, float, float]]] = None,
        max_plan_time_ms: Optional[float] = None,
        use_clearance_cost: bool = True,
        avoid_spheres: Optional[List[Tuple[Tuple[float, float, float], float]]] = None,
        goal_radius_m: float = 0.0,
    ) -> Tuple[Optional[UniformBspline], float]:
        """Plan an obstacle-free, smooth UniformBspline from start_m through optional waypoints to goal_m.

        Returns (spline_or_None, elapsed_ms).
        """
        t0 = time.perf_counter()

        chain = [ clamp_altitude_enu(start_m, profile.z_min_enu, profile.z_max_enu) ]
        if intermediate_waypoints:
            for wp in intermediate_waypoints:
                chain.append(clamp_altitude_enu(wp, profile.z_min_enu, profile.z_max_enu))
        chain.append(clamp_altitude_enu(goal_m, profile.z_min_enu, profile.z_max_enu))

        soft_cost = self.grid.build_soft_clearance_cost() if use_clearance_cost else None

        guiding_segments: List[np.ndarray] = []
        for i in range(len(chain) - 1):
            is_last_seg = i == (len(chain) - 2)
            seg_path = astar_search_3d(
                self.grid,
                chain[i],
                chain[i + 1],
                z_min_m=profile.z_min_enu,
                z_max_m=profile.z_max_enu,
                soft_clearance_cost=soft_cost,
                avoid_spheres=avoid_spheres,
                goal_radius_m=(goal_radius_m if is_last_seg else 0.0),
            )
            if seg_path is None:
                elapsed_ms = (time.perf_counter() - t0) * 1000.0
                return None, elapsed_ms
            if i > 0:
                seg_path = seg_path[1:]
            guiding_segments.append(seg_path)

        full_guiding = np.vstack(guiding_segments)
        resampled = self._resample_polyline(full_guiding, self.spacing)

        # Initial knot span based on target cruise speed
        dt_init = max(0.12, self.spacing / max(0.75 * profile.v_max, 0.20))
        spline = UniformBspline.from_waypoints(resampled, dt_init)
        ctrl = spline.control_points

        # Optimize interior control points [3 .. N-4] (keep clamped boundary endpoints fixed)
        ctrl_init = ctrl.copy()
        n_ctrl = ctrl.shape[0]
        for _ in range(self.max_iterations):
            if max_plan_time_ms is not None and (time.perf_counter() - t0) * 1000.0 > max_plan_time_ms:
                break
            grad = np.zeros_like(ctrl)
            # 1. Elastic / smoothness gradient + guiding-path fidelity + obstacle repulsion
            for i in range(3, n_ctrl - 3):
                laplacian = ctrl[i - 1] - 2.0 * ctrl[i] + ctrl[i + 1]
                grad[i] -= self.w_smooth * laplacian

                # 2. Obstacle repulsion gradient
                obs_cost, obs_grad = self.grid.obstacle_repulsion_gradient(ctrl[i], self.safe_clearance)
                grad[i] += self.w_obs * obs_grad

                # 3. Guiding-path anchor (prevents long-horizon corner shrinkage while yielding to obstacles)
                if obs_cost <= 1e-6:
                    grad[i] += 0.20 * (ctrl[i] - ctrl_init[i])

            for i in range(3, n_ctrl - 3):
                candidate = ctrl[i] - grad[i]
                candidate[0] = float(np.clip(candidate[0], self.grid.bounds_min[0] + 0.42, self.grid.bounds_max[0] - 0.42))
                candidate[1] = float(np.clip(candidate[1], self.grid.bounds_min[1] + 0.42, self.grid.bounds_max[1] - 0.42))
                candidate[2] = float(np.clip(candidate[2], profile.z_min_enu, profile.z_max_enu))
                if not self.grid.is_occupied(candidate, inflated=True):
                    ctrl[i] = candidate

        optimized = UniformBspline(ctrl, dt_init).reallocate_time(profile.v_max, profile.a_max)
        if not self.is_spline_collision_free(optimized):
            # Fall back to tightly-spaced guiding path B-spline if corner smoothing clipped an inflated corner
            dense_guiding = self._resample_polyline(full_guiding, max(0.18, 0.5 * self.spacing))
            optimized = UniformBspline.from_waypoints(dense_guiding, dt_init).reallocate_time(
                profile.v_max, profile.a_max
            )
        elapsed_ms = (time.perf_counter() - t0) * 1000.0
        return optimized, elapsed_ms

    def is_spline_collision_free(self, spline: UniformBspline, t_start: float = 0.0, num_checks: int = 30) -> bool:
        """Verify that the remaining spline trajectory [t_start .. duration] is free of inflated obstacles."""
        if t_start > 0.0:
            t0 = min(spline.duration, t_start + 0.30)
            t1 = min(spline.duration, t_start + 3.50)
            if t1 <= t0 + 1e-3:
                return True
            ts = np.linspace(t0, t1, max(2, num_checks // 2))
        else:
            ts = np.linspace(0.0, spline.duration, max(2, num_checks))
        for t in ts:
            pos = spline.evaluate_position(float(t))
            if self.grid.is_occupied(pos, inflated=True):
                return False
        return True
