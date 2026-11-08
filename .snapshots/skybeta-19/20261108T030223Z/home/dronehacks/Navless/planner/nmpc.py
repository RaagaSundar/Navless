"""Nonlinear Model Predictive Control (NMPC / MPCC) Short-Term Trajectory Planner.

Replaces open-loop B-spline carrot chasing with a receding-horizon 3D Model Predictive
Contouring Controller (MPCC / NMPC) that optimizes control accelerations u_0..u_{N-1}
directly from the drone's measured state (p_0, v_0):

  1. Discrete-time 3D double-integrator dynamics with jerk regularization:
       p_{k+1} = p_k + v_k * dt + 0.5 * u_k * dt^2
       v_{k+1} = v_k + u_k * dt
  2. Curvature-aware reference speed modulation:
       Automatically decelerates ahead of 90-deg maze/chicane turns within the braking
       horizon so the drone enters corners at a controlled speed instead of overshooting.
  3. Anisotropic Contouring vs. Lag Cost (MPCC):
       Penalizes cross-track error perpendicular to the corridor centerline 4x stronger
       than along-track lag error:
         Q_k = w_contour * I_3 - (w_contour - w_lag) * t_k * t_k^T
       so any lateral drift toward a wall is corrected immediately before advancing.
  4. Multi-Voxel ESDF Obstacle Barrier Potential:
       Uses `OccupancyGrid3D.multi_obstacle_repulsion_vector` across the prediction
       horizon to push the trajectory away from walls, corners, and unmapped obstacles.
"""

from __future__ import annotations

import math
import time
from dataclasses import dataclass
from typing import List, Optional, Tuple

import numpy as np

from .bspline import UniformBspline, astar_search_3d
from .config import StageProfile
from .frames import clamp_altitude_enu
from .grid_map import OccupancyGrid3D


@dataclass
class NMPCRollout:
    """Optimal short-term receding-horizon trajectory solved by `NonlinearMPCPlanner`."""

    dt: float
    positions_enu: np.ndarray       # shape (N + 1, 3)
    velocities_enu: np.ndarray      # shape (N + 1, 3)
    accelerations_enu: np.ndarray   # shape (N, 3)
    target_pos_enu: np.ndarray      # shape (3,) short-term position setpoint for PX4
    target_vel_enu: np.ndarray      # shape (3,) short-term velocity feedforward
    target_acc_enu: np.ndarray      # shape (3,) short-term acceleration feedforward
    target_yaw_enu: float           # camera heading aligned with short-term velocity/tangent
    ref_progress_t: float           # updated reference time along the guiding spline
    solve_time_ms: float
    min_clearance_m: float


class NonlinearMPCPlanner:
    """Receding-Horizon 3D Nonlinear MPC (MPCC) local trajectory planner and tracker."""

    def __init__(
        self,
        grid: OccupancyGrid3D,
        *,
        horizon_steps: int = 12,
        dt: float = 0.10,
        w_contour: float = 28.0,
        w_lag: float = 7.0,
        w_terminal: float = 36.0,
        w_vel: float = 6.5,
        w_acc: float = 0.85,
        w_jerk: float = 0.18,
        w_obs: float = 42.0,
        safe_clearance_m: float = 0.62,
        curvature_slowdown_gain: float = 0.65,
        max_iterations: int = 8,
    ) -> None:
        self.grid = grid
        self.N = int(max(4, horizon_steps))
        self.dt = float(dt)
        self.w_contour = float(w_contour)
        self.w_lag = float(w_lag)
        self.w_terminal = float(w_terminal)
        self.w_vel = float(w_vel)
        self.w_acc = float(w_acc)
        self.w_jerk = float(w_jerk)
        self.w_obs = float(w_obs)
        self.safe_clearance = float(safe_clearance_m)
        self.curvature_slowdown_gain = float(curvature_slowdown_gain)
        self.max_iterations = int(max_iterations)

        # Precompute condensed sensitivity matrices S_p, S_v, D_j and nominal Cholesky inverse
        self._Sp, self._Sv, self._Dj, self._H_inv = self._build_condensed_matrices(self.N, self.dt)
        self._warm_U: Optional[np.ndarray] = None

    def _build_condensed_matrices(
        self,
        n_steps: int,
        dt: float,
    ) -> Tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
        """Precompute lower-triangular rollout matrices for p = P0 + Sp @ U, v = V0 + Sv @ U."""
        sp = np.zeros((n_steps, n_steps), dtype=np.float64)
        sv = np.zeros((n_steps, n_steps), dtype=np.float64)
        for k in range(1, n_steps + 1):
            for j in range(k):
                sv[k - 1, j] = dt
                sp[k - 1, j] = (k - j - 0.5) * (dt * dt)

        dj = np.zeros((max(1, n_steps - 1), n_steps), dtype=np.float64)
        for k in range(n_steps - 1):
            dj[k, k] = -1.0 / dt
            dj[k, k + 1] = 1.0 / dt

        # Nominal isotropic Hessian for fast preconditioned quasi-Newton steps
        w_pos_diag = np.full(n_steps, self.w_contour, dtype=np.float64)
        w_pos_diag[-1] = max(self.w_contour, self.w_terminal)
        h_nom = (
            sp.T @ (w_pos_diag[:, None] * sp)
            + self.w_vel * (sv.T @ sv)
            + self.w_acc * np.eye(n_steps, dtype=np.float64)
            + self.w_jerk * (dj.T @ dj)
        )
        h_inv = np.linalg.inv(h_nom)
        return sp, sv, dj, h_inv

    @staticmethod
    def _resample_polyline(path_pts: np.ndarray, step_m: float) -> np.ndarray:
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

    def _project_controls(
        self,
        u_mat: np.ndarray,
        p0: np.ndarray,
        v0: np.ndarray,
        profile: StageProfile,
        *,
        z_min: Optional[float] = None,
        z_max: Optional[float] = None,
        xy_limit_m: Optional[float] = None,
    ) -> np.ndarray:
        """Enforce stage acceleration norms ||u_k|| <= a_max and velocity damping when ||v_k|| > v_max."""
        z_lo = float(z_min if z_min is not None else profile.z_min_enu)
        z_hi = float(z_max if z_max is not None else profile.z_max_enu)
        u_proj = u_mat.copy()
        a_norms = np.linalg.norm(u_proj, axis=1, keepdims=True)
        over_a = a_norms > profile.a_max
        if np.any(over_a):
            u_proj = np.where(over_a, u_proj * (profile.a_max / np.maximum(a_norms, 1e-6)), u_proj)

        # Forward pass to softly clamp velocity & altitude excursions
        v_cur = v0.copy()
        p_cur = p0.copy()
        for k in range(u_proj.shape[0]):
            v_next = v_cur + u_proj[k] * self.dt
            v_norm = float(np.linalg.norm(v_next))
            if v_norm > profile.v_max:
                v_next = v_next * (profile.v_max / v_norm)
                u_proj[k] = (v_next - v_cur) / self.dt
                a_n = float(np.linalg.norm(u_proj[k]))
                if a_n > profile.a_max:
                    u_proj[k] *= profile.a_max / a_n
                    v_next = v_cur + u_proj[k] * self.dt

            p_next = p_cur + v_cur * self.dt + 0.5 * u_proj[k] * (self.dt * self.dt)
            if p_next[2] < z_lo and v_next[2] < 0.0:
                u_proj[k, 2] = min(profile.a_max, max(0.0, -v_cur[2] / self.dt))
                v_next[2] = v_cur[2] + u_proj[k, 2] * self.dt
            elif p_next[2] > z_hi and v_next[2] > 0.0:
                u_proj[k, 2] = max(-profile.a_max, min(0.0, -v_cur[2] / self.dt))
                v_next[2] = v_cur[2] + u_proj[k, 2] * self.dt

            p_cur = p_cur + v_cur * self.dt + 0.5 * u_proj[k] * (self.dt * self.dt)
            v_cur = v_next
        return u_proj

    def solve_short_horizon(
        self,
        pos_enu: np.ndarray | Tuple[float, float, float],
        vel_enu: np.ndarray | Tuple[float, float, float],
        current_yaw_enu: float,
        ref_spline: UniformBspline,
        t_ref_start: float,
        profile: StageProfile,
        *,
        z_min_override: Optional[float] = None,
        xy_limit_m: Optional[float] = None,
        max_lead_m: float = 0.45,
    ) -> NMPCRollout:
        """Solve the receding-horizon 3D NMPC / MPCC problem from live state (pos_enu, vel_enu).

        Returns an `NMPCRollout` containing the full optimal state prediction and the immediate
        short-term lookahead target for PX4.
        """
        t0 = time.perf_counter()
        z_min = float(z_min_override if z_min_override is not None else profile.z_min_enu)
        z_max = float(profile.z_max_enu)
        p0 = clamp_altitude_enu(pos_enu, z_min, z_max, xy_limit_m=xy_limit_m)
        v0 = np.asarray(vel_enu, dtype=np.float64).copy()
        v0_norm = float(np.linalg.norm(v0))
        if v0_norm > profile.v_max * 1.5:
            v0 = v0 * (profile.v_max * 1.5 / v0_norm)

        # 1. Anchor reference time t_anchor on ref_spline in [t_ref_start - 0.35, t_ref_start] closest to p0:
        t_carrot = float(np.clip(t_ref_start, 0.0, ref_spline.duration))
        t_lo_search = max(0.0, t_carrot - 0.35)
        t_hi_search = t_carrot
        ts_local = np.linspace(t_lo_search, t_hi_search, 9)
        best_t = t_lo_search
        best_d = float("inf")
        for tc in ts_local:
            dc = float(np.linalg.norm(ref_spline.evaluate_position(float(tc)) - p0))
            if dc < best_d:
                best_d = dc
                best_t = float(tc)
        t_anchor = best_t

        # Compute upcoming corner curvature ahead of best_t (only when horizontal speed >= 0.15 m/s)
        yaw_bwd = ref_spline.evaluate_yaw(max(0.0, best_t - 0.25), fallback_yaw=current_yaw_enu)
        yaw_mid = ref_spline.evaluate_yaw(min(ref_spline.duration, best_t + 0.35), fallback_yaw=current_yaw_enu)
        yaw_fwd = ref_spline.evaluate_yaw(min(ref_spline.duration, best_t + 0.75), fallback_yaw=current_yaw_enu)
        ang1 = abs(math.atan2(math.sin(yaw_mid - yaw_bwd), math.cos(yaw_mid - yaw_bwd)))
        ang2 = abs(math.atan2(math.sin(yaw_fwd - yaw_mid), math.cos(yaw_fwd - yaw_mid)))
        upcoming_turn = max(ang1, ang2)
        turn_ratio = min(1.0, max(0.0, (upcoming_turn - 0.18) / 0.70))
        corner_speed_scale = 1.0 / (1.0 + self.curvature_slowdown_gain * turn_ratio)
        effective_lead_m = max_lead_m * max(0.46, corner_speed_scale) if max_lead_m > 0.0 else 0.45

        # 2. Sample curvature-modulated reference trajectory (p_ref, v_ref, tangents) over N steps
        p_ref = np.zeros((self.N, 3), dtype=np.float64)
        v_ref = np.zeros((self.N, 3), dtype=np.float64)
        tangents = np.zeros((self.N, 3), dtype=np.float64)
        t_cursor = t_anchor
        for k in range(self.N):
            y0 = ref_spline.evaluate_yaw(max(0.0, t_cursor - 0.20), fallback_yaw=current_yaw_enu)
            y1 = ref_spline.evaluate_yaw(min(ref_spline.duration, t_cursor + 0.45), fallback_yaw=current_yaw_enu)
            dk_yaw = abs(math.atan2(math.sin(y1 - y0), math.cos(y1 - y0)))
            k_turn = min(1.0, max(0.0, (dk_yaw - 0.18) / 0.70))
            step_scale = 1.0 / (1.0 + self.curvature_slowdown_gain * k_turn)
            t_cursor = min(ref_spline.duration, max(t_carrot if k >= 3 else 0.0, t_cursor + self.dt * step_scale))

            pk = clamp_altitude_enu(
                ref_spline.evaluate_position(t_cursor),
                z_min,
                z_max,
                xy_limit_m=xy_limit_m,
            )
            vk = ref_spline.evaluate_velocity(t_cursor) * step_scale
            vk_norm = float(np.linalg.norm(vk))
            if vk_norm > 1e-3:
                tk = vk / vk_norm
            else:
                p_ahead = ref_spline.evaluate_position(min(ref_spline.duration, t_cursor + 0.25))
                diff = p_ahead - pk
                d_norm = float(np.linalg.norm(diff))
                tk = (diff / d_norm) if d_norm > 1e-4 else np.array([math.cos(current_yaw_enu), math.sin(current_yaw_enu), 0.0])
            p_ref[k] = pk
            v_ref[k] = vk
            tangents[k] = tk

        # 3. Free-response rollout P_free, V_free (zero control)
        ks = np.arange(1, self.N + 1, dtype=np.float64)[:, None]
        p_free = p0[None, :] + (ks * self.dt) * v0[None, :]
        v_free = np.repeat(v0[None, :], self.N, axis=0)

        # Warm-start control sequence U in R^(N x 3)
        if self._warm_U is not None and self._warm_U.shape == (self.N, 3):
            u_mat = np.vstack([self._warm_U[1:], self._warm_U[-1:]])
        else:
            u_mat = np.zeros((self.N, 3), dtype=np.float64)
        u_mat = self._project_controls(u_mat, p0, v0, profile, z_min=z_min, z_max=z_max, xy_limit_m=xy_limit_m)

        # 4. Preconditioned Quasi-Newton / Nesterov optimization with Anisotropic MPCC + Obstacle Barrier
        u_prev = u_mat.copy()
        cached_obs_grad = np.zeros((self.N, 3), dtype=np.float64)
        for it in range(self.max_iterations):
            beta = 0.28 if it > 0 else 0.0
            u_eval = u_mat + beta * (u_mat - u_prev)
            p_roll = p_free + self._Sp @ u_eval
            v_roll = v_free + self._Sv @ u_eval

            # Anisotropic MPCC position gradient: decompose error into lag (along tangent) & contour (cross-track)
            err_p = p_roll - p_ref
            dot_t = np.sum(err_p * tangents, axis=1, keepdims=True)
            err_lag = dot_t * tangents
            err_contour = err_p - err_lag
            grad_p = self.w_lag * err_lag + self.w_contour * err_contour
            grad_p[-1] += (self.w_terminal - self.w_contour) * err_p[-1]

            # Soft altitude & arena fence barrier gradients
            low_z = p_roll[:, 2] < z_min + 0.04
            high_z = p_roll[:, 2] > z_max - 0.04
            if np.any(low_z):
                grad_p[low_z, 2] -= 45.0 * (z_min + 0.04 - p_roll[low_z, 2])
            if np.any(high_z):
                grad_p[high_z, 2] += 45.0 * (p_roll[high_z, 2] - (z_max - 0.04))

            # Multi-voxel obstacle repulsion barrier gradient across the horizon (skip during vertical fly-home climb)
            if z_min_override is None or z_min_override >= 0.20:
                if it % 3 == 0:
                    cached_obs_grad.fill(0.0)
                    for k in range(self.N):
                        obs_cost, obs_grad = self.grid.obstacle_repulsion_gradient(
                            p_roll[k],
                            self.safe_clearance,
                        )
                        if obs_cost > 1e-6:
                            cached_obs_grad[k] = self.w_obs * obs_grad
                grad_p += cached_obs_grad

            # Velocity tracking + speed limit gradient
            grad_v = self.w_vel * (v_roll - v_ref)
            v_norms = np.linalg.norm(v_roll, axis=1, keepdims=True)
            over_v = v_norms > (0.95 * profile.v_max)
            if np.any(over_v):
                excess = np.maximum(0.0, v_norms - 0.95 * profile.v_max)
                grad_v += 18.0 * excess * (v_roll / np.maximum(v_norms, 1e-6))

            # Total gradient w.r.t. U in R^(N x 3)
            grad_u = (
                self._Sp.T @ grad_p
                + self._Sv.T @ grad_v
                + self.w_acc * u_eval
                + self.w_jerk * (self._Dj.T @ (self._Dj @ u_eval))
            )

            u_prev = u_mat
            step_u = self._H_inv @ grad_u
            u_cand = u_eval - 0.65 * step_u
            u_mat = self._project_controls(u_cand, p0, v0, profile, z_min=z_min, z_max=z_max, xy_limit_m=xy_limit_m)
            if float(np.max(np.abs(u_mat - u_prev))) < 1e-3:
                break

        self._warm_U = u_mat.copy()
        p_opt = np.vstack([p0[None, :], p_free + self._Sp @ u_mat])
        v_opt = np.vstack([v0[None, :], v_free + self._Sv @ u_mat])
        for k in range(p_opt.shape[0]):
            p_opt[k] = clamp_altitude_enu(p_opt[k], z_min, z_max, xy_limit_m=xy_limit_m)

        # Select short-term lookahead setpoint on the solved NMPC rollout using PX4 position-loop
        # inversion (combining mid-horizon NMPC state p_opt[look_idx], carrot target, and immediate control u_0)
        look_idx = min(6, self.N)
        for k in range(1, min(7, self.N + 1)):
            if float(np.linalg.norm(p_opt[k] - p0)) <= max(effective_lead_m, 0.25):
                look_idx = k
        target_vel = v_opt[look_idx].copy()
        target_acc = u_mat[0].copy()
        p_carrot = clamp_altitude_enu(
            ref_spline.evaluate_position(t_carrot),
            z_min,
            z_max,
            xy_limit_m=xy_limit_m,
        )
        raw_target_pos = 0.50 * p_opt[look_idx] + 0.50 * p_carrot + 0.18 * target_acc
        target_pos = clamp_altitude_enu(
            raw_target_pos,
            z_min,
            z_max,
            xy_limit_m=xy_limit_m,
        )
        lead_vec = target_pos - p0
        lead_dist = float(np.linalg.norm(lead_vec))
        if effective_lead_m > 0.0 and lead_dist > effective_lead_m:
            target_pos = clamp_altitude_enu(
                p0 + (effective_lead_m / lead_dist) * lead_vec,
                z_min,
                z_max,
                xy_limit_m=xy_limit_m,
            )

        # Camera yaw aligned with the short-term NMPC velocity / reference tangent
        t_yaw_ref = max(0.0, t_anchor - 0.10)
        target_yaw = ref_spline.evaluate_yaw(t_yaw_ref, fallback_yaw=current_yaw_enu)

        min_clear = self.safe_clearance
        for k in range(1, min(6, self.N + 1)):
            obs_c, _ = self.grid.obstacle_repulsion_gradient(p_opt[k], self.safe_clearance)
            if obs_c > 0.0:
                min_clear = min(min_clear, max(0.0, self.safe_clearance - math.sqrt(obs_c)))

        elapsed_ms = (time.perf_counter() - t0) * 1000.0
        next_ref_t = min(ref_spline.duration, t_anchor + self.dt * corner_speed_scale)
        return NMPCRollout(
            dt=self.dt,
            positions_enu=p_opt,
            velocities_enu=v_opt,
            accelerations_enu=u_mat,
            target_pos_enu=target_pos,
            target_vel_enu=target_vel,
            target_acc_enu=target_acc,
            target_yaw_enu=target_yaw,
            ref_progress_t=next_ref_t,
            solve_time_ms=elapsed_ms,
            min_clearance_m=min_clear,
        )

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
        """Plan a 3D A* guiding path and optimize it via NMPC contouring + obstacle barrier.

        Drop-in compatible with `EgoBsplineOptimizer.plan_spline`, returning `(UniformBspline, elapsed_ms)`.
        """
        soft_cost = self.grid.build_soft_clearance_cost() if use_clearance_cost else None
        t0 = time.perf_counter()

        chain = [clamp_altitude_enu(start_m, profile.z_min_enu, profile.z_max_enu)]
        if intermediate_waypoints:
            for wp in intermediate_waypoints:
                chain.append(clamp_altitude_enu(wp, profile.z_min_enu, profile.z_max_enu))
        chain.append(clamp_altitude_enu(goal_m, profile.z_min_enu, profile.z_max_enu))

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
        spacing = 0.32
        pts = self._resample_polyline(full_guiding, spacing)
        n_pts = pts.shape[0]

        # Optimize interior trajectory states with Anisotropic MPCC Contouring + Multi-Voxel Barrier
        if use_clearance_cost and n_pts >= 4:
            ref_pts = pts.copy()
            tangents = np.zeros_like(ref_pts)
            diffs = np.diff(ref_pts, axis=0)
            norms = np.maximum(np.linalg.norm(diffs, axis=1, keepdims=True), 1e-6)
            tangents[:-1] = diffs / norms
            tangents[-1] = tangents[-2]

            for _ in range(18):
                if max_plan_time_ms is not None and (time.perf_counter() - t0) * 1000.0 >= 0.85 * max_plan_time_ms:
                    break
                grad = np.zeros_like(pts)
                # 4th-order jerk + acceleration smoothness
                grad[1:-1] += 0.22 * (2.0 * pts[1:-1] - pts[:-2] - pts[2:])
                if n_pts >= 5:
                    grad[2:-2] += 0.06 * (
                        pts[:-4] - 4.0 * pts[1:-3] + 6.0 * pts[2:-2] - 4.0 * pts[3:-1] + pts[4:]
                    )
                # Anisotropic MPCC contouring pullback (keeps corner vertices anchored against cutting)
                err = pts[1:-1] - ref_pts[1:-1]
                t_int = tangents[1:-1]
                err_lag = np.sum(err * t_int, axis=1, keepdims=True) * t_int
                err_cont = err - err_lag
                grad[1:-1] += 0.35 * err_cont + 0.08 * err_lag

                # Multi-voxel obstacle barrier repulsion
                for i in range(1, n_pts - 1):
                    obs_cost, obs_grad = self.grid.obstacle_repulsion_gradient(
                        pts[i],
                        self.safe_clearance,
                    )
                    if obs_cost > 1e-6:
                        grad[i] += 0.95 * obs_grad

                step = np.clip(0.18 * grad[1:-1], -0.05, 0.05)
                cand = pts[1:-1] - step
                cand[:, 0] = np.clip(cand[:, 0], self.grid.bounds_min[0] + 0.42, self.grid.bounds_max[0] - 0.42)
                cand[:, 1] = np.clip(cand[:, 1], self.grid.bounds_min[1] + 0.42, self.grid.bounds_max[1] - 0.42)
                cand[:, 2] = np.clip(cand[:, 2], profile.z_min_enu, profile.z_max_enu)
                for i in range(1, n_pts - 1):
                    if not self.grid.is_occupied(cand[i - 1], inflated=True):
                        pts[i] = cand[i - 1]

        dt_init = max(0.08, spacing / max(0.95 * profile.v_max, 0.20))
        spline = UniformBspline.from_waypoints(pts, dt_init).reallocate_time(profile.v_max, profile.a_max)
        if not self.is_spline_collision_free(spline):
            dense_guiding = self._resample_polyline(full_guiding, max(0.18, 0.5 * spacing))
            spline = UniformBspline.from_waypoints(dense_guiding, dt_init).reallocate_time(
                profile.v_max, profile.a_max
            )
        elapsed_ms = (time.perf_counter() - t0) * 1000.0
        return spline, elapsed_ms

    def is_spline_collision_free(
        self,
        spline: UniformBspline,
        t_start: float = 0.0,
        num_checks: int = 30,
    ) -> bool:
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
