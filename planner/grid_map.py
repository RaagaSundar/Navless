"""3D Voxel Occupancy Grid for EGO-Planner with 3D Layout Prior & OAK-D Depth integration.

Supports:
1. Pre-seeding static course geometry from Koji's 3D prior (axis-aligned boxes/walls,
   point clouds, or Wavefront .obj meshes).
2. Local rolling-window updates (6.0m x 6.0m x 2.0m @ 0.1m resolution) from OAK-D stereo
   depth images (`16UC1` mm or `32FC1` m) or 3D point clouds.
3. Obstacle inflation by drone safety radius (0.25-0.30 m).
4. Fast collision checking, obstacle repulsion gradient evaluation (no full ESDF needed),
   and Window aperture clearance/confidence scoring from ~1 m away.
"""

from __future__ import annotations

import json
import math
from dataclasses import dataclass
from pathlib import Path
from typing import Dict, Iterable, List, Optional, Tuple

import numpy as np


@dataclass(frozen=True)
class WindowInspectResult:
    """Result of evaluating the Window opening from the ~1 m inspection pose."""

    confidence: float
    clear_ratio: float
    wall_detected_ratio: float
    is_passable: bool
    reason: str
    aperture_center_m: Optional[Tuple[float, float, float]] = None
    approach_point_m: Optional[Tuple[float, float, float]] = None
    exit_point_m: Optional[Tuple[float, float, float]] = None


@dataclass(frozen=True)
class BarrierJumpSpec:
    """Dynamically detected wall/barrier between landing position and start origin."""

    wall_detected: bool
    wall_xy_m: Tuple[float, float]
    wall_top_z_m: float
    safe_hop_z_m: float
    apex_waypoint_m: Tuple[float, float, float]


@dataclass(frozen=True)
class DetectedCourseFeatures:
    """Course features dynamically discovered from the 3D occupancy map (no hardcoded turns)."""

    tunnel_entry_m: Tuple[float, float, float]
    tunnel_mid_m: Tuple[float, float, float]
    tunnel_exit_m: Tuple[float, float, float]
    window_center_m: Optional[Tuple[float, float, float]]
    window_normal_xy: Tuple[float, float]
    window_approach_m: Optional[Tuple[float, float, float]]
    window_exit_m: Optional[Tuple[float, float, float]]


class OccupancyGrid3D:
    """3D Voxel Grid storing raw occupancy and safety-inflated occupancy."""

    def __init__(
        self,
        bounds_min_m: Tuple[float, float, float] = (-3.1, -3.1, 0.0),
        bounds_max_m: Tuple[float, float, float] = (3.1, 3.1, 2.6),
        resolution_m: float = 0.10,
        inflation_radius_m: float = 0.22,
        local_window_m: Tuple[float, float, float] = (6.0, 6.0, 2.0),
    ) -> None:
        if resolution_m <= 0.0:
            raise ValueError("resolution_m must be positive")
        self.bounds_min = np.asarray(bounds_min_m, dtype=np.float64)
        self.bounds_max = np.asarray(bounds_max_m, dtype=np.float64)
        self.resolution = float(resolution_m)
        self.inflation_radius = float(inflation_radius_m)
        self.local_window = np.asarray(local_window_m, dtype=np.float64)

        span = self.bounds_max - self.bounds_min
        self.shape: Tuple[int, int, int] = tuple(
            max(1, int(math.ceil(s / self.resolution))) for s in span
        )
        # Raw occupancy: True where static prior or live depth obstacles exist
        self.raw_grid = np.zeros(self.shape, dtype=bool)
        # Inflated occupancy: True within inflation_radius_m of any occupied voxel
        self.inflated_grid = np.zeros(self.shape, dtype=bool)
        self._inflation_offsets = self._build_spherical_offsets(self.inflation_radius)

    def _build_spherical_offsets(self, radius_m: float) -> np.ndarray:
        r_vox = int(math.ceil(radius_m / self.resolution))
        if r_vox <= 0:
            return np.zeros((1, 3), dtype=np.int32)
        rng = np.arange(-r_vox, r_vox + 1, dtype=np.int32)
        gx, gy, gz = np.meshgrid(rng, rng, rng, indexing="ij")
        dist_sq = (gx * self.resolution) ** 2 + (gy * self.resolution) ** 2 + (gz * self.resolution) ** 2
        mask = dist_sq <= (radius_m + 1e-6) ** 2
        return np.stack([gx[mask], gy[mask], gz[mask]], axis=-1)

    def world_to_voxel(self, point_m: np.ndarray | Tuple[float, float, float]) -> Tuple[int, int, int]:
        """Convert world coordinates [x, y, z] (m) to grid indices (ix, iy, iz)."""
        p = np.asarray(point_m, dtype=np.float64)
        idx = np.floor((p - self.bounds_min) / self.resolution).astype(np.int32)
        return (int(idx[0]), int(idx[1]), int(idx[2]))

    def voxel_to_world(self, idx: Tuple[int, int, int] | np.ndarray) -> np.ndarray:
        """Convert grid indices (ix, iy, iz) to voxel center [x, y, z] (m)."""
        i = np.asarray(idx, dtype=np.float64)
        return self.bounds_min + (i + 0.5) * self.resolution

    def in_bounds_voxel(self, idx: Tuple[int, int, int]) -> bool:
        ix, iy, iz = idx
        nx, ny, nz = self.shape
        return (0 <= ix < nx) and (0 <= iy < ny) and (0 <= iz < nz)

    def in_bounds_world(self, point_m: np.ndarray | Tuple[float, float, float]) -> bool:
        return self.in_bounds_voxel(self.world_to_voxel(point_m))

    def is_occupied(self, point_m: np.ndarray | Tuple[float, float, float], *, inflated: bool = True) -> bool:
        """Return True if point is out of bounds or occupied."""
        idx = self.world_to_voxel(point_m)
        if not self.in_bounds_voxel(idx):
            return True
        grid = self.inflated_grid if inflated else self.raw_grid
        return bool(grid[idx])

    def _inflate_voxels(self, voxel_indices: np.ndarray) -> None:
        """Inflate a batch of (N, 3) occupied voxel indices into self.inflated_grid."""
        if voxel_indices.size == 0:
            return
        nx, ny, nz = self.shape
        for dx, dy, dz in self._inflation_offsets:
            ix = voxel_indices[:, 0] + dx
            iy = voxel_indices[:, 1] + dy
            iz = voxel_indices[:, 2] + dz
            valid = (ix >= 0) & (ix < nx) & (iy >= 0) & (iy < ny) & (iz >= 0) & (iz < nz)
            self.inflated_grid[ix[valid], iy[valid], iz[valid]] = True

    def rebuild_inflation(self) -> None:
        """Recompute inflated_grid from scratch using raw_grid."""
        self.inflated_grid.fill(False)
        occ = np.argwhere(self.raw_grid)
        self._inflate_voxels(occ)

    def add_aabb_obstacle(
        self,
        min_corner_m: Tuple[float, float, float],
        max_corner_m: Tuple[float, float, float],
        *,
        update_inflation: bool = True,
    ) -> None:
        """Mark an axis-aligned bounding box [xmin..xmax, ymin..ymax, zmin..zmax] as occupied."""
        imin = np.maximum(0, np.floor((np.asarray(min_corner_m) - self.bounds_min) / self.resolution).astype(int))
        imax = np.minimum(
            np.asarray(self.shape) - 1,
            np.floor((np.asarray(max_corner_m) - self.bounds_min) / self.resolution).astype(int),
        )
        if np.any(imin > imax):
            return
        self.raw_grid[
            imin[0] : imax[0] + 1,
            imin[1] : imax[1] + 1,
            imin[2] : imax[2] + 1,
        ] = True
        if update_inflation:
            gx, gy, gz = np.meshgrid(
                np.arange(imin[0], imax[0] + 1),
                np.arange(imin[1], imax[1] + 1),
                np.arange(imin[2], imax[2] + 1),
                indexing="ij",
            )
            vox = np.stack([gx.ravel(), gy.ravel(), gz.ravel()], axis=-1)
            self._inflate_voxels(vox)

    def carve_aabb_opening(
        self,
        min_corner_m: Tuple[float, float, float],
        max_corner_m: Tuple[float, float, float],
        *,
        rebuild: bool = True,
    ) -> None:
        """Clear an aperture (such as a Gate or Window opening) inside a wall."""
        imin = np.maximum(0, np.floor((np.asarray(min_corner_m) - self.bounds_min) / self.resolution).astype(int))
        imax = np.minimum(
            np.asarray(self.shape) - 1,
            np.floor((np.asarray(max_corner_m) - self.bounds_min) / self.resolution).astype(int),
        )
        if np.any(imin > imax):
            return
        self.raw_grid[
            imin[0] : imax[0] + 1,
            imin[1] : imax[1] + 1,
            imin[2] : imax[2] + 1,
        ] = False
        if rebuild:
            self.rebuild_inflation()

    def load_prior_from_json(self, path: str | Path) -> None:
        """Pre-seed map from a JSON layout file containing `walls` (boxes) and `openings`."""
        data = json.loads(Path(path).read_text(encoding="utf-8"))
        for wall in data.get("walls", []):
            self.add_aabb_obstacle(tuple(wall["min"]), tuple(wall["max"]), update_inflation=False)
        for opening in data.get("openings", []):
            self.carve_aabb_opening(tuple(opening["min"]), tuple(opening["max"]), rebuild=False)
        if "points" in data and len(data["points"]) > 0:
            pts = np.asarray(data["points"], dtype=np.float64)
            self.insert_point_cloud(pts, sensor_origin_m=None)
        self.rebuild_inflation()

    def load_prior_from_obj(
        self,
        path: str | Path,
        sample_step_m: float = 0.08,
        *,
        min_obstacle_z_m: float = 0.04,
        max_obstacle_z_m: float = 2.95,
    ) -> int:
        """Pre-seed static obstacles from a Wavefront .obj 3D course mesh (`dronehacks-course-v3.obj`).

        Filters out flat floor markings (z <= min_obstacle_z_m) and the 3.048m roof net
        (z >= max_obstacle_z_m), and vectorizes triangle sampling for large wall/tunnel faces.
        """
        vertices: List[Tuple[float, float, float]] = []
        faces: List[Tuple[int, int, int]] = []
        for raw_line in Path(path).read_text(encoding="utf-8", errors="ignore").splitlines():
            line = raw_line.strip()
            if line.startswith("v "):
                parts = line.split()
                if len(parts) >= 4:
                    vertices.append((float(parts[1]), float(parts[2]), float(parts[3])))
            elif line.startswith("f "):
                parts = line.split()[1:]
                idxs = [int(p.split("/")[0]) - 1 for p in parts if p]
                for i in range(1, len(idxs) - 1):
                    faces.append((idxs[0], idxs[i], idxs[i + 1]))

        if not vertices:
            return 0

        v_arr = np.asarray(vertices, dtype=np.float64)
        valid_v = (v_arr[:, 2] > min_obstacle_z_m) & (v_arr[:, 2] < max_obstacle_z_m)
        sampled_batches: List[np.ndarray] = [v_arr[valid_v]]

        if faces:
            f_arr = np.asarray(faces, dtype=np.int32)
            v0 = v_arr[f_arr[:, 0]]
            v1 = v_arr[f_arr[:, 1]]
            v2 = v_arr[f_arr[:, 2]]
            tri_z_max = np.maximum(np.maximum(v0[:, 2], v1[:, 2]), v2[:, 2])
            tri_z_min = np.minimum(np.minimum(v0[:, 2], v1[:, 2]), v2[:, 2])
            e1 = np.linalg.norm(v1 - v0, axis=1)
            e2 = np.linalg.norm(v2 - v0, axis=1)
            e3 = np.linalg.norm(v2 - v1, axis=1)
            max_edge = np.maximum(np.maximum(e1, e2), e3)

            # Only large structural triangles (walls, barrier, tunnel) need interior sampling;
            # dense boulder meshes already have vertices < 2 cm apart.
            large_mask = (
                (tri_z_max > min_obstacle_z_m)
                & (tri_z_min < max_obstacle_z_m)
                & (max_edge > sample_step_m)
            )
            for idx in np.flatnonzero(large_mask):
                p0, p1, p2 = v0[idx], v1[idx], v2[idx]
                n_steps = max(2, int(math.ceil(float(max_edge[idx]) / max(sample_step_m, 0.04))))
                us = np.linspace(0.0, 1.0, n_steps + 1)
                uu, vv = np.meshgrid(us, us, indexing="ij")
                mask = (uu + vv) <= 1.0 + 1e-6
                u_flat = uu[mask][:, None]
                v_flat = vv[mask][:, None]
                pts = p0[None, :] + u_flat * (p1 - p0)[None, :] + v_flat * (p2 - p0)[None, :]
                z_ok = (pts[:, 2] > min_obstacle_z_m) & (pts[:, 2] < max_obstacle_z_m)
                if np.any(z_ok):
                    sampled_batches.append(pts[z_ok])

        all_pts = np.vstack(sampled_batches)
        return self.insert_point_cloud(all_pts, sensor_origin_m=None)

    def insert_point_cloud(
        self,
        points_world_m: np.ndarray,
        sensor_origin_m: Optional[np.ndarray | Tuple[float, float, float]] = None,
    ) -> int:
        """Insert 3D obstacle points into the local rolling window (or global grid if sensor_origin_m is None)."""
        pts = np.asarray(points_world_m, dtype=np.float64).reshape(-1, 3)
        if pts.size == 0:
            return 0

        if sensor_origin_m is not None:
            origin = np.asarray(sensor_origin_m, dtype=np.float64)
            half_win = 0.5 * self.local_window
            in_win = np.all(np.abs(pts - origin) <= half_win, axis=1)
            pts = pts[in_win]
            if pts.size == 0:
                return 0

        idx = np.floor((pts - self.bounds_min) / self.resolution).astype(np.int32)
        nx, ny, nz = self.shape
        valid = (
            (idx[:, 0] >= 0)
            & (idx[:, 0] < nx)
            & (idx[:, 1] >= 0)
            & (idx[:, 1] < ny)
            & (idx[:, 2] >= 0)
            & (idx[:, 2] < nz)
        )
        idx = np.unique(idx[valid], axis=0)
        if idx.size == 0:
            return 0

        self.raw_grid[idx[:, 0], idx[:, 1], idx[:, 2]] = True
        self._inflate_voxels(idx)
        return int(idx.shape[0])

    def update_from_depth_image(
        self,
        depth_metres: np.ndarray,
        fx: float,
        fy: float,
        cx: float,
        cy: float,
        drone_pos_enu: np.ndarray | Tuple[float, float, float],
        drone_yaw_rad: float,
        *,
        stride: int = 8,
        min_depth_m: float = 0.30,
        max_depth_m: float = 3.50,
    ) -> int:
        """Project a ROS depth image (`16UC1` converted to m or `32FC1` m) into the local voxel grid.

        Follows `dronehacks-flight-notebook.ipynb` Chapter 03 rules:
        excludes zero/NaN/infinite pixels and restricts updates to the local window.
        """
        if depth_metres.ndim != 2:
            raise ValueError("depth_metres must be a 2D array")
        sub = depth_metres[::stride, ::stride]
        rows, cols = np.indices(sub.shape)
        u = cols * stride
        v = rows * stride
        z_cam = sub.astype(np.float64)
        valid = np.isfinite(z_cam) & (z_cam >= min_depth_m) & (z_cam <= max_depth_m)
        if not np.any(valid):
            return 0

        z_c = z_cam[valid]
        u_c = u[valid]
        v_c = v[valid]

        # Optical frame (z forward, x right, y down) -> Body FLU (x forward, y left, z up)
        x_flu = z_c
        y_flu = -(u_c - cx) * z_c / fx
        z_flu = -(v_c - cy) * z_c / fy

        c, s = math.cos(drone_yaw_rad), math.sin(drone_yaw_rad)
        pos = np.asarray(drone_pos_enu, dtype=np.float64)
        x_w = pos[0] + c * x_flu - s * y_flu
        y_w = pos[1] + s * x_flu + c * y_flu
        z_w = pos[2] + z_flu

        pts_world = np.stack([x_w, y_w, z_w], axis=-1)
        return self.insert_point_cloud(pts_world, sensor_origin_m=pos)

    def obstacle_repulsion_gradient(
        self,
        point_m: np.ndarray | Tuple[float, float, float],
        safe_distance_m: float = 0.45,
    ) -> Tuple[float, np.ndarray]:
        """Compute local collision cost and repulsive gradient away from nearby raw obstacles.

        Mirrors EGO-Planner's ESDF-free local repulsion check within a small neighborhood.
        Returns (cost, d_cost_d_pos).
        """
        p = np.asarray(point_m, dtype=np.float64)
        center_idx = self.world_to_voxel(p)
        r_vox = max(1, int(math.ceil(safe_distance_m / self.resolution)))
        nx, ny, nz = self.shape

        ix0, ix1 = max(0, center_idx[0] - r_vox), min(nx - 1, center_idx[0] + r_vox)
        iy0, iy1 = max(0, center_idx[1] - r_vox), min(ny - 1, center_idx[1] + r_vox)
        iz0, iz1 = max(0, center_idx[2] - r_vox), min(nz - 1, center_idx[2] + r_vox)

        sub = self.raw_grid[ix0 : ix1 + 1, iy0 : iy1 + 1, iz0 : iz1 + 1]
        if not np.any(sub):
            return 0.0, np.zeros(3, dtype=np.float64)

        occ_local = np.argwhere(sub)
        occ_global = occ_local + np.array([ix0, iy0, iz0], dtype=np.int32)
        occ_centers = self.bounds_min + (occ_global.astype(np.float64) + 0.5) * self.resolution

        diffs = p[None, :] - occ_centers
        dists = np.linalg.norm(diffs, axis=1)
        min_i = int(np.argmin(dists))
        d = float(dists[min_i])
        if d >= safe_distance_m:
            return 0.0, np.zeros(3, dtype=np.float64)

        penetration = safe_distance_m - d
        cost = 0.5 * (penetration ** 2)
        if d > 1e-6:
            direction = diffs[min_i] / d
        else:
            direction = np.array([0.0, 1.0, 0.0], dtype=np.float64)
        # Gradient of cost wrt p (points toward obstacle; subtracting gradient pushes p away)
        grad = -penetration * direction
        return cost, grad

    def inspect_window_aperture(
        self,
        window_center_m: Tuple[float, float, float],
        *,
        aperture_half_width_y_m: float = 0.25,
        aperture_half_height_z_m: float = 0.20,
        frame_ring_thickness_m: float = 0.25,
        confidence_threshold: float = 0.80,
    ) -> WindowInspectResult:
        """Evaluate whether the Window aperture is open and clearly observable in the voxel grid.

        Used at ~1 m distance (Goal 2 & Step 5):
        - Checks that the inner cutout aperture is free of raw/inflated obstacles.
        - Checks whether the surrounding wall frame has observed occupancy (or prior occupancy).
        """
        wx, wy, wz = window_center_m
        # Sample grid points inside the inner window opening
        ys = np.linspace(wy - aperture_half_width_y_m, wy + aperture_half_width_y_m, 5)
        zs = np.linspace(wz - aperture_half_height_z_m, wz + aperture_half_height_z_m, 5)
        inner_clear = 0
        inner_total = 0
        for y in ys:
            for z in zs:
                inner_total += 1
                if not self.is_occupied((wx, float(y), float(z)), inflated=False):
                    inner_clear += 1
        clear_ratio = inner_clear / max(1, inner_total)

        # Also check that the center trajectory point is free in the inflated grid
        center_inflated_free = not self.is_occupied((wx, wy, wz), inflated=True)

        # Check surrounding frame ring for wall structure presence
        frame_hits = 0
        frame_total = 0
        frame_offsets_y = [
            wy - aperture_half_width_y_m - frame_ring_thickness_m,
            wy + aperture_half_width_y_m + frame_ring_thickness_m,
        ]
        for fy in frame_offsets_y:
            for z in zs:
                frame_total += 1
                if self.is_occupied((wx, float(fy), float(z)), inflated=False):
                    frame_hits += 1
        wall_ratio = frame_hits / max(1, frame_total)

        # Confidence combines aperture openness and center safety clearance
        if not center_inflated_free or clear_ratio < 0.85:
            confidence = 0.30 * clear_ratio
            return WindowInspectResult(
                confidence=confidence,
                clear_ratio=clear_ratio,
                wall_detected_ratio=wall_ratio,
                is_passable=False,
                reason="window_aperture_blocked_or_narrow",
            )

        # High confidence when aperture is clear and surrounding wall/prior is consistent
        confidence = min(1.0, 0.75 * clear_ratio + 0.25 * max(0.5, wall_ratio))
        is_passable = confidence >= confidence_threshold
        reason = "window_clear_and_confident" if is_passable else "window_confidence_below_threshold"
        return WindowInspectResult(
            confidence=confidence,
            clear_ratio=clear_ratio,
            wall_detected_ratio=wall_ratio,
            is_passable=is_passable,
            reason=reason,
            aperture_center_m=window_center_m,
        )

    def detect_window_cutout(
        self,
        *,
        search_center_m: Optional[Tuple[float, float, float]] = None,
        search_radius_xy_m: float = 3.5,
        standoff_distance_m: float = 1.0,
        approach_from_xy: Optional[Tuple[float, float]] = None,
    ) -> Optional[Tuple[Tuple[float, float, float], Tuple[float, float], Tuple[float, float, float], Tuple[float, float, float]]]:
        """Dynamically detect a Window cutout in the 3D voxel grid from geometry/sensor data.

        Identifies (ix, iy) columns that have:
          - Occupied sill voxels below (0.05m <= z <= 0.35m)
          - Occupied lintel voxels above (1.20m <= z <= 1.70m)
          - Clear middle aperture (0.45m <= z <= 1.05m)
        Computes the 3D centroid, wall normal vector (via PCA), ~1m approach point, and exit point.
        """
        iz_sill_0 = max(0, int(math.floor((0.05 - self.bounds_min[2]) / self.resolution)))
        iz_sill_1 = min(self.shape[2] - 1, int(math.ceil((0.35 - self.bounds_min[2]) / self.resolution)))
        iz_mid_0 = max(0, int(math.floor((0.45 - self.bounds_min[2]) / self.resolution)))
        iz_mid_1 = min(self.shape[2] - 1, int(math.floor((1.05 - self.bounds_min[2]) / self.resolution)))
        iz_lint_low_0 = max(0, int(math.floor((1.22 - self.bounds_min[2]) / self.resolution)))
        iz_lint_low_1 = min(self.shape[2] - 1, int(math.ceil((1.45 - self.bounds_min[2]) / self.resolution)))
        iz_lint_top_0 = max(0, int(math.floor((1.55 - self.bounds_min[2]) / self.resolution)))
        iz_lint_top_1 = min(self.shape[2] - 1, int(math.ceil((1.74 - self.bounds_min[2]) / self.resolution)))
        has_sill = np.any(self.raw_grid[:, :, iz_sill_0 : iz_sill_1 + 1], axis=2)
        has_lintel = (
            np.any(self.raw_grid[:, :, iz_lint_low_0 : iz_lint_low_1 + 1], axis=2)
            & np.any(self.raw_grid[:, :, iz_lint_top_0 : iz_lint_top_1 + 1], axis=2)
        )
        mid_clear = ~np.any(self.raw_grid[:, :, iz_mid_0 : iz_mid_1 + 1], axis=2)

        cutout_cols = np.argwhere(has_sill & has_lintel & mid_clear)
        if cutout_cols.shape[0] < 3:
            return None

        xy_world = (
            self.bounds_min[:2][None, :]
            + (cutout_cols.astype(np.float64) + 0.5) * self.resolution
        )
        # Exclude tunnel side-wall ribs by clustering cutout columns and selecting the
        # compact wall-aperture cluster in the post-tunnel region (or nearest to search_center_m).
        if search_center_m is not None:
            sc = np.asarray(search_center_m[:2], dtype=np.float64)
            dists = np.linalg.norm(xy_world - sc[None, :], axis=1)
            xy_world = xy_world[dists <= search_radius_xy_m]
        else:
            # Group into connected components (within 0.25m in XY) and pick the window cluster
            clusters: List[np.ndarray] = []
            visited = np.zeros(xy_world.shape[0], dtype=bool)
            for i in range(xy_world.shape[0]):
                if visited[i]:
                    continue
                comp = [i]
                visited[i] = True
                queue = [i]
                while queue:
                    cur = queue.pop()
                    d = np.linalg.norm(xy_world - xy_world[cur][None, :], axis=1)
                    nbrs = np.flatnonzero((~visited) & (d <= 0.25))
                    for n_i in nbrs:
                        visited[n_i] = True
                        comp.append(int(n_i))
                        queue.append(int(n_i))
                if len(comp) >= 3:
                    clusters.append(xy_world[comp])
            if not clusters:
                return None
            # Select cluster with the lowest mean Y (post-tunnel southern chicane wall)
            clusters.sort(key=lambda cl: float(np.mean(cl[:, 1])))
            xy_world = clusters[0]

        if xy_world.shape[0] < 3:
            return None

        center_xy = np.mean(xy_world, axis=0)
        # Estimate vertical span of the opening at the centroid
        c_ix, c_iy, _ = self.world_to_voxel((float(center_xy[0]), float(center_xy[1]), 0.75))
        col_occ = self.raw_grid[c_ix, c_iy, :]
        z_centers = self.bounds_min[2] + (np.arange(self.shape[2]) + 0.5) * self.resolution
        sill_zs = z_centers[(z_centers <= 0.45) & col_occ]
        lintel_zs = z_centers[(z_centers >= 1.10) & (z_centers <= 1.75) & col_occ]
        z_bot = float(np.max(sill_zs)) if sill_zs.size else 0.305
        z_top = float(np.min(lintel_zs)) if lintel_zs.size else 1.219
        center_z = 0.5 * (z_bot + z_top)

        # Principal component along XY gives wall tangent; normal is perpendicular
        centered = xy_world - center_xy[None, :]
        cov = centered.T @ centered
        eigvals, eigvecs = np.linalg.eigh(cov)
        tangent = eigvecs[:, int(np.argmax(eigvals))]
        normal = np.array([-tangent[1], tangent[0]], dtype=np.float64)
        normal /= max(np.linalg.norm(normal), 1e-6)

        # Orient normal toward the drone's approach side (default: East/+X side from tunnel exit)
        ref_xy = np.asarray(approach_from_xy if approach_from_xy is not None else (2.0, -1.0), dtype=np.float64)
        if np.dot(ref_xy - center_xy, normal) < 0.0:
            normal = -normal

        win_center = (float(center_xy[0]), float(center_xy[1]), float(center_z))
        approach_pt = (
            float(center_xy[0] + standoff_distance_m * normal[0]),
            float(center_xy[1] + standoff_distance_m * normal[1]),
            float(center_z),
        )
        exit_pt = (
            float(center_xy[0] - standoff_distance_m * normal[0]),
            float(center_xy[1] - standoff_distance_m * normal[1]),
            float(center_z),
        )
        return win_center, (float(normal[0]), float(normal[1])), approach_pt, exit_pt

    def build_soft_clearance_cost(self, buffer_radius_m: float = 0.55) -> np.ndarray:
        """Build a 3D soft cost field (in [0, 3.0]) that penalizes proximity to walls.

        Used by 3D A* so paths naturally stay centered in corridors and turn smoothly
        around maze & chicane walls without needing hardcoded intermediate waypoints.
        """
        cost = np.zeros(self.shape, dtype=np.float32)
        occ = np.argwhere(self.raw_grid)
        if occ.size == 0:
            return cost
        offsets = self._build_spherical_offsets(buffer_radius_m)
        nx, ny, nz = self.shape
        for dx, dy, dz in offsets:
            dist = math.sqrt((dx * self.resolution) ** 2 + (dy * self.resolution) ** 2 + (dz * self.resolution) ** 2)
            if dist <= self.inflation_radius or dist > buffer_radius_m:
                continue
            penalty = float(2.5 * ((buffer_radius_m - dist) / max(buffer_radius_m - self.inflation_radius, 0.05)) ** 2)
            ix = occ[:, 0] + dx
            iy = occ[:, 1] + dy
            iz = occ[:, 2] + dz
            valid = (ix >= 0) & (ix < nx) & (iy >= 0) & (iy < ny) & (iz >= 0) & (iz < nz)
            np.maximum.at(cost, (ix[valid], iy[valid], iz[valid]), penalty)
        return cost

    def discover_course_features(
        self,
        *,
        standoff_distance_m: float = 0.90,
        cruise_z_m: float = 0.85,
    ) -> DetectedCourseFeatures:
        """Automatically discover Tunnel entrance/exit and Window aperture from the 3D voxel map."""
        # 1. Detect tunnel from overhead arch structure (1.74m <= z <= 1.95m) with clear interior
        iz_arch_0 = max(0, int(math.floor((1.74 - self.bounds_min[2]) / self.resolution)))
        iz_arch_1 = min(self.shape[2] - 1, int(math.ceil((1.95 - self.bounds_min[2]) / self.resolution)))
        iz_mid_0 = max(0, int(math.floor((0.55 - self.bounds_min[2]) / self.resolution)))
        iz_mid_1 = min(self.shape[2] - 1, int(math.floor((1.10 - self.bounds_min[2]) / self.resolution)))

        has_arch = np.any(self.raw_grid[:, :, iz_arch_0 : iz_arch_1 + 1], axis=2)
        interior_clear = ~np.any(self.inflated_grid[:, :, iz_mid_0 : iz_mid_1 + 1], axis=2)
        tunnel_cols = np.argwhere(has_arch & interior_clear)

        if tunnel_cols.shape[0] >= 5:
            xy_tun = (
                self.bounds_min[:2][None, :]
                + (tunnel_cols.astype(np.float64) + 0.5) * self.resolution
            )
            tun_x = float(0.5 * (np.min(xy_tun[:, 0]) + np.max(xy_tun[:, 0])))
            tun_y_max = float(np.max(xy_tun[:, 1]))
            tun_y_min = float(np.min(xy_tun[:, 1]))
            tunnel_entry = (tun_x, tun_y_max + 0.35, cruise_z_m)
            tunnel_mid = (tun_x, 0.5 * (tun_y_max + tun_y_min), cruise_z_m)
            tunnel_exit = (tun_x, tun_y_min - 0.40, cruise_z_m)
        else:
            tunnel_entry = (2.311, 1.550, cruise_z_m)
            tunnel_mid = (2.311, 0.450, cruise_z_m)
            tunnel_exit = (2.311, -0.750, cruise_z_m)

        # 2. Detect Window cutout from sill + lintel + clear aperture signature
        win_info = self.detect_window_cutout(
            standoff_distance_m=standoff_distance_m,
            approach_from_xy=(tunnel_exit[0], tunnel_exit[1]),
        )
        if win_info is not None:
            win_center, win_normal, win_approach, win_exit = win_info
        else:
            win_center = None
            win_normal = (0.923, -0.385)
            win_approach = None
            win_exit = None

        return DetectedCourseFeatures(
            tunnel_entry_m=tunnel_entry,
            tunnel_mid_m=tunnel_mid,
            tunnel_exit_m=tunnel_exit,
            window_center_m=win_center,
            window_normal_xy=win_normal,
            window_approach_m=win_approach,
            window_exit_m=win_exit,
        )

    def detect_barrier_between(
        self,
        from_pos_m: np.ndarray | Tuple[float, float, float],
        to_pos_m: np.ndarray | Tuple[float, float, float],
        *,
        clearance_above_wall_m: float = 0.28,
        min_hop_z_m: float = 1.85,
        max_hop_z_m: float = 2.30,
    ) -> BarrierJumpSpec:
        """Inspect the 3D occupancy map along the direct line from landing pos to start pos.

        Finds the barrier wall location and height between `from_pos_m` and `to_pos_m` and
        computes the dynamic apex waypoint to jump across the wall back to the start.
        """
        p0 = np.asarray(from_pos_m, dtype=np.float64)
        p1 = np.asarray(to_pos_m, dtype=np.float64)
        dist_xy = float(np.linalg.norm(p1[:2] - p0[:2]))
        n_samples = max(10, int(math.ceil(dist_xy / (0.5 * self.resolution))))

        ts = np.linspace(0.0, 1.0, n_samples)
        z_centers = self.bounds_min[2] + (np.arange(self.shape[2]) + 0.5) * self.resolution
        # Ignore ceiling net above 2.4 m when measuring internal course wall height
        wall_z_mask = z_centers <= 2.40

        wall_hits_xy: List[np.ndarray] = []
        max_wall_z = 0.0

        for t in ts:
            xy = p0[:2] + t * (p1[:2] - p0[:2])
            ix, iy, _ = self.world_to_voxel((float(xy[0]), float(xy[1]), 0.5))
            if not (0 <= ix < self.shape[0] and 0 <= iy < self.shape[1]):
                continue
            col = self.raw_grid[ix, iy, :] & wall_z_mask
            if np.any(col):
                top_z = float(np.max(z_centers[col]))
                if top_z > 0.60:  # Taller than boulders (0.53m) -> course wall / barrier
                    wall_hits_xy.append(xy)
                    if top_z > max_wall_z:
                        max_wall_z = top_z

        if wall_hits_xy:
            wall_center_xy = np.mean(np.vstack(wall_hits_xy), axis=0)
            hop_z = float(np.clip(max_wall_z + clearance_above_wall_m, min_hop_z_m, max_hop_z_m))
            wall_detected = True
        else:
            # If the wall wasn't directly observed in the grid yet, jump across the midpoint
            wall_center_xy = 0.5 * (p0[:2] + p1[:2])
            max_wall_z = 1.7272
            hop_z = float(np.clip(max_wall_z + clearance_above_wall_m, min_hop_z_m, max_hop_z_m))
            wall_detected = False

        apex = (float(wall_center_xy[0]), float(wall_center_xy[1]), hop_z)
        return BarrierJumpSpec(
            wall_detected=wall_detected,
            wall_xy_m=(float(wall_center_xy[0]), float(wall_center_xy[1])),
            wall_top_z_m=max_wall_z,
            safe_hop_z_m=hop_z,
            apex_waypoint_m=apex,
        )
