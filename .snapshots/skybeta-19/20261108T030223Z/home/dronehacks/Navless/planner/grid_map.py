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
class ConcentricPadTarget:
    """Concentric-circle landing target discovered from floor ring geometry or camera observations.

    In the DroneHacks arena, the landing target consists of 3 concentric rings (r9 = 0.229 m,
    r20 = 0.508 m, r30 = 0.762 m) around a central bullseye cross. Closer to the center is better.
    """

    center_xy_m: Tuple[float, float]
    ring_radii_m: Tuple[float, float, float] = (0.229, 0.508, 0.762)
    outer_radius_m: float = 0.762

    def radial_cost_bonus(self, x_m: float, y_m: float) -> float:
        """Return a smooth radial cost in [0, 2.0] inside/near the concentric rings (0 at bullseye)."""
        r = math.hypot(x_m - self.center_xy_m[0], y_m - self.center_xy_m[1])
        if r >= self.outer_radius_m:
            return 2.0
        # Stepped + quadratic gradient across r30 -> r20 -> r9 -> bullseye so closer is strictly better
        ring_step = sum(0.25 for rad in self.ring_radii_m if r > rad)
        return ring_step + 1.25 * ((r / max(self.outer_radius_m, 1e-3)) ** 2)


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
    concentric_pad: Optional[ConcentricPadTarget] = None
    launch_mark_xy_m: Optional[Tuple[float, float]] = None


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
        # Snapshot of static prior occupancy so live sensor discrepancies (raw_grid & ~prior_grid) can be tracked
        self.prior_grid = np.zeros(self.shape, dtype=bool)
        # 2-voxel (0.20 m) tolerance mask around prior walls so VIO/stereo jitter on known walls
        # never thickens corridors, while genuine physical discrepancies (> 0.20 m or in window) are mapped live
        self.prior_near_mask = np.zeros(self.shape, dtype=bool)
        # Per-voxel hit accumulator for multi-ray / multi-frame live depth confirmation
        self.live_hit_counts = np.zeros(self.shape, dtype=np.uint16)
        # Inflated occupancy: True within inflation_radius_m of any occupied voxel
        self.inflated_grid = np.zeros(self.shape, dtype=bool)
        self._inflation_offsets = self._build_spherical_offsets(self.inflation_radius)
        self.detected_concentric_pad: Optional[ConcentricPadTarget] = None
        self.detected_launch_mark_xy: Optional[Tuple[float, float]] = None
        self._soft_cost_cache: Dict[float, np.ndarray] = {}

        # If this grid covers the 20x20 ft course arena, permanently bake in the perimeter net border
        # because the thin mesh net at x,y = ±3.048 m is invisible to the OAK-D stereo depth sensor.
        if (
            self.bounds_min[0] <= -3.0
            and self.bounds_min[1] <= -3.0
            and self.bounds_max[0] >= 3.0
            and self.bounds_max[1] >= 3.0
        ):
            self.add_arena_net_border()

    def add_arena_net_border(
        self,
        arena_half_extent_m: float = 3.048,
        net_inward_pad_m: float = 0.06,
    ) -> None:
        """Permanently mark the outer perimeter net (x, y = ±3.048 m) as solid walls in the grid.

        Since the thin arena net cannot be reliably detected by stereo depth, baking this border
        into the map (plus `inflation_radius_m = 0.22 m` -> hard keep-out at ±2.75 m and soft
        repulsion out to ±2.45 m) prevents local replanning from ever dodging boulders into the net.
        """
        inner = arena_half_extent_m - net_inward_pad_m  # 2.988 m (inflated to ~2.75 m)
        z_lo = float(self.bounds_min[2])
        z_hi = float(self.bounds_max[2])
        bmin_x, bmax_x = float(self.bounds_min[0]), float(self.bounds_max[0])
        bmin_y, bmax_y = float(self.bounds_min[1]), float(self.bounds_max[1])

        # West, East, South, and North net walls
        self.add_aabb_obstacle((bmin_x, bmin_y, z_lo), (-inner, bmax_y, z_hi))
        self.add_aabb_obstacle((inner, bmin_y, z_lo), (bmax_x, bmax_y, z_hi))
        self.add_aabb_obstacle((bmin_x, bmin_y, z_lo), (bmax_x, -inner, z_hi))
        self.add_aabb_obstacle((bmin_x, inner, z_lo), (bmax_x, bmax_y, z_hi))
        self.prior_grid[:] = self.raw_grid

    def _build_spherical_offsets(self, radius_m: float) -> np.ndarray:
        """Build quadrotor-shaped (cylindrical XY = radius_m, vertical Z = min(radius_m, 0.15 m)) inflation offsets."""
        r_xy = int(math.ceil(radius_m / self.resolution))
        if r_xy <= 0:
            return np.zeros((1, 3), dtype=np.int32)
        z_rad = min(radius_m, 0.15)
        r_z = int(math.ceil(z_rad / self.resolution))
        rng_xy = np.arange(-r_xy, r_xy + 1, dtype=np.int32)
        rng_z = np.arange(-r_z, r_z + 1, dtype=np.int32)
        gx, gy, gz = np.meshgrid(rng_xy, rng_xy, rng_z, indexing="ij")
        dist_xy_sq = (gx * self.resolution) ** 2 + (gy * self.resolution) ** 2
        mask = (dist_xy_sq <= (radius_m + 1e-6) ** 2) & (np.abs(gz * self.resolution) <= z_rad + 1e-6)
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

        # Incrementally update cached soft clearance cost fields for small live depth batches
        if self._soft_cost_cache:
            if voxel_indices.shape[0] <= 600:
                for buf_r, cost_arr in self._soft_cost_cache.items():
                    self._apply_soft_clearance_voxels(cost_arr, voxel_indices, buf_r)
            else:
                self._soft_cost_cache.clear()

    def rebuild_inflation(self) -> None:
        """Recompute inflated_grid from scratch using raw_grid."""
        self._soft_cost_cache.clear()
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

    def _update_prior_snapshot(self) -> None:
        """Snapshot `prior_grid` and build a 2-voxel (0.20 m) VIO-jitter tolerance mask around static walls."""
        self.prior_grid[:] = self.raw_grid
        self.prior_near_mask.fill(False)
        occ = np.argwhere(self.prior_grid)
        if occ.size == 0:
            return
        nx, ny, nz = self.shape
        for dx in (-2, -1, 0, 1, 2):
            for dy in (-2, -1, 0, 1, 2):
                for dz in (-1, 0, 1):
                    ix = occ[:, 0] + dx
                    iy = occ[:, 1] + dy
                    iz = occ[:, 2] + dz
                    valid = (ix >= 0) & (ix < nx) & (iy >= 0) & (iy < ny) & (iz >= 0) & (iz < nz)
                    self.prior_near_mask[ix[valid], iy[valid], iz[valid]] = True
        # Never mask out the central Window opening core so a blocked window discrepancy is always detected
        wmin = self.world_to_voxel((-0.62, -2.52, 0.52))
        wmax = self.world_to_voxel((-0.32, -2.02, 1.00))
        self.prior_near_mask[
            max(0, wmin[0]) : min(nx, wmax[0] + 1),
            max(0, wmin[1]) : min(ny, wmax[1] + 1),
            max(0, wmin[2]) : min(nz, wmax[2] + 1),
        ] = False

    def load_prior_from_json(self, path: str | Path) -> None:
        """Pre-seed map from a JSON layout file containing `walls` (boxes), `openings`, or packed voxel snapshot."""
        import base64
        import zlib
        data = json.loads(Path(path).read_text(encoding="utf-8"))
        if "packed_raw_grid_b64" in data and list(data.get("shape", [])) == list(self.shape):
            raw_bytes = zlib.decompress(base64.b64decode(data["packed_raw_grid_b64"]))
            bits = np.unpackbits(np.frombuffer(raw_bytes, dtype=np.uint8))[: self.raw_grid.size]
            self.raw_grid[:] = bits.reshape(self.shape).astype(bool)
        for wall in data.get("walls", []):
            self.add_aabb_obstacle(tuple(wall["min"]), tuple(wall["max"]), update_inflation=False)
        for opening in data.get("openings", []):
            self.carve_aabb_opening(tuple(opening["min"]), tuple(opening["max"]), rebuild=False)
        if "points" in data and len(data["points"]) > 0:
            pts = np.asarray(data["points"], dtype=np.float64)
            self.insert_point_cloud(pts, sensor_origin_m=None)
        if "concentric_pad" in data and data["concentric_pad"]:
            cp_data = data["concentric_pad"]
            self.detected_concentric_pad = ConcentricPadTarget(
                center_xy_m=(float(cp_data["center_xy_m"][0]), float(cp_data["center_xy_m"][1])),
                ring_radii_m=tuple(float(v) for v in cp_data.get("ring_radii_m", (0.229, 0.508, 0.762))),
                outer_radius_m=float(cp_data.get("outer_radius_m", 0.762)),
            )
        if "launch_mark_xy" in data and data["launch_mark_xy"]:
            self.detected_launch_mark_xy = (float(data["launch_mark_xy"][0]), float(data["launch_mark_xy"][1]))
        self.rebuild_inflation()
        self._update_prior_snapshot()

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
        # Detect concentric landing pad rings (r9, r20, r30) and launch mark from floor geometry
        floor_mask = (v_arr[:, 2] >= 0.0015) & (v_arr[:, 2] <= 0.012)
        if np.any(floor_mask):
            self.detect_concentric_pad_from_floor_points(v_arr[floor_mask])

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
        added = self.insert_point_cloud(all_pts, sensor_origin_m=None)
        self._update_prior_snapshot()
        return added

    def get_discovered_discrepancy_points(
        self,
        *,
        z_min_m: float = 0.15,
        z_max_m: float = 1.70,
    ) -> np.ndarray:
        """Return (M, 3) world coordinates of occupied voxels discovered by live sensors that were not in the prior map."""
        diff_mask = self.raw_grid & (~self.prior_grid)
        if not np.any(diff_mask):
            return np.zeros((0, 3), dtype=np.float64)
        vox = np.argwhere(diff_mask)
        pts = self.bounds_min + (vox.astype(np.float64) + 0.5) * self.resolution
        keep = (pts[:, 2] >= z_min_m) & (pts[:, 2] <= z_max_m)
        return pts[keep]

    def insert_point_cloud(
        self,
        points_world_m: np.ndarray,
        sensor_origin_m: Optional[np.ndarray | Tuple[float, float, float]] = None,
        *,
        min_live_hits: int = 1,
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
        idx_valid = idx[valid]
        if idx_valid.size == 0:
            return 0

        if sensor_origin_m is not None and min_live_hits > 1:
            uniq_idx, counts = np.unique(idx_valid, axis=0, return_counts=True)
            cur = self.live_hit_counts[uniq_idx[:, 0], uniq_idx[:, 1], uniq_idx[:, 2]].astype(np.int32) + counts
            self.live_hit_counts[uniq_idx[:, 0], uniq_idx[:, 1], uniq_idx[:, 2]] = np.minimum(65535, cur).astype(np.uint16)
            confirmed = cur >= int(min_live_hits)
            idx = uniq_idx[confirmed]
        else:
            idx = np.unique(idx_valid, axis=0)
        if idx.size == 0:
            return 0

        if sensor_origin_m is not None:
            not_wall_jitter = ~self.prior_near_mask[idx[:, 0], idx[:, 1], idx[:, 2]]
            idx = idx[not_wall_jitter]
            if idx.size == 0:
                return 0

        new_mask = ~self.raw_grid[idx[:, 0], idx[:, 1], idx[:, 2]]
        new_idx = idx[new_mask]
        if new_idx.size == 0:
            return 0

        self.raw_grid[new_idx[:, 0], new_idx[:, 1], new_idx[:, 2]] = True
        self._inflate_voxels(new_idx)
        return int(new_idx.shape[0])

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
        min_live_hits: int = 1,
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
        # Filter out floor hits (z < 0.12 m) and above-wall/net grazing hits (z > 1.70 m or outside ±2.85 m)
        # so VIO altitude/attitude drift during over-wall fly-home never spawns phantom obstacles at z ~ 2 m.
        in_course_z = (pts_world[:, 2] >= 0.12) & (pts_world[:, 2] <= 1.70)
        if (
            self.bounds_min[0] <= -3.0
            and self.bounds_min[1] <= -3.0
            and self.bounds_max[0] >= 3.0
            and self.bounds_max[1] >= 3.0
        ):
            in_course_xy = (np.abs(pts_world[:, 0]) <= 2.85) & (np.abs(pts_world[:, 1]) <= 2.85)
            pts_world = pts_world[in_course_z & in_course_xy]
        else:
            pts_world = pts_world[in_course_z]
        if pts_world.size == 0:
            return 0
        return self.insert_point_cloud(pts_world, sensor_origin_m=pos, min_live_hits=min_live_hits)

    def obstacle_repulsion_gradient(
        self,
        point_m: np.ndarray | Tuple[float, float, float],
        safe_distance_m: float = 0.58,
    ) -> Tuple[float, np.ndarray]:
        """Compute local collision cost and symmetric multi-voxel repulsive gradient away from nearby raw obstacles.

        By averaging repulsive vectors over all occupied voxels within `safe_distance_m`:
        - At one-sided walls and turn corners, pushes the trajectory outward to `safe_distance_m`.
        - Between opposing walls (corridors, tunnel, window), left/right repulsions cancel at the exact centerline.
        """
        p = np.asarray(point_m, dtype=np.float64)
        center_idx = self.world_to_voxel(p)
        r_vox_xy = max(1, int(math.ceil(safe_distance_m / self.resolution)))
        r_vox_z = max(1, int(math.ceil(min(safe_distance_m, 0.20) / self.resolution)))
        nx, ny, nz = self.shape

        ix0, ix1 = max(0, center_idx[0] - r_vox_xy), min(nx - 1, center_idx[0] + r_vox_xy)
        iy0, iy1 = max(0, center_idx[1] - r_vox_xy), min(ny - 1, center_idx[1] + r_vox_xy)
        iz0, iz1 = max(0, center_idx[2] - r_vox_z), min(nz - 1, center_idx[2] + r_vox_z)

        sub = self.raw_grid[ix0 : ix1 + 1, iy0 : iy1 + 1, iz0 : iz1 + 1]
        if not np.any(sub):
            return 0.0, np.zeros(3, dtype=np.float64)

        occ_local = np.argwhere(sub)
        occ_global = occ_local + np.array([ix0, iy0, iz0], dtype=int)
        occ_centers = self.bounds_min + (occ_global.astype(np.float64) + 0.5) * self.resolution

        diffs = p[None, :] - occ_centers
        dists = np.linalg.norm(diffs, axis=1)
        within = dists < safe_distance_m
        if not np.any(within):
            return 0.0, np.zeros(3, dtype=np.float64)

        d_in = np.maximum(dists[within], 1e-6)
        diff_in = diffs[within]
        penetrations = safe_distance_m - d_in
        # Weight closer voxels more strongly (inverse-distance weighting) so corner tips repel strongly
        weights = (penetrations / safe_distance_m) ** 2
        w_sum = float(np.sum(weights))
        dirs = diff_in / d_in[:, None]
        # Net repulsive gradient wrt p (points toward obstacles; subtracting grad pushes p away toward centerline)
        grad = -np.sum((weights * penetrations)[:, None] * dirs, axis=0) / max(w_sum, 1e-6)
        max_pen = float(np.max(penetrations))
        cost = 0.5 * (max_pen ** 2)
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
        # Keep approach and exit points at least 0.47 m inside the ±3.048 m perimeter net (|x|,|y| <= 2.58 m)
        approach_pt = (
            float(np.clip(center_xy[0] + standoff_distance_m * normal[0], -2.58, 2.58)),
            float(np.clip(center_xy[1] + standoff_distance_m * normal[1], -2.58, 2.58)),
            float(center_z),
        )
        exit_pt = (
            float(np.clip(center_xy[0] - standoff_distance_m * normal[0], -2.58, 2.58)),
            float(np.clip(center_xy[1] - standoff_distance_m * normal[1], -2.58, 2.58)),
            float(center_z),
        )
        return win_center, (float(normal[0]), float(normal[1])), approach_pt, exit_pt

    def _apply_soft_clearance_voxels(
        self,
        cost: np.ndarray,
        occ: np.ndarray,
        buffer_radius_m: float,
    ) -> None:
        offsets = self._build_spherical_offsets(buffer_radius_m)
        nx, ny, nz = self.shape
        for dx, dy, dz in offsets:
            dist = math.sqrt((dx * self.resolution) ** 2 + (dy * self.resolution) ** 2 + (dz * self.resolution) ** 2)
            if dist <= self.inflation_radius or dist > buffer_radius_m:
                continue
            penalty = float(4.2 * ((buffer_radius_m - dist) / max(buffer_radius_m - self.inflation_radius, 0.05)) ** 2)
            ix = occ[:, 0] + dx
            iy = occ[:, 1] + dy
            iz = occ[:, 2] + dz
            valid = (ix >= 0) & (ix < nx) & (iy >= 0) & (iy < ny) & (iz >= 0) & (iz < nz)
            np.maximum.at(cost, (ix[valid], iy[valid], iz[valid]), penalty)
        # Keep the narrow Window aperture core low-cost when open so Window routing remains preferred over Gate B
        if self.bounds_min[0] <= -3.0 and self.bounds_max[0] >= 3.0:
            wmin = self.world_to_voxel((-0.62, -2.35, 0.68))
            wmax = self.world_to_voxel((-0.32, -2.18, 0.85))
            cost[
                max(0, wmin[0]) : min(nx, wmax[0] + 1),
                max(0, wmin[1]) : min(ny, wmax[1] + 1),
                max(0, wmin[2]) : min(nz, wmax[2] + 1),
            ] *= 0.25

    def build_soft_clearance_cost(self, buffer_radius_m: float = 0.72) -> np.ndarray:
        """Build a 3D soft cost field (in [0, 3.0]) that penalizes proximity to walls.

        Used by 3D A* so paths naturally stay centered in corridors and turn smoothly
        around maze & chicane walls without needing hardcoded intermediate waypoints.
        """
        key = round(float(buffer_radius_m), 4)
        if key in self._soft_cost_cache:
            return self._soft_cost_cache[key]
        cost = np.zeros(self.shape, dtype=np.float32)
        occ = np.argwhere(self.raw_grid)
        if occ.size == 0:
            self._soft_cost_cache[key] = cost
            return cost
        self._apply_soft_clearance_voxels(cost, occ, buffer_radius_m)
        self._soft_cost_cache[key] = cost
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
            concentric_pad=self.detected_concentric_pad,
            launch_mark_xy_m=self.detected_launch_mark_xy,
        )

    def detect_concentric_pad_from_floor_points(
        self,
        floor_points_m: np.ndarray,
    ) -> Optional[ConcentricPadTarget]:
        """Geometrically detect the concentric-circle landing target (and launch mark) from floor vertices."""
        pts = np.asarray(floor_points_m, dtype=np.float64)
        if pts.shape[0] < 20:
            return None
        xy = pts[:, :2]
        dists = np.linalg.norm(xy[:, None, :] - xy[None, :, :], axis=2)
        neighbor_counts = (dists <= 0.80).sum(axis=1)
        best_i = int(np.argmax(neighbor_counts))
        in_pad_mask = dists[best_i] <= 0.82
        pad_xy = xy[in_pad_mask]
        if pad_xy.shape[0] < 15:
            return None

        center_xy = 0.5 * (np.min(pad_xy, axis=0) + np.max(pad_xy, axis=0))
        radii = np.linalg.norm(pad_xy - center_xy[None, :], axis=1)
        r9, r20, r30 = [float(x) for x in np.percentile(radii, [15, 50, 85])]
        outer_r = float(np.percentile(radii, 98))
        self.detected_concentric_pad = ConcentricPadTarget(
            center_xy_m=(float(center_xy[0]), float(center_xy[1])),
            ring_radii_m=(round(r9, 3), round(r20, 3), round(r30, 3)),
            outer_radius_m=round(max(outer_r, r30), 3),
        )

        # Remaining floor marking cluster (outside 1.2 m of landing pad) is the Launch Mark
        rem_xy = xy[dists[best_i] > 1.20]
        if rem_xy.shape[0] >= 4:
            rem_d = np.linalg.norm(rem_xy[:, None, :] - rem_xy[None, :, :], axis=2)
            rem_best = int(np.argmax((rem_d <= 0.15).sum(axis=1)))
            launch_cluster = rem_xy[rem_d[rem_best] <= 0.15]
            if launch_cluster.shape[0] >= 4:
                l_xy = 0.5 * (np.min(launch_cluster, axis=0) + np.max(launch_cluster, axis=0))
                self.detected_launch_mark_xy = (float(l_xy[0]), float(l_xy[1]))

        return self.detected_concentric_pad

    def update_concentric_target_observation(
        self,
        center_xy_m: Tuple[float, float],
        ring_radii_m: Tuple[float, float, float] = (0.229, 0.508, 0.762),
    ) -> ConcentricPadTarget:
        """Update or refine the concentric-circle landing target from live camera ring detection."""
        self.detected_concentric_pad = ConcentricPadTarget(
            center_xy_m=(float(center_xy_m[0]), float(center_xy_m[1])),
            ring_radii_m=ring_radii_m,
            outer_radius_m=float(max(ring_radii_m)),
        )
        return self.detected_concentric_pad

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
