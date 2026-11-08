"""Interactive 3D WebGL + PX4-Autopilot Mission & Live OAK-D Depth Simulation Viewer for DroneHacks 2026.

Serves an interactive 3D Three.js visualizer on http://localhost:8090 that:
- Renders the actual `Navless/dronehacks-course-asbuilt.obj` 3D arena (phone-LiDAR measured walls
  M1, M2, M3, A1, C1 confirmed in the net room, plus dark tunnel, window cutout, boulders, launch mark,
  and concentric landing pad).
- Runs our real `GlobalCourseRouter` + `PX4TrajectoryTrackerCore.step_px4_mission` + `QuadrotorPhysicsAndVIOSim`
  (with PX4 `MPC_XY_P = 0.95` position loop gain, Offboard setpoint priming, `VEHICLE_CMD_DO_SET_MODE`,
  `VEHICLE_CMD_COMPONENT_ARM_DISARM`, `ALIGN` centering, and `VEHICLE_CMD_NAV_LAND` auto-disarm)
  + `RaycastOakDDepthCamera`, streaming:
  - True 3D physical drone pose & velocity (with PX4 `MPC_XY_P = 0.95`, 2nd-order PD, drag, lag, and wind)
  - Drifted/noisy VIO estimated pose (shown as a translucent ghost frame)
  - Live minimum wall clearance (`min_clearance_cm`) against the as-built course mesh
  - Live OAK-D `16UC1` depth camera ray hits & newly discovered obstacle voxels
  - Active EGO B-spline (default) or 12-step 3D NMPC trajectory & mid-flight replans
"""

from __future__ import annotations

import json
import math
from http.server import BaseHTTPRequestHandler, HTTPServer
from pathlib import Path
from typing import Dict, List, Optional
from urllib.parse import parse_qs, urlparse

import numpy as np

from planner.config import PlannerConfig
from planner.frames import enu_to_ned_pos, ned_to_enu_pos, wrap_pi
from planner.fsm_router import CourseStage, GlobalCourseRouter
from planner.grid_map import OccupancyGrid3D
from planner.px4_bridge_node import (
    ARMING_STATE_ARMED,
    NAVIGATION_STATE_AUTO_LAND,
    NAVIGATION_STATE_OFFBOARD,
    VEHICLE_CMD_COMPONENT_ARM_DISARM,
    VEHICLE_CMD_DO_SET_MODE,
    VEHICLE_CMD_NAV_LAND,
    PX4TrajectoryTrackerCore,
)
from tests.test_ros2_gazebo_sim import QuadrotorPhysicsAndVIOSim, RaycastOakDDepthCamera


REPO_ROOT = Path(__file__).resolve().parent.parent
OBJ_PATH = (
    REPO_ROOT / "dronehacks-course-asbuilt.obj"
    if (REPO_ROOT / "dronehacks-course-asbuilt.obj").exists()
    else (
        REPO_ROOT / "dronehacks-course-v3.obj"
        if (REPO_ROOT / "dronehacks-course-v3.obj").exists()
        else REPO_ROOT.parent / "dronehacks-course-v3.obj"
    )
)

DISARMED, POSCTL = 1, 2
SPAWN_ENU = (-2.515, 1.114, 0.067)  # FlyX lab spawn (body origin standing on the floor)
CONTACT_M = 0.20  # body centre closer than this to raw occupied voxels counts as contact (7" props)

_BASE_GRID: OccupancyGrid3D | None = None
_OLD_BASE_GRID: OccupancyGrid3D | None = None
_BASE_OCC_M: np.ndarray | None = None


def _get_base_grid(cfg: PlannerConfig) -> OccupancyGrid3D:
    global _BASE_GRID, _BASE_OCC_M
    if _BASE_GRID is None:
        g = OccupancyGrid3D(
            bounds_min_m=cfg.global_map_bounds_min_m,
            bounds_max_m=cfg.global_map_bounds_max_m,
            resolution_m=cfg.voxel_resolution_m,
            inflation_radius_m=cfg.obstacle_inflation_m,
        )
        g.load_prior_from_obj(OBJ_PATH)
        g.build_soft_clearance_cost()
        g.build_soft_clearance_cost(0.55)
        g.get_discovered_discrepancy_points()
        occ = np.argwhere(g.raw_grid)
        _BASE_OCC_M = (occ + 0.5) * g.resolution + g.bounds_min
        _BASE_GRID = g
    return _BASE_GRID


def _get_old_base_grid(cfg: PlannerConfig) -> OccupancyGrid3D:
    global _OLD_BASE_GRID
    if _OLD_BASE_GRID is None:
        g = OccupancyGrid3D(
            bounds_min_m=cfg.global_map_bounds_min_m,
            bounds_max_m=cfg.global_map_bounds_max_m,
            resolution_m=cfg.voxel_resolution_m,
            inflation_radius_m=cfg.obstacle_inflation_m,
        )
        g.load_old_asbuilt_prior_from_obj(OBJ_PATH)
        g.build_soft_clearance_cost()
        g.build_soft_clearance_cost(0.55)
        g.get_discovered_discrepancy_points()
        _OLD_BASE_GRID = g
    return _OLD_BASE_GRID


def _clone_grid(src: OccupancyGrid3D, cfg: PlannerConfig) -> OccupancyGrid3D:
    g = OccupancyGrid3D(
        bounds_min_m=cfg.global_map_bounds_min_m,
        bounds_max_m=cfg.global_map_bounds_max_m,
        resolution_m=cfg.voxel_resolution_m,
        inflation_radius_m=cfg.obstacle_inflation_m,
    )
    g.raw_grid[:] = src.raw_grid
    g.prior_grid[:] = src.prior_grid
    g.prior_near_mask[:] = src.prior_near_mask
    g.inflated_grid[:] = src.inflated_grid
    g.detected_concentric_pad = src.detected_concentric_pad
    g.detected_launch_mark_xy = src.detected_launch_mark_xy
    g._soft_cost_cache = {k: v.copy() for k, v in src._soft_cost_cache.items()}
    g._features_cache = dict(src._features_cache)
    g._discrepancy_cache = dict(src._discrepancy_cache)
    return g


def run_simulation_episode(
    *,
    fast_mode: bool = True,
    use_old_mapping: bool = False,
    spawn_unmapped_boulder: bool = False,
    block_window: bool = False,
    kp_pos: float = 0.95,
) -> Dict:
    """Execute a closed-loop PX4-Autopilot (`step_px4_mission`) + noisy VIO + OAK-D depth episode."""
    cfg = PlannerConfig(fast_mode=fast_mode)
    base_grid = _get_base_grid(cfg)
    prior_src = _get_old_base_grid(cfg) if use_old_mapping else base_grid
    planner_grid = _clone_grid(prior_src, cfg)
    initial_raw = planner_grid.raw_grid.copy()
    truth_grid = _clone_grid(base_grid, cfg)

    unmapped_boxes: List[Dict] = []
    old_ghost_walls: List[Dict] = []
    if use_old_mapping:
        old_ghost_walls = [
            {"start": [-2.066, 0.508], "end": [-1.786, 2.456], "label": "Old M1 (Removed in New Map)"},
            {"start": [-0.590, 1.539], "end": [-0.458, 3.048], "label": "Old M2 (Removed in New Map)"},
            {"start": [1.019, 0.508], "end": [1.078, 2.431], "label": "Old M3 (Removed in New Map)"},
        ]
    if spawn_unmapped_boulder:
        # Place an unmapped discrepancy boulder between the tunnel exit and window approach on the as-built course
        # (south-west of C1's south end at (1.52, -1.14)) so OAK-D depth discovers it and replans around it!
        bmin, bmax = (0.85, -1.95, 0.0), (1.10, -1.70, 1.45)
        truth_grid.add_aabb_obstacle(bmin, bmax)
        unmapped_boxes.append({"min": list(bmin), "max": list(bmax), "label": "Unmapped Boulder (Map Discrepancy)"})

    if block_window:
        features = planner_grid.discover_course_features(standoff_distance_m=cfg.window_inspect_distance_m)
        if features.window_center_m is not None:
            wx, wy, wz = features.window_center_m
            wmin, wmax = (wx - 0.18, wy - 0.35, 0.15), (wx + 0.18, wy + 0.35, 1.35)
            truth_grid.add_aabb_obstacle(wmin, wmax)
            unmapped_boxes.append({"min": list(wmin), "max": list(wmax), "label": "Blocked Window Panel (Map Discrepancy)"})

    occ = np.argwhere(truth_grid.raw_grid)
    occ_m = (occ + 0.5) * truth_grid.resolution + truth_grid.bounds_min

    router = GlobalCourseRouter(cfg, planner_grid)
    tracker = PX4TrajectoryTrackerCore(router, launch_origin_enu=SPAWN_ENU, prearm_settle_s=2.0)
    phys = QuadrotorPhysicsAndVIOSim(SPAWN_ENU, SPAWN_ENU, seed=7)
    phys.kp_pos = float(kp_pos)  # PX4 MPC_XY_P = 0.95 default
    depth_cam = RaycastOakDDepthCamera(seed=101)
    has_discrepancy = bool(use_old_mapping or spawn_unmapped_boulder or block_window)

    dt = 1.0 / 30.0
    max_steps = int(140.0 / dt)
    frames: List[Dict] = []
    init_spline, _ = router.optimizer.plan_spline(
        (SPAWN_ENU[0], SPAWN_ENU[1], 0.90),
        router.current_goal_enu(),
        router.profile_for_stage(),
        goal_radius_m=router.landing_target_enu()[1],
    )
    active_spline_pts: List[List[float]] = (
        np.round(init_spline.sample_points(110), 3).tolist() if init_spline is not None else []
    )
    depth_hits_world: List[List[float]] = []

    armed = False
    nav = POSCTL
    landed_since: Optional[float] = None
    last_sp_t = -1e9
    stream_since: Optional[float] = None
    sp = None
    min_clear = 9.9
    cur_clear = 9.9

    for step_i in range(max_steps):
        t = round(step_i * dt, 3)
        p_ned, v_ned, hdg = phys.observe_noisy_vio_ned(dt)
        est_enu = ned_to_enu_pos(p_ned, origin_enu=tracker.launch_origin_enu)
        est_yaw_enu = wrap_pi(0.5 * math.pi - hdg)
        z = float(phys.true_pos_enu[2])
        on_ground = z <= SPAWN_ENU[2] + 0.03 and float(np.linalg.norm(phys.true_vel_enu)) < 0.15

        tracker.update_local_position(t, *p_ned, *v_ned, hdg, True, True)
        tracker.update_vehicle_status(t, ARMING_STATE_ARMED if armed else DISARMED, nav)
        tracker.update_landed(t, on_ground)
        tracker.update_estimator_health(True)

        # Live OAK-D 16UC1 depth raycasting when testing old->new mapping, unmapped obstacle, or blocked window
        new_vox_discovered = False
        if has_discrepancy and armed and z > 0.45 and step_i % 6 == 0:
            raw_bytes = depth_cam.render_16uc1_mm(truth_grid, phys.true_pos_enu, phys.true_yaw_enu)
            depth_m = (
                np.frombuffer(raw_bytes, dtype=np.uint16)
                .reshape(depth_cam.height, depth_cam.width)
                .astype(np.float32)
                * 0.001
            )
            added = planner_grid.update_from_depth_image(
                depth_m,
                depth_cam.fx,
                depth_cam.fy,
                depth_cam.cx,
                depth_cam.cy,
                est_enu,
                est_yaw_enu,
                carve_free_space=use_old_mapping,
            )
            if added > 0:
                diff_vox = np.argwhere(planner_grid.raw_grid & ~initial_raw)
                if diff_vox.size > 0:
                    centers = planner_grid.bounds_min + (diff_vox.astype(np.float64) + 0.5) * planner_grid.resolution
                    depth_hits_world = np.round(centers[:: max(1, len(centers) // 160)], 3).tolist()
                    new_vox_discovered = True

        cmd = tracker.step_px4_mission(t)
        if cmd.setpoint is not None:
            sp = cmd.setpoint
            if t - last_sp_t > 0.5:
                stream_since = t
            last_sp_t = t

        for c, p1, _p2 in cmd.commands:  # PX4 Offboard / Arm / NAV_LAND command handling
            if c == VEHICLE_CMD_DO_SET_MODE and stream_since is not None and t - stream_since >= 0.5:
                nav = NAVIGATION_STATE_OFFBOARD
            elif c == VEHICLE_CMD_COMPONENT_ARM_DISARM:
                armed = p1 > 0.5
            elif c == VEHICLE_CMD_NAV_LAND and armed:
                nav = NAVIGATION_STATE_AUTO_LAND

        if nav == NAVIGATION_STATE_OFFBOARD and t - last_sp_t > 0.5:  # Offboard loss failsafe
            nav = NAVIGATION_STATE_AUTO_LAND

        if not armed:
            phys.true_vel_enu[:] = 0.0
            sp_enu = phys.true_pos_enu.copy()
        elif nav == NAVIGATION_STATE_AUTO_LAND:
            phys.step_physics(dt, t, (p_ned[0], p_ned[1], p_ned[2] + 0.5), (0.0, 0.0, 0.5), hdg, 0.6)
            sp_enu = np.array([est_enu[0], est_enu[1], max(SPAWN_ENU[2], est_enu[2] - 0.5)], dtype=np.float64)
            if on_ground:
                landed_since = t if landed_since is None else landed_since
                if t - landed_since > 1.0:
                    armed, nav, landed_since = False, POSCTL, None
            else:
                landed_since = None
        elif nav == NAVIGATION_STATE_OFFBOARD and sp is not None:
            vel = tuple(0.0 if not math.isfinite(v) else v for v in sp.velocity_ned)
            phys.step_physics(dt, t, tracker.to_px4_frame(sp.position_ned), vel, sp.yaw_ned, 1.2)
            sp_enu = ned_to_enu_pos(sp.position_ned, origin_enu=tracker.launch_origin_enu)
        else:
            phys.step_physics(dt, t, tuple(p_ned), (0.0, 0.0, 0.0), hdg, 1.2)
            sp_enu = est_enu.copy()

        if phys.true_pos_enu[2] <= SPAWN_ENU[2] and phys.true_vel_enu[2] < 0.0:
            phys.true_pos_enu[2] = SPAWN_ENU[2]
            phys.true_vel_enu[:] = 0.0

        if armed and z > 0.25:
            cur_clear = float(np.min(np.linalg.norm(occ_m - phys.true_pos_enu, axis=1)))
            if cur_clear < min_clear:
                min_clear = cur_clear

        if router.active_spline is not None and (step_i % 6 == 0 or not active_spline_pts):
            active_spline_pts = np.round(router.active_spline.sample_points(110), 3).tolist()

        if step_i % 2 == 0 or cmd.state in ("DONE", "ABORT"):
            speed_mps = float(np.linalg.norm(phys.true_vel_enu))
            vio_err_cm = float(np.linalg.norm(est_enu - phys.true_pos_enu) * 100.0)
            nav_label = (
                "OFFBOARD"
                if nav == NAVIGATION_STATE_OFFBOARD
                else ("AUTO_LAND" if nav == NAVIGATION_STATE_AUTO_LAND else "POSCTL")
            )
            px4_state = f"{cmd.state} ({'ARMED ' + nav_label if armed else 'DISARMED'})"

            frames.append(
                {
                    "t": t,
                    "stage": router.stage.value,
                    "px4_state": px4_state,
                    "armed": armed,
                    "branch": router.selected_branch,
                    "reason": cmd.reason,
                    "true_enu": [round(float(x), 4) for x in phys.true_pos_enu],
                    "true_yaw": round(float(phys.true_yaw_enu), 4),
                    "vio_enu": [round(float(x), 4) for x in est_enu],
                    "sp_enu": [round(float(x), 4) for x in sp_enu],
                    "speed_mps": round(speed_mps, 3),
                    "vio_err_cm": round(vio_err_cm, 2),
                    "clearance_cm": round(float(cur_clear if cur_clear < 9.0 else min_clear) * 100.0, 1),
                    "min_clearance_cm": round(float(min_clear if min_clear < 9.0 else 0.0) * 100.0, 1),
                    "spline": active_spline_pts if (step_i % 6 == 0 or cmd.reason != "tracking_active_spline") else None,
                    "discovered_voxels": depth_hits_world if new_vox_discovered else None,
                }
            )

        if cmd.state in ("DONE", "ABORT"):
            break

    return {
        "fast_mode": fast_mode,
        "use_old_mapping": use_old_mapping,
        "obj_file": OBJ_PATH.name,
        "spawn_unmapped_boulder": spawn_unmapped_boulder,
        "block_window": block_window,
        "total_time_s": frames[-1]["t"] if frames else 0.0,
        "min_clearance_cm": round(float(min_clear if min_clear < 9.0 else 0.0) * 100.0, 1),
        "pass_mission": bool(tracker.px4_state == "DONE" and min_clear >= CONTACT_M),
        "selected_branch": router.selected_branch,
        "unmapped_boxes": unmapped_boxes,
        "old_ghost_walls": old_ghost_walls,
        "frames": frames,
    }


HTML_PAGE = """<!DOCTYPE html>
<html lang="en">
<head>
  <meta charset="UTF-8" />
  <title>DroneHacks 2026 — EGO-Planner + PX4-Autopilot Sim (As-Built Course)</title>
  <style>
    * { box-sizing: border-box; margin: 0; padding: 0; }
    body { background: #0b0f19; color: #e2e8f0; font-family: system-ui, -apple-system, sans-serif; overflow: hidden; }
    #canvas-container { width: 100vw; height: 100vh; }
    #hud {
      position: absolute; top: 16px; left: 16px; width: 396px;
      background: rgba(15, 23, 42, 0.90); border: 1px solid #334155;
      border-radius: 12px; padding: 16px; backdrop-filter: blur(8px);
      box-shadow: 0 10px 25px rgba(0,0,0,0.5);
    }
    h1 { font-size: 15px; font-weight: 700; color: #38bdf8; margin-bottom: 10px; }
    .row { display: flex; justify-content: space-between; font-size: 13px; margin: 5px 0; }
    .label { color: #94a3b8; }
    .val { font-family: monospace; font-weight: 600; color: #f8fafc; }
    .badge { padding: 2px 8px; border-radius: 999px; font-size: 11px; font-weight: 700; }
    .badge-stage { background: #1e3a8a; color: #93c5fd; }
    .badge-win { background: #065f46; color: #6ee7b7; }
    .badge-gateb { background: #78350f; color: #fcd34d; }
    .badge-disarmed { background: #7f1d1d; color: #fca5a5; }
    .controls { margin-top: 12px; border-top: 1px solid #334155; padding-top: 12px; display: flex; flex-direction: column; gap: 8px; }
    .btn-group { display: flex; gap: 6px; flex-wrap: wrap; }
    button {
      background: #1e293b; color: #e2e8f0; border: 1px solid #475569;
      padding: 6px 10px; border-radius: 6px; cursor: pointer; font-size: 12px; font-weight: 600;
      transition: all 0.15s;
    }
    button:hover { background: #334155; border-color: #38bdf8; }
    button.active { background: #0284c7; border-color: #38bdf8; color: white; }
    input[type=range] { width: 100%; accent-color: #38bdf8; }
    .legend { margin-top: 10px; font-size: 11px; color: #94a3b8; line-height: 1.6; }
    .dot { display: inline-block; width: 9px; height: 9px; border-radius: 50%; margin-right: 5px; }
  </style>
  <script type="importmap">
    {
      "imports": {
        "three": "https://unpkg.com/three@0.160.0/build/three.module.js",
        "three/addons/": "https://unpkg.com/three@0.160.0/examples/jsm/"
      }
    }
  </script>
</head>
<body>
  <div id="canvas-container"></div>
  <div id="hud">
    <h1>DroneHacks 2026 — EGO-Planner + PX4-Autopilot Sim</h1>
    <div class="row"><span class="label">Course Mesh:</span><span class="val" id="val-obj">dronehacks-course-asbuilt.obj</span></div>
    <div class="row"><span class="label">Sim Time:</span><span class="val" id="val-t">0.00 s</span></div>
    <div class="row"><span class="label">PX4 Mission State:</span><span class="val badge badge-win" id="val-px4">PREFLIGHT</span></div>
    <div class="row"><span class="label">FSM Stage:</span><span class="val badge badge-stage" id="val-stage">LOADING...</span></div>
    <div class="row"><span class="label">Chicane Branch:</span><span class="val badge badge-win" id="val-branch">PENDING</span></div>
    <div class="row"><span class="label">Min Wall Clearance:</span><span class="val badge badge-win" id="val-clear">-</span></div>
    <div class="row"><span class="label">Planner Event:</span><span class="val" id="val-reason">-</span></div>
    <div class="row"><span class="label">True Speed:</span><span class="val" id="val-speed">0.00 m/s</span></div>
    <div class="row"><span class="label">True ENU (m):</span><span class="val" id="val-pos">(0, 0, 0)</span></div>
    <div class="row"><span class="label">VIO Drift Error:</span><span class="val" id="val-vio">0.00 cm</span></div>

    <div class="controls">
      <div class="btn-group">
        <button id="btn-play" class="active">Pause</button>
        <button id="btn-restart">Restart</button>
        <button id="btn-cam">Follow Cam: OFF</button>
      </div>
      <input type="range" id="timeline" min="0" max="100" value="0" />
      <div style="font-size:12px; color:#94a3b8; margin-top:4px;">Speed Profile (PX4 MPC_XY_P = 0.95):</div>
      <div class="btn-group">
        <button id="speed-fast" class="active">Fast Mode (1.35 m/s)</button>
        <button id="speed-norm">Standard FlyX (0.60 m/s)</button>
      </div>
      <div style="font-size:12px; color:#94a3b8; margin-top:4px;">PX4-Autopilot Sim Scenario (2026-11-07 Course):</div>
      <div class="btn-group">
        <button id="preset-asbuilt" class="active">New Mapping -> Traverse New Map</button>
        <button id="preset-oldmap">Old Mapping -> Traverse New Map</button>
        <button id="preset-gateb">Blocked Window -> Gate B</button>
        <button id="preset-boulder">New Map + Unmapped Obstacle</button>
      </div>
    </div>

    <div class="legend">
      <div><span class="dot" style="background:#22d3ee;"></span>True Physical Drone (PX4 MPC_XY_P=0.95 + Wind)</div>
      <div><span class="dot" style="background:#f43f5e;"></span>Noisy/Drifted VIO Estimate (No GPS)</div>
      <div><span class="dot" style="background:#4ade80;"></span>Active EGO B-Spline Trajectory</div>
      <div><span class="dot" style="background:#fb923c;"></span>Live OAK-D Discovered Obstacle Voxels</div>
      <div><span class="dot" style="background:#a855f7;"></span>Old Mapping Ghost Walls (M1, M2, M3 — Carved Live)</div>
    </div>
  </div>

  <script type="module">
    import * as THREE from 'three';
    import { OrbitControls } from 'three/addons/controls/OrbitControls.js';
    import { OBJLoader } from 'three/addons/loaders/OBJLoader.js';

    const container = document.getElementById('canvas-container');
    const scene = new THREE.Scene();
    scene.background = new THREE.Color(0x0b0f19);

    // Course ENU has +X East, +Y North, +Z Up.
    THREE.Object3D.DEFAULT_UP.set(0, 0, 1);

    const camera = new THREE.PerspectiveCamera(50, window.innerWidth / window.innerHeight, 0.1, 50);
    camera.position.set(-5.2, -5.2, 5.5);

    const renderer = new THREE.WebGLRenderer({ antialias: true });
    renderer.setSize(window.innerWidth, window.innerHeight);
    renderer.setPixelRatio(window.devicePixelRatio);
    container.appendChild(renderer.domElement);

    const controls = new OrbitControls(camera, renderer.domElement);
    controls.target.set(0, 0, 0.8);
    controls.update();

    scene.add(new THREE.AmbientLight(0xffffff, 0.7));
    const dirLight = new THREE.DirectionalLight(0xffffff, 0.9);
    dirLight.position.set(5, -5, 10);
    scene.add(dirLight);

    const gridHelper = new THREE.GridHelper(6.096, 20, 0x334155, 0x1e293b);
    gridHelper.rotation.x = Math.PI / 2;
    scene.add(gridHelper);

    // Visual Start Pad (-2.515, 0.914) and Primary Landing Pad (-2.042, -0.792) on the floor
    const startPadMesh = new THREE.Mesh(
      new THREE.CylinderGeometry(0.32, 0.32, 0.02, 32),
      new THREE.MeshBasicMaterial({ color: 0x22c55e, transparent: true, opacity: 0.65 })
    );
    startPadMesh.rotation.x = Math.PI / 2;
    startPadMesh.position.set(-2.515, 0.914, 0.01);
    scene.add(startPadMesh);

    // Concentric Circle Landing Target: r9 (0.229m), r20 (0.508m), r30 (0.762m) + bullseye
    for (const [rIn, rOut, col, op] of [[0.0, 0.08, 0x38bdf8, 0.9], [0.20, 0.23, 0x38bdf8, 0.75], [0.48, 0.51, 0x0284c7, 0.65], [0.73, 0.76, 0x0369a1, 0.55]]) {
      const ringMesh = new THREE.Mesh(
        new THREE.RingGeometry(rIn, rOut, 48),
        new THREE.MeshBasicMaterial({ color: col, transparent: true, opacity: op, side: THREE.DoubleSide })
      );
      ringMesh.position.set(-2.042, -0.792, 0.015);
      scene.add(ringMesh);
    }

    // 20x20 ft (±3.048 m) Outer Perimeter Net Cage wireframe
    const netGeom = new THREE.EdgesGeometry(new THREE.BoxGeometry(6.096, 6.096, 2.4));
    const netWire = new THREE.LineSegments(netGeom, new THREE.LineBasicMaterial({ color: 0xef4444, transparent: true, opacity: 0.35 }));
    netWire.position.set(0, 0, 1.2);
    scene.add(netWire);

    // Load actual Navless/dronehacks-course-asbuilt.obj mesh
    const loader = new OBJLoader();
    loader.load('/course.obj', (obj) => {
      obj.traverse((child) => {
        if (child.isMesh) {
          const nm = ((child.name || '') + ' ' + (child.parent ? child.parent.name || '' : '')).toLowerCase();
          if (nm.includes('net ')) {
            child.visible = false;
            return;
          }
          child.material = new THREE.MeshStandardMaterial({
            color: 0x64748b,
            transparent: true,
            opacity: 0.52,
            roughness: 0.6,
            side: THREE.DoubleSide
          });
          const edges = new THREE.EdgesGeometry(child.geometry, 25);
          const line = new THREE.LineSegments(edges, new THREE.LineBasicMaterial({ color: 0x94a3b8, transparent: true, opacity: 0.40 }));
          child.add(line);
        }
      });
      scene.add(obj);
    });

    // True Physical Drone mesh (Cyan quadrotor + heading arrow)
    const droneGroup = new THREE.Group();
    const bodyMesh = new THREE.Mesh(
      new THREE.CylinderGeometry(0.18, 0.18, 0.08, 24),
      new THREE.MeshStandardMaterial({ color: 0x22d3ee, emissive: 0x0891b2, emissiveIntensity: 0.4 })
    );
    bodyMesh.rotation.x = Math.PI / 2;
    droneGroup.add(bodyMesh);
    const noseArrow = new THREE.ArrowHelper(new THREE.Vector3(1, 0, 0), new THREE.Vector3(0, 0, 0), 0.45, 0x22d3ee, 0.12, 0.08);
    droneGroup.add(noseArrow);
    scene.add(droneGroup);

    // Drifted VIO Estimate ghost mesh (Red wireframe sphere)
    const vioMesh = new THREE.Mesh(
      new THREE.SphereGeometry(0.14, 16, 16),
      new THREE.MeshBasicMaterial({ color: 0xf43f5e, wireframe: true })
    );
    scene.add(vioMesh);

    // Trail line for true trajectory
    const trailMat = new THREE.LineBasicMaterial({ color: 0x22d3ee, linewidth: 2 });
    const trailGeom = new THREE.BufferGeometry();
    const trailLine = new THREE.Line(trailGeom, trailMat);
    scene.add(trailLine);

    // Active B-Spline line (Green)
    const splineMat = new THREE.LineBasicMaterial({ color: 0x4ade80, linewidth: 3 });
    const splineGeom = new THREE.BufferGeometry();
    const splineLine = new THREE.Line(splineGeom, splineMat);
    scene.add(splineLine);

    // Unmapped obstacle group, old ghost walls group & discovered OAK-D voxels group
    const unmappedGroup = new THREE.Group();
    scene.add(unmappedGroup);
    const oldGhostGroup = new THREE.Group();
    scene.add(oldGhostGroup);
    const voxGroup = new THREE.Group();
    scene.add(voxGroup);

    let episode = null;
    let frameIdx = 0;
    let playing = true;
    let followCam = false;
    let currentFastMode = true;
    let currentOldMap = false;
    let currentBoulder = false;
    let currentBlockWindow = false;
    let lastTick = performance.now();

    const timeline = document.getElementById('timeline');
    const btnPlay = document.getElementById('btn-play');
    const btnRestart = document.getElementById('btn-restart');
    const btnCam = document.getElementById('btn-cam');
    const speedFast = document.getElementById('speed-fast');
    const speedNorm = document.getElementById('speed-norm');
    const presetAsbuilt = document.getElementById('preset-asbuilt');
    const presetOldMap = document.getElementById('preset-oldmap');
    const presetGateB = document.getElementById('preset-gateb');
    const presetBoulder = document.getElementById('preset-boulder');

    async function loadScenario(fastMode = currentFastMode, oldMap = currentOldMap, boulder = currentBoulder, blockWindow = currentBlockWindow) {
      currentFastMode = fastMode;
      currentOldMap = oldMap;
      currentBoulder = boulder;
      currentBlockWindow = blockWindow;
      document.getElementById('val-stage').textContent = 'SIMULATING...';
      const res = await fetch(`/api/sim?fast_mode=${fastMode ? 1 : 0}&old_map=${oldMap ? 1 : 0}&boulder=${boulder ? 1 : 0}&block_window=${blockWindow ? 1 : 0}`);
      episode = await res.json();
      frameIdx = 0;
      timeline.max = Math.max(0, episode.frames.length - 1);
      timeline.value = 0;
      if (episode.obj_file) {
        document.getElementById('val-obj').textContent = oldMap ? `${episode.obj_file} (Old Prior -> New Map)` : episode.obj_file;
      }

      // Render unmapped ground-truth boxes
      unmappedGroup.clear();
      for (const b of (episode.unmapped_boxes || [])) {
        const sx = b.max[0] - b.min[0], sy = b.max[1] - b.min[1], sz = b.max[2] - b.min[2];
        const m = new THREE.Mesh(
          new THREE.BoxGeometry(sx, sy, sz),
          new THREE.MeshStandardMaterial({ color: 0xef4444, transparent: true, opacity: 0.45 })
        );
        m.position.set(0.5*(b.min[0]+b.max[0]), 0.5*(b.min[1]+b.max[1]), 0.5*(b.min[2]+b.max[2]));
        unmappedGroup.add(m);
      }

      // Render Old Mapping ghost walls (M1, M2, M3) when Old Mapping -> Traverse New Map is selected
      oldGhostGroup.clear();
      for (const w of (episode.old_ghost_walls || [])) {
        const dx = w.end[0] - w.start[0], dy = w.end[1] - w.start[1];
        const len = Math.hypot(dx, dy);
        const geom = new THREE.BoxGeometry(len, 0.1016, 1.7272);
        const mat = new THREE.MeshBasicMaterial({ color: 0xa855f7, transparent: true, opacity: 0.22, wireframe: false });
        const mesh = new THREE.Mesh(geom, mat);
        const wire = new THREE.LineSegments(new THREE.EdgesGeometry(geom), new THREE.LineBasicMaterial({ color: 0xc084fc, transparent: true, opacity: 0.75 }));
        mesh.add(wire);
        mesh.position.set(0.5 * (w.start[0] + w.end[0]), 0.5 * (w.start[1] + w.end[1]), 0.8636);
        mesh.rotation.z = Math.atan2(dy, dx);
        oldGhostGroup.add(mesh);
      }
      voxGroup.clear();
    }

    function applyFrame(idx) {
      if (!episode || !episode.frames.length) return;
      const f = episode.frames[idx];
      droneGroup.position.set(f.true_enu[0], f.true_enu[1], f.true_enu[2]);
      droneGroup.rotation.set(0, 0, f.true_yaw);
      vioMesh.position.set(f.vio_enu[0], f.vio_enu[1], f.vio_enu[2]);

      const pts = [];
      for (let i = 0; i <= idx; i++) {
        const p = episode.frames[i].true_enu;
        pts.push(new THREE.Vector3(p[0], p[1], p[2]));
      }
      trailGeom.setFromPoints(pts);

      if (f.spline) {
        splineGeom.setFromPoints(f.spline.map(p => new THREE.Vector3(p[0], p[1], p[2])));
      }
      if (f.discovered_voxels) {
        voxGroup.clear();
        const vGeom = new THREE.BoxGeometry(0.09, 0.09, 0.09);
        const vMat = new THREE.MeshBasicMaterial({ color: 0xfb923c });
        for (const v of f.discovered_voxels) {
          const vm = new THREE.Mesh(vGeom, vMat);
          vm.position.set(v[0], v[1], v[2]);
          voxGroup.add(vm);
        }
      }

      if (followCam) {
        controls.target.set(f.true_enu[0], f.true_enu[1], f.true_enu[2]);
        controls.update();
      }

      document.getElementById('val-t').textContent = `${f.t.toFixed(2)} s / ${episode.total_time_s.toFixed(2)} s`;
      const px4El = document.getElementById('val-px4');
      if (px4El) {
        px4El.textContent = f.px4_state || 'ARMED (OFFBOARD)';
        px4El.className = 'val badge ' + (f.armed === false ? 'badge-disarmed' : (f.px4_state && f.px4_state.includes('LAND') ? 'badge-gateb' : 'badge-win'));
      }
      const clEl = document.getElementById('val-clear');
      if (clEl) {
        const mc = episode.min_clearance_cm || f.min_clearance_cm || 0;
        const passTxt = episode.pass_mission ? 'PASS' : 'WARN';
        clEl.textContent = `${mc.toFixed(1)} cm (${passTxt} >= 20 cm)`;
        clEl.className = 'val badge ' + (mc >= 20.0 ? 'badge-win' : 'badge-disarmed');
      }
      bodyMesh.material.color.setHex(f.armed === false ? 0xf59e0b : 0x22d3ee);
      document.getElementById('val-stage').textContent = f.stage;
      const brEl = document.getElementById('val-branch');
      brEl.textContent = f.branch;
      brEl.className = 'val badge ' + (f.branch === 'GATE_B' ? 'badge-gateb' : 'badge-win');
      document.getElementById('val-reason').textContent = f.reason;
      document.getElementById('val-speed').textContent = `${f.speed_mps.toFixed(2)} m/s`;
      document.getElementById('val-pos').textContent = `(${f.true_enu[0].toFixed(2)}, ${f.true_enu[1].toFixed(2)}, ${f.true_enu[2].toFixed(2)})`;
      document.getElementById('val-vio').textContent = `${f.vio_err_cm.toFixed(2)} cm`;
    }

    function setActivePreset(activeBtn) {
      for (const b of [presetAsbuilt, presetOldMap, presetGateB, presetBoulder]) {
        b.classList.toggle('active', b === activeBtn);
      }
    }

    btnPlay.onclick = () => {
      playing = !playing;
      btnPlay.textContent = playing ? 'Pause' : 'Play';
      btnPlay.classList.toggle('active', playing);
    };
    btnRestart.onclick = () => { frameIdx = 0; timeline.value = 0; applyFrame(0); };
    btnCam.onclick = () => {
      followCam = !followCam;
      btnCam.textContent = `Follow Cam: ${followCam ? 'ON' : 'OFF'}`;
      btnCam.classList.toggle('active', followCam);
    };
    timeline.oninput = (e) => {
      frameIdx = parseInt(e.target.value, 10);
      applyFrame(frameIdx);
    };
    speedFast.onclick = () => {
      speedFast.classList.add('active'); speedNorm.classList.remove('active');
      loadScenario(true, currentOldMap, currentBoulder, currentBlockWindow);
    };
    speedNorm.onclick = () => {
      speedNorm.classList.add('active'); speedFast.classList.remove('active');
      loadScenario(false, currentOldMap, currentBoulder, currentBlockWindow);
    };
    presetAsbuilt.onclick = () => {
      setActivePreset(presetAsbuilt);
      loadScenario(currentFastMode, false, false, false);
    };
    presetOldMap.onclick = () => {
      setActivePreset(presetOldMap);
      loadScenario(currentFastMode, true, false, false);
    };
    presetGateB.onclick = () => {
      setActivePreset(presetGateB);
      loadScenario(currentFastMode, false, false, true);
    };
    presetBoulder.onclick = () => {
      setActivePreset(presetBoulder);
      loadScenario(currentFastMode, false, true, false);
    };

    window.addEventListener('resize', () => {
      camera.aspect = window.innerWidth / window.innerHeight;
      camera.updateProjectionMatrix();
      renderer.setSize(window.innerWidth, window.innerHeight);
    });

    loadScenario(true, false, false, false);

    function animate(now) {
      requestAnimationFrame(animate);
      if (playing && episode && episode.frames.length > 0 && (now - lastTick) >= 50) {
        lastTick = now;
        frameIdx = (frameIdx + 1) % episode.frames.length;
        timeline.value = frameIdx;
        applyFrame(frameIdx);
      }
      renderer.render(scene, camera);
    }
    requestAnimationFrame(animate);
  </script>
</body>
</html>
"""


class SimViewerHandler(BaseHTTPRequestHandler):
    _cache: Dict[str, bytes] = {}

    def log_message(self, format: str, *args) -> None:
        pass

    def do_GET(self) -> None:
        parsed = urlparse(self.path)
        if parsed.path == "/":
            body = HTML_PAGE.encode("utf-8")
            self.send_response(200)
            self.send_header("Content-Type", "text/html; charset=utf-8")
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            self.wfile.write(body)
            return

        if parsed.path == "/course.obj":
            if "obj" not in self._cache:
                self._cache["obj"] = OBJ_PATH.read_bytes()
            body = self._cache["obj"]
            self.send_response(200)
            self.send_header("Content-Type", "text/plain; charset=utf-8")
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            self.wfile.write(body)
            return

        if parsed.path == "/api/sim":
            qs = parse_qs(parsed.query)
            fast = qs.get("fast_mode", ["1"])[0] == "1"
            old_map = qs.get("old_map", ["0"])[0] == "1"
            boulder = qs.get("boulder", ["0"])[0] == "1"
            block_win = qs.get("block_window", ["0"])[0] == "1"
            key = f"ego_{fast}_{old_map}_{boulder}_{block_win}"
            if key not in self._cache:
                ep = run_simulation_episode(
                    fast_mode=fast,
                    use_old_mapping=old_map,
                    spawn_unmapped_boulder=boulder,
                    block_window=block_win,
                )
                self._cache[key] = json.dumps(ep).encode("utf-8")
            body = self._cache[key]
            self.send_response(200)
            self.send_header("Content-Type", "application/json")
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            self.wfile.write(body)
            return

        self.send_error(404)


def main(port: int = 8090) -> None:
    import threading

    def _warm() -> None:
        for fm in (True, False):
            for om in (False, True):
                key = f"ego_{fm}_{om}_False_False"
                if key not in SimViewerHandler._cache:
                    ep = run_simulation_episode(
                        fast_mode=fm,
                        use_old_mapping=om,
                        spawn_unmapped_boulder=False,
                        block_window=False,
                    )
                    SimViewerHandler._cache[key] = json.dumps(ep).encode("utf-8")

    threading.Thread(target=_warm, daemon=True).start()
    server = HTTPServer(("0.0.0.0", port), SimViewerHandler)
    print(f"3D Simulation Viewer live at http://localhost:{port}", flush=True)
    server.serve_forever()


if __name__ == "__main__":
    main()
