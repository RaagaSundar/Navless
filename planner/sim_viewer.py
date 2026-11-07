"""Interactive 3D WebGL + Live Physics/VIO/OAK-D Depth Simulation Viewer for DroneHacks 2026.

Serves an interactive 3D Three.js visualizer on http://localhost:8090 that:
- Renders the actual `dronehacks-course-v3.obj` 3D arena (walls, dark tunnel, window cutout,
  boulders, launch mark, and landing pad).
- Runs our real `GlobalCourseRouter` + `PX4TrajectoryTrackerCore` + `QuadrotorPhysicsAndVIOSim`
  + `RaycastOakDDepthCamera` in real time or pre-recorded telemetry steps, streaming:
  - True 3D physical drone pose & velocity (with 2nd-order PD control, drag, lag, and wind)
  - Drifted/noisy VIO estimated pose (shown as a translucent ghost frame)
  - Live OAK-D `16UC1` depth camera ray hits & newly discovered obstacle voxels
  - Active EGO B-spline trajectory & mid-flight collision-avoidance replans
"""

from __future__ import annotations

import json
import math
from http.server import BaseHTTPRequestHandler, HTTPServer
from pathlib import Path
from typing import Dict, List
from urllib.parse import parse_qs, urlparse

import numpy as np

from planner.config import PlannerConfig
from planner.frames import enu_to_ned_pos, ned_to_enu_pos, wrap_pi
from planner.fsm_router import CourseStage, GlobalCourseRouter
from planner.grid_map import OccupancyGrid3D
from planner.px4_bridge_node import (
    ARMING_STATE_ARMED,
    NAVIGATION_STATE_OFFBOARD,
    PX4TrajectoryTrackerCore,
)
from tests.test_ros2_gazebo_sim import QuadrotorPhysicsAndVIOSim, RaycastOakDDepthCamera


REPO_ROOT = Path(__file__).resolve().parent.parent
OBJ_PATH = REPO_ROOT / "dronehacks-course-v3.obj"


_BASE_GRID: OccupancyGrid3D | None = None


def _get_base_grid(cfg: PlannerConfig) -> OccupancyGrid3D:
    global _BASE_GRID
    if _BASE_GRID is None:
        g = OccupancyGrid3D(
            bounds_min_m=cfg.global_map_bounds_min_m,
            bounds_max_m=cfg.global_map_bounds_max_m,
            resolution_m=cfg.voxel_resolution_m,
            inflation_radius_m=cfg.obstacle_inflation_m,
        )
        g.load_prior_from_obj(OBJ_PATH)
        _BASE_GRID = g
    return _BASE_GRID


def _clone_grid(src: OccupancyGrid3D, cfg: PlannerConfig) -> OccupancyGrid3D:
    g = OccupancyGrid3D(
        bounds_min_m=cfg.global_map_bounds_min_m,
        bounds_max_m=cfg.global_map_bounds_max_m,
        resolution_m=cfg.voxel_resolution_m,
        inflation_radius_m=cfg.obstacle_inflation_m,
    )
    g.raw_grid[:] = src.raw_grid
    g.inflated_grid[:] = src.inflated_grid
    return g


def run_simulation_episode(
    *,
    fast_mode: bool = True,
    spawn_unmapped_boulder: bool = True,
    block_window: bool = False,
) -> Dict:
    """Execute a complete closed-loop physics + noisy VIO + raycasted depth episode and return telemetry."""
    cfg = PlannerConfig(fast_mode=fast_mode)
    base_grid = _get_base_grid(cfg)
    planner_grid = _clone_grid(base_grid, cfg)
    initial_raw = planner_grid.raw_grid.copy()
    truth_grid = _clone_grid(base_grid, cfg)

    unmapped_boxes: List[Dict] = []
    if spawn_unmapped_boulder:
        bmin, bmax = (1.30, -1.75, 0.0), (1.70, -1.35, 1.45)
        truth_grid.add_aabb_obstacle(bmin, bmax)
        unmapped_boxes.append({"min": list(bmin), "max": list(bmax), "label": "Unmapped Boulder"})

    if block_window:
        features = planner_grid.discover_course_features(standoff_distance_m=cfg.window_inspect_distance_m)
        if features.window_center_m is not None:
            wx, wy, wz = features.window_center_m
            wmin, wmax = (wx - 0.15, wy - 0.30, wz - 0.30), (wx + 0.15, wy + 0.30, wz + 0.30)
            truth_grid.add_aabb_obstacle(wmin, wmax)
            unmapped_boxes.append({"min": list(wmin), "max": list(wmax), "label": "Blocked Window Panel"})

    router = GlobalCourseRouter(cfg, planner_grid)
    tracker = PX4TrajectoryTrackerCore(router, lookahead_s=0.35)
    phys = QuadrotorPhysicsAndVIOSim(cfg.checkpoints.start_pad, tracker.launch_origin_enu, seed=42)
    depth_cam = RaycastOakDDepthCamera(seed=101)

    dt = 0.08
    frames: List[Dict] = []
    active_spline_pts: List[List[float]] = []

    for step_i in range(800):
        sim_t = round(step_i * dt, 3)
        meas_pos_ned, meas_vel_ned, meas_yaw_ned = phys.observe_noisy_vio_ned(dt)
        est_enu = ned_to_enu_pos(meas_pos_ned, origin_enu=tracker.launch_origin_enu)
        est_yaw_enu = wrap_pi(0.5 * math.pi - meas_yaw_ned)

        tracker.update_local_position(
            sim_t,
            float(meas_pos_ned[0]),
            float(meas_pos_ned[1]),
            float(meas_pos_ned[2]),
            float(meas_vel_ned[0]),
            float(meas_vel_ned[1]),
            float(meas_vel_ned[2]),
            float(meas_yaw_ned),
            True,
            True,
        )
        tracker.update_vehicle_status(sim_t, ARMING_STATE_ARMED, NAVIGATION_STATE_OFFBOARD)
        tracker.update_estimator_health(True)

        depth_hits_world: List[List[float]] = []
        if step_i % 3 == 0:
            raw_bytes = depth_cam.render_16uc1_mm(truth_grid, phys.true_pos_enu, phys.true_yaw_enu)
            depth_m = (
                np.frombuffer(raw_bytes, dtype=np.uint16)
                .reshape(depth_cam.height, depth_cam.width)
                .astype(np.float32)
                * 0.001
            )
            planner_grid.update_from_depth_image(
                depth_m,
                depth_cam.fx,
                depth_cam.fy,
                depth_cam.cx,
                depth_cam.cy,
                est_enu,
                est_yaw_enu,
            )
            # Extract recently discovered voxels for 3D visualization
            diff_vox = np.argwhere(planner_grid.raw_grid & ~initial_raw)
            if diff_vox.size > 0:
                centers = planner_grid.bounds_min + (diff_vox.astype(np.float64) + 0.5) * planner_grid.resolution
                depth_hits_world = np.round(centers[:: max(1, len(centers) // 120)], 3).tolist()

        sample, reason = tracker.sample_setpoint(
            sim_t,
            external_window_confidence=(0.35 if block_window else 0.95),
        )
        if sample is not None:
            if router.active_spline is not None:
                active_spline_pts = np.round(router.active_spline.sample_points(30), 3).tolist()
            prof = router.profile_for_stage(router.stage)
            phys.step_physics(
                dt,
                sim_t,
                sample.position_ned,
                sample.velocity_ned,
                sample.yaw_ned,
                prof.v_max,
            )
            sp_enu = ned_to_enu_pos(sample.position_ned, origin_enu=tracker.launch_origin_enu)
        else:
            sp_enu = phys.true_pos_enu

        speed_mps = float(np.linalg.norm(phys.true_vel_enu))
        vio_err_cm = float(np.linalg.norm(est_enu - phys.true_pos_enu) * 100.0)

        frames.append(
            {
                "t": sim_t,
                "stage": router.stage.value,
                "branch": router.selected_branch,
                "reason": reason,
                "true_enu": [round(float(x), 4) for x in phys.true_pos_enu],
                "true_yaw": round(float(phys.true_yaw_enu), 4),
                "vio_enu": [round(float(x), 4) for x in est_enu],
                "sp_enu": [round(float(x), 4) for x in sp_enu],
                "speed_mps": round(speed_mps, 3),
                "vio_err_cm": round(vio_err_cm, 2),
                "spline": active_spline_pts if (step_i % 3 == 0 or reason != "tracking_active_spline") else None,
                "discovered_voxels": depth_hits_world if depth_hits_world else None,
            }
        )

        if router.stage == CourseStage.MISSION_COMPLETE:
            break

    return {
        "fast_mode": fast_mode,
        "spawn_unmapped_boulder": spawn_unmapped_boulder,
        "block_window": block_window,
        "total_time_s": frames[-1]["t"] if frames else 0.0,
        "selected_branch": router.selected_branch,
        "unmapped_boxes": unmapped_boxes,
        "frames": frames,
    }


HTML_PAGE = """<!DOCTYPE html>
<html lang="en">
<head>
  <meta charset="UTF-8" />
  <title>DroneHacks 2026 — Live 3D EGO-Planner + Physics + VIO/Depth Simulator</title>
  <style>
    * { box-sizing: border-box; margin: 0; padding: 0; }
    body { background: #0b0f19; color: #e2e8f0; font-family: system-ui, -apple-system, sans-serif; overflow: hidden; }
    #canvas-container { width: 100vw; height: 100vh; }
    #hud {
      position: absolute; top: 16px; left: 16px; width: 360px;
      background: rgba(15, 23, 42, 0.88); border: 1px solid #334155;
      border-radius: 12px; padding: 16px; backdrop-filter: blur(8px);
      box-shadow: 0 10px 25px rgba(0,0,0,0.5);
    }
    h1 { font-size: 16px; font-weight: 700; color: #38bdf8; margin-bottom: 10px; }
    .row { display: flex; justify-content: space-between; font-size: 13px; margin: 5px 0; }
    .label { color: #94a3b8; }
    .val { font-family: monospace; font-weight: 600; color: #f8fafc; }
    .badge { padding: 2px 8px; border-radius: 999px; font-size: 11px; font-weight: 700; }
    .badge-stage { background: #1e3a8a; color: #93c5fd; }
    .badge-win { background: #065f46; color: #6ee7b7; }
    .badge-gateb { background: #78350f; color: #fcd34d; }
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
    <h1>DroneHacks 2026 — 3D Sim & EGO-Planner</h1>
    <div class="row"><span class="label">Sim Time:</span><span class="val" id="val-t">0.00 s</span></div>
    <div class="row"><span class="label">FSM Stage:</span><span class="val badge badge-stage" id="val-stage">LOADING...</span></div>
    <div class="row"><span class="label">Chicane Branch:</span><span class="val badge badge-win" id="val-branch">PENDING</span></div>
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
      <div style="font-size:12px; color:#94a3b8; margin-top:4px;">Scenario Presets (runs live Python physics + 3D A*):</div>
      <div class="btn-group">
        <button id="preset-win" class="active">Window + Unmapped Boulder</button>
        <button id="preset-gateb">Blocked Window -> Gate B</button>
      </div>
    </div>

    <div class="legend">
      <div><span class="dot" style="background:#22d3ee;"></span>True Physical Drone (2nd-Order PX4 + Wind)</div>
      <div><span class="dot" style="background:#f43f5e;"></span>Noisy/Drifted VIO Estimate (No GPS)</div>
      <div><span class="dot" style="background:#4ade80;"></span>Active EGO B-Spline Trajectory</div>
      <div><span class="dot" style="background:#fb923c;"></span>Live OAK-D Discovered Obstacle Voxels</div>
    </div>
  </div>

  <script type="module">
    import * as THREE from 'three';
    import { OrbitControls } from 'three/addons/controls/OrbitControls.js';
    import { OBJLoader } from 'three/addons/loaders/OBJLoader.js';

    const container = document.getElementById('canvas-container');
    const scene = new THREE.Scene();
    scene.background = new THREE.Color(0x0b0f19);

    // Note: Course ENU has +X East, +Y North, +Z Up. We set THREE Object3D.DEFAULT_UP to (0,0,1).
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

    // Load actual dronehacks-course-v3.obj mesh
    const loader = new OBJLoader();
    loader.load('/course.obj', (obj) => {
      obj.traverse((child) => {
        if (child.isMesh) {
          child.material = new THREE.MeshStandardMaterial({
            color: 0x64748b,
            transparent: true,
            opacity: 0.48,
            roughness: 0.6,
            side: THREE.DoubleSide
          });
          const edges = new THREE.EdgesGeometry(child.geometry, 25);
          const line = new THREE.LineSegments(edges, new THREE.LineBasicMaterial({ color: 0x94a3b8, transparent: true, opacity: 0.35 }));
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

    // Unmapped obstacle group & discovered OAK-D voxels group
    const unmappedGroup = new THREE.Group();
    scene.add(unmappedGroup);
    const voxGroup = new THREE.Group();
    scene.add(voxGroup);

    let episode = null;
    let frameIdx = 0;
    let playing = true;
    let followCam = false;
    let lastTick = performance.now();

    const timeline = document.getElementById('timeline');
    const btnPlay = document.getElementById('btn-play');
    const btnRestart = document.getElementById('btn-restart');
    const btnCam = document.getElementById('btn-cam');
    const presetWin = document.getElementById('preset-win');
    const presetGateB = document.getElementById('preset-gateb');

    async function loadScenario(blockWindow) {
      document.getElementById('val-stage').textContent = 'SIMULATING...';
      const res = await fetch(`/api/sim?fast_mode=1&boulder=1&block_window=${blockWindow ? 1 : 0}`);
      episode = await res.json();
      frameIdx = 0;
      timeline.max = Math.max(0, episode.frames.length - 1);
      timeline.value = 0;

      // Render unmapped ground-truth boxes
      unmappedGroup.clear();
      for (const b of episode.unmapped_boxes) {
        const sx = b.max[0] - b.min[0], sy = b.max[1] - b.min[1], sz = b.max[2] - b.min[2];
        const m = new THREE.Mesh(
          new THREE.BoxGeometry(sx, sy, sz),
          new THREE.MeshStandardMaterial({ color: 0xef4444, transparent: true, opacity: 0.45 })
        );
        m.position.set(0.5*(b.min[0]+b.max[0]), 0.5*(b.min[1]+b.max[1]), 0.5*(b.min[2]+b.max[2]));
        unmappedGroup.add(m);
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
      document.getElementById('val-stage').textContent = f.stage;
      const brEl = document.getElementById('val-branch');
      brEl.textContent = f.branch;
      brEl.className = 'val badge ' + (f.branch === 'GATE_B' ? 'badge-gateb' : 'badge-win');
      document.getElementById('val-reason').textContent = f.reason;
      document.getElementById('val-speed').textContent = `${f.speed_mps.toFixed(2)} m/s`;
      document.getElementById('val-pos').textContent = `(${f.true_enu[0].toFixed(2)}, ${f.true_enu[1].toFixed(2)}, ${f.true_enu[2].toFixed(2)})`;
      document.getElementById('val-vio').textContent = `${f.vio_err_cm.toFixed(2)} cm`;
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
    presetWin.onclick = () => {
      presetWin.classList.add('active'); presetGateB.classList.remove('active');
      loadScenario(false);
    };
    presetGateB.onclick = () => {
      presetGateB.classList.add('active'); presetWin.classList.remove('active');
      loadScenario(true);
    };

    window.addEventListener('resize', () => {
      camera.aspect = window.innerWidth / window.innerHeight;
      camera.updateProjectionMatrix();
      renderer.setSize(window.innerWidth, window.innerHeight);
    });

    loadScenario(false);

    function animate(now) {
      requestAnimationFrame(animate);
      if (playing && episode && episode.frames.length > 0 && (now - lastTick) >= 65) {
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
            boulder = qs.get("boulder", ["1"])[0] == "1"
            block_win = qs.get("block_window", ["0"])[0] == "1"
            key = f"{fast}_{boulder}_{block_win}"
            if key not in self._cache:
                ep = run_simulation_episode(
                    fast_mode=fast,
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
        for bw in (False, True):
            key = f"True_True_{bw}"
            if key not in SimViewerHandler._cache:
                ep = run_simulation_episode(fast_mode=True, spawn_unmapped_boulder=True, block_window=bw)
                SimViewerHandler._cache[key] = json.dumps(ep).encode("utf-8")

    threading.Thread(target=_warm, daemon=True).start()
    server = HTTPServer(("0.0.0.0", port), SimViewerHandler)
    print(f"3D Simulation Viewer live at http://localhost:{port}", flush=True)
    server.serve_forever()


if __name__ == "__main__":
    main()
