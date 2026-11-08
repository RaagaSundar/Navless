"""Live Top-Down Course Map & Multithreaded Planner Visualizer for skybeta-19.

Serves an interactive browser dashboard (default port 8082) on its own daemon thread
inside course_mission so you can smoke-test cuVSLAM + stereo_cloud.py + the
multithreaded 3D occupancy mapper + 3D A* / EGO B-spline replanner on the Jetson
(even while disarmed on the ground in WAIT_FOR_OFFBOARD).

Endpoints:
  GET  /                         HTML5 top-down course + stereo discrepancy + B-spline viewer
  GET  /api/map_state            JSON snapshot of prior walls, live stereo voxels, routes, pose, threads
  POST /api/inject_test_obstacle Inject a test obstacle 1.2 m ahead on the route to smoke-test replanning
  POST /api/reset_map            Clear live discrepancy obstacles and restore the clean initial route
"""
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import json
import math
import threading
import time

import numpy as np

from navless_nav.route import course_to_odom, odom_to_course


HTML_PAGE = """<!doctype html>
<html lang="en">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>Navless · Live Top-Down Planner & Stereo Map</title>
<style>
:root {
  color-scheme: dark;
  --bg: #0b1014;
  --panel: #121a21;
  --edge: #263542;
  --ink: #ecf3f8;
  --sub: #8fa6b8;
  --green: #38e596;
  --cyan: #4cc9f0;
  --amber: #f7b84b;
  --red: #ff5c72;
}
* { box-sizing: border-box; }
body {
  margin: 0;
  background: var(--bg);
  color: var(--ink);
  font: 14px/1.4 'Inter', system-ui, -apple-system, sans-serif;
}
header {
  display: flex;
  flex-wrap: wrap;
  justify-content: space-between;
  align-items: center;
  padding: 14px 22px;
  border-bottom: 1px solid var(--edge);
  background: #0e151b;
  gap: 12px;
}
h1 { font-size: 18px; margin: 0; letter-spacing: -0.02em; }
.sub { color: var(--sub); font-size: 12px; }
.badges { display: flex; gap: 8px; align-items: center; flex-wrap: wrap; }
.pill {
  padding: 5px 11px;
  border-radius: 999px;
  font-size: 12px;
  font-weight: 600;
  background: #251d1a;
  color: var(--amber);
  border: 1px solid #473628;
}
.pill.ok { background: #132c23; color: var(--green); border-color: #215742; }
.pill.err { background: #30171d; color: var(--red); border-color: #5e2734; }
.layout {
  display: grid;
  grid-template-columns: minmax(380px, 640px) 1fr;
  gap: 18px;
  max-width: 1360px;
  margin: 0 auto;
  padding: 18px 22px;
}
@media (max-width: 960px) {
  .layout { grid-template-columns: 1fr; }
}
.card {
  background: var(--panel);
  border: 1px solid var(--edge);
  border-radius: 12px;
  padding: 16px;
}
.canvas-wrap {
  position: relative;
  width: 100%;
  aspect-ratio: 1 / 1;
  background: #090d11;
  border: 1px solid var(--edge);
  border-radius: 8px;
  overflow: hidden;
}
canvas { display: block; width: 100%; height: 100%; }
.legend {
  display: flex;
  flex-wrap: wrap;
  gap: 14px;
  margin-top: 12px;
  font-size: 12px;
  color: var(--sub);
}
.dot {
  display: inline-block;
  width: 10px;
  height: 10px;
  border-radius: 2px;
  margin-right: 5px;
  vertical-align: middle;
}
.stats-grid {
  display: grid;
  grid-template-columns: repeat(auto-fit, minmax(165px, 1fr));
  gap: 10px;
  margin-bottom: 14px;
}
.stat {
  background: #0e151b;
  border: 1px solid var(--edge);
  border-radius: 8px;
  padding: 10px 12px;
}
.stat dt { color: var(--sub); font-size: 11px; text-transform: uppercase; letter-spacing: 0.04em; }
.stat dd { margin: 4px 0 0; font-size: 16px; font-weight: 600; font-family: ui-monospace, SFMono-Regular, monospace; }
.controls {
  display: flex;
  flex-wrap: wrap;
  gap: 10px;
  margin: 14px 0;
}
button {
  cursor: pointer;
  padding: 9px 14px;
  border-radius: 8px;
  border: 1px solid var(--edge);
  background: #1a2631;
  color: var(--ink);
  font-weight: 600;
  font-size: 13px;
  transition: background 0.15s;
}
button:hover { background: #243544; }
button.primary { background: #16382b; border-color: #27694e; color: var(--green); }
button.primary:hover { background: #1e4b3a; }
button.warn { background: #332024; border-color: #63323c; color: #ff8596; }
button.warn:hover { background: #45292f; }
.checklist {
  margin: 10px 0 0;
  padding-left: 18px;
  color: var(--sub);
  font-size: 13px;
}
.checklist li { margin-bottom: 6px; }
.checklist strong { color: var(--ink); }
.mono { font-family: ui-monospace, SFMono-Regular, monospace; }
</style>
</head>
<body>
<header>
  <div>
    <h1>Navless · Live Top-Down Course Map &amp; Multithreaded Planner</h1>
    <div class="sub" id="subtitle">Connecting to course_mission live map server...</div>
  </div>
  <div class="badges">
    <span class="pill" id="badge-vio">VIO: --</span>
    <span class="pill" id="badge-threads">Threads: --</span>
    <span class="pill" id="badge-prearm">Pre-Arm: --</span>
  </div>
</header>

<div class="layout">
  <div class="card">
    <div class="canvas-wrap">
      <canvas id="mapCanvas"></canvas>
    </div>
    <div class="legend">
      <span><i class="dot" style="background:#33495e"></i>Prior Course Walls (0.85m slice)</span>
      <span><i class="dot" style="background:#ff5c72"></i>Live Stereo Discrepancies (/team/stereo/points)</span>
      <span><i class="dot" style="background:#4cc9f0"></i>Initial Baseline Route</span>
      <span><i class="dot" style="background:#38e596"></i>Active 3D A* + B-Spline Route</span>
      <span><i class="dot" style="background:#f7b84b"></i>Carrot Setpoint</span>
    </div>
  </div>

  <div class="card">
    <h2 style="margin:0 0 12px;font-size:16px;">Live Jetson Telemetry &amp; Thread Health</h2>
    <dl class="stats-grid">
      <div class="stat"><dt>Mission State</dt><dd id="st-state">--</dd></div>
      <div class="stat"><dt>Route Mode / Kind</dt><dd id="st-kind" style="font-size:12px">--</dd></div>
      <div class="stat"><dt>Course Pose (ENU)</dt><dd id="st-pose">--</dd></div>
      <div class="stat"><dt>Nose / Travel Yaw</dt><dd id="st-yaw">--</dd></div>
      <div class="stat"><dt>Stereo Clouds Rx</dt><dd id="st-clouds">--</dd></div>
      <div class="stat"><dt>Live Obstacle Voxels</dt><dd id="st-voxels">--</dd></div>
      <div class="stat"><dt>3D A* + B-Spline Replans</dt><dd id="st-replans">--</dd></div>
      <div class="stat"><dt>Waypoints / Carrot</dt><dd id="st-wps">--</dd></div>
    </dl>

    <h3 style="margin:16px 0 8px;font-size:14px;">Bench Smoke-Test Controls (Disarmed Safe)</h3>
    <div class="controls">
      <button class="primary" onclick="smokeInject()">1. Inject Test Obstacle 1.2m Ahead</button>
      <button class="warn" onclick="smokeReset()">2. Clear Live Obstacles &amp; Reset Route</button>
    </div>
    <div id="action-msg" class="sub mono" style="min-height:18px;margin-bottom:10px;"></div>

    <h3 style="margin:14px 0 6px;font-size:14px;">Quick Jetson Smoke-Test Procedure</h3>
    <ol class="checklist">
      <li><strong>Verify Threads &amp; VIO:</strong> Check that <span class="mono">VIO: TRACKING</span> and <span class="mono">Threads: 2 ALIVE</span> are green above.</li>
      <li><strong>Test Real Stereo Camera Mapping:</strong> With the drone disarmed on the launch mark, hold a box or stand ~1.2 m in front of the OAK-D camera. Watch red discrepancy voxels appear in the yellow camera FOV cone and watch the green B-spline route bend around them!</li>
      <li><strong>Test Synthetic Replan:</strong> Click <em>"1. Inject Test Obstacle 1.2m Ahead"</em> to verify the background <span class="mono">navless_ego_replanner</span> thread detours the green path around the obstacle in real time.</li>
      <li><strong>Reset Before Arming:</strong> Click <em>"2. Clear Live Obstacles &amp; Reset Route"</em> so the map and route are clean before arming in Altitude &rarr; Position.</li>
    </ol>
  </div>
</div>

<script>
const NET_HALF = 3.048;
const VIEW_EXTENT = 3.25;
let lastState = null;

async function smokeInject() {
  const el = document.getElementById('action-msg');
  el.textContent = 'Injecting test obstacle ahead of drone...';
  try {
    const r = await fetch('/api/inject_test_obstacle', {method: 'POST'});
    const d = await r.json();
    el.textContent = d.message || 'Injected.';
  } catch (e) {
    el.textContent = 'Error: ' + e;
  }
}

async function smokeReset() {
  const el = document.getElementById('action-msg');
  el.textContent = 'Clearing live obstacles and restoring initial route...';
  try {
    const r = await fetch('/api/reset_map', {method: 'POST'});
    const d = await r.json();
    el.textContent = d.message || 'Reset complete.';
  } catch (e) {
    el.textContent = 'Error: ' + e;
  }
}

function drawMap(s) {
  const canvas = document.getElementById('mapCanvas');
  const rect = canvas.getBoundingClientRect();
  const dpr = window.devicePixelRatio || 1;
  canvas.width = Math.round(rect.width * dpr);
  canvas.height = Math.round(rect.height * dpr);
  const ctx = canvas.getContext('2d');
  ctx.scale(dpr, dpr);
  const W = rect.width, H = rect.height;

  const toX = x => W * (0.5 + x / (2 * VIEW_EXTENT));
  const toY = y => H * (0.5 - y / (2 * VIEW_EXTENT));
  const mToPx = m => (m / (2 * VIEW_EXTENT)) * Math.min(W, H);

  ctx.fillStyle = '#090d11';
  ctx.fillRect(0, 0, W, H);

  // 1m grid lines
  ctx.strokeStyle = '#15202b';
  ctx.lineWidth = 1;
  for (let g = -3; g <= 3; g++) {
    ctx.beginPath();
    ctx.moveTo(toX(g), 0); ctx.lineTo(toX(g), H);
    ctx.moveTo(0, toY(g)); ctx.lineTo(W, toY(g));
    ctx.stroke();
  }

  // 20x20 ft perimeter net box
  ctx.strokeStyle = '#3b5266';
  ctx.lineWidth = 2;
  ctx.strokeRect(
    toX(-NET_HALF),
    toY(NET_HALF),
    toX(NET_HALF) - toX(-NET_HALF),
    toY(-NET_HALF) - toY(NET_HALF)
  );

  if (!s) return;

  // Prior course wall cells (slice around flight altitude)
  const voxPx = Math.max(2, mToPx(s.voxel_res || 0.10));
  ctx.fillStyle = '#33495e';
  for (const [wx, wy] of (s.prior_xy || [])) {
    ctx.fillRect(toX(wx) - voxPx * 0.5, toY(wy) - voxPx * 0.5, voxPx, voxPx);
  }

  // Launch mark & Landing pad
  if (s.launch_xy) {
    ctx.strokeStyle = '#4cc9f0';
    ctx.lineWidth = 2;
    ctx.beginPath();
    ctx.arc(toX(s.launch_xy[0]), toY(s.launch_xy[1]), mToPx(0.18), 0, Math.PI * 2);
    ctx.stroke();
  }
  if (s.pad_xy) {
    for (const r of [0.23, 0.51, 0.76]) {
      ctx.strokeStyle = 'rgba(247, 184, 75, 0.45)';
      ctx.lineWidth = 1.5;
      ctx.beginPath();
      ctx.arc(toX(s.pad_xy[0]), toY(s.pad_xy[1]), mToPx(r), 0, Math.PI * 2);
      ctx.stroke();
    }
  }

  // Initial baseline route (dashed cyan)
  if (s.initial_route_enu && s.initial_route_enu.length > 1) {
    ctx.save();
    ctx.strokeStyle = 'rgba(76, 201, 240, 0.45)';
    ctx.lineWidth = 2;
    ctx.setLineDash([5, 5]);
    ctx.beginPath();
    s.initial_route_enu.forEach(([x, y], i) => {
      if (i === 0) ctx.moveTo(toX(x), toY(y));
      else ctx.lineTo(toX(x), toY(y));
    });
    ctx.stroke();
    ctx.restore();
  }

  // Active 3D A* + EGO B-spline route (solid green)
  if (s.route_enu && s.route_enu.length > 1) {
    ctx.strokeStyle = '#38e596';
    ctx.lineWidth = 3;
    ctx.beginPath();
    s.route_enu.forEach(([x, y], i) => {
      if (i === 0) ctx.moveTo(toX(x), toY(y));
      else ctx.lineTo(toX(x), toY(y));
    });
    ctx.stroke();

    ctx.fillStyle = '#38e596';
    for (let i = 0; i < s.route_enu.length; i += 3) {
      const [x, y] = s.route_enu[i];
      ctx.beginPath();
      ctx.arc(toX(x), toY(y), 2.2, 0, Math.PI * 2);
      ctx.fill();
    }
  }

  // Live discovered stereo discrepancy voxels (glowing red/orange)
  ctx.fillStyle = '#ff5c72';
  for (const [dx, dy] of (s.discrepancy_xy || [])) {
    ctx.fillRect(toX(dx) - voxPx * 0.65, toY(dy) - voxPx * 0.65, voxPx * 1.3, voxPx * 1.3);
  }

  // Carrot setpoint
  if (s.carrot_enu) {
    ctx.fillStyle = '#f7b84b';
    ctx.beginPath();
    ctx.arc(toX(s.carrot_enu[0]), toY(s.carrot_enu[1]), 6, 0, Math.PI * 2);
    ctx.fill();
  }

  // Drone pose + OAK-D stereo FOV cone + travel heading arrow
  if (s.drone_enu) {
    const [dx, dy] = s.drone_enu;
    const yaw = (s.yaw_enu_rad != null) ? s.yaw_enu_rad : (Math.PI * 0.5);
    const hfov = 0.62; // ~70 deg total HFOV
    const coneLen = mToPx(1.8);

    // Camera FOV wedge
    ctx.save();
    ctx.fillStyle = 'rgba(247, 184, 75, 0.13)';
    ctx.strokeStyle = 'rgba(247, 184, 75, 0.45)';
    ctx.lineWidth = 1;
    ctx.beginPath();
    ctx.moveTo(toX(dx), toY(dy));
    ctx.lineTo(toX(dx) + Math.cos(yaw - hfov) * coneLen, toY(dy) - Math.sin(yaw - hfov) * coneLen);
    ctx.lineTo(toX(dx) + Math.cos(yaw + hfov) * coneLen, toY(dy) - Math.sin(yaw + hfov) * coneLen);
    ctx.closePath();
    ctx.fill();
    ctx.stroke();
    ctx.restore();

    // Drone body circle
    ctx.strokeStyle = '#ffffff';
    ctx.fillStyle = '#4cc9f0';
    ctx.lineWidth = 2;
    ctx.beginPath();
    ctx.arc(toX(dx), toY(dy), mToPx(0.2375), 0, Math.PI * 2);
    ctx.stroke();
    ctx.beginPath();
    ctx.arc(toX(dx), toY(dy), 5, 0, Math.PI * 2);
    ctx.fill();

    // Nose direction line
    ctx.strokeStyle = '#ffffff';
    ctx.lineWidth = 2.5;
    ctx.beginPath();
    ctx.moveTo(toX(dx), toY(dy));
    ctx.lineTo(toX(dx) + Math.cos(yaw) * mToPx(0.45), toY(dy) - Math.sin(yaw) * mToPx(0.45));
    ctx.stroke();
  }

  // Axis labels
  ctx.fillStyle = '#8fa6b8';
  ctx.font = '11px ui-monospace, monospace';
  ctx.fillText('NORTH (+Y = +3.05m)', toX(-0.55), 14);
  ctx.fillText('SOUTH (-Y)', toX(-0.30), H - 6);
  ctx.fillText('WEST (-X)', 6, toY(0) - 6);
  ctx.fillText('EAST (+X)', W - 64, toY(0) - 6);
}

function updateUI(s) {
  lastState = s;
  document.getElementById('subtitle').textContent =
    `Updated ${new Date().toLocaleTimeString()} · heading_mode=${s.heading_mode} · route_source=${s.route_source}`;

  const vioBadge = document.getElementById('badge-vio');
  if (s.vio_ok) {
    vioBadge.textContent = 'VIO: TRACKING (vo_state=1)';
    vioBadge.className = 'pill ok';
  } else {
    vioBadge.textContent = `VIO: WAITING/LOST (vo_state=${s.vo_state ?? 'none'})`;
    vioBadge.className = 'pill err';
  }

  const thBadge = document.getElementById('badge-threads');
  if (s.cloud_thread_alive && s.replan_thread_alive) {
    thBadge.textContent = s.bg_replanning ? 'Threads: 2 ALIVE (REPLANNING...)' : 'Threads: 2 ALIVE (Mapper + Replanner)';
    thBadge.className = 'pill ok';
  } else {
    thBadge.textContent = 'Threads: SYNC MODE';
    thBadge.className = 'pill';
  }

  const paBadge = document.getElementById('badge-prearm');
  if (s.prearm_passed) {
    paBadge.textContent = 'Pre-Arm: PASSED (Ready)';
    paBadge.className = 'pill ok';
  } else {
    paBadge.textContent = 'Pre-Arm: Checking...';
    paBadge.className = 'pill';
  }

  document.getElementById('st-state').textContent = `${s.state} (${s.armed ? 'ARMED' : 'DISARMED'})`;
  document.getElementById('st-kind').textContent = s.route_kind || 'DYNAMIC_3D_ASTAR_BSPLINE';
  document.getElementById('st-pose').textContent = s.drone_enu
    ? s.drone_enu.map(v => v.toFixed(2)).join(', ') + ' m'
    : 'waiting for odom';
  document.getElementById('st-yaw').textContent =
    `${s.yaw_enu_deg != null ? s.yaw_enu_deg.toFixed(1) + '°' : '--'} / ${s.want_yaw_enu_deg != null ? s.want_yaw_enu_deg.toFixed(1) + '°' : '--'}`;
  document.getElementById('st-clouds').textContent = `${s.cloud_rx_count || 0} frames`;
  document.getElementById('st-voxels').textContent = `${(s.discrepancy_xy || []).length} cells`;
  document.getElementById('st-replans').textContent = `${s.replans || 0} replans`;
  document.getElementById('st-wps').textContent = `${s.carrot_index ?? 0} / ${(s.route_enu || []).length} wps`;

  drawMap(s);
}

async function poll() {
  try {
    const res = await fetch('/api/map_state', {cache: 'no-store'});
    if (res.ok) {
      updateUI(await res.json());
    }
  } catch (e) {
    document.getElementById('subtitle').textContent = 'Disconnected from course_mission map server';
  }
  setTimeout(poll, 250);
}

window.addEventListener('resize', () => drawMap(lastState));
poll();
</script>
</body>
</html>
"""


class LiveMapVisualizer:
    """Lightweight HTTP server exposing a top-down 2D map of course_mission's 3D grid + B-spline."""

    def __init__(self, node, port=8082):
        self.node = node
        self.port = int(port)
        self._server = None
        self._thread = None
        self._prior_xy_cache = None

    def _compute_prior_xy(self):
        if self._prior_xy_cache is not None:
            return self._prior_xy_cache
        grid = self.node.planner_grid
        # Slice between z = 0.45 m and 1.35 m so floor/ceiling aren't drawn as solid walls
        iz_min = max(0, int(math.floor((0.45 - grid.bounds_min[2]) / grid.resolution)))
        iz_max = min(grid.shape[2], int(math.ceil((1.35 - grid.bounds_min[2]) / grid.resolution)))
        occ_2d = np.any(grid.prior_grid[:, :, iz_min:iz_max], axis=2)
        ij = np.argwhere(occ_2d)
        pts = []
        for ix, iy in ij:
            wx = round(float(grid.bounds_min[0] + (ix + 0.5) * grid.resolution), 3)
            wy = round(float(grid.bounds_min[1] + (iy + 0.5) * grid.resolution), 3)
            pts.append([wx, wy])
        self._prior_xy_cache = pts
        return pts

    def snapshot(self):
        node = self.node
        if node.position is None and getattr(node, '_bg_replan_result', None) is not None:
            node.maybe_replan_for_discrepancies(node.now_s())
        grid = node.planner_grid
        frame = node.frame or ((-2.515, 0.914), 0.0)
        launch_xy, bearing_deg = frame[0], frame[1]

        with node._planner_lock:
            disc_pts = grid.get_discovered_discrepancy_points()
            route_odom = list(node.route or [])
            init_odom = list(getattr(node, 'initial_route_odom', None) or route_odom)

        disc_xy = []
        if disc_pts.size > 0:
            # Project unique 2D cells of discovered live obstacles
            ij = np.unique(np.round(disc_pts[:, :2], 2), axis=0)
            disc_xy = [[float(p[0]), float(p[1])] for p in ij]

        route_enu = [
            [round(v, 3) for v in odom_to_course(q, launch_xy, bearing_deg, node.start_height)]
            for q in route_odom
        ]
        init_enu = [
            [round(v, 3) for v in odom_to_course(q, launch_xy, bearing_deg, node.start_height)]
            for q in init_odom
        ]

        drone_enu = None
        if node.position is not None:
            c = odom_to_course(node.corr_to_route(node.position), launch_xy, bearing_deg, node.start_height)
            drone_enu = [round(float(c[0]), 3), round(float(c[1]), 3), round(float(c[2]), 3)]

        carrot_enu = None
        carrot_idx = 0
        if node.carrot is not None:
            cc = odom_to_course(node.carrot.point, launch_xy, bearing_deg, node.start_height)
            carrot_enu = [round(float(cc[0]), 3), round(float(cc[1]), 3), round(float(cc[2]), 3)]
            carrot_idx = int(node.carrot.index)

        yaw_enu_rad = 0.5 * math.pi - math.radians(bearing_deg)
        if node.yaw is not None:
            yaw_enu_rad = math.atan2(
                math.sin(yaw_enu_rad + node.yaw - node.corr[2]),
                math.cos(yaw_enu_rad + node.yaw - node.corr[2]),
            )
        want_yaw_deg = None
        if node.want_yaw is not None:
            w_rad = (0.5 * math.pi - math.radians(bearing_deg)) + node.want_yaw
            want_yaw_deg = round(math.degrees(math.atan2(math.sin(w_rad), math.cos(w_rad))), 1)

        pad_xy = [-2.042, -0.792]
        if grid.detected_concentric_pad is not None:
            pad_xy = [
                round(float(grid.detected_concentric_pad.center_xy_m[0]), 3),
                round(float(grid.detected_concentric_pad.center_xy_m[1]), 3),
            ]

        cloud_t = getattr(node, '_cloud_thread', None)
        replan_t = getattr(node, '_bg_thread', None)

        return {
            't': round(time.time(), 3),
            'state': node.state,
            'armed': bool(node.armed),
            'offboard': bool(node.offboard_active),
            'prearm_passed': bool(node.prearm_passed),
            'vio_ok': bool(node.vio_ok()),
            'vo_state': node.vo_state,
            'heading_mode': node.heading_mode,
            'route_source': 'file' if node.static_asbuilt_route else 'planner',
            'route_kind': getattr(node, 'route_kind', 'DYNAMIC_3D_ASTAR_BSPLINE'),
            'voxel_res': float(grid.resolution),
            'launch_xy': [round(float(launch_xy[0]), 3), round(float(launch_xy[1]), 3)],
            'pad_xy': pad_xy,
            'prior_xy': self._compute_prior_xy(),
            'discrepancy_xy': disc_xy,
            'initial_route_enu': init_enu,
            'route_enu': route_enu,
            'drone_enu': drone_enu,
            'carrot_enu': carrot_enu,
            'carrot_index': carrot_idx,
            'yaw_enu_rad': round(float(yaw_enu_rad), 4),
            'yaw_enu_deg': round(math.degrees(yaw_enu_rad), 1),
            'want_yaw_enu_deg': want_yaw_deg,
            'cloud_rx_count': int(getattr(node, 'cloud_rx_count', 0)),
            'replans': int(node.replans),
            'cloud_thread_alive': bool(cloud_t is not None and cloud_t.is_alive()),
            'replan_thread_alive': bool(replan_t is not None and replan_t.is_alive()),
            'bg_replanning': bool(node._bg_replanning),
        }

    def inject_test_obstacle(self):
        """Inject a 35x35 cm obstacle ~1.2 m ahead along the planned route to smoke-test replanning."""
        node = self.node
        if node.frame is None or not node.route:
            return {'ok': False, 'message': 'No route loaded yet.'}
        launch_xy, bearing_deg = node.frame
        # Pick a waypoint ~1.2 m ahead of the drone (or index 8 from start when disarmed)
        start_idx = node.carrot.index if node.carrot is not None else 0
        target_idx = min(len(node.route) - 3, start_idx + 8)
        wp_odom = node.route[max(1, target_idx)]
        cx, cy, cz = odom_to_course(wp_odom, launch_xy, bearing_deg, node.start_height)
        xs = np.linspace(cx - 0.18, cx + 0.18, 4)
        ys = np.linspace(cy - 0.18, cy + 0.18, 4)
        zs = np.linspace(0.35, 1.35, 5)
        pts = np.array([[x, y, z] for x in xs for y in ys for z in zs], dtype=np.float64)
        cur_c = odom_to_course(
            node.corr_to_route(node.position or (0.0, 0.0, 0.78)),
            launch_xy, bearing_deg, node.start_height,
        )
        with node._planner_lock:
            for _ in range(3):
                node.planner_grid.insert_point_cloud(pts, sensor_origin_m=cur_c, min_live_hits=2)
            node.live_grid_updated = True
            node.last_replan_at = 0.0
        node.maybe_replan_for_discrepancies(node.now_s() + 1.0)
        return {
            'ok': True,
            'message': f'Injected test obstacle at course ({cx:+.2f}, {cy:+.2f}) m and triggered 3D A* + B-spline replan.',
        }

    def reset_map(self):
        """Clear live discrepancy obstacles and restore the initial clean route."""
        node = self.node
        with node._planner_lock:
            node.planner_grid.raw_grid[:] = node.planner_grid.prior_grid
            node.planner_grid.live_hit_counts.fill(0)
            node.planner_grid.rebuild_inflation()
            node.live_grid_updated = False
            node._bg_replan_req = None
            node._bg_replan_result = None
            if getattr(node, 'initial_route_odom', None):
                node.route = list(node.initial_route_odom)
                node.route_kind = getattr(node, 'initial_route_kind', 'DYNAMIC_3D_ASTAR_BSPLINE')
                if node.carrot is not None:
                    eff_sag = node.max_sagitta if node.max_sagitta > 0.0 else 0.04
                    from navless_nav.route import Carrot
                    node.carrot = Carrot(
                        node.route, node.cruise_speed, node.max_lead,
                        node.max_lat_accel, eff_sag,
                    )
                    if node.position is not None:
                        node.carrot.catch_up(node.corr_to_route(node.position))
        return {'ok': True, 'message': 'Cleared live stereo obstacles and restored clean baseline route.'}

    def start(self):
        if self._server is not None or self.port <= 0:
            return
        viz = self

        class Handler(BaseHTTPRequestHandler):
            def do_GET(self):
                if self.path == '/' or self.path.startswith('/index.html'):
                    body = HTML_PAGE.encode('utf-8')
                    mime = 'text/html; charset=utf-8'
                elif self.path.startswith('/api/map_state'):
                    body = json.dumps(viz.snapshot()).encode('utf-8')
                    mime = 'application/json'
                else:
                    self.send_error(404)
                    return
                self.send_response(200)
                self.send_header('Content-Type', mime)
                self.send_header('Cache-Control', 'no-store')
                self.send_header('Content-Length', str(len(body)))
                self.end_headers()
                self.wfile.write(body)

            def do_POST(self):
                if self.path.startswith('/api/inject_test_obstacle'):
                    res = viz.inject_test_obstacle()
                elif self.path.startswith('/api/reset_map'):
                    res = viz.reset_map()
                else:
                    self.send_error(404)
                    return
                body = json.dumps(res).encode('utf-8')
                self.send_response(200)
                self.send_header('Content-Type', 'application/json')
                self.send_header('Cache-Control', 'no-store')
                self.send_header('Content-Length', str(len(body)))
                self.end_headers()
                self.wfile.write(body)

            def log_message(self, *args):
                pass

        try:
            self._server = ThreadingHTTPServer(('0.0.0.0', self.port), Handler)
            self._thread = threading.Thread(
                target=self._server.serve_forever, name='navless_map_viz_http', daemon=True
            )
            self._thread.start()
            self.node.get_logger().info(
                f'live top-down planner & stereo map visualizer: http://0.0.0.0:{self._server.server_port}/'
            )
        except OSError as exc:
            self.node.get_logger().warn(f'live map visualizer port {self.port} unavailable ({exc})')

    def stop(self):
        if self._server is not None:
            try:
                self._server.shutdown()
                self._server.server_close()
            except OSError:
                pass
            self._server = None
