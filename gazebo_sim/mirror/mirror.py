#!/usr/bin/env python3
"""Live mirror: the real drone's VIO pose replayed in the Gazebo course, plus what its camera would see.

  # 1. on the drone (props off), inside the stack container with the team overlay sourced:
  ros2 run navless_nav mirror_relay
  # 2. on this laptop, in WSL:
  python3 gazebo_sim/mirror/mirror.py --drone 192.168.55.1           # drone on USB
  python3 gazebo_sim/mirror/mirror.py --drone <drone-wifi-ip> --gui   # also open the Gazebo window
  # 3. open http://localhost:8090 in a browser on this laptop
  # no drone? replay a recorded sim flight through the same path:
  python3 gazebo_sim/mirror/replay_relay.py gazebo_sim/runs/<run>/ &  python3 gazebo_sim/mirror/mirror.py --drone 127.0.0.1

Nothing here can command the drone: the relay is read-only, and this end only reads from it.

Frames: the relay sends cuVSLAM's odom pose (ENU at power-up: x = nose, y = left, z = up, origin =
base_link at power-up). It is placed in the course exactly the way navless_nav places its route:
launch mark + start bearing + start height. Power the drone up on the real launch mark, nose north
(bearing 0), or the mirror shows the right motion from the wrong place.
"""
import argparse
import http.server
import io
import json
import math
import os
import signal
import socket
import subprocess
import sys
import threading
import time

HERE = os.path.dirname(os.path.abspath(__file__))
SIM = os.path.dirname(HERE)
REPO = os.path.dirname(SIM)
sys.path.insert(0, os.path.join(REPO, "navless_nav"))
from navless_nav.route import odom_to_course, resolve_route_path  # noqa: E402

IN = 0.0254
HALF = 3.048
LAUNCH = (-2.5146, 0.9144)
# base_link -> camera mount when the drone's static TF never arrives: the hardware checklist's
# typical mount (CAMERA_X 0.10, Z -0.02, PITCH +0.35 rad = ~20 deg nose-down), not a measured one.
DEFAULT_CAM = ((0.10, 0.0, -0.02), (0.0, 0.17408, 0.0, 0.98473))
# The mount TF (oak_parent_frame) is the camera body; cuVSLAM's view is the left eye, half the
# 7.5 cm baseline to the left (the stack's TF: base_link -> left optical = mount + (0, 0.038, 0)).
LEFT_EYE = (0.0, 0.0375, 0.0)
HIDDEN = (0.0, 0.0, -1.0)


# ------------------------------------------------------------------------------------ quaternions (x, y, z, w)
def qmul(a, b):
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return (aw * bx + ax * bw + ay * bz - az * by, aw * by - ax * bz + ay * bw + az * bx,
            aw * bz + ax * by - ay * bx + az * bw, aw * bw - ax * bx - ay * by - az * bz)


def qrot(q, v):
    x, y, z, _ = qmul(qmul(q, (v[0], v[1], v[2], 0.0)), (-q[0], -q[1], -q[2], q[3]))
    return (x, y, z)


def qz(a):
    return (0.0, 0.0, math.sin(a / 2), math.cos(a / 2))


def yaw_of(q):
    x, y, z, w = q
    return math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))


def unit(q):
    n = math.sqrt(sum(v * v for v in q))
    return tuple(v / n for v in q) if n > 1e-9 else (0.0, 0.0, 0.0, 1.0)


class Placement:
    """odom (power-up frame) -> course/world, as navless_nav's route.py does it.

    rezero(p, q) makes the drone's CURRENT pose the launch mark, nose along the start
    bearing (for benches and rooms that are not the course); unzero() goes back to the
    power-up frame. The flight code never sees this: it is only where the mirror draws.
    """

    def __init__(self, launch, bearing_deg, start_height):
        self.launch, self.bearing, self.height = launch, bearing_deg, start_height
        self.q0 = qz(math.radians(90.0 - bearing_deg))      # ENU heading of odom +x
        self.ref = None                                      # (odom p, odom yaw) taken as the launch mark

    def rezero(self, p, q):
        self.ref = (tuple(p), yaw_of(q))

    def unzero(self):
        self.ref = None

    def _local(self, p):
        if self.ref is None:
            return p
        (rx, ry, rz), yaw = self.ref
        c, s = math.cos(-yaw), math.sin(-yaw)
        dx, dy = p[0] - rx, p[1] - ry
        return (c * dx - s * dy, s * dx + c * dy, p[2] - rz)

    def position(self, p):
        return odom_to_course(self._local(p), self.launch, self.bearing, self.height)

    def orientation(self, q):
        if self.ref is not None:
            q = qmul(qz(-self.ref[1]), unit(q))
        return unit(qmul(self.q0, unit(q)))


# ------------------------------------------------------------------------------------ inputs
class RelayClient:
    """Connects to navless_nav's mirror_relay and keeps the latest frame. Reconnects forever."""

    def __init__(self, host, port):
        self.host, self.port = host, port
        self.lock = threading.Lock()
        self.frame, self.frame_at, self.connected, self.count = None, None, False, 0
        threading.Thread(target=self._run, daemon=True).start()

    def _run(self):
        while True:
            try:
                with socket.create_connection((self.host, self.port), timeout=3) as s:
                    s.settimeout(3)
                    with self.lock:
                        self.connected = True
                    print(f"[mirror] connected to relay {self.host}:{self.port}", flush=True)
                    for line in s.makefile("r"):
                        try:
                            f = json.loads(line)
                        except ValueError:
                            continue
                        with self.lock:
                            self.frame, self.frame_at = f, time.time()
                            self.count += 1
            except OSError as e:
                with self.lock:
                    was = self.connected
                    self.connected = False
                if was:
                    print(f"[mirror] relay connection lost ({e}); retrying", flush=True)
            time.sleep(1.0)

    def latest(self):
        with self.lock:
            return self.frame, self.frame_at, self.connected, self.count


class CameraFeed:
    """Latest simulated camera frame as JPEG. Grey by default: the OAK-D-S2's stereo pair is
    monochrome (OV9282), and cuVSLAM sees mono8."""

    def __init__(self, node, topic="/mirror/camera", color=False):
        from gz.msgs10.image_pb2 import Image
        from PIL import Image as PILImage
        self.PILImage, self.color = PILImage, color
        self.cond = threading.Condition()
        self.jpeg, self.seq, self.size = None, 0, None
        node.subscribe(Image, topic, self._cb)

    def _cb(self, m):
        if m.pixel_format_type != 3 or len(m.data) < m.width * m.height * 3:      # RGB_INT8
            return
        img = self.PILImage.frombytes("RGB", (m.width, m.height), bytes(m.data))
        if not self.color:
            img = img.convert("L")
        buf = io.BytesIO()
        img.save(buf, format="JPEG", quality=80)
        with self.cond:
            self.jpeg, self.size = buf.getvalue(), (m.width, m.height)
            self.seq += 1
            self.cond.notify_all()

    def wait(self, after, timeout=1.0):
        with self.cond:
            self.cond.wait_for(lambda: self.seq != after, timeout)
            return self.seq, self.jpeg


# ------------------------------------------------------------------------------------ course map for the page
def course_map(route_path):
    c = json.load(open(os.path.join(SIM, "course", "course_v3.json")))
    w = lambda x, y: [round(x * IN - HALF, 4), round(y * IN - HALF, 4)]          # noqa: E731
    walls = [{"name": x["name"], "a": w(*x["start_in"]), "b": w(*x["end_in"]),
              "t": x.get("thickness_in", 4) * IN} for x in c["walls"]]
    tf = c["tunnel"]["footprint_in"]
    pad = c["landing_pad_in"]
    route = []
    if route_path:
        try:
            route = [[round(p[0], 3), round(p[1], 3)]
                     for p in json.load(open(resolve_route_path(route_path)))["waypoints_enu"]]
        except (OSError, ValueError, KeyError) as e:
            print(f"[mirror] route not drawn ({e})", flush=True)
    return {"half": HALF, "walls": walls,
            "tunnel": [w(tf["x_min"], tf["y_min"]), w(tf["x_max"], tf["y_max"])],
            "boulders": [{"c": w(*b["center"]), "r": b["diameter"] / 2 * IN} for b in c["boulders_in"]],
            "pad": {"c": w(*pad["center"]), "r": [pad["outer_ring_radius"] * IN] + [r * IN for r in pad["inner_ring_radii"]]},
            "launch": w(*c["launch_mark_in"]), "route": route}


PAGE = """<!doctype html><html><head><meta charset="utf-8"><title>Drone mirror</title>
<meta name="viewport" content="width=device-width,initial-scale=1">
<style>
:root{--bg:#0f1115;--panel:#181b22;--text:#e6e8ec;--muted:#9aa3b2;--ok:#3ecf8e;--warn:#f5a524;--bad:#f2555a;--accent:#5aa9ff}
@media (prefers-color-scheme: light){:root{--bg:#f4f5f7;--panel:#fff;--text:#1b1f27;--muted:#5d6675}}
body{margin:0;background:var(--bg);color:var(--text);font:14px/1.4 system-ui,sans-serif}
header{padding:12px 16px;display:flex;gap:12px;align-items:center;flex-wrap:wrap}
h1{font-size:16px;margin:0;font-weight:600}.pill{padding:2px 8px;border-radius:999px;font-size:12px;background:var(--panel)}
main{display:grid;grid-template-columns:minmax(0,3fr) minmax(0,2fr);gap:12px;padding:0 16px 16px}
@media (max-width:900px){main{grid-template-columns:1fr}}
.card{background:var(--panel);border-radius:10px;padding:10px}.card h2{font-size:13px;margin:0 0 8px;color:var(--muted);font-weight:500}
img,canvas{width:100%;height:auto;border-radius:6px;display:block;background:#000}
dl{display:grid;grid-template-columns:auto 1fr;gap:4px 12px;margin:10px 0 0;font-variant-numeric:tabular-nums}dt{color:var(--muted)}dd{margin:0}
</style></head><body>
<header><h1>Drone mirror</h1><span id="link" class="pill">connecting...</span><span id="vio" class="pill">VIO ?</span>
<span id="arm" class="pill">disarmed</span><span class="pill" style="color:var(--muted)">read-only: this page cannot command the drone</span>
<button id="zero" title="Draw the drone's current pose as the launch mark, nose north. Changes only this page's drawing.">Put drone on launch mark</button>
<button id="unzero" title="Back to the drone's power-up frame">Use power-up pose</button></header>
<main><section class="card"><h2>What the drone would see (simulated left stereo lens)</h2><img src="/stream.mjpg" alt="simulated camera"></section>
<section class="card"><h2>Where it is in the course (top-down, north up)</h2><canvas id="map" width="560" height="560"></canvas>
<dl id="info"></dl></section></main>
<script>
const cv=document.getElementById('map'),g=cv.getContext('2d');let C=null,trail=[];
fetch('/course').then(r=>r.json()).then(j=>C=j);
const S=()=>cv.width/(2*C.half+0.4),X=x=>(x+C.half+0.2)*S(),Y=y=>(C.half+0.2-y)*S();
function draw(s){if(!C)return;const css=getComputedStyle(document.body);g.clearRect(0,0,cv.width,cv.height);
g.fillStyle='#22262f';g.fillRect(X(-C.half),Y(C.half),2*C.half*S(),2*C.half*S());
g.strokeStyle='#4a5160';g.lineWidth=2;g.strokeRect(X(-C.half),Y(C.half),2*C.half*S(),2*C.half*S());
const t=C.tunnel;g.fillStyle='#3a3320';g.fillRect(X(t[0][0]),Y(t[1][1]),(t[1][0]-t[0][0])*S(),(t[1][1]-t[0][1])*S());
for(const r of C.pad.r){g.strokeStyle='#d0b030';g.lineWidth=1.5;g.beginPath();g.arc(X(C.pad.c[0]),Y(C.pad.c[1]),r*S(),0,7);g.stroke();}
g.fillStyle='#6b5a44';for(const b of C.boulders){g.beginPath();g.arc(X(b.c[0]),Y(b.c[1]),b.r*S(),0,7);g.fill();}
g.strokeStyle='#c9ced8';g.lineCap='butt';for(const w of C.walls){g.lineWidth=Math.max(2,w.t*S());g.beginPath();g.moveTo(X(w.a[0]),Y(w.a[1]));g.lineTo(X(w.b[0]),Y(w.b[1]));g.stroke();}
if(C.route.length){g.strokeStyle='rgba(90,169,255,.45)';g.lineWidth=1.5;g.setLineDash([4,4]);g.beginPath();C.route.forEach((p,i)=>i?g.lineTo(X(p[0]),Y(p[1])):g.moveTo(X(p[0]),Y(p[1])));g.stroke();g.setLineDash([]);}
g.fillStyle='#e0e0e0';g.fillRect(X(C.launch[0])-3,Y(C.launch[1])-3,6,6);
if(trail.length>1){g.strokeStyle='rgba(62,207,142,.8)';g.lineWidth=2;g.beginPath();trail.forEach((p,i)=>i?g.lineTo(X(p[0]),Y(p[1])):g.moveTo(X(p[0]),Y(p[1])));g.stroke();}
if(s&&s.setpoint){g.fillStyle='#3ecf8e';g.beginPath();g.arc(X(s.setpoint[0]),Y(s.setpoint[1]),4,0,7);g.fill();}
if(s&&s.world){const [x,y]=s.world,a=s.yaw_enu;g.save();g.translate(X(x),Y(y));g.rotate(-a);g.fillStyle=s.vo_state==2?'#f2555a':'#5aa9ff';
g.beginPath();g.moveTo(12,0);g.lineTo(-8,7);g.lineTo(-4,0);g.lineTo(-8,-7);g.closePath();g.fill();g.restore();}}
document.getElementById('zero').onclick=()=>{trail=[];fetch('/zero',{method:'POST'});};
document.getElementById('unzero').onclick=()=>{trail=[];fetch('/unzero',{method:'POST'});};
function pill(id,text,color){const e=document.getElementById(id);e.textContent=text;e.style.color=color;}
async function poll(){try{const s=await (await fetch('/status')).json();
pill('link',s.connected?`relay ${s.relay} · ${s.relay_hz.toFixed(0)} Hz`:`no relay (${s.relay})`,s.connected&&s.frame_age<1?'var(--ok)':'var(--bad)');
pill('vio',s.vo_state==1?'VIO tracking':s.vo_state==2?'VIO LOST':'VIO unknown',s.vo_state==1?'var(--ok)':s.vo_state==2?'var(--bad)':'var(--warn)');
pill('arm',s.armed?(s.offboard?'ARMED · OFFBOARD':'ARMED'):'disarmed',s.armed?'var(--warn)':'var(--muted)');
if(s.world){const l=trail[trail.length-1];if(!l||Math.hypot(l[0]-s.world[0],l[1]-s.world[1])>0.02)trail.push(s.world.slice(0,2));if(trail.length>600)trail.shift();}
const f=v=>v==null?'—':v.map(x=>x.toFixed(2)).join(', ');
document.getElementById('info').innerHTML=`<dt>course xyz (m)</dt><dd>${f(s.world)}</dd><dt>heading</dt><dd>${s.world?(90-s.yaw_enu*57.2958).toFixed(0)+'° from north':'—'}</dd>
<dt>odom xyz (m)</dt><dd>${f(s.odom)}</dd><dt>setpoint</dt><dd>${f(s.setpoint)}</dd><dt>mode</dt><dd>${s.nav||'—'}</dd>
<dt>placed by</dt><dd>${s.placement}</dd><dt>camera mount</dt><dd>${s.cam_source}</dd><dt>sim camera</dt><dd>${s.camera_hz.toFixed(0)} Hz</dd><dt>frame age</dt><dd>${s.frame_age==null?'—':s.frame_age.toFixed(2)+' s'}</dd>`;
draw(s);}catch(e){pill('link','mirror offline','var(--bad)');}setTimeout(poll,200);}poll();
</script></body></html>"""


# ------------------------------------------------------------------------------------ main
def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--drone", required=True, help="host running navless_nav mirror_relay (e.g. 192.168.55.1)")
    ap.add_argument("--port", type=int, default=8775, help="relay TCP port")
    ap.add_argument("--start-bearing", type=float, default=0.0, help="nose bearing at power-up, deg clockwise from course north")
    ap.add_argument("--start-height", type=float, default=0.07, help="base_link height above the floor at power-up, m")
    ap.add_argument("--launch", type=float, nargs=2, default=LAUNCH, metavar=("X", "Y"), help="power-up point, world m")
    ap.add_argument("--route", default="gazebo_sim/course/route_centreline_window.json",
                    help="route to draw on the map, as navless_nav's route_file ('' for none)")
    ap.add_argument("--http-port", type=int, default=8090)
    ap.add_argument("--listen", default="127.0.0.1",
                    help="address for the page; 0.0.0.0 shares live drone telemetry with the network")
    ap.add_argument("--gui", action="store_true", help="also open the Gazebo window")
    ap.add_argument("--color", action="store_true", help="colour camera view (the real stereo pair is mono)")
    ap.add_argument("--no-sim", action="store_true", help="use an already running mirror world")
    a = ap.parse_args()

    env = dict(os.environ, GZ_PARTITION="dhm", GZ_IP="127.0.0.1",
               GZ_SIM_RESOURCE_PATH=os.path.join(SIM, "models") + ":" + os.environ.get("GZ_SIM_RESOURCE_PATH", ""))
    os.environ.update({k: env[k] for k in ("GZ_PARTITION", "GZ_IP", "GZ_SIM_RESOURCE_PATH")})
    procs = []
    if not a.no_sim:
        log = open("/tmp/dh_mirror_gz.log", "w")
        procs.append(subprocess.Popen(["gz", "sim", "-s", "-r", "--headless-rendering",
                                       os.path.join(HERE, "mirror_world.sdf")],
                                      env=env, stdout=log, stderr=subprocess.STDOUT, start_new_session=True))
        if a.gui:
            procs.append(subprocess.Popen(["gz", "sim", "-g"], env=env, stdout=log, stderr=subprocess.STDOUT,
                                          start_new_session=True))

    def stop(*_):
        for p in procs:
            try:
                os.killpg(p.pid, signal.SIGTERM)
            except OSError:
                pass
        sys.exit(0)

    signal.signal(signal.SIGINT, stop)
    signal.signal(signal.SIGTERM, stop)

    from gz.msgs10.boolean_pb2 import Boolean
    from gz.msgs10.pose_v_pb2 import Pose_V
    from gz.transport13 import Node
    node = Node()
    camera = CameraFeed(node, color=a.color)
    relay = RelayClient(a.drone, a.port)
    place = Placement(tuple(a.launch), a.start_bearing, a.start_height)

    def set_poses(poses):
        msg = Pose_V()
        for name, p, q in poses:
            m = msg.pose.add()
            m.name = name
            m.position.x, m.position.y, m.position.z = p
            m.orientation.x, m.orientation.y, m.orientation.z, m.orientation.w = q
        ok, _ = node.request("/world/mirror/set_pose_vector", msg, Pose_V, Boolean, 200)
        return ok

    t_end = time.time() + 60
    while not set_poses([("mirror_setpoint", HIDDEN, (0, 0, 0, 1))]):
        if time.time() > t_end:
            print("[mirror] the mirror world never came up; see /tmp/dh_mirror_gz.log", flush=True)
            stop()
        time.sleep(0.5)
    print("[mirror] mirror world up", flush=True)

    status = {"connected": False, "relay": f"{a.drone}:{a.port}", "relay_hz": 0.0, "camera_hz": 0.0,
              "frame_age": None, "world": None, "yaw_enu": None, "odom": None, "setpoint": None,
              "vo_state": None, "armed": False, "offboard": False, "nav": None, "cam_source": "waiting",
              "placement": "power-up pose (launch mark, nose north)"}
    status_lock = threading.Lock()
    cmap = json.dumps(course_map(a.route))

    class Handler(http.server.BaseHTTPRequestHandler):
        def log_message(self, *args):
            pass

        def _send(self, body, ctype):
            self.send_response(200)
            self.send_header("Content-Type", ctype)
            self.send_header("Cache-Control", "no-store")
            self.end_headers()
            self.wfile.write(body)

        def do_POST(self):
            f, _, _, _ = relay.latest()
            if self.path == "/zero" and f is not None:
                place.rezero(f["p"], f["q"])
                label = time.strftime("re-zeroed here at %H:%M:%S (drawn as the launch mark, nose north)")
            elif self.path == "/unzero":
                place.unzero()
                label = "power-up pose (launch mark, nose north)"
            else:
                self.send_error(409 if self.path == "/zero" else 404)
                return
            with status_lock:
                status["placement"] = label
            self._send(b"ok", "text/plain")

        def do_GET(self):
            if self.path == "/":
                self._send(PAGE.encode(), "text/html; charset=utf-8")
            elif self.path == "/course":
                self._send(cmap.encode(), "application/json")
            elif self.path == "/status":
                with status_lock:
                    self._send(json.dumps(status).encode(), "application/json")
            elif self.path == "/frame.jpg":
                _, jpg = camera.wait(-1, 0)
                self._send(jpg or b"", "image/jpeg")
            elif self.path == "/stream.mjpg":
                self.send_response(200)
                self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=frame")
                self.send_header("Cache-Control", "no-store")
                self.end_headers()
                seq = -1
                try:
                    while True:
                        seq, jpg = camera.wait(seq)
                        if jpg:
                            self.wfile.write(b"--frame\r\nContent-Type: image/jpeg\r\nContent-Length: "
                                             + str(len(jpg)).encode() + b"\r\n\r\n" + jpg + b"\r\n")
                except (BrokenPipeError, ConnectionResetError):
                    return
            else:
                self.send_error(404)

    httpd = http.server.ThreadingHTTPServer((a.listen, a.http_port), Handler)
    threading.Thread(target=httpd.serve_forever, daemon=True).start()
    print(f"[mirror] open http://localhost:{a.http_port}  (relay {a.drone}:{a.port}, start bearing "
          f"{a.start_bearing:.0f} deg, launch {tuple(a.launch)})", flush=True)

    last_n, last_seq, t_rate = 0, camera.seq, time.time()
    while True:
        f, f_at, connected, n = relay.latest()
        now = time.time()
        if now - t_rate >= 1.0:
            with status_lock:
                status["relay_hz"] = (n - last_n) / (now - t_rate)
                status["camera_hz"] = (camera.seq - last_seq) / (now - t_rate)
            last_n, last_seq, t_rate = n, camera.seq, now
        if f is not None:
            p, q = place.position(f["p"]), place.orientation(f["q"])
            cam = f.get("cam")
            cp, cq = (cam["p"], cam["q"]) if cam else DEFAULT_CAM
            eye = tuple(a_ + b_ for a_, b_ in zip(cp, qrot(cq, LEFT_EYE)))     # in base_link
            cam_p = tuple(a_ + b_ for a_, b_ in zip(p, qrot(q, eye)))
            sp = place.position(f["setpoint"]) if f.get("setpoint") else None
            set_poses([("mirror_drone", p, q), ("mirror_cam", cam_p, unit(qmul(q, cq))),
                       ("mirror_setpoint", sp or HIDDEN, (0, 0, 0, 1))])
            with status_lock:
                status.update(connected=connected, frame_age=round(now - f_at, 2),
                              world=[round(v, 3) for v in p], yaw_enu=round(yaw_of(q), 4),
                              odom=f["p"], setpoint=None if sp is None else [round(v, 3) for v in sp],
                              vo_state=f.get("vo_state"), armed=f.get("armed"), offboard=f.get("offboard"),
                              nav=f.get("nav"),
                              cam_source="drone's static TF" if cam else "typical mount (not measured)")
        else:
            with status_lock:
                status["connected"] = connected
        time.sleep(1 / 30)


if __name__ == "__main__":
    main()
