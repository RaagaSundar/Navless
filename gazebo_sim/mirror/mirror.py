#!/usr/bin/env python3
"""Live dashboard: the drone in the CAD course, with what its camera and side ToF would see.

Two sources, either or both:
  --run 127.0.0.1:8776   a live sim run: fly_navless.py --relay 8776 flies the REAL navless_nav node
                         through the course in Gazebo + PX4 SITL; its pose drives the mirror.
  --drone <ip>           the real drone: navless_nav's read-only mirror_relay (TCP 8775) sends its
                         VIO pose, side ToF ranges and left-camera thumbnails. Without --run, its
                         pose drives the mirror; with --run, it supplies the real sensor panels.

  python3 gazebo_sim/mirror/mirror.py --drone 192.168.68.88                    # real drone only
  python3 gazebo_sim/mirror/mirror.py --run 127.0.0.1:8776 --drone 192.168.68.88   # sim run + real sensors
  python3 gazebo_sim/mirror/mirror.py --run 127.0.0.1:8776                     # sim run only
  then open http://localhost:8090   (--gui also opens the Gazebo window)

Nothing here can command anything: both sources are read-only streams.

Frames: odom = ENU at power-up (x = nose, y = left, z = up, origin = base_link at power-up), placed in
the course the way navless_nav places its route (launch mark + start bearing + start height). The page's
"Put drone on launch mark" re-zeroes that placement for benches and rooms that are not the course.
"""
import argparse
import base64
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
sys.path.insert(0, SIM)
from navless_nav.route import odom_to_course, resolve_route_path  # noqa: E402

try:                                    # simulated side ToF, from the course collision model
    from dh.tof import side_ranges_q    # noqa: E402  (pos_world, body FLU -> world quaternion xyzw)
except Exception:                       # noqa: BLE001 - optional; the page says it is missing
    side_ranges_q = None

IN = 0.0254
HALF = 3.048
LAUNCH = (-2.5146, 0.9144)
# base_link -> camera mount when the drone's static TF never arrives: the hardware checklist's
# typical mount (CAMERA_X 0.10, Z -0.02, PITCH +0.35 rad = ~20 deg nose-down), not a measured one.
DEFAULT_CAM = ((0.10, 0.0, -0.02), (0.0, 0.17408, 0.0, 0.98473))
# The mount TF (oak_parent_frame) is the camera body; cuVSLAM's view is the left eye, half the
# 7.5 cm baseline to the left (the stack's TF: base_link -> left optical = mount + (0, 0.038, 0)).
LEFT_EYE = (0.0, 0.0375, 0.0)
TOF_MOUNT_Y = 0.075                     # tube/tof_driver.py: sensor faces at body y = +/-0.075 m
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


def roll_pitch_of(q):
    x, y, z, w = q
    roll = math.atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y))
    pitch = math.asin(max(-1.0, min(1.0, 2 * (w * y - z * x))))
    return roll, pitch


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
    """Connects to a relay (navless_nav mirror_relay, or fly_navless.py --relay) and keeps the latest
    pose frame and camera thumbnail. Reconnects forever."""

    def __init__(self, name, host, port):
        self.name, self.host, self.port = name, host, port
        self.lock = threading.Lock()
        self.img_cond = threading.Condition()
        self.frame, self.frame_at, self.connected, self.count = None, None, False, 0
        self.jpeg, self.img_seq, self.img_at = None, 0, None
        threading.Thread(target=self._run, daemon=True).start()

    def _run(self):
        while True:
            try:
                with socket.create_connection((self.host, self.port), timeout=3) as s:
                    s.settimeout(5)
                    with self.lock:
                        self.connected = True
                    print(f"[dash] {self.name}: connected to {self.host}:{self.port}", flush=True)
                    for line in s.makefile("r"):
                        try:
                            f = json.loads(line)
                        except ValueError:
                            continue
                        if "img" in f:
                            jpg = base64.b64decode(f["img"])
                            with self.img_cond:
                                self.jpeg, self.img_at = jpg, time.time()
                                self.img_seq += 1
                                self.img_cond.notify_all()
                            continue
                        with self.lock:
                            self.frame, self.frame_at = f, time.time()
                            self.count += 1
            except OSError as e:
                with self.lock:
                    was = self.connected
                    self.connected = False
                if was:
                    print(f"[dash] {self.name}: connection lost ({e}); retrying", flush=True)
            time.sleep(1.0)

    def latest(self):
        with self.lock:
            return self.frame, self.frame_at, self.connected, self.count

    def fresh(self, max_age=1.0):
        f, at, connected, _ = self.latest()
        return f if connected and f is not None and time.time() - at < max_age else None

    def wait_image(self, after, timeout=1.0):
        with self.img_cond:
            self.img_cond.wait_for(lambda: self.img_seq != after, timeout)
            return self.img_seq, self.jpeg


class CameraFeed:
    """Latest simulated camera frame as JPEG. Grey by default: the OAK-D-S2's stereo pair is
    monochrome (OV9282), and cuVSLAM sees mono8."""

    def __init__(self, node, topic="/mirror/camera", color=False):
        from gz.msgs10.image_pb2 import Image
        from PIL import Image as PILImage
        self.PILImage, self.color = PILImage, color
        self.cond = threading.Condition()
        self.jpeg, self.seq = None, 0
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
            self.jpeg = buf.getvalue()
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
            print(f"[dash] route not drawn ({e})", flush=True)
    return {"half": HALF, "walls": walls,
            "tunnel": [w(tf["x_min"], tf["y_min"]), w(tf["x_max"], tf["y_max"])],
            "boulders": [{"c": w(*b["center"]), "r": b["diameter"] / 2 * IN} for b in c["boulders_in"]],
            "pad": {"c": w(*pad["center"]), "r": [pad["outer_ring_radius"] * IN] + [r * IN for r in pad["inner_ring_radii"]]},
            "launch": w(*c["launch_mark_in"]), "route": route}


PAGE = r"""<!doctype html><html><head><meta charset="utf-8"><title>Navless live</title>
<meta name="viewport" content="width=device-width,initial-scale=1">
<style>
:root{--bg:#0f1115;--panel:#181b22;--line:#262b35;--text:#e6e8ec;--muted:#9aa3b2;--ok:#3ecf8e;--warn:#f5a524;--bad:#f2555a;--sim:#5aa9ff;--real:#ff9f43}
@media (prefers-color-scheme: light){:root{--bg:#f4f5f7;--panel:#fff;--line:#e3e6ea;--text:#1b1f27;--muted:#5d6675}}
*{box-sizing:border-box}body{margin:0;background:var(--bg);color:var(--text);font:14px/1.4 system-ui,sans-serif}
header{padding:10px 16px;display:flex;gap:8px;align-items:center;flex-wrap:wrap}
h1{font-size:16px;margin:0 8px 0 0;font-weight:650}.pill{padding:2px 9px;border-radius:999px;font-size:12px;background:var(--panel);border:1px solid var(--line)}
button{font:inherit;font-size:12px;padding:3px 10px;border-radius:6px;border:1px solid var(--line);background:var(--panel);color:var(--text);cursor:pointer}
main{display:grid;grid-template-columns:repeat(2,minmax(0,1fr));gap:12px;padding:0 16px 16px}
@media (max-width:900px){main{grid-template-columns:1fr}}
.card{background:var(--panel);border:1px solid var(--line);border-radius:10px;padding:10px;min-width:0}
.card h2{font-size:12px;margin:0 0 8px;color:var(--muted);font-weight:550;text-transform:uppercase;letter-spacing:.04em}
.cam{position:relative;aspect-ratio:16/10;background:#000;border-radius:6px;overflow:hidden}
.cam img{width:100%;height:100%;object-fit:contain;display:block}.cam .msg{position:absolute;inset:0;display:flex;align-items:center;justify-content:center;color:#8a93a3;text-align:center;padding:16px}
canvas{width:100%;height:auto;display:block;border-radius:6px;background:#000}
.tof{display:grid;grid-template-columns:52px 1fr 64px;gap:6px 10px;align-items:center;margin-bottom:12px;font-variant-numeric:tabular-nums}
.bar{height:12px;border-radius:6px;background:var(--line);overflow:hidden}.bar i{display:block;height:100%;border-radius:6px}
.lbl{color:var(--muted);font-size:12px}.val{text-align:right}
.node{font-size:22px;font-weight:700;letter-spacing:.02em;margin:2px 0 10px}
dl{display:grid;grid-template-columns:auto 1fr;gap:3px 12px;margin:0;font-variant-numeric:tabular-nums}dt{color:var(--muted)}dd{margin:0}
.key{display:flex;gap:12px;font-size:12px;color:var(--muted);margin-top:6px}.key b{display:inline-block;width:18px;height:3px;vertical-align:middle;margin-right:4px}
</style></head><body>
<header><h1>Navless live</h1><span id="run" class="pill">sim run: off</span><span id="drone" class="pill">drone: off</span>
<span id="vio" class="pill">VIO ?</span><span id="arm" class="pill">disarmed</span>
<button id="zero" title="Draw the current pose as the launch mark, nose north (only changes this page)">Put drone on launch mark</button>
<button id="unzero" title="Back to the power-up frame">Use power-up pose</button>
<span class="pill" style="color:var(--muted)">read-only: nothing here commands a drone</span></header>
<main>
<section class="card"><h2>Simulated camera (left stereo lens, CAD course)</h2><div class="cam"><img src="/stream.mjpg" alt="simulated camera"></div></section>
<section class="card"><h2>Real camera (drone left lens)</h2><div class="cam"><img id="realimg" src="/real.mjpg" alt="real camera"><div id="realmsg" class="msg">waiting for the drone's camera thumbnails...</div></div></section>
<section class="card"><h2>Course (top-down, north up)</h2><canvas id="map" width="620" height="620"></canvas>
<div class="key"><span><b style="background:var(--sim)"></b>simulated ToF beam</span><span><b style="background:var(--real)"></b>real ToF beam</span><span><b style="background:#3ecf8e"></b>trail / setpoint</span></div></section>
<section class="card"><h2>Mission &amp; sensors</h2><div class="node" id="node">-</div>
<div class="tof" id="tof"></div><dl id="info"></dl></section>
</main>
<script>
const cv=document.getElementById('map'),g=cv.getContext('2d');let C=null,trail=[],pose_src=null;
fetch('/course').then(r=>r.json()).then(j=>C=j);
const S=()=>cv.width/(2*C.half+0.4),X=x=>(x+C.half+0.2)*S(),Y=y=>(C.half+0.2-y)*S();
function beam(x,y,a,len,color,dash){g.save();g.strokeStyle=color;g.lineWidth=2;g.setLineDash(dash?[5,4]:[]);g.beginPath();g.moveTo(X(x),Y(y));g.lineTo(X(x+len*Math.cos(a)),Y(y+len*Math.sin(a)));g.stroke();g.restore();}
function draw(s){if(!C)return;g.clearRect(0,0,cv.width,cv.height);
g.fillStyle='#22262f';g.fillRect(X(-C.half),Y(C.half),2*C.half*S(),2*C.half*S());g.strokeStyle='#4a5160';g.lineWidth=2;g.strokeRect(X(-C.half),Y(C.half),2*C.half*S(),2*C.half*S());
const t=C.tunnel;g.fillStyle='#3a3320';g.fillRect(X(t[0][0]),Y(t[1][1]),(t[1][0]-t[0][0])*S(),(t[1][1]-t[0][1])*S());
for(const r of C.pad.r){g.strokeStyle='#d0b030';g.lineWidth=1.5;g.beginPath();g.arc(X(C.pad.c[0]),Y(C.pad.c[1]),r*S(),0,7);g.stroke();}
g.fillStyle='#6b5a44';for(const b of C.boulders){g.beginPath();g.arc(X(b.c[0]),Y(b.c[1]),b.r*S(),0,7);g.fill();}
g.strokeStyle='#c9ced8';for(const w of C.walls){g.lineWidth=Math.max(2,w.t*S());g.beginPath();g.moveTo(X(w.a[0]),Y(w.a[1]));g.lineTo(X(w.b[0]),Y(w.b[1]));g.stroke();}
if(C.route.length){g.strokeStyle='rgba(90,169,255,.35)';g.lineWidth=1.5;g.setLineDash([4,4]);g.beginPath();C.route.forEach((p,i)=>i?g.lineTo(X(p[0]),Y(p[1])):g.moveTo(X(p[0]),Y(p[1])));g.stroke();g.setLineDash([]);}
g.fillStyle='#e0e0e0';g.fillRect(X(C.launch[0])-3,Y(C.launch[1])-3,6,6);
if(trail.length>1){g.strokeStyle='rgba(62,207,142,.85)';g.lineWidth=2;g.beginPath();trail.forEach((p,i)=>i?g.lineTo(X(p[0]),Y(p[1])):g.moveTo(X(p[0]),Y(p[1])));g.stroke();}
if(!s||!s.world)return;const [x,y]=s.world,a=s.yaw_enu,L=a+Math.PI/2,R=a-Math.PI/2;
for(const [side,ang] of [['left',L],['right',R]]){const sm=s.tof_sim&&s.tof_sim[side],rl=s.tof_real&&s.tof_real[side];
 beam(x+0.075*Math.cos(ang),y+0.075*Math.sin(ang),ang,sm==null?4:sm,'#5aa9ff',false);
 if(rl!=null)beam(x+0.075*Math.cos(ang),y+0.075*Math.sin(ang),ang,rl,'#ff9f43',true);}
if(s.setpoint){g.fillStyle='#3ecf8e';g.beginPath();g.arc(X(s.setpoint[0]),Y(s.setpoint[1]),4,0,7);g.fill();}
g.save();g.translate(X(x),Y(y));g.rotate(-a);g.fillStyle=s.vo_state==2?'#f2555a':'#ffffff';g.beginPath();g.moveTo(12,0);g.lineTo(-8,7);g.lineTo(-4,0);g.lineTo(-8,-7);g.closePath();g.fill();g.restore();}
function pill(id,text,color){const e=document.getElementById(id);e.textContent=text;e.style.color=color;}
const cm=v=>v==null?'—':(v*100).toFixed(1)+' cm';
function bar(v,color){const pct=v==null?0:Math.min(100,v/2.0*100);return `<div class="bar"><i style="width:${pct}%;background:${color}"></i></div>`;}
document.getElementById('zero').onclick=()=>{trail=[];fetch('/zero',{method:'POST'});};
document.getElementById('unzero').onclick=()=>{trail=[];fetch('/unzero',{method:'POST'});};
async function poll(){try{const s=await (await fetch('/status')).json();
const R=s.run,D=s.drone;
pill('run',R.enabled?(R.connected?`sim run · ${R.hz.toFixed(0)} Hz`:'sim run: waiting'):'sim run: off',R.enabled?(R.connected?'var(--ok)':'var(--warn)'):'var(--muted)');
pill('drone',D.enabled?(D.connected?`drone ${D.host} · ${D.hz.toFixed(0)} Hz`:`drone ${D.host}: no relay`):'drone: off',D.enabled?(D.connected?'var(--ok)':'var(--bad)'):'var(--muted)');
pill('vio',s.vo_state==1?'VIO tracking':s.vo_state==2?'VIO LOST':'VIO ?',s.vo_state==1?'var(--ok)':s.vo_state==2?'var(--bad)':'var(--warn)');
pill('arm',s.armed?(s.offboard?'ARMED · OFFBOARD':'ARMED'):'disarmed',s.armed?'var(--warn)':'var(--muted)');
document.getElementById('realmsg').style.display=s.real_cam?'none':'flex';
document.getElementById('realmsg').textContent=D.enabled?(D.connected?"no camera thumbnails from the drone yet (relay img_hz)":"drone relay not connected"):"no drone connected (--drone)";
document.getElementById('node').textContent=s.node||(s.pose_src=='drone'?'drone (live pose)':'-');
if(pose_src!==s.pose_src){trail=[];pose_src=s.pose_src;}
if(s.world){const l=trail[trail.length-1];if(!l||Math.hypot(l[0]-s.world[0],l[1]-s.world[1])>0.02)trail.push(s.world.slice(0,2));if(trail.length>1500)trail.shift();}
const ts=s.tof_sim||{},tr=s.tof_real||{};
document.getElementById('tof').innerHTML=['left','right'].map(k=>`<span class="lbl">${k} sim</span>${bar(ts[k],'var(--sim)')}<span class="val">${s.tof_sim?cm(ts[k]):'n/a'}</span><span class="lbl">${k} real</span>${bar(tr[k],'var(--real)')}<span class="val">${s.tof_real?cm(tr[k]):'no sensor'}</span>`).join('');
const f=v=>v==null?'—':v.map(x=>x.toFixed(2)).join(', ');
document.getElementById('info').innerHTML=`<dt>pose from</dt><dd>${s.pose_src||'—'}</dd><dt>course xyz (m)</dt><dd>${f(s.world)}</dd>
<dt>heading</dt><dd>${s.world?(90-s.yaw_enu*57.2958).toFixed(0)+'° from north':'—'}</dd><dt>speed</dt><dd>${s.speed==null?'—':s.speed.toFixed(2)+' m/s'}</dd>
<dt>setpoint</dt><dd>${f(s.setpoint)}</dd><dt>sim ToF</dt><dd>${s.tof_sim_note}</dd><dt>real ToF</dt><dd>${s.tof_real_note}</dd>
<dt>placed by</dt><dd>${s.placement}</dd><dt>camera mount</dt><dd>${s.cam_source}</dd><dt>sim camera</dt><dd>${s.camera_hz.toFixed(0)} Hz</dd>`;
draw(s);}catch(e){pill('run','dashboard offline','var(--bad)');}setTimeout(poll,150);}poll();
</script></body></html>"""


def host_port(text, default_port):
    host, _, port = text.rpartition(":") if ":" in text else (text, "", "")
    return (host or "127.0.0.1"), int(port or default_port)


# ------------------------------------------------------------------------------------ main
def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--drone", help="real drone running navless_nav mirror_relay: HOST[:PORT] (port 8775)")
    ap.add_argument("--run", help="live sim run from fly_navless.py --relay PORT: HOST:PORT (e.g. 127.0.0.1:8776)")
    ap.add_argument("--start-bearing", type=float, default=0.0, help="nose bearing at power-up, deg clockwise from course north")
    ap.add_argument("--start-height", type=float, default=0.07, help="base_link height above the floor at power-up, m")
    ap.add_argument("--launch", type=float, nargs=2, default=LAUNCH, metavar=("X", "Y"), help="power-up point, world m")
    ap.add_argument("--route", default="gazebo_sim/course/route_centreline_window.json",
                    help="route to draw on the map, as navless_nav's route_file ('' for none)")
    ap.add_argument("--http-port", type=int, default=8090)
    ap.add_argument("--listen", default="127.0.0.1",
                    help="address for the page; 0.0.0.0 shares live drone telemetry with the network")
    ap.add_argument("--gui", action="store_true", help="also open the Gazebo window")
    ap.add_argument("--color", action="store_true", help="colour simulated camera (the real stereo pair is mono)")
    ap.add_argument("--no-sim", action="store_true", help="use an already running mirror world")
    a = ap.parse_args()
    if not (a.drone or a.run):
        ap.error("give --drone and/or --run")

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
    run = RelayClient("sim run", *host_port(a.run, 8776)) if a.run else None
    drone = RelayClient("drone", *host_port(a.drone, 8775)) if a.drone else None
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
            print("[dash] the mirror world never came up; see /tmp/dh_mirror_gz.log", flush=True)
            stop()
        time.sleep(0.5)
    print("[dash] mirror world up", flush=True)

    def src_status(c):
        if c is None:
            return {"enabled": False, "connected": False, "hz": 0.0, "host": None}
        return {"enabled": True, "connected": c.latest()[2], "hz": 0.0, "host": f"{c.host}:{c.port}"}

    status = {"run": src_status(run), "drone": src_status(drone), "pose_src": None, "node": None,
              "camera_hz": 0.0, "world": None, "yaw_enu": None, "speed": None, "setpoint": None,
              "vo_state": None, "armed": False, "offboard": False, "cam_source": "waiting", "real_cam": False,
              "tof_sim": None, "tof_real": None,
              "tof_sim_note": "course model raycast" if side_ranges_q else "dh/tof.py not available yet",
              "tof_real_note": "waiting", "placement": "power-up pose (launch mark, nose north)"}
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

        def _mjpeg(self, wait):
            self.send_response(200)
            self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=frame")
            self.send_header("Cache-Control", "no-store")
            self.end_headers()
            seq = -1
            try:
                while True:
                    seq, jpg = wait(seq)
                    if jpg:
                        self.wfile.write(b"--frame\r\nContent-Type: image/jpeg\r\nContent-Length: "
                                         + str(len(jpg)).encode() + b"\r\n\r\n" + jpg + b"\r\n")
            except (BrokenPipeError, ConnectionResetError):
                return

        def do_POST(self):
            src = run if run is not None and run.fresh() else drone
            f = src.fresh() if src is not None else None
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
            elif self.path == "/stream.mjpg":
                self._mjpeg(camera.wait)
            elif self.path == "/real.mjpg":
                if drone is None:
                    self.send_error(404)
                else:
                    self._mjpeg(drone.wait_image)
            else:
                self.send_error(404)

    httpd = http.server.ThreadingHTTPServer((a.listen, a.http_port), Handler)
    threading.Thread(target=httpd.serve_forever, daemon=True).start()
    print(f"[dash] open http://localhost:{a.http_port}  (run {a.run or '-'}, drone {a.drone or '-'})", flush=True)

    counts = {"run": 0, "drone": 0}
    last_seq, t_rate = camera.seq, time.time()
    last_world, last_t = None, None
    while True:
        now = time.time()
        if now - t_rate >= 1.0:
            with status_lock:
                for key, c in (("run", run), ("drone", drone)):
                    if c is not None:
                        n = c.latest()[3]
                        status[key]["hz"], counts[key] = (n - counts[key]) / (now - t_rate), n
                status["camera_hz"] = (camera.seq - last_seq) / (now - t_rate)
            last_seq, t_rate = camera.seq, now
        fr = run.fresh() if run is not None else None
        fd = drone.fresh() if drone is not None else None
        f, src = (fr, "sim run") if fr is not None else (fd, "drone") if fd is not None else (None, None)
        with status_lock:
            for key, c in (("run", run), ("drone", drone)):
                if c is not None:
                    status[key]["connected"] = c.latest()[2]
            status["real_cam"] = bool(drone is not None and drone.img_at and now - drone.img_at < 3)
            if fd is not None and fd.get("tof"):
                status["tof_real"], status["tof_real_note"] = fd["tof"], f"live from the drone ({fd['tof'].get('age')} s old)"
            elif fd is not None:
                status["tof_real"], status["tof_real_note"] = None, "no /tube/*/range from the drone (sensors not answering)"
            elif drone is not None:
                status["tof_real"], status["tof_real_note"] = None, "drone relay not connected"
            else:
                status["tof_real"], status["tof_real_note"] = None, "no drone connected"
        if f is not None:
            p, q = place.position(f["p"]), place.orientation(f["q"])
            # the camera mount comes from the real drone's TF when we have it, even during a sim run
            cam = f.get("cam") or (fd.get("cam") if fd else None)
            cp, cq = (cam["p"], cam["q"]) if cam else DEFAULT_CAM
            eye = tuple(a_ + b_ for a_, b_ in zip(cp, qrot(cq, LEFT_EYE)))     # in base_link
            cam_p = tuple(a_ + b_ for a_, b_ in zip(p, qrot(q, eye)))
            sp = place.position(f["setpoint"]) if f.get("setpoint") else None
            set_poses([("mirror_drone", p, q), ("mirror_cam", cam_p, unit(qmul(q, cq))),
                       ("mirror_setpoint", sp or HIDDEN, (0, 0, 0, 1))])
            tof_sim = None
            if side_ranges_q is not None:
                try:
                    r = side_ranges_q(p, q)
                    tof_sim = {k: (None if not math.isfinite(r[k][0]) else round(float(r[k][0]), 3))
                               for k in ("left", "right")}
                except Exception as e:      # noqa: BLE001
                    tof_sim = None
                    status["tof_sim_note"] = f"raycast error: {e}"
            speed = None
            if last_world is not None and now > last_t:
                speed = math.dist(p, last_world) / (now - last_t)
            last_world, last_t = p, now
            with status_lock:
                status.update(pose_src=src, node=f.get("node"), world=[round(v, 3) for v in p],
                              yaw_enu=round(yaw_of(q), 4), speed=None if speed is None else round(speed, 2),
                              setpoint=None if sp is None else [round(v, 3) for v in sp],
                              vo_state=f.get("vo_state"), armed=f.get("armed"), offboard=f.get("offboard"),
                              tof_sim=tof_sim,
                              cam_source="drone's static TF" if cam else "typical mount (not measured)")
        time.sleep(1 / 30)


if __name__ == "__main__":
    main()
