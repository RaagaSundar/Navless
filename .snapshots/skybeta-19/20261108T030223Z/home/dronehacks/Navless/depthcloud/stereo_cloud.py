#!/usr/bin/env python3
"""
stereo_cloud.py — point cloud + camera feed from the skybeta stack's EXISTING camera topics.

The skybeta stack owns the OAK-D-S2 and publishes only the raw stereo pair (no depth node):
    /oak/left/image_raw, /oak/right/image_raw   bgr8 1024x640 @ 30 Hz
    /oak/left/camera_info, /oak/right/camera_info
This node never touches the camera. It subscribes to those topics, computes stereo depth itself
(OpenCV SGBM on the CPU, at reduced resolution and rate), and outputs:

    /team/stereo/points           sensor_msgs/PointCloud2  (left optical frame)
    /team/stereo/depth            sensor_msgs/Image 32FC1, metres (rectified left camera)
    http://<jetson>:8081/         browser view: rectified left image | colourised depth, and below it
                                  an interactive 3D point-cloud viewer (drag to orbit, scroll to zoom)
    http://<jetson>:8081/cloud.ply   download the latest cloud (open in CloudCompare / MeshLab)
    http://<jetson>:8081/cloud.bin   compact cloud the 3D viewer polls (int16 mm xyz + uint8 grey)

The 3D viewer is rendered by the laptop's browser (WebGL, no internet needed); the Jetson only
packs points when a viewer actually asks for them.

It runs on the same CPU as the flight stack, so it is throttled by default (5 Hz, half resolution)
and shows its own compute time in the HUD. Treat it as a dev/debug tool: check the HUD's ms figure,
and stop it (or lower `rate_hz`) before flying.

Run INSIDE the stack container:   host/run.sh shell
    python3 /root/team_ws/src/dronehacks/stereo_cloud.py
    python3 /root/team_ws/src/dronehacks/stereo_cloud.py --ros-args -p rate_hz:=3.0 -p scale:=0.5
"""
import math
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import cv2
import numpy as np


# ----------------------------------------------------------------------------------------------
# Stereo core (no ROS) — the offline test exercises this
# ----------------------------------------------------------------------------------------------

class StereoDepth:
    """Rectify a stereo pair, run SGBM, return depth and 3D points in the LEFT optical frame."""

    def __init__(self, num_disp=64, block=5, min_depth=0.3, max_depth=5.0, texture_filter=False):
        self.num_disp = int(math.ceil(num_disp / 16) * 16)
        self.min_depth, self.max_depth = min_depth, max_depth
        self.texture_filter = texture_filter
        self.sgbm = cv2.StereoSGBM_create(
            minDisparity=0, numDisparities=self.num_disp, blockSize=block,
            P1=8 * block * block, P2=32 * block * block, disp12MaxDiff=1,
            uniquenessRatio=10, speckleWindowSize=100, speckleRange=2,
            mode=cv2.STEREO_SGBM_MODE_SGBM_3WAY)
        self._key = None

    def setup(self, K1, D1, K2, D2, size, R, T):
        """K/D per camera at working resolution; R,T map LEFT-camera points into the RIGHT camera:
        X_r = R X_l + T (for a side-by-side pair, T ≈ [-baseline, 0, 0])."""
        key = (K1.tobytes(), D1.tobytes(), K2.tobytes(), D2.tobytes(), size, R.tobytes(), T.tobytes())
        if key == self._key:
            return
        R1, R2, P1, P2, Q, _, _ = cv2.stereoRectify(K1, D1, K2, D2, size, R, T,
                                                    flags=cv2.CALIB_ZERO_DISPARITY, alpha=0)
        self.map_l = cv2.initUndistortRectifyMap(K1, D1, R1, P1, size, cv2.CV_16SC2)
        self.map_r = cv2.initUndistortRectifyMap(K2, D2, R2, P2, size, cv2.CV_16SC2)
        self.Q, self.R1 = Q, R1
        self.fx, self.baseline = P1[0, 0], float(np.linalg.norm(T))
        self._key = key

    def compute(self, gray_l, gray_r):
        rl = cv2.remap(gray_l, *self.map_l, cv2.INTER_LINEAR)
        rr = cv2.remap(gray_r, *self.map_r, cv2.INTER_LINEAR)
        disp = self.sgbm.compute(rl, rr).astype(np.float32) / 16.0
        xyz_rect = cv2.reprojectImageTo3D(disp, self.Q)               # rectified-left frame
        z = xyz_rect[..., 2]
        valid = (disp > 0.5) & np.isfinite(z) & (z > self.min_depth) & (z < self.max_depth)
        if self.texture_filter:
            # SGBM can propagate a rim disparity through an untextured opening.
            # Such pixels are not independent depth measurements for wall fitting.
            intensity = rl.astype(np.float32)
            mean = cv2.boxFilter(intensity, -1, (5, 5))
            variance = cv2.boxFilter(intensity*intensity, -1, (5, 5)) - mean*mean
            valid &= variance > 30.0
        depth = np.where(valid, z, 0.0).astype(np.float32)
        pts = xyz_rect[valid] @ self.R1                                # rect -> left optical (R1^T)
        return rl, depth, valid, pts.astype(np.float32)


def colourise(depth, valid, dmin, dmax):
    d = np.clip((depth - dmin) / (dmax - dmin), 0.0, 1.0)
    out = cv2.applyColorMap((255 * (1.0 - d)).astype(np.uint8), cv2.COLORMAP_TURBO)
    out[~valid] = 0
    return out


def ply_bytes(pts):
    header = (f"ply\nformat binary_little_endian 1.0\nelement vertex {len(pts)}\n"
              "property float x\nproperty float y\nproperty float z\nend_header\n").encode("ascii")
    return header + np.ascontiguousarray(pts, dtype="<f4").tobytes()


def view_bytes(pts, grey, max_pts):
    """Compact cloud for the browser viewer: uint32 n, n*3 int16 xyz (mm, left optical frame), n uint8 grey."""
    k = max(1, int(math.ceil(len(pts) / max(1, max_pts))))
    p, g = pts[::k], grey[::k]
    q = np.clip(np.round(p * 1000.0), -32767, 32767).astype("<i2")
    return (np.array([len(p)], "<u4").tobytes() + np.ascontiguousarray(q).tobytes()
            + np.ascontiguousarray(g, dtype=np.uint8).tobytes())


VIEWER_HTML = r"""<!doctype html><html><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1"><title>Stereo cloud</title>
<style>
body{margin:0;background:#111;color:#ddd;font:14px sans-serif}
#stream{width:100%;display:block}
#wrap{position:relative;height:70vh;border-top:1px solid #333}
#gl{width:100%;height:100%;display:block;touch-action:none;cursor:grab}
#bar{position:absolute;top:8px;left:8px;background:rgba(0,0,0,.65);padding:6px 8px;border-radius:4px;line-height:1.9}
#bar button{background:#333;color:#ddd;border:1px solid #555;border-radius:3px;padding:2px 8px;margin-right:4px;cursor:pointer}
#help{color:#999;font-size:12px}
a{color:#9cf}
</style></head><body>
<img id="stream" src="/stream" alt="camera and depth stream">
<div id="wrap"><canvas id="gl"></canvas>
<div id="bar"><span id="info">waiting for cloud...</span><br>
<button id="col">colour: image</button><button id="pause">pause</button><button id="reset">camera view</button><button id="top">top view</button>
<a href="/cloud.ply">download .ply</a><br>
<span id="help">drag: orbit &middot; shift/right-drag: pan &middot; scroll: zoom &middot; white lines: camera</span></div></div>
<script>
"use strict";
const cv = document.getElementById('gl'), gl = cv.getContext('webgl');
const info = document.getElementById('info');
if (!gl) { info.textContent = 'WebGL not available in this browser'; throw new Error('no webgl'); }

function shader(type, src) {
  const s = gl.createShader(type); gl.shaderSource(s, src); gl.compileShader(s);
  if (!gl.getShaderParameter(s, gl.COMPILE_STATUS)) throw new Error(gl.getShaderInfoLog(s));
  return s;
}
const prog = gl.createProgram();
// p: left-optical-frame millimetres (x right, y down, z forward) -> GL metres (x right, y up, -z forward)
gl.attachShader(prog, shader(gl.VERTEX_SHADER, `
attribute vec3 p; attribute float g;
uniform mat4 m; uniform float mode, psize; varying vec3 c;
vec3 turbo(float t){ t = clamp(t, 0., 1.); return clamp(vec3(
  0.1357+t*(4.5974-t*(42.3277-t*(130.5887-t*(150.5666-t*58.1375)))),
  0.0914+t*(2.1856+t*(4.8052-t*(14.0195-t*(4.2109+t*2.7747)))),
  0.1067+t*(12.5925-t*(60.1097-t*(109.0745-t*(88.5066-t*26.8183))))), 0., 1.); }
void main(){
  vec3 P = p * vec3(0.001, -0.001, -0.001);
  vec4 q = m * vec4(P, 1.);
  gl_Position = q;
  gl_PointSize = clamp(psize / max(q.w, 0.05), 1., 8.);
  c = mode < 0.5 ? vec3(g) : turbo(1. - (-P.z - 0.3) / 4.7);
}`));
gl.attachShader(prog, shader(gl.FRAGMENT_SHADER,
  'precision mediump float; varying vec3 c; void main(){ gl_FragColor = vec4(c, 1.); }'));
gl.linkProgram(prog);
if (!gl.getProgramParameter(prog, gl.LINK_STATUS)) throw new Error(gl.getProgramInfoLog(prog));
const pL = gl.getAttribLocation(prog, 'p'), gL = gl.getAttribLocation(prog, 'g');
const mL = gl.getUniformLocation(prog, 'm'), modeL = gl.getUniformLocation(prog, 'mode'),
      sizeL = gl.getUniformLocation(prog, 'psize');

// camera frustum (optical mm): approx OAK-D-S2 FOV, 0.3 m deep
const fx = 231, fy = 144, fz = 300;
const cs = [[-fx,-fy,fz],[fx,-fy,fz],[fx,fy,fz],[-fx,fy,fz]], fr = [];
for (let i = 0; i < 4; i++) { fr.push(0,0,0, ...cs[i], ...cs[i], ...cs[(i+1)%4]); }
const fb = gl.createBuffer(); gl.bindBuffer(gl.ARRAY_BUFFER, fb);
gl.bufferData(gl.ARRAY_BUFFER, new Float32Array(fr), gl.STATIC_DRAW);

const pb = gl.createBuffer(), gb = gl.createBuffer();
let n = 0, mode = 0, paused = false;

// --- orbit camera ---
const sub=(a,b)=>[a[0]-b[0],a[1]-b[1],a[2]-b[2]], dot=(a,b)=>a[0]*b[0]+a[1]*b[1]+a[2]*b[2];
const cross=(a,b)=>[a[1]*b[2]-a[2]*b[1],a[2]*b[0]-a[0]*b[2],a[0]*b[1]-a[1]*b[0]];
const norm=a=>{const l=Math.hypot(...a)||1;return [a[0]/l,a[1]/l,a[2]/l];};
function lookAt(e,t,u){const z=norm(sub(e,t)),x=norm(cross(u,z)),y=cross(z,x);
  return [x[0],y[0],z[0],0, x[1],y[1],z[1],0, x[2],y[2],z[2],0, -dot(x,e),-dot(y,e),-dot(z,e),1];}
function persp(f,a,nr,fa){const t=1/Math.tan(f/2);
  return [t/a,0,0,0, 0,t,0,0, 0,0,(fa+nr)/(nr-fa),-1, 0,0,2*fa*nr/(nr-fa),0];}
function mul(a,b){const r=new Array(16);for(let i=0;i<4;i++)for(let j=0;j<4;j++){let s=0;
  for(let k=0;k<4;k++)s+=a[k*4+j]*b[i*4+k];r[i*4+j]=s;}return r;}
let cam;
function camView(){ cam = {t:[0,0,-1.5], yaw:0, pitch:0, dist:1.5}; }
function topView(){ cam = {t:[0,0,-2], yaw:0, pitch:1.45, dist:5}; }
camView();
function eye(){ const cp=Math.cos(cam.pitch);
  return [cam.t[0]+cam.dist*cp*Math.sin(cam.yaw), cam.t[1]+cam.dist*Math.sin(cam.pitch), cam.t[2]+cam.dist*cp*Math.cos(cam.yaw)]; }

let drag = null;
cv.addEventListener('contextmenu', e => e.preventDefault());
cv.addEventListener('pointerdown', e => { drag = {x:e.clientX, y:e.clientY, pan: e.button === 2 || e.shiftKey};
  cv.setPointerCapture(e.pointerId); cv.style.cursor = 'grabbing'; });
cv.addEventListener('pointerup', () => { drag = null; cv.style.cursor = 'grab'; });
cv.addEventListener('pointermove', e => {
  if (!drag) return;
  const dx = e.clientX - drag.x, dy = e.clientY - drag.y; drag.x = e.clientX; drag.y = e.clientY;
  if (drag.pan) {
    const sy=Math.sin(cam.yaw), cy=Math.cos(cam.yaw), sp=Math.sin(cam.pitch), cp=Math.cos(cam.pitch);
    const r=[cy,0,-sy], u=[-sp*sy,cp,-sp*cy], k=cam.dist*0.0015;
    for (let i = 0; i < 3; i++) cam.t[i] += (-dx*r[i] + dy*u[i]) * k;
  } else {
    cam.yaw -= dx * 0.006;
    cam.pitch = Math.max(-1.55, Math.min(1.55, cam.pitch + dy * 0.006));
  }
});
cv.addEventListener('wheel', e => { e.preventDefault();
  cam.dist = Math.max(0.1, Math.min(30, cam.dist * Math.exp(e.deltaY * 0.001))); }, {passive:false});

document.getElementById('col').onclick = e => { mode = 1 - mode; e.target.textContent = 'colour: ' + (mode ? 'depth' : 'image'); };
document.getElementById('pause').onclick = e => { paused = !paused; e.target.textContent = paused ? 'resume' : 'pause'; };
document.getElementById('reset').onclick = camView;
document.getElementById('top').onclick = topView;

// --- data ---
function load(buf) {
  const cnt = new DataView(buf).getUint32(0, true);
  gl.bindBuffer(gl.ARRAY_BUFFER, pb); gl.bufferData(gl.ARRAY_BUFFER, new Int16Array(buf, 4, cnt * 3), gl.DYNAMIC_DRAW);
  gl.bindBuffer(gl.ARRAY_BUFFER, gb); gl.bufferData(gl.ARRAY_BUFFER, new Uint8Array(buf, 4 + cnt * 6, cnt), gl.DYNAMIC_DRAW);
  n = cnt; info.textContent = cnt.toLocaleString() + ' pts' + (paused ? ' (paused)' : '');
}
async function poll() {
  if (!paused) {
    try {
      const r = await fetch('/cloud.bin', {cache: 'no-store'});
      if (r.ok) load(await r.arrayBuffer()); else info.textContent = 'waiting for cloud...';
    } catch (e) { info.textContent = 'connection lost, retrying...'; }
  }
  setTimeout(poll, 400);
}
poll();

function frame() {
  const dpr = window.devicePixelRatio || 1, w = Math.round(cv.clientWidth * dpr), h = Math.round(cv.clientHeight * dpr);
  if (cv.width !== w || cv.height !== h) { cv.width = w; cv.height = h; }
  gl.viewport(0, 0, w, h);
  gl.clearColor(0.07, 0.07, 0.07, 1); gl.clear(gl.COLOR_BUFFER_BIT | gl.DEPTH_BUFFER_BIT);
  gl.enable(gl.DEPTH_TEST);
  gl.useProgram(prog);
  gl.uniformMatrix4fv(mL, false, new Float32Array(mul(persp(1.0, w / Math.max(h, 1), 0.02, 100), lookAt(eye(), cam.t, [0,1,0]))));
  gl.uniform1f(sizeL, 2.5 * dpr);
  gl.enableVertexAttribArray(pL);
  if (n) {
    gl.bindBuffer(gl.ARRAY_BUFFER, pb); gl.vertexAttribPointer(pL, 3, gl.SHORT, false, 0, 0);
    gl.bindBuffer(gl.ARRAY_BUFFER, gb); gl.enableVertexAttribArray(gL); gl.vertexAttribPointer(gL, 1, gl.UNSIGNED_BYTE, true, 0, 0);
    gl.uniform1f(modeL, mode); gl.drawArrays(gl.POINTS, 0, n);
  }
  gl.bindBuffer(gl.ARRAY_BUFFER, fb); gl.vertexAttribPointer(pL, 3, gl.FLOAT, false, 0, 0);
  gl.disableVertexAttribArray(gL); gl.vertexAttrib1f(gL, 0.9); gl.uniform1f(modeL, 0);
  gl.drawArrays(gl.LINES, 0, fr.length / 3);
  requestAnimationFrame(frame);
}
requestAnimationFrame(frame);
</script></body></html>
""".encode("utf-8")


def hud(img, lines):
    y = 8
    for t in lines:
        (tw, th), _ = cv2.getTextSize(t, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 1)
        cv2.rectangle(img, (6, y - 2), (12 + tw, y + th + 6), (0, 0, 0), -1)
        cv2.putText(img, t, (9, y + th + 1), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1, cv2.LINE_AA)
        y += th + 10


def image_to_gray(msg):
    h, w, enc, step = msg.height, msg.width, msg.encoding.lower(), msg.step
    buf = np.frombuffer(msg.data, np.uint8)
    if enc == "mono8":
        return buf.reshape(h, step)[:, :w].copy()
    if enc in ("bgr8", "rgb8"):
        a = buf.reshape(h, step)[:, : w * 3].reshape(h, w, 3)
        return cv2.cvtColor(a, cv2.COLOR_BGR2GRAY if enc == "bgr8" else cv2.COLOR_RGB2GRAY)
    raise ValueError(f"unsupported encoding {msg.encoding}")


class WebServer:
    """Page with MJPEG stream + 3D viewer at /, latest cloud at /cloud.ply and /cloud.bin.
    Cloud files are packed on request, so they cost nothing when no one is looking."""

    def __init__(self, port, max_view_points=40000):
        self.jpg, self.pts, self.grey = None, None, None
        self.max_view_points = max_view_points
        self.cond = threading.Condition()
        srv = self

        class H(BaseHTTPRequestHandler):
            def _send(self, ctype, body, extra=()):
                self.send_response(200)
                self.send_header("Content-Type", ctype)
                self.send_header("Content-Length", str(len(body)))
                for k, v in extra:
                    self.send_header(k, v)
                self.end_headers()
                self.wfile.write(body)

            def do_GET(self):
                if self.path == "/":
                    self._send("text/html; charset=utf-8", VIEWER_HTML)
                elif self.path in ("/cloud.ply", "/cloud.bin"):
                    with srv.cond:
                        pts, grey = srv.pts, srv.grey
                    if pts is None:
                        self.send_error(503, "no cloud yet")
                    elif self.path == "/cloud.ply":
                        self._send("application/octet-stream", ply_bytes(pts),
                                   [("Content-Disposition", f"attachment; filename=cloud_{int(time.time())}.ply")])
                    else:
                        self._send("application/octet-stream", view_bytes(pts, grey, srv.max_view_points),
                                   [("Cache-Control", "no-store")])
                elif self.path == "/stream":
                    self.send_response(200)
                    self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=frame")
                    self.send_header("Cache-Control", "no-cache")
                    self.end_headers()
                    last = None
                    try:
                        while True:
                            with srv.cond:
                                srv.cond.wait_for(lambda: srv.jpg is not last, timeout=2.0)
                                jpg = srv.jpg
                            if jpg is None or jpg is last:
                                continue
                            last = jpg
                            self.wfile.write(b"--frame\r\nContent-Type: image/jpeg\r\nContent-Length: "
                                             + str(len(jpg)).encode() + b"\r\n\r\n" + jpg + b"\r\n")
                    except (BrokenPipeError, ConnectionResetError):
                        pass
                else:
                    self.send_error(404)

            def log_message(self, *a):
                pass

        self.httpd = ThreadingHTTPServer(("0.0.0.0", port), H)
        self.httpd.daemon_threads = True
        threading.Thread(target=self.httpd.serve_forever, daemon=True).start()

    def push(self, jpg, pts, grey):
        with self.cond:
            self.jpg, self.pts, self.grey = jpg, pts, grey
            self.cond.notify_all()


# ----------------------------------------------------------------------------------------------
# ROS 2 node
# ----------------------------------------------------------------------------------------------

def main():
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from rclpy.time import Time
    from rclpy.duration import Duration
    from sensor_msgs.msg import Image, CameraInfo, PointCloud2
    from sensor_msgs_py import point_cloud2
    import message_filters
    import tf2_ros

    def quat_to_rot(x, y, z, w):
        return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                         [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                         [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])

    class StereoCloud(Node):
        def __init__(self):
            super().__init__("team_stereo_cloud")
            p = self.declare_parameter
            self.rate_hz = p("rate_hz", 5.0).value
            self.scale = p("scale", 0.5).value              # 0.5 -> 512x320 matching
            self.cloud_stride = int(p("cloud_stride", 2).value)
            self.fallback_baseline = p("fallback_baseline_m", 0.075).value
            self.require_calibration_tf = p("require_calibration_tf", False).value
            self.stereo = StereoDepth(num_disp=int(p("num_disparities", 64).value),
                                      block=int(p("block_size", 5).value),
                                      min_depth=p("min_depth", 0.3).value,
                                      max_depth=p("max_depth", 5.0).value,
                                      texture_filter=p("texture_filter", False).value)
            self.web = WebServer(int(p("http_port", 8081).value),
                                 int(p("max_view_points", 40000).value))
            self.pub_cloud = self.create_publisher(PointCloud2, "/team/stereo/points", 2)
            self.pub_depth = self.create_publisher(Image, "/team/stereo/depth", 2)
            self.tf_buffer = tf2_ros.Buffer()
            self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
            self.last = 0.0
            self.ms = 0.0
            self.warned_tf = False

            subs = [message_filters.Subscriber(self, Image, "/oak/left/image_raw", qos_profile=qos_profile_sensor_data),
                    message_filters.Subscriber(self, Image, "/oak/right/image_raw", qos_profile=qos_profile_sensor_data),
                    message_filters.Subscriber(self, CameraInfo, "/oak/left/camera_info", qos_profile=qos_profile_sensor_data),
                    message_filters.Subscriber(self, CameraInfo, "/oak/right/camera_info", qos_profile=qos_profile_sensor_data)]
            self.sync = message_filters.ApproximateTimeSynchronizer(subs, queue_size=6, slop=0.01)
            self.sync.registerCallback(self.on_pair)
            self.get_logger().info("Waiting for /oak/left + /oak/right ... view at http://<jetson>:8081")

        def left_to_right(self, lf, rf):
            """R,T with X_right = R X_left + T, from TF (the stack's calibration), else a fallback."""
            try:
                tf = self.tf_buffer.lookup_transform(rf, lf, Time(), Duration(seconds=0.05))
                q, t = tf.transform.rotation, tf.transform.translation
                return quat_to_rot(q.x, q.y, q.z, q.w), np.array([[t.x], [t.y], [t.z]])
            except tf2_ros.TransformException:
                if self.require_calibration_tf:
                    if not self.warned_tf:
                        self.get_logger().warn(f"Missing stereo calibration TF {lf}->{rf}; no depth published")
                        self.warned_tf = True
                    return None
                if not self.warned_tf:
                    self.get_logger().warn(f"No TF {lf}->{rf}; assuming parallel cameras, "
                                           f"{self.fallback_baseline} m baseline")
                    self.warned_tf = True
                return np.eye(3), np.array([[-self.fallback_baseline], [0.0], [0.0]])

        def on_pair(self, l_msg, r_msg, l_info, r_info):
            now = time.monotonic()
            if now - self.last < 1.0 / self.rate_hz:
                return
            self.last = now
            t0 = time.perf_counter()

            gl, gr = image_to_gray(l_msg), image_to_gray(r_msg)
            s = self.scale
            if s != 1.0:
                gl = cv2.resize(gl, None, fx=s, fy=s, interpolation=cv2.INTER_AREA)
                gr = cv2.resize(gr, None, fx=s, fy=s, interpolation=cv2.INTER_AREA)
            S = np.diag([s, s, 1.0])
            K1 = S @ np.array(l_info.k, float).reshape(3, 3)
            K2 = S @ np.array(r_info.k, float).reshape(3, 3)
            D1 = np.array(l_info.d, float) if len(l_info.d) else np.zeros(5)
            D2 = np.array(r_info.d, float) if len(r_info.d) else np.zeros(5)
            extrinsics = self.left_to_right(l_msg.header.frame_id, r_msg.header.frame_id)
            if extrinsics is None:
                return
            R, T = extrinsics
            size = (gl.shape[1], gl.shape[0])
            self.stereo.setup(K1, D1, K2, D2, size, R, T)
            rect, depth, valid, pts = self.stereo.compute(gl, gr)
            pts = pts[:: self.cloud_stride]
            grey = rect[valid][:: self.cloud_stride]       # same mask order as pts -> per-point intensity

            # ROS outputs
            header = l_msg.header
            self.pub_cloud.publish(point_cloud2.create_cloud_xyz32(header, pts))
            dmsg = Image()
            dmsg.header = header
            dmsg.height, dmsg.width = depth.shape
            dmsg.encoding, dmsg.step = "32FC1", depth.shape[1] * 4
            dmsg.data = depth.tobytes()
            self.pub_depth.publish(dmsg)

            # browser view
            self.ms = 0.8 * self.ms + 0.2 * (time.perf_counter() - t0) * 1000.0
            left = cv2.cvtColor(rect, cv2.COLOR_GRAY2BGR)
            right = colourise(depth, valid, self.stereo.min_depth, self.stereo.max_depth)
            med = float(np.median(depth[valid])) if valid.any() else float("nan")
            hud(left, [f"valid depth {100 * valid.mean():4.0f}%   median {med:4.2f} m",
                       f"{len(pts):,} pts   compute {self.ms:5.0f} ms @ {self.rate_hz:.0f} Hz",
                       f"baseline {self.stereo.baseline * 100:.1f} cm   fx {self.stereo.fx:.0f} px"])
            frame = np.hstack([left, right])
            ok, jpg = cv2.imencode(".jpg", frame, [cv2.IMWRITE_JPEG_QUALITY, 70])
            if ok:
                self.web.push(jpg.tobytes(), pts, grey)

    rclpy.init()
    node = StereoCloud()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    if rclpy.ok():                  # Ctrl+C already shuts the context down in Humble
        rclpy.shutdown()


if __name__ == "__main__":
    main()
