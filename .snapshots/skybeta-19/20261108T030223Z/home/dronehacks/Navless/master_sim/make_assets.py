#!/usr/bin/env python3
"""Generate the master sim's Gazebo assets from gazebo_sim/ (which stays untouched).

  python3 make_assets.py OUT_DIR

OUT_DIR/dh_quad_stereo.sdf
    gazebo_sim/models/dh_quad/model.sdf plus the OAK-D-S2 stereo pair in base_link, at the
    model's existing `stereo_camera` box: 7.5 cm baseline, 1024x640 like the skybeta stack's
    /oak/{left,right}/image_raw. The frame ids are the optical frames sim_core.py puts in TF.
OUT_DIR/models/dh_course/
    The course with a speckle texture on every face. The official mesh is flat grey, and stereo
    matching finds nothing on a flat colour. The real walls' texture is unknown, so the depth
    quality in this sim is a guess either way. Put OUT_DIR/models first on GZ_SIM_RESOURCE_PATH.
"""
import math
import os
import random
import shutil
import struct
import sys
import zlib

HERE = os.path.dirname(os.path.abspath(__file__))
SIM = os.path.join(os.path.dirname(HERE), "gazebo_sim", "models")

# OAK-D-S2 stereo pair (OV9282). ponytail: HFOV and mount are datasheet/CAD guesses; replace
# with the drone's /oak/left/camera_info and base_link -> camera TF once someone dumps them.
CAM_X, CAM_Z, BASELINE = 0.115, 0.01, 0.075
# The stack streams 30 Hz; stereo_cloud.py only uses 5 (rate_hz), and rendering is what slows the
# sim on a laptop, so render at 5. DH_CAMERA_HZ overrides.
WIDTH, HEIGHT, HFOV_RAD = 1024, 640, math.radians(80.0)
RATE_HZ = float(os.environ.get("DH_CAMERA_HZ", 5))

TEXTURE_TILE_M = 0.6          # texture repeats every this many metres along a wall


def camera_sensor(side, y):
    return f"""
      <sensor name="oak_{side}" type="camera">
        <pose>{CAM_X} {y} {CAM_Z} 0 0 0</pose>
        <always_on>1</always_on>
        <update_rate>{RATE_HZ}</update_rate>
        <topic>/oak/{side}/image_raw</topic>
        <gz_frame_id>oak_{side}_camera_optical_frame</gz_frame_id>
        <camera>
          <camera_info_topic>/oak/{side}/camera_info</camera_info_topic>
          <horizontal_fov>{HFOV_RAD:.6f}</horizontal_fov>
          <image><width>{WIDTH}</width><height>{HEIGHT}</height><format>R8G8B8</format></image>
          <clip><near>0.1</near><far>12</far></clip>
          <noise><type>gaussian</type><mean>0</mean><stddev>0.007</stddev></noise>
        </camera>
      </sensor>"""


def make_quad(out):
    src = open(os.path.join(SIM, "dh_quad", "model.sdf")).read()
    i = src.index("</link>")              # the first link is base_link
    assert src.rfind('<link name="base_link">', 0, i) != -1, "dh_quad: first link is not base_link"
    sdf = src[:i] + camera_sensor("left", BASELINE / 2) + camera_sensor("right", -BASELINE / 2) + "\n    " + src[i:]
    path = os.path.join(out, "dh_quad_stereo.sdf")
    open(path, "w").write(sdf)
    return path


def write_png_gray(path, rows):
    raw = b"".join(b"\x00" + bytes(r) for r in rows)
    chunk = lambda tag, data: (struct.pack(">I", len(data)) + tag + data
                               + struct.pack(">I", zlib.crc32(tag + data) & 0xFFFFFFFF))
    h, w = len(rows), len(rows[0])
    png = (b"\x89PNG\r\n\x1a\n" + chunk(b"IHDR", struct.pack(">IIBBBBB", w, h, 8, 0, 0, 0, 0))
           + chunk(b"IDAT", zlib.compress(raw, 9)) + chunk(b"IEND", b""))
    open(path, "wb").write(png)


def speckle(n=256, seed=7):
    """Tileable blotchy grey noise, 0.45-1.0: enough texture for SGBM at 0.3-5 m."""
    rnd = random.Random(seed)
    g = [[rnd.random() for _ in range(n)] for _ in range(n)]
    for _ in range(2):                    # wrap-around box blur: blobs, still tileable
        g = [[(g[y][x] + g[y][(x + 1) % n] + g[y][x - 1] + g[(y + 1) % n][x] + g[y - 1][x]) / 5
              for x in range(n)] for y in range(n)]
    lo = min(map(min, g))
    hi = max(map(max, g))
    return [[int(255 * (0.45 + 0.55 * (v - lo) / (hi - lo))) for v in row] for row in g]


def make_course(out):
    src_dir = os.path.join(SIM, "dh_course")
    dst_dir = os.path.join(out, "models", "dh_course")
    os.makedirs(os.path.join(dst_dir, "meshes"), exist_ok=True)
    for name in ("model.config", "model.sdf"):
        shutil.copy(os.path.join(src_dir, name), dst_dir)
    write_png_gray(os.path.join(dst_dir, "meshes", "speckle.png"), speckle())

    mtl = []
    for line in open(os.path.join(src_dir, "meshes", "course.mtl")):
        mtl.append(line)
        if line.startswith("newmtl"):
            mtl.append("map_Kd speckle.png\n")
    open(os.path.join(dst_dir, "meshes", "course.mtl"), "w").writelines(mtl)

    # Planar UVs per face, projected on the plane the face mostly lies in.
    verts, out_lines, vt = [], [], []
    for line in open(os.path.join(src_dir, "meshes", "course_visual.obj")):
        if line.startswith("v "):
            verts.append(tuple(map(float, line.split()[1:4])))
        if not line.startswith("f "):
            out_lines.append(line)
            continue
        refs = [r.split("/") for r in line.split()[1:]]
        p = [verts[int(r[0]) - 1] for r in refs]
        u = [p[1][k] - p[0][k] for k in range(3)]
        w = [p[2][k] - p[0][k] for k in range(3)]
        nrm = [abs(u[1] * w[2] - u[2] * w[1]), abs(u[2] * w[0] - u[0] * w[2]), abs(u[0] * w[1] - u[1] * w[0])]
        a, b = [(1, 2), (0, 2), (0, 1)][nrm.index(max(nrm))]
        face = []
        for r, q in zip(refs, p):
            vt.append(f"vt {q[a] / TEXTURE_TILE_M:.5f} {q[b] / TEXTURE_TILE_M:.5f}\n")
            vn = r[2] if len(r) > 2 else ""
            face.append(f"{r[0]}/{len(vt)}/{vn}" if vn else f"{r[0]}/{len(vt)}")
        out_lines.append("f " + " ".join(face) + "\n")
    with open(os.path.join(dst_dir, "meshes", "course_visual.obj"), "w") as f:
        f.write("# gazebo_sim course_visual.obj + planar UVs (master_sim/make_assets.py)\n")
        f.writelines(vt)
        f.writelines(out_lines)
    return dst_dir


def main():
    out = os.path.abspath(sys.argv[1] if len(sys.argv) > 1 else os.path.join(HERE, "build"))
    os.makedirs(out, exist_ok=True)
    print(make_quad(out))
    print(make_course(out))


if __name__ == "__main__":
    main()
