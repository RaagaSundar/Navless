#!/usr/bin/env python3
"""Build the Gazebo course model from the official OBJ (visual) and the course map (collisions).

  python3 tools/build_course_model.py /path/to/dronehacks-course-v3.obj [course/course_v3.json]

Writes models/dh_course/:
  meshes/course_visual.obj  official mesh minus the net, with flat normals added (the official file has none)
  meshes/course.mtl         official materials
  model.sdf                 visual mesh + collision primitives built from the map:
                            walls/barrier as boxes, the window wall split around its opening,
                            the tube as a ring of boxes on its 58 x 72 in oval, boulders as half-ellipsoids,
                            the net as thin boxes.

Gazebo world frame: x = east, y = north, z = up, metres, origin = floor centre (official OBJ frame).
Map frame (course JSON): inches from the SW corner of the net. world = map * 0.0254 - 3.048.
"""
import json
import math
import os
import shutil
import sys

IN = 0.0254
HALF = 3.048
HERE = os.path.dirname(os.path.abspath(__file__))
MODEL = os.path.join(HERE, "..", "models", "dh_course")


def w(xy_in):
    return (xy_in[0] * IN - HALF, xy_in[1] * IN - HALF)


# ------------------------------------------------------------------ visual mesh
def build_visual(obj_path):
    os.makedirs(os.path.join(MODEL, "meshes"), exist_ok=True)
    verts, out_faces, cur, keep = [], [], None, True
    lines = []
    with open(obj_path, "r", errors="replace") as f:
        for line in f:
            if line.startswith("v "):
                p = line.split()
                verts.append((float(p[1]), float(p[2]), float(p[3])))
            lines.append(line)
    normals = []
    body = []
    for line in lines:
        if line.startswith("g "):
            cur = line[2:].strip()
            keep = not cur.startswith("Net ")
            if keep:
                body.append(line)
        elif line.startswith("usemtl") and keep:
            body.append(line)
        elif line.startswith("f ") and keep:
            idx = [int(t.split("/")[0]) for t in line.split()[1:]]
            idx = [i if i > 0 else len(verts) + 1 + i for i in idx]
            a, b, c = (verts[idx[0] - 1], verts[idx[1] - 1], verts[idx[2] - 1])
            u = (b[0] - a[0], b[1] - a[1], b[2] - a[2])
            v = (c[0] - a[0], c[1] - a[1], c[2] - a[2])
            n = (u[1] * v[2] - u[2] * v[1], u[2] * v[0] - u[0] * v[2], u[0] * v[1] - u[1] * v[0])
            L = math.sqrt(n[0] ** 2 + n[1] ** 2 + n[2] ** 2) or 1.0
            normals.append((n[0] / L, n[1] / L, n[2] / L))
            k = len(normals)
            body.append("f " + " ".join(f"{i}//{k}" for i in idx) + "\n")
    with open(os.path.join(MODEL, "meshes", "course_visual.obj"), "w") as o:
        o.write("# official DroneHacks course v3 mesh without the net, flat normals added\nmtllib course.mtl\n")
        o.writelines(f"v {x:.6f} {y:.6f} {z:.6f}\n" for x, y, z in verts)
        o.writelines(f"vn {x:.5f} {y:.5f} {z:.5f}\n" for x, y, z in normals)
        o.writelines(body)
    mtl = os.path.splitext(obj_path)[0] + ".mtl"
    if os.path.exists(mtl):
        shutil.copyfile(mtl, os.path.join(MODEL, "meshes", "course.mtl"))
    return len(normals)


# ------------------------------------------------------------------ collision primitives
def box(name, cx, cy, cz, sx, sy, sz, yaw=0.0, roll=0.0):
    return (f'      <collision name="{name}"><pose>{cx:.4f} {cy:.4f} {cz:.4f} {roll:.5f} 0 {yaw:.5f}</pose>'
            f'<geometry><box><size>{sx:.4f} {sy:.4f} {sz:.4f}</size></box></geometry></collision>\n')


def wall_boxes(wall):
    """A straight wall as one box, or several around a window opening."""
    p, q = w(wall["start_in"]), w(wall["end_in"])
    L = math.hypot(q[0] - p[0], q[1] - p[1])
    ux, uy = (q[0] - p[0]) / L, (q[1] - p[1]) / L
    yaw = math.atan2(uy, ux)
    t, h = wall["thickness_in"] * IN, wall["height_in"] * IN
    name = wall["name"]

    def seg(s0, s1, z0, z1, tag):
        m = (s0 + s1) / 2
        return box(f"{name}_{tag}", p[0] + ux * m, p[1] + uy * m, (z0 + z1) / 2, s1 - s0, t, z1 - z0, yaw)

    if "window" not in wall:
        return seg(0, L, 0, h, "wall")
    win = wall["window"]
    c = w(win["center_in"])
    sc = (c[0] - p[0]) * ux + (c[1] - p[1]) * uy
    hw = win["width_in"] * IN / 2
    zb, zt = win["bottom_above_floor_in"] * IN, win["top_above_floor_in"] * IN
    return (seg(0, sc - hw, 0, h, "a") + seg(sc + hw, L, 0, h, "b") +
            seg(sc - hw, sc + hw, 0, zb, "below") + seg(sc - hw, sc + hw, zt, h, "above"))


def tube_boxes(t, n=24, wall=0.012):
    fp = t["footprint_in"]
    cx = ((fp["x_min"] + fp["x_max"]) / 2) * IN - HALF
    cy = ((fp["y_min"] + fp["y_max"]) / 2) * IN - HALF
    length = (fp["y_max"] - fp["y_min"]) * IN
    z0, z1 = t["interior_z_in"]
    cz = (z0 + z1) / 2 * IN
    a = t["interior_oval_in"]["width"] / 2 * IN   # half width (x)
    b = t["interior_oval_in"]["height"] / 2 * IN  # half height (z)
    out = ""
    for k in range(n):
        th0, th1 = 2 * math.pi * k / n, 2 * math.pi * (k + 1) / n
        p0 = (a * math.cos(th0), b * math.sin(th0))
        p1 = (a * math.cos(th1), b * math.sin(th1))
        mx, mz = (p0[0] + p1[0]) / 2, (p0[1] + p1[1]) / 2
        chord = math.hypot(p1[0] - p0[0], p1[1] - p0[1])
        ang = math.atan2(p1[1] - p0[1], p1[0] - p0[0])  # chord angle in the x-z plane
        # outward normal of the chord, push the box outward by half its thickness
        nx, nz = math.sin(ang), -math.cos(ang)
        if nx * mx + nz * mz < 0:
            nx, nz = -nx, -nz
        bx, bz = cx + mx + nx * wall / 2, cz + mz + nz * wall / 2
        # box: x = chord (rotated about world y by -ang), y = tube length, z = shell thickness
        out += (f'      <collision name="tube_{k:02d}"><pose>{bx:.4f} {cy:.4f} {bz:.4f} 0 {-ang:.5f} 0</pose>'
                f'<geometry><box><size>{chord * 1.02:.4f} {length:.4f} {wall:.4f}</size></box></geometry></collision>\n')
    # cradles under the tube
    for i, yy in enumerate((108.5, 138.0, 167.5)):
        out += box(f"tube_cradle_{i + 1}", cx, yy * IN - HALF, 10.9 / 2 * IN, 40 * IN, 1 * IN, 10.9 * IN)
    return out


def boulders(bs):
    out = ""
    for i, b in enumerate(bs):
        x, y = w(b["center"])
        r, h = b["diameter"] / 2 * IN, b["height"] * IN
        out += (f'      <collision name="boulder_{i + 1}"><pose>{x:.4f} {y:.4f} 0 0 0 0</pose>'
                f'<geometry><ellipsoid><radii>{r:.4f} {r:.4f} {h:.4f}</radii></ellipsoid></geometry></collision>\n')
    return out


NET = """      <collision name="net_north"><pose>0 3.058 1.524 0 0 0</pose><geometry><box><size>6.2 0.02 3.048</size></box></geometry></collision>
      <collision name="net_south"><pose>0 -3.058 1.524 0 0 0</pose><geometry><box><size>6.2 0.02 3.048</size></box></geometry></collision>
      <collision name="net_east"><pose>3.058 0 1.524 0 0 0</pose><geometry><box><size>0.02 6.2 3.048</size></box></geometry></collision>
      <collision name="net_west"><pose>-3.058 0 1.524 0 0 0</pose><geometry><box><size>0.02 6.2 3.048</size></box></geometry></collision>
      <collision name="net_roof"><pose>0 0 3.058 0 0 0</pose><geometry><box><size>6.2 6.2 0.02</size></box></geometry></collision>
      <visual name="net_north_v"><pose>0 3.058 1.524 0 0 0</pose><geometry><box><size>6.2 0.005 3.048</size></box></geometry><transparency>0.9</transparency><cast_shadows>false</cast_shadows><material><diffuse>0.1 0.1 0.1 1</diffuse></material></visual>
      <visual name="net_east_v"><pose>3.058 0 1.524 0 0 0</pose><geometry><box><size>0.005 6.2 3.048</size></box></geometry><transparency>0.9</transparency><cast_shadows>false</cast_shadows><material><diffuse>0.1 0.1 0.1 1</diffuse></material></visual>
      <visual name="net_west_v"><pose>-3.058 0 1.524 0 0 0</pose><geometry><box><size>0.005 6.2 3.048</size></box></geometry><transparency>0.9</transparency><cast_shadows>false</cast_shadows><material><diffuse>0.1 0.1 0.1 1</diffuse></material></visual>
"""


def build_sdf(course):
    cols = "".join(wall_boxes(wl) for wl in course["walls"])
    cols += tube_boxes(course["tunnel"]) + boulders(course["boulders_in"]) + NET
    sdf = f"""<?xml version="1.0"?>
<!-- GENERATED by tools/build_course_model.py. DroneHacks 2026 course v3.
     World frame: x = east, y = north, z = up, origin = floor centre.
     Visual = official OBJ (net removed). Collisions = primitives from course/course_v3.json. -->
<sdf version="1.9">
  <model name="dh_course">
    <static>true</static>
    <link name="course">
      <visual name="course"><geometry><mesh><uri>model://dh_course/meshes/course_visual.obj</uri></mesh></geometry></visual>
{cols}    </link>
  </model>
</sdf>
"""
    with open(os.path.join(MODEL, "model.sdf"), "w") as o:
        o.write(sdf)
    return cols.count("<collision")


if __name__ == "__main__":
    obj = sys.argv[1]
    cj = sys.argv[2] if len(sys.argv) > 2 else os.path.join(HERE, "..", "course", "course_v3.json")
    n = build_visual(obj)
    k = build_sdf(json.load(open(cj)))
    print(f"visual mesh: {n} faces with normals; model.sdf: {k} collision primitives")
