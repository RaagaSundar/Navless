"""Course geometry for clearance checks, read straight from the Gazebo collision model.

Frames
  WORLD : Gazebo world, metres, x = east, y = north, z = up, origin = floor centre.
  MAP   : course JSON, inches from the SW corner of the net.  world = map * 0.0254 - 3.048
  NED   : PX4 local frame in this sim (EKF origin = world origin): n = y, e = x, d = -z.
"""
import math
import os
import xml.etree.ElementTree as ET

import numpy as np

IN = 0.0254
HALF = 3.048
HERE = os.path.dirname(os.path.abspath(__file__))
COURSE_SDF = os.path.join(HERE, "..", "models", "dh_course", "model.sdf")


def map_to_world(x_in, y_in):
    return (x_in * IN - HALF, y_in * IN - HALF)


def world_to_ned(p):
    p = np.asarray(p, float)
    return np.stack([p[..., 1], p[..., 0], -p[..., 2]], axis=-1)


def ned_to_world(p):
    p = np.asarray(p, float)
    return np.stack([p[..., 1], p[..., 0], -p[..., 2]], axis=-1)


def rpy_matrix(r, p, y):
    cr, sr, cp, sp, cy, sy = math.cos(r), math.sin(r), math.cos(p), math.sin(p), math.cos(y), math.sin(y)
    return np.array([[cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
                     [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
                     [-sp, cp * sr, cp * cr]])


class Obstacles:
    """Every collision primitive of the course model (boxes and ellipsoids)."""

    def __init__(self, sdf_path=COURSE_SDF):
        self.names, self.kind, self.c, self.R, self.size = [], [], [], [], []
        for col in ET.parse(sdf_path).getroot().iter("collision"):
            pose = [float(v) for v in (col.findtext("pose") or "0 0 0 0 0 0").split()]
            g = col.find("geometry")
            if g.find("box") is not None:
                k, s = "box", [float(v) / 2 for v in g.find("box").findtext("size").split()]
            elif g.find("ellipsoid") is not None:
                k, s = "ell", [float(v) for v in g.find("ellipsoid").findtext("radii").split()]
            else:
                continue
            self.names.append(col.get("name"))
            self.kind.append(k)
            self.c.append(pose[:3])
            self.R.append(rpy_matrix(*pose[3:]))
            self.size.append(s)
        self.c, self.R, self.size = np.array(self.c), np.array(self.R), np.array(self.size)
        self.is_box = np.array([k == "box" for k in self.kind])

    def sdf(self, pts):
        """Signed distance from each point (N,3) to each obstacle -> (N, M)."""
        pts = np.atleast_2d(pts)
        d = pts[:, None, :] - self.c[None, :, :]                 # N, M, 3
        local = np.einsum("mji,nmj->nmi", self.R, d)              # R^T d
        q = np.abs(local) - self.size[None]
        box = np.linalg.norm(np.maximum(q, 0), axis=-1) + np.minimum(q.max(-1), 0)
        u = local / self.size[None]
        k = np.maximum(np.linalg.norm(u, axis=-1), 1e-9)
        grad = np.linalg.norm(u / self.size[None], axis=-1) / k
        ell = (k - 1) / np.maximum(grad, 1e-9)                    # first-order distance to the ellipsoid
        return np.where(self.is_box[None], box, ell)


class RayCaster:
    """Rays vs. the course's collision primitives (boxes incl. the net, ellipsoid boulders) and the floor z = 0."""

    def __init__(self, obstacles=None):
        o = obstacles or Obstacles()
        self.c, self.R, self.h, self.box = o.c, o.R, o.size, o.is_box

    def cast_many(self, origins, directions):
        """Distance along each unit ray to the first hit (inf if none). origins, directions: (K, 3) world."""
        o = np.atleast_2d(np.asarray(origins, float))
        d = np.atleast_2d(np.asarray(directions, float))
        best = np.where(d[:, 2] < -1e-9, -o[:, 2] / np.where(d[:, 2] < -1e-9, d[:, 2], -1.0), np.inf)   # floor
        ol = np.einsum("mji,kmj->kmi", self.R, o[:, None, :] - self.c[None])   # R^T (o - c), (K, M, 3)
        dl = np.einsum("mji,kj->kmi", self.R, d)
        dl = np.where(np.abs(dl) < 1e-12, 1e-12, dl)
        # boxes: slab test, hits in front of the origin only
        t1, t2 = (-self.h[None] - ol) / dl, (self.h[None] - ol) / dl
        tmin, tmax = np.minimum(t1, t2).max(-1), np.maximum(t1, t2).min(-1)
        hit = self.box[None] & (tmax >= np.maximum(tmin, 0)) & (tmin > 0)
        best = np.minimum(best, np.where(hit, tmin, np.inf).min(-1))
        # ellipsoids: unit sphere after scaling
        os_, ds = ol / self.h[None], dl / self.h[None]
        a = (ds * ds).sum(-1)
        b = 2 * (os_ * ds).sum(-1)
        cc = (os_ * os_).sum(-1) - 1
        disc = b * b - 4 * a * cc
        t = (-b - np.sqrt(np.maximum(disc, 0))) / (2 * a)
        ok = ~self.box[None] & (disc >= 0) & (t > 0)
        return np.minimum(best, np.where(ok, t, np.inf).min(-1))

    def cast(self, origin, direction):
        return float(self.cast_many([origin], [direction])[0])


# Quad stand-in parts in body FLU (x fwd, y left, z up): (x, y, z, radius). Props on the measured rotor geometry.
DRONE_PARTS = np.array([
    [0.08, -0.160, 0.03, 0.0775], [-0.15, 0.140, 0.03, 0.0775],     # props
    [0.08, 0.160, 0.03, 0.0775], [-0.15, -0.140, 0.03, 0.0775],
    [0.07, 0.0, 0.01, 0.055], [0.0, 0.0, 0.01, 0.055], [-0.07, 0.0, 0.01, 0.055],  # body
    [0.10, -0.12, -0.05, 0.02], [0.10, 0.12, -0.05, 0.02],         # feet
    [-0.10, -0.11, -0.05, 0.02], [-0.10, 0.11, -0.05, 0.02],
])


def drone_points(pos_world, yaw_ned):
    """World positions + radii of the drone's parts. yaw_ned: PX4 yaw (0 = nose north, cw positive)."""
    pos_world = np.atleast_2d(pos_world)
    yaw_ned = np.atleast_1d(yaw_ned)
    psi = math.pi / 2 - yaw_ned                                    # ENU heading of body x
    c, s = np.cos(psi), np.sin(psi)
    ox, oy, oz, r = DRONE_PARTS.T
    wx = pos_world[:, None, 0] + c[:, None] * ox - s[:, None] * oy
    wy = pos_world[:, None, 1] + s[:, None] * ox + c[:, None] * oy
    wz = pos_world[:, None, 2] + oz[None, :]
    return np.stack([wx, wy, wz], -1), r


def clearance(obs, pos_world, yaw_ned, with_ground=False):
    """Min clearance (m) between the drone and the course for each pose; also the nearest obstacle name."""
    pts, r = drone_points(pos_world, yaw_ned)
    N, P, _ = pts.shape
    d = obs.sdf(pts.reshape(-1, 3)).reshape(N, P, -1) - r[None, :, None]
    if with_ground:
        g = (pts[..., 2] - r[None, :])[..., None]
        d = np.concatenate([d, g], -1)
    flat = d.reshape(N, -1)
    j = flat.argmin(1)
    m = flat.min(1)
    names = obs.names + (["ground"] if with_ground else [])
    return m, [names[k % d.shape[2]] for k in j]
