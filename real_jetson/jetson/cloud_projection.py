"""Measured-pose stereo projection for the live course planner (no commands).

Coordinates are optical cloud -> odometry child/body via timestamped calibrated
TF -> odom via measured full attitude -> corrected course frame. No assumed
optical mount, fixed heading, simulator truth or latest-TF fallback is used.
"""
from collections import deque
from dataclasses import dataclass
import math
import numpy as np


def stamp_s(header):
    stamp = header.stamp
    if not 0 <= stamp.nanosec < 1_000_000_000:
        raise ValueError('invalid source nanoseconds')
    value = float(stamp.sec) + float(stamp.nanosec) * 1e-9
    if not math.isfinite(value) or value <= 0:
        raise ValueError('missing/invalid source timestamp')
    return value


def rotation(q):
    q = np.asarray(q, dtype=float)
    if q.shape != (4,) or not np.all(np.isfinite(q)) or abs(float(q @ q)-1) > .02:
        raise ValueError('invalid rotation quaternion')
    q = q / np.linalg.norm(q)
    x, y, z, w = q
    return np.array([[1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
                     [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
                     [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)]])


def xyz(v):
    value = np.array([v.x, v.y, v.z], dtype=float)
    if not np.all(np.isfinite(value)):
        raise ValueError('nonfinite translation')
    return value


def xyzw(q):
    return (q.x, q.y, q.z, q.w)


@dataclass(frozen=True)
class MeasuredPose:
    stamp: float
    parent: str
    child: str
    position: np.ndarray
    attitude: np.ndarray
    sigma_z: float


class PoseHistory:
    def __init__(self):
        self.samples = deque(maxlen=120)

    def add(self, msg):
        stamp = stamp_s(msg.header)
        if msg.header.frame_id != 'odom' or not msg.child_frame_id:
            raise ValueError('odometry must name odom parent and body child')
        cov = np.asarray(msg.pose.covariance, dtype=float)
        if cov.shape != (36,) or not np.all(np.isfinite(cov)) or np.any(cov[[0,7,14,21,28,35]] < 0):
            raise ValueError('invalid pose covariance')
        if all(cov[i] == 1 for i in (0,7,14,21,28,35)):
            raise ValueError('pose covariance unavailable sentinel')
        # Zero covariance does not assert perfect hardware accuracy: the floor
        # model assumes at least 3 cm vertical uncertainty even for ideal tests.
        sigma = max(.03, math.sqrt(float(cov[14])))
        if sigma > .10:
            raise ValueError('pose vertical uncertainty too large')
        pose = MeasuredPose(stamp, msg.header.frame_id, msg.child_frame_id,
                            xyz(msg.pose.pose.position), rotation(xyzw(msg.pose.pose.orientation)), sigma)
        if self.samples and stamp <= self.samples[-1].stamp:
            raise ValueError('nonmonotonic odometry source time')
        self.samples.append(pose)

    def at(self, stamp, now, *, max_skew=.05, max_age=.25):
        if not -0.05 <= now-stamp <= max_age:
            raise ValueError('stale/future cloud source time')
        if not self.samples:
            raise ValueError('missing timestamped pose')
        pose = min(self.samples, key=lambda p: abs(p.stamp-stamp))
        if abs(pose.stamp-stamp) > max_skew or not -0.05 <= now-pose.stamp <= max_age:
            raise ValueError('no time-aligned fresh pose')
        return pose


def decode_xyz(msg, stride=8):
    """Read organized FLOAT32 XYZ using declared offsets and row padding."""
    fields = {f.name: f for f in msg.fields}
    if any(name not in fields or fields[name].datatype != 7 or fields[name].count != 1 for name in ('x','y','z')):
        raise ValueError('unsupported cloud: FLOAT32 XYZ fields required')
    width, height = int(msg.width), int(msg.height)
    step, row = int(msg.point_step), int(msg.row_step)
    offsets = [int(fields[name].offset) for name in ('x','y','z')]
    if (width <= 0 or height <= 0 or step <= 0 or row < width*step
            or any(o < 0 or o+4 > step for o in offsets) or len(set(offsets)) != 3
            or len(msg.data) != row*height):
        raise ValueError('unsupported cloud layout/buffer')
    dtype = np.dtype('>f4' if msg.is_bigendian else '<f4')
    buf = bytes(msg.data)
    return np.column_stack([np.ndarray((height,width), dtype=dtype, buffer=buf,
                            offset=o, strides=(row,step)).reshape(-1)[::max(1,stride)] for o in offsets]).astype(float)


def project_cloud(msg, pose, camera_to_body, *, now, launch_xy, bearing_deg,
                  start_height, zero_offset=(0.,0.,0.), correction=(0.,0.,0.),
                  floor_z=0., stride=8, max_depth=2.8):
    """Return (course obstacle points, camera origin) or reject prerequisites.

    The course map's actual floor is z=0. Reject its uncertainty band after full
    projection, not camera-relative 'down' pixels. Floor/model uncertainty 3 cm,
    stereo depth 2 cm, pose 3 sigma and attitude 0.01 rad are conservative assumed
    errors, not measured qualification. Low objects inside that band remain
    unknown; this insertion never clears the prior or claims free floor space.
    """
    stamp = stamp_s(msg.header)
    if not -0.05 <= now-stamp <= .25 or abs(pose.stamp-stamp) > .05:
        raise ValueError('stale cloud or misaligned pose')
    if not msg.header.frame_id or camera_to_body is None:
        raise ValueError('missing calibrated camera-to-body transform')
    if (camera_to_body.header.frame_id != pose.child
            or camera_to_body.child_frame_id != msg.header.frame_id):
        raise ValueError('camera transform frame mismatch')
    tf_stamp = camera_to_body.header.stamp.sec + camera_to_body.header.stamp.nanosec*1e-9
    # Static TF has stamp zero. Dynamic TF must match the requested measurement.
    if tf_stamp and abs(tf_stamp-stamp) > .05:
        raise ValueError('stale camera transform')
    t = camera_to_body.transform
    mount_r, mount_t = rotation(xyzw(t.rotation)), xyz(t.translation)
    pts = decode_xyz(msg, stride)
    pts = pts[np.all(np.isfinite(pts), axis=1) & (pts[:,2] >= .30) & (pts[:,2] <= max_depth)]
    odom = (pts @ mount_r.T + mount_t) @ pose.attitude.T + pose.position
    origin = mount_t @ pose.attitude.T + pose.position
    # Correct the measured points exactly as the route's position is corrected.
    def course(points):
        points = np.asarray(points) - np.asarray(zero_offset)
        dx,dy,a = correction
        ca,sa = math.cos(a), math.sin(a)
        points = points - np.array([dx,dy,0.])
        points = points @ np.array([[ca,-sa,0.],[sa,ca,0.],[0.,0.,1.]])
        b = math.radians(bearing_deg)
        sb,cb = math.sin(b),math.cos(b)
        return points @ np.array([[sb,cb,0.],[-cb,sb,0.],[0.,0.,1.]]) + np.array([*launch_xy,start_height])
    world, origin = course(odom), course(origin)
    floor_band = .03 + .02 + 3*pose.sigma_z + max_depth*.01
    world = world[world[:,2] > floor_z + floor_band]
    return world, origin
