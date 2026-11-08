#!/usr/bin/env python3
"""Offline image-level stereo tube simulation, NOT PX4/Gazebo/cuVSLAM qualification.

Oracle use is confined to this sensor producer and the scoring/dynamics here.
Analytic raycasts render both cameras against the course-sized oval tube;
production OpenCV SGBM -> production two-wall fit -> production guidance run
without access to the true lateral position. SLAM is modeled as continuous
odometry with a declared lateral bias; no real cuVSLAM tracking is exercised.
"""
import argparse
import hashlib
import json
import math
import platform
from pathlib import Path
import sys

import cv2
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path[:0] = [str(ROOT / 'depthcloud'), str(ROOT / 'navless_nav')]
from stereo_cloud import StereoDepth
from navless_nav.stereo_tube import fit_tube
from navless_nav.tube_guidance import TubeGuidance

OPTICAL_TO_BODY = np.array([[0, 0, 1], [-1, 0, 0], [0, -1, 0]], float)


class StereoTubeRenderer:
    def __init__(self, seed=1, half_length=0.762):
        self.w, self.h, self.f, self.baseline = 512, 320, 305.0, 0.075
        self.half_length = half_length
        self.k = np.array([[self.f, 0, self.w/2], [0, self.f, self.h/2], [0, 0, 1.]])
        u, v = np.meshgrid(np.arange(self.w), np.arange(self.h))
        self.rays = np.stack((np.ones_like(u), -(u-self.w/2)/self.f,
                              -(v-self.h/2)/self.f), axis=-1)
        rng = np.random.default_rng(seed)
        self.texture = cv2.GaussianBlur(rng.integers(30, 230, (1600, 2400), dtype=np.uint8), (3, 3), 0)
        self.stereo = StereoDepth(num_disp=64, texture_filter=True)
        self.stereo.setup(self.k, np.zeros(5), self.k, np.zeros(5), (self.w, self.h),
                          np.eye(3), np.array([[-self.baseline], [0.], [0.]]))

    def eye(self, position, side, *, texture=True):
        origin = np.asarray(position) + [0.1053, side*self.baseline/2, 0.0304]
        ray = self.rays
        # Course interior: 58 x 72 inches, bottom 1 inch above floor.
        a, b, zc = 0.7366, 0.9144, 0.9398
        y, z = origin[1], origin[2]-zc
        aa = (ray[:, :, 1]/a)**2 + (ray[:, :, 2]/b)**2
        bb = 2*(y*ray[:, :, 1]/a**2 + z*ray[:, :, 2]/b**2)
        cc = (y/a)**2 + (z/b)**2 - 1
        with np.errstate(divide='ignore', invalid='ignore'):
            distance = (-bb+np.sqrt(np.maximum(0, bb*bb-4*aa*cc)))/(2*aa)
        points = origin + distance[:, :, None]*ray
        valid = (distance > 0) & (np.abs(points[:, :, 0]) < self.half_length)
        theta = np.arctan2((points[:, :, 2]-zc)/b, points[:, :, 1]/a)
        tx = np.mod(points[:, :, 0]*550, self.texture.shape[1]).astype(np.float32)
        ty = np.mod((theta+math.pi)*220, self.texture.shape[0]).astype(np.float32)
        gray = cv2.remap(self.texture, tx, ty, cv2.INTER_LINEAR, borderMode=cv2.BORDER_WRAP)
        if not texture:
            gray[:] = 140
        return np.where(valid, gray, 180).astype(np.uint8)

    def cloud(self, position, *, texture=True):
        left, right = self.eye(position, 1, texture=texture), self.eye(position, -1, texture=texture)
        _, _, _, optical = self.stereo.compute(left, right)
        body = optical @ OPTICAL_TO_BODY.T + [0.1053, self.baseline/2, 0.0304]
        return body, left, right


def run_case(seed, direction, correction, *, texture=True, camera_faces_travel=True):
    renderer = StereoTubeRenderer(seed)
    guide = TubeGuidance(timeout=3.0)
    dt, speed, bias = 0.1, 0.5, 0.12
    pos = np.array([-direction*1.20, 0.10, 0.85], float)
    velocity = np.array([direction*speed, 0., 0.])
    records = []
    valid_count = 0
    for i in range(40):
        t = i*dt
        odom = pos + [0, bias, 0]
        camera_sign = direction if camera_faces_travel else 1
        # For camera-facing-travel runs, rotate camera/body by pi for the
        # reverse traversal, while keeping the same navigation/odom frame.
        camera_pose = pos * [camera_sign, camera_sign, 1]
        body_points, _, _ = renderer.cloud(camera_pose, texture=texture)
        world_relative = body_points * [camera_sign, camera_sign, 1]
        fit = fit_tube(world_relative+odom, odom)
        if fit is not None:
            valid_count += 1
            guide.update(t, fit.center, fit.yaw, t)
        nominal = odom + [direction*0.40, -odom[1], 0]
        if abs(pos[0]) < 0.762:
            goal = guide.apply(nominal, odom, 0, t, enabled=correction, max_lead=0.6)
        else:
            guide.shift, guide.last_time = 0., None
            goal = nominal
        # Second-order lateral response, bounded acceleration; modeled controller,
        # not a claim about the hardware or actual PX4 tuning.
        acc = np.clip(5*(goal[1]-odom[1])-3*velocity[1] + 0.015*math.sin(2*t), -0.8, 0.8)
        velocity[1] += acc*dt
        pos += velocity*dt
        clearance = 0.7366*math.sqrt(1-((pos[2]-0.9398)/0.9144)**2)-abs(pos[1])-0.2375
        records.append({'t': round(t, 2), 'x': float(pos[0]), 'true_y': float(pos[1]),
                        'target_y_odom': float(goal[1]), 'fit': fit is not None, 'clearance': float(clearance)})
    return {'seed': seed, 'direction': direction, 'centering': correction, 'texture': texture,
            'camera_faces_travel': camera_faces_travel,
            'valid_frames': valid_count, 'frames': len(records),
            'final_abs_error_m': abs(records[-1]['true_y']),
            'min_lateral_clearance_m': min(r['clearance'] for r in records), 'trace': records}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--output', type=Path, required=True)
    args = ap.parse_args()
    cases = [run_case(seed, direction, enabled)
             for seed in (1, 2, 3) for direction in (-1, 1) for enabled in (False, True)]
    cases += [run_case(1, -1, True, texture=False),
              run_case(1, -1, True, camera_faces_travel=False)]
    checks = []
    for seed in (1, 2, 3):
        for direction in (-1, 1):
            off, on = [c for c in cases if c['seed'] == seed and c['direction'] == direction
                       and c['texture'] and c['camera_faces_travel']]
            checks.append({'seed': seed, 'direction': direction,
                           'improved': bool(on['final_abs_error_m'] < off['final_abs_error_m']),
                           'error_within_3cm': bool(on['final_abs_error_m'] < 0.03),
                           'positive_clearance': on['min_lateral_clearance_m'] > 0.10})
    summary = {'kind': 'offline rendered stereo + modeled odometry/dynamics; no PX4 or cuVSLAM',
               'runtime': {'python': platform.python_version(), 'numpy': np.__version__, 'opencv': cv2.__version__},
               'revision_before_changes': '9c57a31',
               'assumptions': {'odom_lateral_bias_m': 0.12, 'speed_mps': 0.5, 'dt_s': 0.1,
                               'image_size': [512, 320], 'fx_px': 305, 'baseline_m': 0.075,
                               'tube_length_m': 1.524, 'aircraft_half_width_m': 0.2375},
               'source_sha256': {str(p.relative_to(ROOT)): hashlib.sha256(p.read_bytes()).hexdigest()
                                 for p in (Path(__file__), ROOT/'depthcloud/stereo_cloud.py',
                                           ROOT/'navless_nav/navless_nav/stereo_tube.py',
                                           ROOT/'navless_nav/navless_nav/tube_guidance.py',
                                           ROOT/'navless_nav/navless_nav/course_mission.py',
                                           ROOT/'navless_nav/launch/navless.launch.py',
                                           ROOT/'navless_nav/launch/navless_course.launch.py',
                                           ROOT/'navless_nav/package.xml', ROOT/'navless_nav/setup.py')},
               'checks': checks, 'cases': cases}
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(summary, indent=2)+'\n')
    for c in cases:
        print(f"seed={c['seed']} dir={c['direction']:+} center={c['centering']} texture={c['texture']} "
              f"faces_travel={c['camera_faces_travel']} fits={c['valid_frames']}/{c['frames']} final={c['final_abs_error_m']:.4f}m "
              f"clearance={c['min_lateral_clearance_m']:.4f}m")
    negative_cases_rejected = all(c['valid_frames'] == 0 for c in cases if not c['texture'] or not c['camera_faces_travel'])
    return 0 if negative_cases_rejected and all(c['improved'] and c['positive_clearance']
                                               and c['error_within_3cm'] for c in checks) else 1


if __name__ == '__main__':
    raise SystemExit(main())
