"""pad_reference.fit_pad on synthetic forward-camera views of the landing pad.

Ring points are projected into a level-ish forward camera (OAK-like: 1280x800,
fx 760, mounted 10.5 cm ahead of base_link), rounded to whole pixels, and sent
back through line_reference's floor_points() exactly as pad_observer does.
Geometry only: no real images, lighting or edge detector.
"""
import math
import os
import sys

import numpy as np

REPO = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
sys.path.insert(0, REPO)
sys.path.insert(0, os.path.join(REPO, 'navless_nav'))

from line_reference.reference import floor_points    # noqa: E402
from navless_nav.pad_reference import RING_RADII_M, fit_pad    # noqa: E402

FX, CX, CY, W, H = 760.0, 640.0, 400.0, 1280, 800
CAM = (0.105, 0.0, 0.03)                     # optical origin from base_link, level frame
OPT_TO_BODY = np.array([[0, 0, 1], [-1, 0, 0], [0, -1, 0]], float)


def rotation(pitch_deg=0.0, roll_deg=0.0):
    """optical -> level; pitch > 0 = nose down."""
    p, r = math.radians(pitch_deg), math.radians(roll_deg)
    ry = np.array([[math.cos(p), 0, math.sin(p)], [0, 1, 0], [-math.sin(p), 0, math.cos(p)]])
    rx = np.array([[1, 0, 0], [0, math.cos(r), -math.sin(r)], [0, math.sin(r), math.cos(r)]])
    return ry @ rx @ OPT_TO_BODY


def view(floor_xy, height, rot, noise_px=0.0, seed=1):
    """Whole-pixel image positions of floor points that land in the frame."""
    rng = np.random.default_rng(seed)
    q = np.column_stack([floor_xy, np.full(len(floor_xy), -height)]) - np.array(CAM)
    o = q @ rot                                   # level -> optical (rot is orthonormal)
    o = o[o[:, 2] > 0.05]
    uv = np.column_stack([FX * o[:, 0] / o[:, 2] + CX, FX * o[:, 1] / o[:, 2] + CY])
    uv += rng.normal(0.0, noise_px, uv.shape)
    uv = np.round(uv)
    return uv[(uv[:, 0] >= 0) & (uv[:, 0] < W) & (uv[:, 1] >= 0) & (uv[:, 1] < H)]


def rings(centre, radii=RING_RADII_M, n=720):
    t = np.linspace(0, 2 * math.pi, n, endpoint=False)
    return np.vstack([np.column_stack([centre[0] + r * np.cos(t), centre[1] + r * np.sin(t)])
                      for r in radii])


def detect(floor_xy, height=0.85, measured_height=None, pitch=0.0, roll=0.0, noise_px=0.5,
           clutter=0, now=0.05):
    rot = rotation(pitch, roll)
    uv = view(floor_xy, height, rot, noise_px)
    if clutter:
        rng = np.random.default_rng(2)
        uv = np.vstack([uv, rng.uniform([0, 0], [W, H], (clutter, 2))])
    pts = floor_points([tuple(x) for x in uv], (FX, FX, CX, CY), rot.tolist(), CAM,
                       measured_height or height, max_range_m=4.0)
    return fit_pad(pts, 0.0, now, origin_xy=CAM[:2])


def err_cm(fix, centre):
    return 100 * math.dist(fix.center_xy, centre)


def test_far_half_of_the_pad_ahead_gives_the_centre():
    # Level at 0.85 m the nearest visible floor is ~1.7 m out: only the rings' far arcs are seen.
    centre = (1.6, 0.3)
    fix = detect(rings(centre))
    assert fix is not None and fix.rings >= 2
    assert err_cm(fix, centre) < 2.0


def test_nose_down_and_rolled_with_clutter():
    centre = (1.3, -0.4)
    fix = detect(rings(centre), pitch=8.0, roll=-4.0, clutter=600)
    assert fix is not None
    assert err_cm(fix, centre) < 2.0


def test_both_edges_of_the_ink_bands():
    # The organisers' rings are ~2.9 cm wide: the edge detector sees R +- 1.45 cm.
    centre = (1.5, 0.2)
    band = [r + e for r in RING_RADII_M for e in (-0.0145, 0.0145)]
    fix = detect(rings(centre, radii=band), pitch=5.0)
    assert fix is not None and fix.edges >= 4
    assert err_cm(fix, centre) < 2.0
    assert abs(fix.scale - 1.0) < 0.02


def test_wrong_height_is_corrected_by_the_ring_sizes():
    # 10% height error scales the whole view by 10%; the known radii undo it.
    centre = (1.8, 0.1)
    fix = detect(rings(centre), height=0.85, measured_height=0.935)
    assert fix is not None
    assert abs(fix.scale - 0.85 / 0.935) < 0.02
    assert err_cm(fix, centre) < 3.0


def test_single_ring_is_rejected():
    assert detect(rings((1.6, 0.3), radii=(0.762,))) is None


def test_rings_with_the_wrong_ratios_are_rejected():
    assert detect(rings((1.6, 0.3), radii=(0.35, 0.60))) is None


def test_straight_lines_are_rejected():
    t = np.linspace(-1.0, 1.0, 800)
    lines = np.vstack([np.column_stack([np.full_like(t, x), t]) for x in (1.6, 2.0, 2.4)])
    assert detect(lines) is None


def test_stale_image_is_rejected():
    assert detect(rings((1.6, 0.3)), now=0.5) is None
