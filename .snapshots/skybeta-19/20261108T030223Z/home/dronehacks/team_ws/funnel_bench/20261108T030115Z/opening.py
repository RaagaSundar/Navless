"""Optical bearing to a bright opening bounded by two darker panels."""
import math
import cv2
import numpy as np


def opening(image, fx, cx, cy):
    if image.ndim != 2 or image.dtype != np.uint8 or not math.isfinite(fx) or fx <= 0:
        return None
    height, width = image.shape
    if not (0 < cx < width and 0.1 * height < cy < 0.9 * height):
        return None
    # Above the horizon the opening shows sky. Median across rows suppresses
    # small texture marks. Reject clipped, ambiguous or nonvertical boundaries.
    pairs = []
    for offset in (0.015, 0.035, 0.055):
        row = round(cy - offset * height)
        stripe = np.median(image[row-2:row+3], axis=0).astype(np.uint8)[None, :]
        smooth = cv2.GaussianBlur(stripe, (5, 1), 1)
        lo, hi = np.percentile(smooth, [10, 98])
        if hi - lo < 35:
            return None
        bright = (smooth.ravel() > (lo + hi) / 2).astype(np.int8)
        transitions = np.diff(np.r_[0, bright, 0])
        spans = [(l, r) for l, r in zip(np.flatnonzero(transitions == 1),
                                          np.flatnonzero(transitions == -1))
                 if width * 0.02 < r-l < width * 0.75 and 4 < l < r < width-4]
        if len(spans) != 1:
            return None
        pairs.append(spans[0])
    if np.ptp(pairs, axis=0).max() > width * 0.012:
        return None
    left, right = map(int, np.median(pairs, axis=0))
    u = (left + right - 1) / 2
    return dict(left_px=left, right_px=right-1, center_px=u,
                bearing_right_deg=math.degrees(math.atan2(u-cx, fx)))
