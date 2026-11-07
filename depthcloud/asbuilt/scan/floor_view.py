"""Top view of the scan's floor in true colour, in the course frame, to find the taped guideline."""
import json
import sys
import numpy as np
import scan_common as sc

ply = sys.argv[1]
raw = open(ply, "rb").read()
h = raw.index(b"end_header\n") + len(b"end_header\n")
n = int([l for l in raw[:h].decode().splitlines() if l.startswith("element vertex")][0].split()[2])
dt = np.dtype([("x", "<f4"), ("y", "<f4"), ("z", "<f4"), ("r", "u1"), ("g", "u1"), ("b", "u1")])
a = np.frombuffer(raw, dt, count=n, offset=h)
RGB = np.stack([a["r"], a["g"], a["b"]], 1)
P = sc.load_level(ply)                       # same order as the file
fit = json.load(open("fit.json"))
W = sc.apply(fit["theta"], np.array(fit["t"]), P)
floor = (np.abs(W[:, 2]) < 0.03) & (np.abs(W[:, 0]) < 3.1) & (np.abs(W[:, 1]) < 3.1)
F, C = W[floor], RGB[floor].astype(float)
print(f"floor points: {len(F):,}")
mx, mn = C.max(1), C.min(1)
sat = (mx - mn) / np.maximum(mx, 1)
print("brightness percentiles", np.percentile(mx, [5, 25, 50, 75, 95]).round())
print("saturation percentiles", np.percentile(sat, [5, 25, 50, 75, 95]).round(2))
# hue of saturated points
hue = np.zeros(len(C))
r_, g_, b_ = C.T
d_ = np.maximum(mx - mn, 1)
hue = np.where(mx == r_, ((g_ - b_) / d_) % 6, np.where(mx == g_, (b_ - r_) / d_ + 2, (r_ - g_) / d_ + 4)) * 60
s_ = sat > 0.35
hist, edges = np.histogram(hue[s_], bins=12, range=(0, 360))
print("hue histogram of saturated floor points (30 deg bins):", hist.tolist())

cv = sc.Canvas(half=3.2, scale=130)
x = ((F[:, 0] + cv.half) * cv.s).astype(int)
y = ((cv.half - F[:, 1]) * cv.s).astype(int)
ok = (x >= 0) & (y >= 0) & (x < cv.img.shape[1]) & (y < cv.img.shape[0])
cv.img[y[ok], x[ok]] = C[ok].astype(np.uint8)
sc.png("floor_rgb.png", cv.img)
# contrast view: dark points and saturated points highlighted
cv2_ = sc.Canvas(half=3.2, scale=130)
dark = mx < np.percentile(mx, 8)
cv2_.put(F[s_], (220, 40, 40))
cv2_.put(F[dark], (20, 20, 20))
sc.png("floor_marks.png", cv2_.img)
print("wrote floor_rgb.png, floor_marks.png")
