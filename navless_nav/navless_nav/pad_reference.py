"""Landing-pad bullseye from the forward camera: floor points in, pad centre out.

Pure numpy, no ROS (pad_observer.py is the node). The caller turns ring edge
pixels into floor points with line_reference.reference.floor_points(), rays
onto the flat floor at the measured height; this finds the printed rings.

Identity check: edges from at least two rings, all concentric, each a
continuous arc lying tightly on its circle, with the pad's radius ratios
(see RING_RADII_M). A printed ring has two edges (either side of the ink);
each is fitted as its own circle and the ring's radius is their mean, so the
radii are taken as ring CENTRELINES. Absolute size is not trusted: a height
error scales every floor point about the camera by one factor, so the known
radii fix that scale and the centre is corrected with it. A reject returns
None, never a cached fix. Thresholds are prototype values, not
hardware-qualified.
"""
from dataclasses import dataclass
import math

import numpy as np

# Ring centrelines. course_asbuilt.json names the rings r9/r20/r30 (in), but in the
# organisers' mesh (dronehacks-course-*.obj, "Landing pad ring r*") each is a dark
# band ~1.1-1.3 in wide whose OUTER edge is at that radius: 7.95-9.05, 18.86-20.14
# and 28.92-30.08 in. Centrelines 8.5, 19.5, 29.5 in. The mesh's "Approach ring
# (stage 4 line)", approach_ring_radius 39 in, spans 38.41-39.59 in: centreline 39.
# It is the biggest ring in view, so leaving it out made it the worst distractor
# (master sim: fits 28 cm off). Measure the real mat and floor ring.
RING_RADII_M = (8.5 * 0.0254, 19.5 * 0.0254, 29.5 * 0.0254, 39.0 * 0.0254)

last_reject = ''    # why the latest fit_pad call returned None (diagnostics only)


def _reject(why):
    global last_reject
    last_reject = why


@dataclass(frozen=True)
class PadFix:
    stamp_s: float
    center_xy: tuple    # pad centre, same frame as the input points, scale-corrected
    scale: float        # true size / measured size (a height error shows up here)
    rms_m: float
    rings: int
    edges: int
    points: int


def _circles(p, idx):
    """Circle through each triple of points: centres (n, 2), radii (n,)."""
    a, b, c = p[idx[:, 0]], p[idx[:, 1]], p[idx[:, 2]]
    m = np.stack([b - a, c - a], axis=1) * 2.0
    rhs = np.stack([(b * b).sum(1) - (a * a).sum(1), (c * c).sum(1) - (a * a).sum(1)], axis=1)
    det = m[:, 0, 0] * m[:, 1, 1] - m[:, 0, 1] * m[:, 1, 0]
    cen = np.full((len(idx), 2), np.nan)
    ok = np.abs(det) > 1e-9
    cen[ok] = np.linalg.solve(m[ok], rhs[ok][..., None])[..., 0]
    return cen, np.linalg.norm(a - cen, axis=1)


def _longest_arc_deg(angles, bin_deg=5.0, min_bin_points=3):
    """Longest run of angular bins holding >= min_bin_points (circular), degrees.
    Edge pixels along a real arc are dense; scattered clutter is not."""
    n = int(round(360.0 / bin_deg))
    occ = np.bincount((np.degrees(angles) % 360.0 / bin_deg).astype(int) % n,
                      minlength=n) >= min_bin_points
    if occ.all():
        return 360.0
    occ = np.roll(occ, -int(np.argmin(occ)))       # start on an empty bin
    run = best = 0
    for o in occ:
        run = run + 1 if o else 0
        best = max(best, run)
    return best * bin_deg


def fit_pad(points, stamp_s, now_s, *, origin_xy=(0.0, 0.0), radii=RING_RADII_M,
            max_age_s=0.15, iterations=600, tol_m=0.04, edge_tol_m=0.008,
            min_edge_points=20, min_arc_deg=40.0, min_rings=2, max_scale_err=0.15,
            max_rms_m=0.006, min_crisp=0.75, score_points=800, seed=0):
    """Pad centre from floor points (N x 2, metres), or None.

    origin_xy is the camera's floor position in the points' frame: a height
    error scales the points about it. tol_m is the ring band half-width
    searched around each nominal radius, at true scale; edge_tol_m is how
    tightly a point must sit on its edge circle.
    """
    # ponytail: radii read as ring centrelines, thin ink assumed (< 2 * tol_m);
    # check both against a photo/measurement of the real mat.
    if not (math.isfinite(stamp_s) and math.isfinite(now_s) and 0.0 <= now_s - stamp_s <= max_age_s):
        return _reject(f'image age {now_s - stamp_s:.3f} s')
    p = np.asarray(points, dtype=float).reshape(-1, 2)
    p = p[np.isfinite(p).all(1)]
    if len(p) < min_rings * min_edge_points:
        return _reject(f'{len(p)} floor points')
    radii = np.asarray(radii, dtype=float)
    origin = np.asarray(origin_xy, dtype=float)

    # RANSAC: a circle through 3 points, read as each known ring in turn (that
    # fixes the scale); keep the centre/scale that puts most points on rings.
    rng = np.random.default_rng(seed)
    idx = np.array([rng.choice(len(p), 3, replace=False) for _ in range(iterations)])
    cen, r = _circles(p, idx)
    scales = radii[None, :] / r[:, None]
    ii, kk = np.nonzero(np.isfinite(scales) & (np.abs(scales - 1.0) <= max_scale_err))
    if not len(ii):
        return _reject('no ring-sized circle')
    hc, hs = cen[ii], scales[ii, kk]
    ps = p[rng.choice(len(p), min(len(p), score_points), replace=False)]  # bounded memory
    d = np.linalg.norm(ps[None, :, :] - hc[:, None, :], axis=2) * hs[:, None]
    score = (np.abs(d[..., None] - radii).min(2) < tol_m).sum(1)   # whole ink band, both edges
    c, s = hc[score.argmax()], float(hs[score.argmax()])

    # Refine: find each ring's ink edges at the current centre/scale, then a
    # geometric fit (Gauss-Newton on point-to-circle distance) of one centre
    # and one scale with the KNOWN radii: edge radius = rho * R_k +- h_k (h_k:
    # that ring's half ink width). Free radii per edge would let a far-side
    # arc trade a centre shift for a constant radius change; known radii don't.
    for _ in range(4):
        rd = np.linalg.norm(p - c, axis=1) * s                     # true-scale radius
        res = rd[:, None] - radii[None, :]
        ring = np.abs(res).argmin(1)
        off = res[np.arange(len(p)), ring]
        edges = []                                                 # (ring, mask, offset)
        for k in range(len(radii)):
            band = (ring == k) & (np.abs(off) < tol_m)
            for _ in range(2):                                     # strongest edge, then the other
                if band.sum() < min_edge_points:
                    break
                hist, e = np.histogram(off[band], bins=np.arange(-tol_m, tol_m + 1e-9, 0.004))
                peak = 0.5 * (e[hist.argmax()] + e[hist.argmax() + 1])
                on = band & (np.abs(off - peak) < edge_tol_m)
                if on.sum() >= min_edge_points:
                    edges.append((k, on, peak))
                band &= np.abs(off - peak) >= 2 * edge_tol_m
        used = sorted({k for k, _, _ in edges})
        if len(used) < min_rings:
            return _reject(f'edges on {len(used)} ring(s)')
        # Every edge is rho * R_k + sign * h: inside (-) or outside (+) of the
        # ink, by where its peak sits; one ink half-width h for all rings (the
        # organisers' mesh: 1.1-1.3 in wide). Thin ink just fits h ~ 0.
        sign = np.array([1.0 if pk >= 0.0 else -1.0 for _, _, pk in edges])
        ring_of = np.array([k for k, _, _ in edges])
        rows = np.concatenate([np.nonzero(m)[0] for _, m, _ in edges])
        col = np.concatenate([np.full(m.sum(), j) for j, (_, m, _) in enumerate(edges)])
        q, rk, sg = p[rows], radii[ring_of][col], sign[col]
        rho, h = 1.0 / s, float(np.mean(np.abs([pk for _, _, pk in edges]))) / s
        for _ in range(8):
            v = q - c
            dist = np.linalg.norm(v, axis=1)
            jac = np.column_stack([-v / dist[:, None], -rk, -sg])
            step = np.linalg.lstsq(jac, -(dist - rho * rk - sg * h), rcond=None)[0]
            if not np.isfinite(step).all():
                return _reject('degenerate fit')
            c, rho, h = c + step[:2], rho + step[2], h + step[3]
        if rho <= 0:
            return _reject('degenerate fit')
        s = 1.0 / rho

    # Gates: scale, ink width, tightness, and each edge a real arc.
    if abs(s - 1.0) > max_scale_err:
        return _reject(f'scale {s:.3f}')
    if not -edge_tol_m <= h * s <= tol_m:
        return _reject(f'ink half-width {h * s * 100:.1f} cm')
    radial = (np.linalg.norm(q - c, axis=1) - rho * rk - sg * h) * s
    rms = float(np.sqrt(np.mean(radial ** 2)))
    # Crisp edges: of the points inside the used rings' bands, most must sit on
    # an edge. A wrong centre smears an arc across a band instead.
    rd = np.linalg.norm(p - c, axis=1) * s
    near = np.abs(rd[:, None] - radii[used][None, :]).min(1) < tol_m
    on_edge = np.zeros(len(p), bool)
    on_edge[rows[np.abs(radial) < edge_tol_m]] = True
    crisp = on_edge.sum() / max(1, near.sum())
    if crisp < min_crisp:
        return _reject(f'only {crisp:.2f} of band points on edges')
    if rms > max_rms_m:
        return _reject(f'rms {rms * 100:.2f} cm')
    arcs = {k for k, m, _ in edges
            if _longest_arc_deg(np.arctan2(*(p[m] - c).T[::-1])) >= min_arc_deg}
    if len(arcs) < min_rings:
        return _reject(f'continuous arcs on {len(arcs)} ring(s)')
    centre = origin + (c - origin) * s
    return PadFix(stamp_s, (float(centre[0]), float(centre[1])), s, rms,
                  len(used), len(edges), int(len(q)))
