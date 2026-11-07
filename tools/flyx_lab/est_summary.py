"""Summarise a sim_truth_recorder CSV: timeline every ~1 s plus first VIO loss / first big error."""
import csv, gzip, sys

lines = [l for l in (gzip.open if sys.argv[1].endswith(".gz") else open)(sys.argv[1], "rt", encoding="utf-8", errors="ignore") if l[:1].isdigit() or l.startswith("sim_t")]
rows = [r for r in csv.DictReader(lines) if r.get("sim_t")]
f = lambda r, k: float(r[k]) if r.get(k) not in (None, "") else float("nan")
print(f"{len(rows)} rows, sim_t {rows[0]['sim_t']}..{rows[-1]['sim_t']}")
armed = [r for r in rows if r["arming_state"] == "2"]
t0 = f(armed[0], "sim_t") if armed else f(rows[0], "sim_t")
print("  t(arm) truthN truthE truthU | px4x  px4y  px4up | err  vo ev mag nav | hdgT  hdgP spYaw")
last_t, prev = -9.0, None
for r in rows:
    t = f(r, "sim_t") - t0
    changed = prev is not None and (prev["vo_state"] != r["vo_state"] or prev["nav_state"] != r["nav_state"])
    if t - last_t >= 1.0 or changed:
        print(f"{t:7.1f} {f(r,'truth_n'):6.2f} {f(r,'truth_e'):6.2f} {f(r,'truth_u'):6.2f} | {f(r,'px4_x'):5.2f} {f(r,'px4_y'):5.2f} {f(r,'px4_up'):5.2f} | "
              f"{f(r,'err_xy'):4.2f} {r['vo_state']:>2} {r['ev_pos']:>2} {r['mag_hdg']:>2} {r['nav_state']:>3} | "
              f"{f(r,'truth_hdg_deg'):5.0f} {f(r,'px4_hdg_deg'):5.0f} {f(r,'sp_yaw_deg'):5.0f}")
        last_t = t
    prev = r
bad = [r for r in armed if r["vo_state"] not in ("1", "")]
print("first VIO not tracking (s after arm):", round(f(bad[0], "sim_t") - t0, 2) if bad else None)
big = [r for r in armed if f(r, "err_xy") > 0.3]
print("first err_xy>0.3 m:", round(f(big[0], "sim_t") - t0, 2) if big else None,
      " max err:", max((f(r, "err_xy") for r in armed), default=None))
