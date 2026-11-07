"""Compact story of the latest run in view.csv: one line each time either side's 2-second median moves by more
than 2 cm or starts/stops seeing something. usage: python3 view_log_summary.py view.csv (from log_view.py)"""
import statistics
import sys

rows = []
for line in open(sys.argv[1]):
    if line.startswith("# start"):
        rows = []
        continue
    parts = line.strip().split(",")
    if len(parts) != 3:
        continue                                    # a line cut off when the run was stopped
    try:
        rows.append((float(parts[0]), float(parts[1]) if parts[1] else None, float(parts[2]) if parts[2] else None))
    except ValueError:
        continue
if not rows:
    sys.exit("no rows")
t0, t1 = rows[0][0], rows[-1][0]
print(f"{len(rows)} frames over {(t1 - t0) / 60:.1f} min ({len(rows) / max(t1 - t0, 1e-6):.1f} Hz)")
for i, side in ((1, "left"), (2, "right")):
    v = [r[i] for r in rows if r[i] is not None]
    print(f"  {side}: valid in {len(v)}/{len(rows)} frames"
          + (f", min {min(v):.1f} median {statistics.median(v):.1f} max {max(v):.1f} cm" if v else ""))
last, t = None, t0
while t <= t1:
    w = [r for r in rows if t <= r[0] < t + 2]
    state = []
    for i in (1, 2):
        v = [r[i] for r in w if r[i] is not None]
        state.append(statistics.median(v) if len(v) * 2 >= len(w) and v else None)
    changed = last is None or any((a is None) != (b is None) or (a is not None and abs(a - b) > 2)
                                  for a, b in zip(state, last))
    if w and changed:
        fmt = lambda x: "  --  " if x is None else f"{x:5.1f}"
        print(f"  {(t - t0) / 60:5.1f} min   L {fmt(state[0])}   R {fmt(state[1])}")
        last = state
    t += 2
