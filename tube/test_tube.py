import math

from tube_node import tube_offset

close = lambda a, b: abs(a - b) < 1e-9

# centred: no offset
off, w = tube_offset(0.3, 0.3, 0.1, 2.0)
assert close(off, 0) and close(w, 0.6)
# closer to the right wall -> centreline is to the left (positive)
off, _ = tube_offset(0.4, 0.2, 0.1, 2.0)
assert close(off, 0.1)
off, _ = tube_offset(0.2, 0.4, 0.1, 2.0)
assert close(off, -0.1)
# sensor faces 5 cm off centreline on each side
off, w = tube_offset(0.25, 0.25, 0.1, 2.0, left_mount=0.05, right_mount=0.05)
assert close(off, 0) and close(w, 0.6)
# prop blade at 4 cm, out of range, NaN or inf: rejected
assert tube_offset(0.04, 0.3, 0.1, 2.0) is None
assert tube_offset(0.3, 2.5, 0.1, 2.0) is None
assert tube_offset(math.nan, 0.3, 0.1, 2.0) is None
assert tube_offset(0.3, math.inf, 0.1, 2.0) is None
# width check: one side sees the tube exit gap -> dropped
assert tube_offset(0.3, 0.3, 0.1, 2.0, expected_width=0.6) is not None
assert tube_offset(0.3, 0.9, 0.1, 2.0, expected_width=0.6) is None
# a pair counts as fresh for at least two of tof_driver's frames
from tof_driver import PERIOD_MS  # noqa: E402
from tube_node import STALE_S  # noqa: E402
assert STALE_S >= 2 * PERIOD_MS / 1000
print("ok")
