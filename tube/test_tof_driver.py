import math

from tof_driver import mm_to_range, view_text

assert mm_to_range(650) == 0.65
assert math.isinf(mm_to_range(0)) and math.isinf(mm_to_range(-1)) and math.isinf(mm_to_range(None))

# centred: O on the centre line
v = view_text(0.6, 0.6, 30.0)
assert "+0.0 cm" in v and "OK " in v
# closer to the right wall: tube centre is to the left (positive), same sign as tube_node,
# and the drone marker sits right of the centre line
v = view_text(0.4, 0.2, 30.0)
assert "+10.0 cm" in v
row = next(l for l in v.splitlines() if "left wall [" in l)
assert row.index("O") > row.index("|")
# a bad side (prop at 4 cm, or no return) shows BAD and no offset
v = view_text(0.04, 0.6, 30.0)
assert "BAD" in v and "n/a" in v
assert "BAD" in view_text(0.6, math.inf, 30.0)
print("ok")
