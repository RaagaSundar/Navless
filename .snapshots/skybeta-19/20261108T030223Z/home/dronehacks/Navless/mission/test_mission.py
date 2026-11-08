import math
import os

from mission import Carrot, course_to_local, fence_for, inside, load_route, check, turn_toward

close = lambda a, b: all(abs(x - y) < 1e-9 for x, y in zip(a, b))

# ENU -> NED swaps x/y and flips z; heading 0 = local x points course north
assert close(course_to_local((1, 2, 3), (0, 0, 0), 0), (2, 1, -3))
# drone started facing course east: course east is local +x, course north is local -y
assert close(course_to_local((1, 0, 0), (0, 0, 0), 90), (1, 0, 0))
assert close(course_to_local((0, 1, 0), (0, 0, 0), 90), (0, -1, 0))
# launch offset
assert close(course_to_local((-2.515, 0.914, 0.85), (-2.515, 0.914, 0), 0), (0, 0, -0.85))

# carrot moves speed*dt per tick, walks corners, and waits when the drone lags
c = Carrot([(0, 0, 0), (1, 0, 0), (1, 1, 0)], speed=1.0, max_lead=0.5)
assert close(c.step((0, 0, 0), 0.25), (0.25, 0, 0))
assert close(c.step((0.25, 0, 0), 1.0), (1, 0.25, 0))
assert close(c.step((0, 0, 0), 1.0), (1, 0.25, 0))  # 1.03 m behind -> hold
for _ in range(10):
    c.step(c.p, 0.1)
assert c.done((1, 1, 0), 0.15) and not c.done((1, 0.5, 0), 0.15)

# camera faces travel: local NED x = north, y = east, so yaw 0 = north, +pi/2 = east
h = Carrot([(0, 0, 0), (0, 0, -1), (1, 0, -1), (1, 1, -1)], speed=1.0, max_lead=5.0)
assert abs(h.heading() - 0.0) < 1e-9  # skips the vertical climb, looks at the first sideways leg
h.step(h.p, 2.5)  # carrot now on the east-going leg
assert abs(h.heading() - math.pi / 2) < 1e-9
assert Carrot([(0, 0, 0), (0, 0, -1)], 1.0, 1.0).heading(0.7) == 0.7  # straight up: keep current yaw
# rate limit, and turns the short way across +-pi
assert abs(turn_toward(0.0, 1.0, 0.1) - 0.1) < 1e-9
assert abs(turn_toward(0.0, 0.05, 0.1) - 0.05) < 1e-9
assert abs(turn_toward(3.0, -3.0, 0.1) - 3.1) < 1e-9  # 0.28 rad gap across pi: step +0.1, not -6.0

f = fence_for([(0, 0, 0), (1, 1, -1)], 0.5)
assert inside((1.4, -0.4, -1.4), f) and not inside((1.6, 0, 0), f)

import json
root = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..")
for name in ("route_gate_b.json", "route_window.json", "route_fly_home.json"):
    f = os.path.join(root, name)
    route, launch = load_route(f, None, 0.0)
    assert close(route[0], (0, 0, -0.85)) and launch == (-2.515, 0.914, 0.0)
    # planner's own NED must agree with ours (it was mirrored once)
    assert all(close(a, b) for a, b in zip(route, json.load(open(f))["waypoints_ned"])), name
    check(route, 0.5)
print("ok")
