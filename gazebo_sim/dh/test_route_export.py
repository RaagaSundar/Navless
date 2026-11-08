"""tools/export_route.py: the plan's routes still export unchanged, and the as-built routes are the ones 19 flies
(the window route; the Gate B route is its backup, navless_course_gate_b.launch.py on drone19/flight).

  python3 -m pytest gazebo_sim/dh/test_route_export.py
"""
import json
import os
import subprocess
import sys

import pytest

GZ = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
COURSE = os.path.join(GZ, "course")


def export(tmp_path, *args):
    out = tmp_path / "route.json"
    subprocess.run([sys.executable, os.path.join(GZ, "tools", "export_route.py"), str(out), *args],
                   check=True, capture_output=True, cwd=GZ)
    return json.load(open(out))


def test_plan_route_exports_unchanged(tmp_path):
    # The detours default to the plan's route: no --course, no detour flags, same file byte for byte.
    assert export(tmp_path) == json.load(open(os.path.join(COURSE, "route_centreline_window.json")))


def test_plan_gate_b_route_exports_unchanged(tmp_path):
    assert export(tmp_path, "--route", "gate_b") == json.load(open(os.path.join(COURSE, "route_centreline_gate_b.json")))


def test_another_course_needs_its_own_collision_model(tmp_path):
    r = subprocess.run([sys.executable, os.path.join(GZ, "tools", "export_route.py"), str(tmp_path / "x.json"),
                        "--course", os.path.join(COURSE, "course_asbuilt.json")], capture_output=True, text=True, cwd=GZ)
    assert r.returncode != 0 and "--sdf" in r.stderr


def test_asbuilt_route_records_how_it_was_made():
    r = json.load(open(os.path.join(COURSE, "route_centreline_window_asbuilt.json")))
    assert r["course_file"] == "gazebo_sim/course/course_asbuilt.json"
    assert (r["tube_exit_drop_in"], r["pad_from_south_in"]) == (42.0, 26.0)
    assert r["window_via_in"] == [136.0, 44.0]
    # The organisers' 2026-11-07 maze (MN, MD, MT), planned from its walls: east through Gate A at maze_y_in.
    assert r["maze"].startswith("2026-11-07") and r["maze_y_in"] == pytest.approx(220.22, abs=0.01)
    assert r["min_clearance_m_nose_north"] >= 0.17
    # The new maze leaves the route >= 0.5 m from the net, so no net clamp moves a setpoint (the course
    # launch's 0.25 or navless.launch.py's 0.4).
    top = max(abs(c) for p in r["waypoints_enu"] for c in p[:2])
    assert top <= 3.048 - 0.45


def test_asbuilt_gate_b_route_records_how_it_was_made():
    r = json.load(open(os.path.join(COURSE, "route_centreline_gate_b_asbuilt.json")))
    assert r["route_type"] == "GATE_B_CENTRELINE"
    assert r["course_file"] == "gazebo_sim/course/course_asbuilt.json"
    assert r["gate_b_turns_in"] == [59.0, 151.5, 118.0, 86.5]
    assert r["pad_from_south_in"] == 26.0
    assert r["maze"].startswith("2026-11-07") and r["maze_y_in"] == pytest.approx(220.22, abs=0.01)
    assert r["min_clearance_m_nose_north"] >= 0.17
    top = max(abs(c) for p in r["waypoints_enu"] for c in p[:2])          # same maze leg as the window route
    assert top <= 3.048 - 0.45


@pytest.mark.parametrize("route", ["route_centreline_window_asbuilt.json", "route_centreline_gate_b_asbuilt.json"])
def test_asbuilt_route_goes_through_the_middle_of_gate_a(route):
    # Gate A since 2026-11-07: the 1 m gap between MD's tip and the north net. North of the barrier line the
    # route crosses the gap's line (x = MD's tip) once, with >= 0.3 m to either end of the gap.
    d = json.load(open(os.path.join(COURSE, "course_asbuilt.json")))
    (gx, y_tip), (_, y_net) = (tuple(c * 0.0254 - 3.048 for c in q) for q in d["gates_in"]["gate_A"]["between"])
    p = json.load(open(os.path.join(COURSE, route)))["waypoints_enu"]
    ys = [a[1] + (b[1] - a[1]) * (gx - a[0]) / (b[0] - a[0]) for a, b in zip(p, p[1:]) if (a[0] - gx) * (b[0] - gx) < 0]
    north = [y for y in ys if y > 0.5]
    assert len(north) == 1, ys
    assert y_tip + 0.3 <= north[0] <= y_net - 0.3, (north[0], y_tip, y_net)


@pytest.mark.parametrize("route", ["route_centreline_window_asbuilt.json", "route_centreline_gate_b_asbuilt.json"])
def test_asbuilt_route_keeps_the_rangefinder_off_the_boulders(route):
    # PX4 takes its height from the downward laser (~1 deg beam, 2.7 cm left of the body origin; nose
    # north, so 2.7 cm west). Over a 0.53 m boulder it reads that much short: keep its footprint >= 20 cm
    # outside every boulder (map: boulders_in, centre and diameter in inches).
    d = json.load(open(os.path.join(COURSE, "course_asbuilt.json")))
    r = json.load(open(os.path.join(COURSE, route)))
    for b in d["boulders_in"]:
        bx, by = (c * 0.0254 - 3.048 for c in b["center"])
        radius = b["diameter"] / 2 * 0.0254
        gap = min(((p[0] - 0.027 - bx) ** 2 + (p[1] - by) ** 2) ** 0.5 for p in r["waypoints_enu"] if p[2] > 0.3)
        assert gap - radius >= 0.20, (b, gap - radius)
