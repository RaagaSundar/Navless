"""tools/export_route.py: the plan's route still exports unchanged, and the as-built route is the one 19 flies.

  python3 -m pytest gazebo_sim/dh/test_route_export.py
"""
import json
import os
import subprocess
import sys

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


def test_another_course_needs_its_own_collision_model(tmp_path):
    r = subprocess.run([sys.executable, os.path.join(GZ, "tools", "export_route.py"), str(tmp_path / "x.json"),
                        "--course", os.path.join(COURSE, "course_asbuilt.json")], capture_output=True, text=True, cwd=GZ)
    assert r.returncode != 0 and "--sdf" in r.stderr


def test_asbuilt_route_records_how_it_was_made():
    r = json.load(open(os.path.join(COURSE, "route_centreline_window_asbuilt.json")))
    assert r["course_file"] == "gazebo_sim/course/course_asbuilt.json"
    assert (r["tube_exit_drop_in"], r["pad_from_south_in"], r["maze_top_north_m"]) == (42.0, 26.0, 0.04)
    assert r["window_via_in"] == [136.0, 44.0]
    assert r["min_clearance_m_nose_north"] >= 0.08
    # The crossings over M1's / M3's north ends sit 0.256 m from the north net: course_mission's net clamp
    # (net_margin_m) must be <= 0.25 to leave them alone. navless.launch.py's 0.4 pulls them into the walls.
    top = max(abs(c) for p in r["waypoints_enu"] for c in p[:2])
    assert 3.048 - 0.4 < top <= 3.048 - 0.25


def test_asbuilt_route_keeps_the_rangefinder_off_the_boulders():
    # PX4 takes its height from the downward laser (~1 deg beam, 2.7 cm left of the body origin; nose
    # north, so 2.7 cm west). Over a 0.53 m boulder it reads that much short: keep its footprint >= 20 cm
    # outside every boulder (map: boulders_in, centre and diameter in inches).
    d = json.load(open(os.path.join(COURSE, "course_asbuilt.json")))
    r = json.load(open(os.path.join(COURSE, "route_centreline_window_asbuilt.json")))
    for b in d["boulders_in"]:
        bx, by = (c * 0.0254 - 3.048 for c in b["center"])
        radius = b["diameter"] / 2 * 0.0254
        gap = min(((p[0] - 0.027 - bx) ** 2 + (p[1] - by) ** 2) ** 0.5 for p in r["waypoints_enu"] if p[2] > 0.3)
        assert gap - radius >= 0.20, (b, gap - radius)
