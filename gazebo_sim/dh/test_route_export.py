"""tools/export_route.py: the plan's routes still export unchanged, and the as-built routes are the ones 19 flies
(the window route; the Gate B route is its backup, navless_course_gate_b.launch.py).

Since the organisers' 2026-11-07 maze change (M1-M3 out; MN, MD and MT in) the course map has no floor line (the
tape on the floor is the old maze's), and dh/route.py maze_2026_11_07 plans the maze from the walls: north-east
up corridor 1 beside MD, round MD's tip through Gate A, then south into the tube.

  python3 -m pytest gazebo_sim/dh/test_route_export.py
"""
import importlib.util
import json
import math
import os
import subprocess
import sys

import numpy as np
import pytest

GZ = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
COURSE = os.path.join(GZ, "course")
ASBUILT = os.path.join(COURSE, "course_asbuilt.json")
sys.path.insert(0, GZ)
from dh.geom import Obstacles, clearance, map_to_world  # noqa: E402
from dh.route import window_route  # noqa: E402

ROUTES = {   # file -> the export flags that made it (also in the launch files' comments)
    "route_centreline_window_asbuilt.json": ["--tube-exit-drop-in", "42", "--pad-from-south-in", "26",
                                             "--window-via-in", "136", "44"],
    "route_centreline_gate_b_asbuilt.json": ["--route", "gate_b", "--pad-from-south-in", "26",
                                             "--gate-b-turns-in", "59", "151.5", "118", "86.5"],
}
NAMES = sorted(ROUTES)


def export(tmp_path, *args):
    out = tmp_path / "route.json"
    subprocess.run([sys.executable, os.path.join(GZ, "tools", "export_route.py"), str(out), *args],
                   check=True, capture_output=True, cwd=GZ)
    return json.load(open(out))


def load(name):
    return json.load(open(os.path.join(COURSE, name)))


@pytest.fixture(scope="module")
def asbuilt_sdf(tmp_path_factory):
    """The as-built course's collision model (tools/build_course_model.py, collisions only), in a scratch dir:
    the repo's models/dh_course is the organisers' plan."""
    spec = importlib.util.spec_from_file_location("build_course_model", os.path.join(GZ, "tools", "build_course_model.py"))
    b = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(b)
    b.MODEL = str(tmp_path_factory.mktemp("asbuilt_model"))
    b.build_sdf(json.load(open(ASBUILT)))
    return os.path.join(b.MODEL, "model.sdf")


def test_plan_route_exports_unchanged(tmp_path):
    # The detours default to the plan's route: no --course, no detour flags, same file byte for byte.
    assert export(tmp_path) == load("route_centreline_window.json")


def test_plan_gate_b_route_exports_unchanged(tmp_path):
    assert export(tmp_path, "--route", "gate_b") == load("route_centreline_gate_b.json")


def test_another_course_needs_its_own_collision_model(tmp_path):
    r = subprocess.run([sys.executable, os.path.join(GZ, "tools", "export_route.py"), str(tmp_path / "x.json"),
                        "--course", ASBUILT], capture_output=True, text=True, cwd=GZ)
    assert r.returncode != 0 and "--sdf" in r.stderr


def test_new_maze_is_planned_from_its_walls_not_the_tape():
    # The tape on the floor is the old maze's (it runs under MD): the map has no floor line, and the route's
    # maze part is the turn round MD's tip, not the old floor-line corners.
    assert "floor_line_in" not in json.load(open(ASBUILT))
    names = [w.name for w in window_route(ASBUILT)[0]]
    assert names[:4] == ["launch", "liftoff", "md_tip", "tube_turn"]
    assert not {"climb", "corridor_end", "m1_top", "gate_a_out"} & set(names)


@pytest.mark.parametrize("name", NAMES)
def test_asbuilt_route_exports_as_committed(tmp_path, asbuilt_sdf, name):
    # The committed file is exactly what the code makes from course_asbuilt.json with the recorded flags.
    assert export(tmp_path, "--course", ASBUILT, "--sdf", asbuilt_sdf, *ROUTES[name]) == load(name)


@pytest.mark.parametrize("name", NAMES)
def test_asbuilt_route_records_how_it_was_made(name):
    r = load(name)
    assert r["course_file"] == "gazebo_sim/course/course_asbuilt.json"
    assert r["maze"].startswith("2026-11-07")
    assert (r["tip_radius_m"], r["tube_turn_y_in"], r["pad_from_south_in"]) == (0.585, 208.0, 26.0)
    assert r["min_clearance_m_nose_north"] >= 0.08
    assert r["launch_origin_enu"][:2] == [-2.5146, 0.9144]
    if "window" in name:
        assert (r["tube_exit_drop_in"], r["window_via_in"]) == (42.0, [136.0, 44.0])
    else:
        assert r["gate_b_turns_in"] == [59.0, 151.5, 118.0, 86.5]


@pytest.mark.parametrize("name", NAMES)
def test_asbuilt_route_keeps_its_clearance_with_the_props(asbuilt_sdf, name):
    # The drone's parts with the props (0.1575 m ahead, 0.2275 m behind, 0.2375 m each side), nose north the
    # whole way (course_mission holds the takeoff heading): >= 8 cm to everything, and the new maze's walls and
    # the north net >= 20 cm, so the carrot's corner cuts and PX4's overshoot have room.
    obs = Obstacles(asbuilt_sdf)
    pts = np.array(load(name)["waypoints_enu"])
    c, names = clearance(obs, pts, np.zeros(len(pts)))
    assert c.min() >= 0.08, (names[int(np.argmin(c))], c.min())
    maze = [ci for ci, ni in zip(c, names) if ni.split("_")[0] in ("MN", "MD", "MT") or ni == "net_north"]
    assert min(maze) >= 0.20, min(maze)


@pytest.mark.parametrize("name", NAMES)
def test_asbuilt_route_heads_away_from_the_west_net(name):
    # At liftoff 19 has slid 25-33 cm west, and on the launch mark its prop tips are 29.6 cm from the west net:
    # the route never goes west of the mark, and 1 m along it the drone is >= 0.6 m further east (course_mission
    # also leans the climb along this leg; navless_nav/test/test_asbuilt_route.py checks that).
    pts = np.array(load(name)["waypoints_enu"])
    lx = pts[0, 0]
    assert pts[:, 0].min() >= lx - 1e-3
    s = np.concatenate([[0], np.cumsum(np.linalg.norm(np.diff(pts, axis=0), axis=1))])
    assert pts[np.searchsorted(s, 1.0), 0] >= lx + 0.6


@pytest.mark.parametrize("name", NAMES)
def test_asbuilt_route_goes_through_gate_a(name):
    # Gate A is the gap between MD's tip and the north net. Crossing its line, the drone's back (0.2275 m) and
    # front (0.1575 m) both stay >= 8 cm off the tip's corner and the net.
    d = json.load(open(ASBUILT))
    (gx, tip_y), (_, net_y) = (map_to_world(*q) for q in d["gates_in"]["gate_A"]["between"])
    md = next(w for w in d["walls"] if w["name"] == "MD")
    corner_y = tip_y + md["thickness_in"] / 2 * 0.0254 * math.sin(math.radians(md["bearing_deg"]))
    pts = np.array(load(name)["waypoints_enu"])
    cross = [i for i in range(len(pts) - 1)
             if (pts[i, 0] - gx) * (pts[i + 1, 0] - gx) < 0 and pts[i, 1] > 1.5]    # (the window leg recrosses it far south)
    assert len(cross) == 1
    y = float(np.interp(gx, pts[cross[0]:cross[0] + 2, 0], pts[cross[0]:cross[0] + 2, 1]))
    assert corner_y + 0.2275 + 0.08 <= y <= net_y - 0.1575 - 0.08, y


@pytest.mark.parametrize("name", NAMES)
def test_asbuilt_route_is_straight_into_the_tube(name):
    # Straight (within 2 mm of one line) from 0.45 m north of the mouth: the drone's back reaches the mouth with
    # its centre 0.2275 m out, so the turn is over before any of it is in the tube. The line is the window
    # route's tube line (1.2 deg off south: 3 in west over the tube), within 3 cm of the tube's axis at the mouth.
    d = json.load(open(ASBUILT))
    tube = d["tunnel"]["footprint_in"]
    mouth = tube["y_max"] * 0.0254 - 3.048
    axis = (tube["x_min"] + tube["x_max"]) / 2 * 0.0254 - 3.048
    pts = np.array(load(name)["waypoints_enu"])
    band = pts[(pts[:, 1] >= mouth) & (pts[:, 1] <= mouth + 0.45) & (pts[:, 0] > 1.5)]
    assert len(band) >= 8
    k, b = np.polyfit(band[:, 1], band[:, 0], 1)
    assert np.abs(band[:, 0] - (k * band[:, 1] + b)).max() < 0.002
    assert abs(math.degrees(math.atan(k))) < 1.5
    assert abs(k * mouth + b - axis) < 0.03


@pytest.mark.parametrize("name", NAMES)
def test_asbuilt_route_keeps_the_rangefinder_off_the_boulders(name):
    # PX4 takes its height from the downward laser (~1 deg beam, 2.7 cm left of the body origin; nose
    # north, so 2.7 cm west). Over a 0.53 m boulder it reads that much short: keep its footprint >= 20 cm
    # outside every boulder (map: boulders_in, centre and diameter in inches).
    d = json.load(open(ASBUILT))
    r = load(name)
    for b in d["boulders_in"]:
        bx, by = (c * 0.0254 - 3.048 for c in b["center"])
        radius = b["diameter"] / 2 * 0.0254
        gap = min(((p[0] - 0.027 - bx) ** 2 + (p[1] - by) ** 2) ** 0.5 for p in r["waypoints_enu"] if p[2] > 0.3)
        assert gap - radius >= 0.20, (b, gap - radius)
