#!/usr/bin/env python3
"""Read the drone's own launch defaults, so the sim runs navless_nav exactly as skybeta does.

  python3 launch_params.py mission [name:=value ...]   ->  -p name:=value ... (navless_nav's params)
  python3 launch_params.py envelope                    ->  --min-alt A --max-alt B --max-speed C

The drone starts navless_course.launch.py (when it exists), which sets a few values (route, bearing,
speed, odometry topic) and forwards them to navless.launch.py; everything else comes from
navless.launch.py's ARGUMENTS and MISSION_PARAMS. Overrides replace a value, e.g.
route_file:=route_gate_b.json.
"""
import importlib.util
import os
import sys

LAUNCH_DIR = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "navless_nav", "launch")


def module(name):
    path = os.path.join(LAUNCH_DIR, name)
    if not os.path.isfile(path):
        return None
    spec = importlib.util.spec_from_file_location(name.replace(".", "_"), path)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


def load():
    base = module("navless.launch.py")
    defaults = {name: default for name, default, _ in base.ARGUMENTS}
    course = module("navless_course.launch.py")       # the drone's entry point, if this branch has it
    if course is not None:
        for name, default, _ in course.ARGUMENTS:
            if name in defaults:                       # only what it forwards to navless.launch.py
                defaults[name] = default
    return defaults, base.MISSION_PARAMS


def main():
    defaults, mission = load()
    if sys.argv[1] == "envelope":
        print(f"--min-alt {defaults['min_altitude_m']} --max-alt {defaults['max_altitude_m']} "
              f"--max-speed {defaults['max_speed_mps']}")
        return
    values = {name: defaults[name] for name in mission}
    for item in sys.argv[2:]:
        name, value = item.split(":=", 1)
        values[name] = value
    print(" ".join(f"-p {k}:={v}" for k, v in values.items()))


if __name__ == "__main__":
    main()
