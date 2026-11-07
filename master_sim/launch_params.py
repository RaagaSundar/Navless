#!/usr/bin/env python3
"""Read the drone's own launch defaults, so the sim runs navless_nav exactly as skybeta does.

  python3 launch_params.py mission [name:=value ...]   ->  -p name:=value ... (navless_nav's params)
  python3 launch_params.py envelope                    ->  --min-alt A --max-alt B --max-speed C

Values come from navless_nav/launch/navless.launch.py (ARGUMENTS, MISSION_PARAMS). Overrides
replace a mission default, e.g. route_file:=route_window.json.
"""
import importlib.util
import os
import sys

LAUNCH = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
                      "navless_nav", "launch", "navless.launch.py")


def load():
    spec = importlib.util.spec_from_file_location("navless_launch", LAUNCH)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return {name: default for name, default, _ in mod.ARGUMENTS}, mod.MISSION_PARAMS


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
