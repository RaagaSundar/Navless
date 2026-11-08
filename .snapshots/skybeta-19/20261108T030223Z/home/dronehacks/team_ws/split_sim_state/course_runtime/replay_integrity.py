"""Replay actual ULog EV packets against PX4 samples available at EV arrival."""

import argparse
import json
from types import SimpleNamespace

from integrity import IntegrityGate
from px4_navigation import PoseHistory, RESET_FIELDS


def replay(path):
    import numpy as np
    from pyulog import ULog

    log = ULog(str(path))
    local = log.get_dataset("vehicle_local_position").data
    ev = log.get_dataset("vehicle_visual_odometry").data
    status = log.get_dataset("vehicle_status").data
    laser = log.get_dataset("distance_sensor").data
    gate, history = IntegrityGate(), PoseHistory()
    position_index = 0
    checked = 0
    fields = ("timestamp_sample", "x", "y", "z", "heading", *RESET_FIELDS)
    for i, arrival in enumerate(ev["timestamp"]):
        while position_index < len(local["timestamp"]) and local["timestamp"][position_index] <= arrival:
            history.add(SimpleNamespace(**{k: local[k][position_index] for k in fields}))
            position_index += 1
        stamp = int(ev["timestamp_sample"][i]) / 1e6
        aligned = history.at(stamp)
        status_index = int(np.searchsorted(status["timestamp"], arrival, side="right")) - 1
        if status_index < 0 or aligned is None:
            if checked:
                raise ValueError(f"cannot align recorded samples at {stamp}")
            continue

        # The deployed arbitrary-heading EV bridge swaps ROS odom X/Y and
        # negates Z for world positions. Undo that exact mapping for the gate.
        xyz = (float(ev["position[1]"][i]), float(ev["position[0]"][i]),
               -float(ev["position[2]"][i]))
        armed = status["arming_state"][status_index] == 2
        ok, reason = gate.observe(stamp, xyz, aligned[2], armed)
        checked += 1
        if not ok:
            # Laser is post-hoc corroboration only and never enters the gate.
            raw = int(np.searchsorted(laser["timestamp"], ev["timestamp_sample"][i], side="right")) - 1
            return dict(checked=checked, reason=reason, source_s=stamp, arrival_s=int(arrival)/1e6,
                        vo_up_m=xyz[2], px4_down_m=aligned[2],
                        prior_laser_m=float(laser["current_distance"][raw]) if raw >= 0 else None)
    return dict(checked=checked, reason=None)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("ulog")
    print(json.dumps(replay(parser.parse_args().ulog), indent=2))
