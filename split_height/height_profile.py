#!/usr/bin/env python3
"""SITL-only reviewed height profiles and passive checks. No ROS/flight commands."""
import argparse
import json
import math
import re

PROFILES = {
    'range': {'EKF2_HGT_REF': 2, 'EKF2_EV_CTRL': 9, 'EKF2_RNG_CTRL': 1, 'EKF2_BARO_CTRL': 1},
    'ev-terrain': {'EKF2_HGT_REF': 3, 'EKF2_EV_CTRL': 11, 'EKF2_RNG_CTRL': 1, 'EKF2_BARO_CTRL': 1},
}


def settings(profile):
    if profile not in PROFILES:
        raise ValueError('unknown reviewed height profile')
    return dict(PROFILES[profile])


def fusion_problem(profile, flags):
    """Mirror the frozen companion guard; no missing flag is considered healthy."""
    settings(profile)
    if flags is None:
        return 'missing estimator fusion flags'
    required = ['cs_rng_hgt']
    faults = ['cs_rng_stuck', 'cs_rng_fault']
    if profile == 'ev-terrain':
        required += ['cs_ev_hgt', 'cs_baro_hgt', 'cs_rng_terrain']
        faults += ['cs_fake_hgt', 'cs_baro_fault', 'fs_bad_acc_vertical']
    if any(getattr(flags, name, None) is not True for name in required):
        return 'required height/terrain fusion inactive or missing'
    if any(getattr(flags, name, None) is not False for name in faults):
        return 'height/terrain integrity fault or missing flag'
    return None


def preflight_problem(profile, listener_text, parameter_text):
    """Validate actual passive PX4 listener/parameter output; no fabricated fix."""
    expected = settings(profile)
    numbers = {key: int(value) for key, value in re.findall(
        r'(EKF2_[A-Z_]+)\n\s*(\d+)\s*(?:\n|$)', parameter_text)}
    if numbers != expected:
        return 'actual PX4 height parameters differ from selected profile'
    for topic in ('distance_sensor', 'vehicle_imu', 'estimator_aid_src_rng_hgt'):
        marker = 'TOPIC: ' + topic + '\n'
        if marker not in listener_text:
            return 'missing ' + topic
        data = listener_text.split(marker, 1)[1].split('TOPIC:', 1)[0]
        match = re.search(r'timestamp: \d+ \(([0-9.]+) seconds ago\)', data)
        if not match or not math.isfinite(float(match[1])) or float(match[1]) > .5:
            return 'missing or stale ' + topic
    required = ['fused: True', 'cs_rng_fault: False', 'cs_rng_stuck: False', 'cs_rng_hgt: True']
    if profile == 'ev-terrain':
        required += ['cs_ev_hgt: True', 'cs_baro_hgt: True', 'cs_rng_terrain: True']
    if any(field not in listener_text for field in required):
        return 'actual range/height/terrain aiding not confirmed'
    return None


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('command', choices=('shell', 'check'))
    parser.add_argument('--profile', choices=tuple(PROFILES), required=True)
    parser.add_argument('--listener')
    parser.add_argument('--parameters')
    args = parser.parse_args()
    if args.command == 'shell':
        for key, value in settings(args.profile).items():
            print('export PX4_PARAM_' + key + '=' + str(value))
    else:
        if not args.listener or not args.parameters:
            parser.error('check requires --listener and --parameters')
        from pathlib import Path
        problem = preflight_problem(args.profile, Path(args.listener).read_text(), Path(args.parameters).read_text())
        print(json.dumps({'height_preflight_ok': problem is None, 'problem': problem,
                          'qualification': 'Passive height checks only; not flight approval.'}))
        raise SystemExit(1 if problem else 0)
