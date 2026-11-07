#!/usr/bin/env python3
"""Passive container isolation check. No start/stop, DDS, or flight commands."""
import argparse
import json
import subprocess
import sys

PROFILES = ('drone19-gazebo-bench', 'drone19-split-sim')


def assess(containers, expected):
    """Return a sanitized inventory and isolation errors, never flight readiness."""
    errors, inventory = [], []
    by_name = {}
    for item in containers:
        name = item.get('Name', '').lstrip('/')
        if name in PROFILES:
            if name in by_name:
                errors.append('duplicate container record: ' + name)
            by_name[name] = item
    running = [name for name, item in by_name.items()
               if item.get('State', {}).get('Running') is True]
    if len(running) > 1:
        errors.append('domain 219 collision: both simulation profiles are running')
    if expected not in running:
        errors.append('expected profile is not running: ' + expected)
    for name, item in by_name.items():
        env = {}
        for value in item.get('Config', {}).get('Env', []) or []:
            key, sep, value = value.partition('=')
            if sep and key in ('ROS_DOMAIN_ID', 'ROS_LOCALHOST_ONLY'):
                env[key] = value
        host = item.get('HostConfig', {})
        devices = [d.get('PathOnHost', '') for d in host.get('Devices', []) or []]
        mounts = item.get('Mounts', []) or []
        # Allow only the verified renderer devices; reject USB/serial/camera access.
        unsafe_devices = [p for p in devices if p not in
                          ('/dev/dri/renderD128', '/dev/dri/renderD129')]
        unsafe_mount = any(m.get('Source', '') == '/dev' or
                           m.get('Source', '').startswith('/dev/') for m in mounts)
        active = name in running
        if active:
            if env.get('ROS_DOMAIN_ID') != '219':
                errors.append(name + ': ROS_DOMAIN_ID must be 219')
            if env.get('ROS_LOCALHOST_ONLY') != '1':
                errors.append(name + ': ROS_LOCALHOST_ONLY must be 1')
            if host.get('Privileged') is not False:
                errors.append(name + ': privileged state is absent or enabled')
            if unsafe_devices or unsafe_mount:
                errors.append(name + ': unexpected device access; inspect locally')
        inventory.append({'profile': name, 'running': active,
                          'image_id': item.get('Image'), 'ros': env,
                          'privileged': host.get('Privileged'),
                          'unexpected_device_access': bool(unsafe_devices or unsafe_mount)})
    return {'schema_version': 1, 'expected_profile': expected,
            'isolation_ok': not errors, 'errors': errors, 'profiles': inventory,
            'qualification': 'Isolation inventory only; sensor health, flight readiness and hardware qualification are NOT established.'}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--expected', choices=PROFILES, required=True)
    parser.add_argument('--inspect-json', help='Offline Docker inspect JSON; otherwise inspect local known simulation containers')
    args = parser.parse_args()
    try:
        if args.inspect_json:
            with open(args.inspect_json, encoding='utf-8') as handle:
                containers = json.load(handle)
        else:
            # One inspect per profile permits an absent alternative profile.
            containers = []
            for name in PROFILES:
                found = subprocess.run(['docker', 'container', 'ls', '-a', '--filter',
                                        'name=^/' + name + '$', '--format', '{{.Names}}'],
                                       capture_output=True, text=True, check=True, timeout=10)
                if name in found.stdout.splitlines():
                    result = subprocess.run(['docker', 'inspect', name], capture_output=True,
                                            text=True, check=True, timeout=10)
                    containers.extend(json.loads(result.stdout))
        if not isinstance(containers, list) or any(not isinstance(c, dict) for c in containers):
            raise ValueError('expected a list of container objects')
        report = assess(containers, args.expected)
    except (OSError, ValueError, TypeError, AttributeError, subprocess.SubprocessError):
        # Docker diagnostics/inspect payloads may contain private paths or env secrets.
        print(json.dumps({'isolation_ok': False, 'errors': ['Inventory unavailable or malformed; inspect locally.']}))
        return 2
    print(json.dumps(report, indent=2))
    return 0 if report['isolation_ok'] else 1


if __name__ == '__main__':
    sys.exit(main())
