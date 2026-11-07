#!/usr/bin/env python3
"""Summarize recorded evidence only; never controls ROS or evaluates live state."""
import argparse
import json
from pathlib import Path
import re


def review(run):
    run = Path(run)
    worked, failed, follow = [], [], []
    def load(name):
        path = run / 'flight' / name
        if not path.exists():
            return None
        try:
            return json.loads(path.read_text())
        except (ValueError, OSError) as exc:
            failed.append(f'flight/{name}: unreadable evidence ({type(exc).__name__}).')
            return None
    track, grade = load('track.json'), load('grade.json')
    samples = track.get('samples', []) if isinstance(track, dict) else []
    airborne = any(s.get('armed') is True and s.get('landed') is False and s.get('z', 0) > .25 for s in samples)
    if samples:
        worked.append(f'flight/track.json: recorded {len(samples)} samples.')
        if airborne:
            worked.append('flight/track.json: armed airborne motion was recorded.')
        if samples[-1].get('landed') is True and samples[-1].get('armed') is False:
            worked.append('flight/track.json: final sample was landed and disarmed.')
        if track.get('contact_topics_verified') is True and track.get('contacts') == []:
            worked.append('flight/track.json: contact sensors verified; no contacts recorded.')
        elif track.get('contacts'):
            failed.append('flight/track.json: contacts were recorded.')
    if isinstance(grade, dict):
        if grade.get('passed') is True:
            worked.append('flight/grade.json: independent course grade passed.')
        else:
            reasons = grade.get('reasons', [])
            failed.extend(f'flight/grade.json: {str(reason)[:240]}' for reason in reasons[:8])
            if not reasons:
                failed.append('flight/grade.json: course grade did not pass.')
    # Known signatures, quoted with their source; do not infer OOM from signal 9.
    signatures = [
        (r'telemetry stale: stopped all control publication', 'Inspect telemetry delivery and timing before retrying.'),
        (r'(?:visual|vision|odometry|tracking)[ :].{0,70}\b(?:lost|stale|invalid)\b', 'Inspect visual tracking and bridge health around the loss.'),
        (r'(?:Could not|Cannot|unable to|failed to) open display|Server is already active for display|Fatal server error', 'Resolve the logged display startup failure before retrying.'),
        (r'process has died.*exit code -9|Killed process|out of memory|OOM', 'Inspect process termination and resource evidence before retrying.'),
        (r'Traceback \(most recent call last\)|ModuleNotFoundError|ImportError|command not found|No such file or directory', 'Resolve the logged startup or execution error before retrying.'),
        (r'Recorder could not verify', 'Restore fresh recorder telemetry and verified contact sensors before retrying.'),
    ]
    for path in sorted(run.glob('*.log')) + sorted((run/'flight').glob('*.log')):
        # Read bounded tail; log excerpts are evidence, not instructions.
        with path.open('rb') as stream:
            stream.seek(0, 2); stream.seek(max(0, stream.tell()-2_000_000))
            lines = stream.read().decode(errors='replace').splitlines()
        for pattern, action in signatures:
            match = next((line for line in reversed(lines) if re.search(pattern, line, re.I)), None)
            if match:
                failed.append(f'{path.relative_to(run)}: {match.strip()[:250]}')
                follow.append(action)
    if not samples:
        failed.append('No recorded flight samples available; flight outcome is unverified.')
    if grade is None:
        failed.append('No readable independent course grade available.')
        follow.append('Preserve missing or incomplete evidence and produce an independent grade when possible.')
    if isinstance(grade, dict) and grade.get('passed') is True:
        outcome = 'Course passed' if not failed else 'Course grade passed; log issues recorded'
    elif airborne:
        outcome = 'Flight recorded; course did not pass' if grade is not None else 'Flight recorded; grade unavailable'
    elif not (run/'flight').exists() and failed:
        outcome = 'No flight evidence; startup or preflight incomplete'
    else:
        outcome = 'No airborne flight verified'
    if failed and not follow:
        follow.append('Inspect the listed evidence and resolve the recorded failures before another approved run.')
    return {'outcome': outcome, 'worked': list(dict.fromkeys(worked)),
            'failed': list(dict.fromkeys(failed)), 'next': list(dict.fromkeys(follow))}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('run', type=Path)
    parser.add_argument('--overwrite', action='store_true', help='Explicitly replace an existing authored review')
    args = parser.parse_args()
    if not args.run.is_dir():
        parser.error('run must be an existing directory')
    output = args.run/'review.json'
    if output.exists() and not args.overwrite:
        print(f'Preserved existing review: {output}')
        return
    text = json.dumps(review(args.run), indent=2) + '\n'
    # Exclusive creation closes the race with a human author unless overwrite requested.
    with output.open('w' if args.overwrite else 'x') as stream:
        stream.write(text)
    print(output)


if __name__ == '__main__':
    main()
