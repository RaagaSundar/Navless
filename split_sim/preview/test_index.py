"""Bounded, read-only inventory of saved test receipts; never executes a test.

Known evidence roots only. Runtime/vendor/source trees are excluded. Outcomes are
reported from explicit receipts, not inferred from code, filenames or file dates.
"""
from __future__ import annotations
import hashlib
import json
import os
import re
import threading
import time
from datetime import datetime, timezone
from pathlib import Path
from urllib.parse import quote

ROOT = Path(__file__).resolve().parents[2]
MAX_FILE = 16 * 1024 * 1024
READ_LIMIT = 128 * 1024
MAX_DEPTH = 5
MAX_FILES = 18000
EXTENSIONS = {'.json', '.jsonl', '.log', '.txt', '.md', '.png', '.jpg', '.jpeg', '.csv', '.yaml', '.yml', '.sha256'}
SKIP_DIRS = {'.git', '__pycache__', 'build', 'install', 'log', 'logs-build', 'node_modules', 'vendor', 'third_party', 'thirdparty', 'src', 'source', 'sources', 'assets', 'models', 'meshes', 'materials', 'textures', 'ros2_ws', 'fast_msgs', 'team-upstream', '.venv', 'venv', 'bin', 'lib', 'include', 'share', 'cmake', 'source-snapshot', 'pre-ekf-source', 'flight-source'}
SENSITIVE = re.compile(r'credential|secret|password|cookie|askpass|keychain|private[-_]?key|authorized_keys|known_hosts|(?:^|[-_.])token(?:[-_.]|$)', re.I)
# Exact roots, not a recursive workspace crawl. Each key is a URL namespace.
ROOTS = {
 'mac-runs': ('local_course/state/runs', 4, 'Mac session archives'),
 'nvidia-runs': ('local_course/state/nvidia-runs', 4, 'NVIDIA session archives'),
 'flyx-runs': ('local_course/state/flyx-runs', 3, 'FlyX session archives'),
 'course-evidence': ('local_course/evidence', 5, 'Course evidence'),
 'course-state': ('local_course/state', 0, 'Planner / estimator receipts'),
 'camera-bench': ('local_course/state/camera-bench', 4, 'Camera benches'),
 'wall-rotation': ('local_course/state/wall-rotation', 4, 'Wall rotation'),
 'return-experiments': ('local_course/state/return-experiments', 4, 'Return replays'),
 'round-trip': ('local_course/state/round-trip-runs', 4, 'Round trip / combined'),
 'return-staging': ('local_course/state/return-staging', 3, 'Return staging / receipts'),
 'map-registration': ('local_course/state/map-registration-study', 4, 'Map registration'),
 'scan-appearance': ('local_course/state/scan-appearance', 4, 'Scan appearance'),
 'openvins': ('local_course/state/openvins', 3, 'VIO replay / build receipts'),
 'replay': ('local_course/replay-evidence', 3, 'Sensor replays'),
 'replay-later': ('local_course/replay-evidence-2', 3, 'Sensor replays'),
 'github-reuse': ('local_course/github-reuse-evidence', 3, 'Estimator reuse studies'),
 'direct-core': ('local_course/direct-core-evidence', 3, 'Controller offline checks'),
 'map-constrained': ('local_course/map-constrained-evidence', 3, 'Constrained map replay'),
 'map-replay': ('local_course/map-replay-evidence', 3, 'Map replay'),
 'map-study': ('local_course/map-study', 3, 'Map model experiments'),
 'offline-ekf': ('local_course/offline_ekf', 3, 'Offline fusion replay'),
 'jetson-evidence': ('jetson_sim/evidence', 4, 'Jetson simulation / sensor receipts'),
 'jetson-state': ('jetson_sim/state', 0, 'Jetson archived receipts'),
 'hardware-evidence': ('hardware_slam/evidence', 3, 'Hardware disarmed bench'),
 'bootstrap-evidence': ('local_sim/evidence', 3, 'Local simulator qualification'),
 'tube-evidence': ('Navless/tube/stereo_evidence', 3, 'Tube stereo experiments'),
 'line-evidence': ('Navless/line_reference', 0, 'Line / fusion evidence'),
}
RUN_ROOTS = {}
_LOCK = threading.Lock()
_CACHE = None
_CACHE_AT = 0.0

def _source_root(relative):
    alias={'local_course/state/runs':'mac','local_course/state/nvidia-runs':'nvidia','local_course/state/flyx-runs':'flyx'}.get(relative)
    if alias in RUN_ROOTS:
        return RUN_ROOTS[alias]
    path=ROOT/relative
    # The local workspace nests the team checkout as Navless; installed main is
    # itself that checkout. Resolve only these two documented relative layouts.
    if not path.exists() and relative.startswith('Navless/'):
        path=ROOT/relative.removeprefix('Navless/')
    return path

def _safe(path: Path, root: Path) -> bool:
    parts=path.relative_to(root).parts
    return (not root.is_symlink() and not any(root.joinpath(*parts[:i]).is_symlink() for i in range(1,len(parts)+1))
            and path.resolve().is_relative_to(root.resolve()) and not any(SENSITIVE.search(p) for p in parts))

def _walk(root: Path, depth: int):
    if not root.is_dir() or root.is_symlink():
        return
    for base, dirs, files in os.walk(root, followlinks=False):
        folder = Path(base)
        level = len(folder.relative_to(root).parts)
        dirs[:] = sorted(d for d in dirs if level < depth and d.lower() not in SKIP_DIRS and not d.startswith('.') and not (folder/d).is_symlink() and not SENSITIVE.search(d))
        for name in sorted(files):
            p = folder/name
            if name.startswith('.'): continue
            if p.suffix.lower() in EXTENSIONS and _safe(p, root):
                yield p

def _read(path):
    try:
        if path.stat().st_size <= READ_LIMIT:
            raw = path.read_text(errors='replace')
            return json.loads(raw) if path.suffix == '.json' else raw
    except (OSError, ValueError):
        pass
    return None

def _receipt(path):
    data = _read(path)
    if isinstance(data, dict):
        summary = next((data[k] for k in ('outcome', 'finding', 'conclusion', 'scope', 'description') if isinstance(data.get(k), str)), '')
        if isinstance(data.get('capture_met_declared_goals'), bool):
            return ('bounded goals met' if data['capture_met_declared_goals'] else 'bounded goals not met', summary, path.name)
        if isinstance(data.get('clearance_and_contact_passed'), bool):
            return ('clearance check passed' if data['clearance_and_contact_passed'] else 'clearance check failed', summary, path.name)
        if isinstance(data.get('passed'), bool):
            return ('passed check' if data['passed'] else 'failed check', summary, path.name)
        # Capture complete does not imply successful tracking or flight.
        if data.get('complete') is True:
            return ('capture complete', summary, path.name)
        for key in ('outcome', 'status', 'qualification'):
            value = data.get(key)
            if isinstance(value, str) and len(value) < 180 and value.lower() in {'passed', 'failed', 'rejected', 'blocked', 'incomplete', 'unqualified', 'proposed', 'completed', 'pass', 'fail'}:
                return ('reported '+value.lower(), summary, path.name)
        return ('unknown', summary, None)
    if isinstance(data, str) and path.suffix in {'.txt', '.log'}:
        failed = re.search(r'(?im)^FAILED\b|\b[1-9]\d* failed\b|^FAIL(?:ED)?[: ]', data)
        passed = re.search(r'(?m)^OK(?:\s*\(.*\))?\s*$|\b\d+ passed\b|^PASS(?:ED)?[: ]', data)
        if failed:
            return ('failed check', 'Saved test output reports a failure; inspect the exact receipt.', path.name)
        if passed:
            return ('passed check', 'Saved test output reports a pass; scope is this check only.', path.name)
    return ('unknown', '', None)

def _kind(text):
    value = text.lower()
    if 'hardware' in value: return 'hardware bench'
    if any(k in value for k in ('unit-test', 'unit_test', 'tests.txt', 'tests.log', 'test-result', 'test_result')): return 'unit / software check'
    if any(k in value for k in ('camera-bench', 'wall-rotation', 'texture', 'static', 'bench', 'net-first-turn')): return 'passive bench'
    if any(k in value for k in ('replay', 'offline', 'map-study', 'registration', 'scan-appearance', 'imu-review', 'fusion')): return 'offline replay / analysis'
    if any(k in value for k in ('astar', 'planner', 'route', 'clearance', 'yawplan', 'launchview')): return 'planner / geometry'
    if any(k in value for k in ('build', 'install', 'bootstrap', 'network', 'transport', 'preflight', 'connection', 'probe')): return 'setup / preflight check'
    return 'recorded evidence / unknown kind'

def _date(identity, files):
    match = re.search(r'(20\d{6})T(\d{6})Z', identity)
    if match:
        try:
            return datetime.strptime(''.join(match.groups()), '%Y%m%d%H%M%S').replace(tzinfo=timezone.utc).isoformat(), 'timestamp in immutable artifact ID'
        except ValueError:
            pass
    stamp = max((p.stat().st_mtime for p in files), default=0)
    return datetime.fromtimestamp(stamp, timezone.utc).isoformat() if stamp else None, 'file modification time only; not an execution date'

def _stem(path):
    name = path.stem
    # Related result/log/image siblings share one item; no execution is inferred.
    return re.sub(r'[-_](?:grade|result|summary|verification|manifest|sources?|sha256|metrics|notes|log)$', '', name, flags=re.I) or name

def _entry(key, relative, label, files, root):
    identity = key+'/'+relative
    rid = 'artifact:'+hashlib.sha256(identity.encode()).hexdigest()[:20]
    date, date_basis = _date(relative, files)
    title = relative.replace('/', ' · ').replace('_',' ').replace('-', ' ')
    outcome, summary, basis = 'unknown', '', None
    outcome_receipts=[]
    receipts = sorted(files, key=lambda p: (not any(s in p.name.lower() for s in ('assessment','result','summary','verification','experiment','evaluation','test')), p.name))
    for path in receipts[:32]:
        if path.suffix not in {'.json','.txt','.log'}: continue
        found, sentence, source = _receipt(path)
        if not summary and sentence: summary = sentence[:500]
        if found != 'unknown':
            outcome_receipts.append(dict(outcome=found,file=str(path.relative_to(root))))
            if outcome == 'unknown': outcome, basis = found, str(path.relative_to(root))
        meta = _read(path) if path.suffix == '.json' and path.name in {'experiment.json','run-comparison.json','assessment.json','manifest.json'} else None
        if isinstance(meta, dict):
            candidate = meta.get('title') or meta.get('name')
            if isinstance(candidate,str): title = candidate[:200]
    if any('failed' in r['outcome'] or 'rejected' in r['outcome'] for r in outcome_receipts) and any('passed' in r['outcome'] for r in outcome_receipts):
        outcome, basis = 'mixed recorded checks', 'multiple receipts; inspect individual results'
    matches = sorted(set(re.findall(r'20\d{6}T\d{6}Z(?:-split-[a-z-]+)?', relative+' '+summary)))
    return dict(id=rid, title=title, group=label, kind=_kind(key+'/'+relative), date=date, date_basis=date_basis,
                outcome=outcome, outcome_basis=basis, outcome_receipts=outcome_receipts, summary=summary or ('Capture receipt reports a complete recording; it does not establish successful tracking or flight.' if outcome=='capture complete' else 'Proposed metadata; no execution is inferred.' if outcome=='reported proposed' else 'An explicit saved receipt reports this check outcome; scope and qualification remain limited to that receipt.' if outcome!='unknown' else 'Saved artifacts exist. Execution and outcome are not established without an explicit receipt.'),
                parent_run_ids=matches, viewer_url=None,
                limits=['Reported results apply only to the named check. Capture completeness, geometric replay and documentation are not flight or hardware qualification.'],
                files=[dict(name=str(p.relative_to(root)), size=p.stat().st_size,
                            url='/test-evidence/'+quote(key,safe='')+'/'+quote(str(p.relative_to(root)),safe='/'),
                            viewable=p.stat().st_size<=MAX_FILE) for p in files], source=key+'/'+relative)

def build(run_rows=None):
    started = time.monotonic()
    records, file_map, scanned, coverage = [], {}, 0, []
    known_runs={('nvidia-runs' if r['id'].startswith('nvidia:') else 'flyx-runs' if r['id'].startswith('flyx:') else 'mac-runs',r.get('run_id',r['id'])) for r in run_rows or []}
    for key, (relative, depth, label) in ROOTS.items():
        root = _source_root(relative)
        groups = {}
        count = 0
        for path in _walk(root, depth) or ():
            if scanned >= MAX_FILES: break
            if path.name=='all-tests-source-inventory.json': continue
            # Source checkout documentation is not a test result inventory.
            if key == 'line-evidence' and path.name not in {'evidence.txt','fusion_evidence.txt'}: continue
            scanned += 1; count += 1
            rel = path.relative_to(root)
            group = rel.parts[0] if len(rel.parts)>1 else _stem(path)
            groups.setdefault(group, []).append(path)
            file_map[(key,str(rel))] = path
        if root.exists(): coverage.append(dict(root=relative, files=count, max_depth=depth))
        for name, files in groups.items():
            if (key,name) in known_runs: continue
            row=_entry(key,name,label,files,root)
            if key in {'mac-runs','nvidia-runs','flyx-runs'}:
                row.update(kind='setup / preflight attempt',outcome='unknown / unqualified session',summary='Historical session files exist, but verified platform/flight/grade metadata is incomplete. No flight or pass is inferred.')
                # Unverified platform sessions remain discoverable as receipts,
                # never promoted into the observed-flight catalog.
                intent=_read(root/name/'trial-intent.json') if root/name/'trial-intent.json' in files else None
                review=_read(root/name/'review.json') if root/name/'review.json' in files else None
                platform=_read(root/name/'platform.json') if root/name/'platform.json' in files else None
                if isinstance(intent,dict) and isinstance(intent.get('name'),str):
                    row['title']=intent['name'][:160]+' · '+name
                if isinstance(review,dict) and isinstance(review.get('failure'),str) and review['failure'].strip():
                    row.update(outcome='reported setup failure',outcome_basis=name+'/review.json',summary=str(review.get('outcome') or 'Archived setup failure')[:500]+' · '+review['failure'][:500])
                elif isinstance(platform,dict) and platform.get('setup_status') in {'preparing','blocked','partial'}:
                    row.update(outcome='setup / '+platform['setup_status'],outcome_basis=name+'/platform.json')
                row['limits'].append('Unverified session: intent and setup receipts do not establish mission execution, airborne telemetry or a course pass.')
            if row['source']=='wall-rotation/20261107T224200Z-experimental-m3-tangent-pair':
                row['viewer_url']='/?run=bench%3Am3-wall-rotation'
            records.append(row)
    # Existing catalog determines genuine flight/pass semantics; never regrade.
    for run in run_rows or []:
        record = dict(id='run:'+run['id'], title=(run.get('comparison') or {}).get('name') or (run.get('review') or {}).get('outcome') or run.get('run_id') or run['id'],
                      group=run.get('platform_label','Simulation'), kind='flight' if run.get('observed_flight') else 'native trajectory' if run.get('trajectory_archive') else 'setup / preflight attempt',
                      date=None,date_basis='run ID',outcome=run.get('status','unknown'),outcome_basis='existing run archive',
                      summary=(run.get('review') or run.get('assessment') or {}).get('outcome','Archived session; no outcome inferred from process exit.'),
                      parent_run_ids=[run['id']],viewer_url='/?run='+quote(run['id'],safe=''),limits=['Original run grading and missing-data limits remain unchanged.'],
                      files=[dict(name=f,url='/evidence/'+quote(run['id'],safe='')+'/'+quote(f,safe='/'),size=None,viewable=True) for f in run.get('files',[])],source='run/'+run['id'])
        record['date'],record['date_basis']=_date(run.get('run_id',run['id']),[])
        records.append(record)
        runkey='nvidia' if run['id'].startswith('nvidia:') else 'flyx' if run['id'].startswith('flyx:') else 'mac'
        runroot = RUN_ROOTS.get(runkey,ROOT / ('local_course/state/nvidia-runs' if runkey=='nvidia' else 'local_course/state/flyx-runs' if runkey=='flyx' else 'local_course/state/runs')) / run.get('run_id', run['id'])
        labels={'m3-capture-result.json':'Bounded M3 capture result','m3-physical-clearance.json':'Physical clearance check','m3-shadow-offline.json':'Offline M3 shadow replay','localization-poses.json':'Localization replay', 'map-localization.json':'CAD / scan map comparison',
                'flight/imu-review.json':'Recorded inertial / fusion review','flight/range-review.json':'Recorded range review',
                'flight/map-match-review.json':'Map matching review','texture-bench/metrics.json':'Fixed-pose texture comparison',
                'flight/frame-rate-comparison.json':'Frame-rate review'}
        for filename,title in labels.items():
            if filename not in run.get('files',[]): continue
            path=runroot/filename
            if not _safe(path,runroot): continue
            data=_read(path)
            found,sentence,basis=_receipt(path)
            experiment=dict(record)
            experiment.update(id='derived:'+hashlib.sha256((run['id']+'/'+filename).encode()).hexdigest()[:20],
                              title=title+' · '+run.get('run_id',run['id']),kind='offline replay / analysis',
                              group='Run-derived experiments',outcome=found,outcome_basis=filename if basis else None,
                              summary=sentence or 'Derived observations of this archived flight; not an additional flight or independent pass.',
                              viewer_url=None,parent_viewer_url=record['viewer_url'],source='derived/'+run['id']+'/'+filename,
                              files=[f for f in record['files'] if f['name']==filename],
                              limits=['Derived analysis of an existing flight, not another simulator session. Source truth/alignment/qualification limits remain in the original evidence.'])
            records.append(experiment)
    records.sort(key=lambda r:(r['date'] or '',r['title']),reverse=True)
    return dict(records=records,coverage=coverage,scanned_files=scanned,scan_seconds=round(time.monotonic()-started,3),
                generated_at=datetime.now(timezone.utc).isoformat(),limits=['Inventory includes saved receipts from explicitly allowlisted roots, not every possible test ever run.',
                'Unknown outcomes and file-modification-only dates are preserved. Code, vendor trees, binaries, raw bags, credentials and symlinks are excluded.',
                'Native trajectories, replay scores, passive benches and software checks do not increment independent flight pass counts.'],_files=file_map)

def inventory(run_rows=None, force=False):
    global _CACHE, _CACHE_AT
    with _LOCK:
        if force or _CACHE is None or time.monotonic()-_CACHE_AT>15:
            _CACHE=build(run_rows);_CACHE_AT=time.monotonic()
        return {k:v for k,v in _CACHE.items() if not k.startswith('_')}

def resolve_file(key, relative):
    if not relative or any(p in ('','.','..') for p in relative.split('/')) or '\\' in relative:
        return None
    if _CACHE is None: inventory()
    path = _CACHE['_files'].get((key,relative))
    definition=ROOTS.get(key)
    if path is None or definition is None: return None
    root=_source_root(definition[0])
    return path if path.is_file() and _safe(path,root) else None
