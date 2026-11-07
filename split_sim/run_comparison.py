#!/usr/bin/env python3
"""Seed an honest run comparison from archived files only. No ROS/flight interfaces."""
import argparse
import hashlib
import json
import math
from pathlib import Path
import os

SETTINGS = ('estimator','height_profile','route','start_bearing_deg','takeoff_height_m',
            'cruise_m_s','max_lead_m','takeoff_vertical_lead_m','max_lat_accel_m_s2',
            'max_sagitta_m','yaw_policy','yaw_rate_deg_s','navigation_source')
ARCHIVES = ('settings.json','flight/planned-route.json','renderer-source.json',
            'appearance.json','height-profile','yaw-profile','course_mission.py')
KINDS = ('flight attempted','bench','replay')


def read(path):
    if path.is_symlink() or not path.is_file():
        return None
    try:
        return json.loads(path.read_text())
    except (ValueError,OSError):
        return None


def identity(run):
    prefix={'nvidia-runs':'nvidia:','flyx-runs':'flyx:'}.get(run.parent.name,'')
    return prefix+run.name


def snapshot(run):
    values={}
    for file,keys in [('settings.json',SETTINGS),('appearance.json',('appearance_profile','synthetic','net_visual_type')),
                      ('renderer-source.json',('world_sha256','spawn_sdf_sha256','course_model_sha256')),
                      ('flight/planned-route.json',('window_target_z_m','height_policy','xy_unchanged'))]:
        data=read(run/file)
        if isinstance(data,dict):
            for key in keys:
                if key in data:
                    values[file+':'+key]=data[key]
    route=read(run/'flight/planned-route.json')
    if isinstance(route,dict) and isinstance(route.get('waypoints_enu'),list):
        values['flight/planned-route.json:waypoints_sha256']=hashlib.sha256(json.dumps(route['waypoints_enu'],sort_keys=True,separators=(',',':')).encode()).hexdigest()
    return values


def telemetry(run):
    file='flight/track.json' if (run/'flight/track.json').is_file() else 'course-truth-track.json'
    data=read(run/file)
    samples=data.get('samples',[]) if isinstance(data,dict) else []
    observed=any(s.get('armed') is True and s.get('landed') is False
                 and isinstance(s.get('z'),(int,float)) and math.isfinite(s['z']) and s['z']>.25
                 for s in samples if isinstance(s,dict))
    known=any(isinstance(s,dict) and isinstance(s.get('armed'),bool) and isinstance(s.get('landed'),bool) for s in samples)
    return dict(state='observed armed airborne samples' if observed else 'no observed airborne flight' if known else 'arm/landed telemetry not archived',
                observed_flight=observed,source=file if data else None,sample_count=len(samples))


def seed(run, name=None, kind='flight attempted', baseline=None, annotations=None):
    run=Path(run).resolve(strict=True)
    identifier=identity(run)
    current=snapshot(run);previous=snapshot(baseline) if baseline else {}
    changes=[]
    if baseline:
        for key in sorted(current.keys()|previous.keys()):
            before=previous.get(key,'not archived');after=current.get(key,'not archived')
            if before!=after:
                file=key.split(':',1)[0]
                changes.append(dict(field=key,before=before,after=after,evidence=[file],provenance='archived field difference'))
    hashes={file:hashlib.sha256((run/file).read_bytes()).hexdigest() for file in ARCHIVES if (run/file).is_file() and not (run/file).is_symlink()}
    settings=read(run/'settings.json') or {}
    appearance=read(run/'appearance.json') or {}
    descriptors=[settings.get('height_profile'),settings.get('yaw_policy'),appearance.get('appearance_profile')]
    descriptors=[v for v in descriptors if isinstance(v,str)]
    label=name or ('Bench review' if kind=='bench' else 'Offline replay' if kind=='replay' else 'Flight attempt')+(' · '+' / '.join(descriptors) if descriptors else '')+' · '+run.name
    result=dict(schema=1,run_id=identifier,name=label,kind=kind,baseline_id=identity(Path(baseline)) if baseline else None,
                changes=changes,notes=[],confounds=['Only archived settings are compared. Equal recorded fields do not establish identical software, startup state, random sensor errors, or scene assets.'],
                evidence=list(hashes),source_sha256=hashes,telemetry=telemetry(run),snapshot=current,
                comparison_state='recorded differences' if changes else 'no recorded differences' if baseline else 'baseline not specified')
    if annotations:
        if any(key in annotations and annotations[key]!=result[key] for key in ('schema','run_id','kind','baseline_id')):
            raise ValueError('Annotations cannot change schema, run identity, kind or baseline')
        for key in ('name','notes','confounds'):
            if key in annotations:result[key]=annotations[key]
        for change in annotations.get('changes',[]):
            if not all(key in change for key in ('field','before','after','evidence')):
                raise ValueError('Annotated change needs field, before, after and evidence')
            if not change['evidence'] or any(not (run/file).is_file() or (run/file).is_symlink() or not (run/file).resolve().is_relative_to(run) for file in change['evidence']):
                raise ValueError('Annotated evidence must name existing run-local files')
            result['changes'].append(dict(change,provenance='human annotation; inspect evidence'))
        if annotations.get('changes'):result['comparison_state']='recorded differences with annotations'
    return result


def write(run, **kwargs):
    run=Path(run).resolve(strict=True);result=seed(run,**kwargs);target=run/'run-comparison.json'
    existing=read(target)
    if existing and existing.get('run_id')!=result['run_id']:
        raise ValueError('Existing immutable run identity differs')
    temp=target.with_suffix('.json.tmp');temp.write_text(json.dumps(result,indent=2,allow_nan=False)+'\n');os.replace(temp,target)
    return result


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('run',type=Path);parser.add_argument('--name');parser.add_argument('--kind',choices=KINDS,default='flight attempted')
    parser.add_argument('--baseline',type=Path);parser.add_argument('--annotations',type=Path)
    args=parser.parse_args()
    annotation=read(args.annotations) if args.annotations else None
    if args.annotations and annotation is None:parser.error('Invalid annotations JSON')
    output=write(args.run,name=args.name,kind=args.kind,baseline=args.baseline.resolve(strict=True) if args.baseline else None,annotations=annotation)
    print(output['run_id']+': '+output['name'])
