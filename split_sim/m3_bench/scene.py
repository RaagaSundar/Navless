#!/usr/bin/env python3
"""Offline oracle fixture builder; altered geometry is evaluation-only, never planner input."""
import datetime
import argparse, collections, difflib, hashlib, json, math, shutil, sys
from pathlib import Path
import xml.etree.ElementTree as ET
import numpy as np
ROOT=next((candidate for candidate in (Path(__file__).resolve().parents[2],Path('/workspace/Navless')) if (candidate/'gazebo_sim/dh/geom.py').is_file()),Path(__file__).resolve().parents[2])
sys.path.insert(0,str(ROOT/'gazebo_sim'))
from dh.geom import Obstacles


def sha(p): return hashlib.sha256(p.read_bytes()).hexdigest()
def rot2(p, pivot, a):
    c,s=math.cos(a),math.sin(a);x,y=p[0]-pivot[0],p[1]-pivot[1]
    return [pivot[0]+c*x-s*y,pivot[1]+s*x+c*y,*p[2:]]

def rotate_obj(text,pivot,angle):
    lines=text.splitlines(keepends=True);vs=[];ns=[];group=None;owned=set();other=set();normal_ids=set();faces=[]
    for i,l in enumerate(lines):
        if l.startswith('v '):vs.append(i)
        elif l.startswith('vn '):ns.append(i)
        elif l.startswith('g '):group=l[2:].strip()
        elif l.startswith('f '):
            tokens=l.split()[1:]
            ids={int(t.split('/')[0]) for t in tokens}
            if group=='M3 maze divider':
                owned|=ids;faces.append((i,tokens))
                normal_ids|={int(t.split('/')[2]) for t in tokens if len(t.split('/'))>2 and t.split('/')[2]}
            else:other|=ids
    assert owned and not owned&other, 'M3 must have exclusive vertex ownership'
    for v in owned:
        q=list(map(float,lines[vs[v-1]].split()[1:]));q=rot2(q,pivot,angle)
        lines[vs[v-1]]='v '+' '.join(f'{x:.9f}' for x in q)+'\n'
    # Normals can be shared; duplicate transformed M3 normals, leaving every other face untouched.
    newids={n:len(ns)+j+1 for j,n in enumerate(sorted(normal_ids))}
    additions=[]
    for n in sorted(normal_ids):
        q=list(map(float,lines[ns[n-1]].split()[1:]));q=rot2(q,[0,0],angle)
        additions.append('vn '+' '.join(f'{x:.9f}' for x in q)+'\n')
    for i,tokens in faces:
        changed=[]
        for t in tokens:
            fields=t.split('/')
            if len(fields)>2 and fields[2]:fields[2]=str(newids[int(fields[2])])
            changed.append('/'.join(fields))
        lines[i]='f '+' '.join(changed)+'\n'
    # OBJ indices are global; definitions precede faces for strict readers.
    first=next(i for i,l in enumerate(lines) if l.startswith('f '))
    lines[first:first]=additions
    return ''.join(lines),{'vertex_count':len(owned),'normal_count':len(normal_ids),'face_count':len(faces)}


def dense(points,step=.005):
    out=[]
    for a,b in zip(points,points[1:]):
        n=max(1,math.ceil(np.linalg.norm(b-a)/step));out.extend(a+(b-a)*i/n for i in range(n))
    return np.array(out+[points[-1]])

def validate(source,out,route):
    base=Obstacles(source/'models/dh_course/model.sdf');changed=Obstacles(out/'models/dh_course/model.sdf')
    p=dense(np.array(json.loads(route.read_text())['waypoints_enu']));idx=base.names.index('M3_wall')
    before=base.sdf(p)[:,idx]-.30;after=changed.sdf(p)[:,idx]-.30
    assert before.min()>0 and after.min()<-.05
    # Evaluation-only existence proof in full collision geometry. Not a planner/controller route.
    xs=np.arange(-.70,1.31,.02);ys=np.arange(.82,2.71,.02);z=.95
    grid=np.array([[x,y,z] for x in xs for y in ys]);ds=changed.sdf(grid).min(axis=1).reshape(len(xs),len(ys))
    safe=ds>.35
    def cell(pt):return (int(np.argmin(abs(xs-pt[0]))),int(np.argmin(abs(ys-pt[1]))))
    start,goal=cell([-.60,1.00]),cell([.70,2.58]);assert safe[start] and safe[goal]
    queue=collections.deque([start]);parents={start:None}
    while queue:
        a=queue.popleft()
        if a==goal:break
        for di,dj in ((1,0),(-1,0),(0,1),(0,-1)):
            b=(a[0]+di,a[1]+dj)
            if 0<=b[0]<len(xs) and 0<=b[1]<len(ys) and safe[b] and b not in parents:parents[b]=a;queue.append(b)
    assert goal in parents,'No .35m collision-free maze corridor'
    path=[];at=goal
    while at is not None:path.append([xs[at[0]],ys[at[1]],z]);at=parents[at]
    path=np.array(path[::-1]);q=dense(path);minclear=float(changed.sdf(q).min()-.30)
    assert minclear>.045
    (out/'evaluation-only-corridor.json').write_text(json.dumps({'warning':'oracle feasibility proof ONLY; never navigation input','points_enu':path.tolist()},indent=2)+'\n')
    return {'route_sha256':sha(route),'conservative_sphere_radius_m':.30,'original_route_M3_margin_m':float(before.min()),'changed_route_M3_margin_m':float(after.min()),'changed_route_intrusion_enu':p[after.argmin()].tolist(),'feasible_corridor_min_sphere_margin_m':minclear,'feasible_corridor_length_m':float(np.linalg.norm(np.diff(path,axis=0),axis=1).sum()),'feasibility_grid_m':.02,'dense_check_m':.005,'feasibility_scope':'evaluation-only full collision model; .35m-inflated 4-connected local maze corridor, not flight or dynamic qualification'}


def build(source,out,route):
    assert not out.exists(),'Immutable output must not exist'
    shutil.copytree(source,out)
    pivot=[.3048,2.12085];angle=math.radians(-18)
    sdf=out/'models/dh_course/model.sdf';obj=out/'models/dh_course/meshes/course_visual.obj'
    tree=ET.parse(sdf);col=tree.find(".//collision[@name='M3_wall']");old=list(map(float,col.findtext('pose').split()));new=rot2(old,pivot,angle);new[5]+=angle
    # Preserve all unrelated XML bytes by replacing exactly the M3 pose text.
    sdf.write_text(sdf.read_text().replace('<pose>'+col.findtext('pose')+'</pose>','<pose>'+' '.join(f'{v:.9f}' for v in new)+'</pose>',1))
    rotated,counts=rotate_obj(obj.read_text(),pivot,angle);obj.write_text(rotated)
    original_files={str(p.relative_to(source)):sha(p) for p in source.rglob('*') if p.is_file()}
    final_files={str(p.relative_to(out)):sha(p) for p in out.rglob('*') if p.is_file()}
    changed=[p for p,h in final_files.items() if original_files.get(p)!=h]
    assert sorted(changed)==['models/dh_course/meshes/course_visual.obj','models/dh_course/model.sdf']
    check=validate(source,out,route)
    manifest={'created_utc':datetime.datetime.now(datetime.timezone.utc).isoformat(),'test_kind':'passive bench, no actual flight','baseline_id':source.name,'what_changed':['M3 visual OBJ vertices/normals and collision pose rotated -18deg at north endpoint (.3048,2.12085)m ENU','All other scene files copied byte-for-byte; original onboard OBJ/map/route unchanged','No sensor or calibration changes; no active world commands'],'fixture':'M3 north-pivot rotation -18deg passive bench','source':str(source),'source_sha256':original_files,'variant_sha256':final_files,'changed_files':changed,'angle_enu_deg':-18,'pivot_enu_xy_m':pivot,'old_collision_pose':old,'new_collision_pose':new,'wall_dimensions_m':[1.6129,.1016,1.7272],'choice':'fixed north tip preserves maze-exit opening, lower tip swings west into intended M2-to-M3 path; modest angle leaves independently verified alternate corridor','obj_edit':counts,'oracle_scope':'environment fault + separate evaluation only; onboard map/OBJ/route untouched','validation':check}
    (out/'fault-fixture.json').write_text(json.dumps(manifest,indent=2)+'\n')
    diff=''.join(difflib.unified_diff((source/'models/dh_course/model.sdf').read_text().splitlines(True),sdf.read_text().splitlines(True),fromfile='baseline/model.sdf',tofile='variant/model.sdf'))
    (out/'collision.diff').write_text(diff)
    # Large mesh line diff uses a linear scan; normals are duplicated and face indices remapped.
    (out/'mesh-edit-summary.json').write_text(json.dumps(counts,indent=2)+'\n')
    print(json.dumps(check,indent=2))


def bundle(source,out,route,variant):
    source,out,route=Path(source).resolve(),Path(out).resolve(),Path(route).resolve()
    if out.exists() or source==out or source in out.parents or out in source.parents:
        raise ValueError('new separate immutable bundle directory required')
    if any(p.is_symlink() for p in source.rglob('*')):raise ValueError('source symlinks forbidden')
    prior=ROOT/'gazebo_sim/models/dh_course/meshes/course_visual.obj'
    def geometry(path):
        lines=path.read_text().splitlines()
        return ([list(map(float,line.split()[1:])) for line in lines if line.startswith('v ')],
                [[int(token.split('/')[0]) for token in line.split()[1:]] for line in lines if line.startswith('f ')])
    if geometry(source/'models/dh_course/meshes/course_visual.obj')!=geometry(prior):
        raise ValueError('nominal actual geometry must coincide with the original public planning prior')
    if sha(source/'models/dh_course/model.sdf')!=sha(ROOT/'gazebo_sim/models/dh_course/model.sdf'):
        raise ValueError('reviewed original-perimeter physical model required')
    out.mkdir(parents=True)
    assets=out/'assets'
    if variant=='changed':
        build(source,assets,route)
        fault=json.loads((assets/'fault-fixture.json').read_text());fault.pop('source',None)
        (assets/'fault-fixture.json').write_text(json.dumps(fault,indent=2)+'\n')
    elif variant=='nominal':
        shutil.copytree(source,assets)
    else:raise ValueError('unknown M3 variant')
    prior=ROOT/'gazebo_sim/models/dh_course/meshes/course_visual.obj'
    for tag,tree in [('baseline',source),('actual',assets)]:
        for member in ('models/dh_course/model.sdf','models/dh_course/meshes/course_visual.obj'):
            destination=out/'map-assets'/tag/member;destination.parent.mkdir(parents=True,exist_ok=True);shutil.copy2(tree/member,destination)
    (out/'provenance').mkdir();shutil.copy2(prior,out/'provenance/preloaded-course_visual.obj')
    profile='m3-bounded-'+variant
    appearance={'schema_version':2,'appearance_profile':profile,'synthetic':True,'asset_root':'.',
       'parent_appearance_sha256':sha(source/'appearance.json'),'world_for_renderer':'worlds/dronehacks-synthetic.sdf',
       'oracle_scope':'environment/evaluation only; navigation original map and route unchanged',
       'verified_textured_files_sha256':{str(p.relative_to(assets)):sha(p) for p in assets.rglob('*') if p.is_file() and p.name!='appearance.json'}}
    (assets/'appearance.json').write_text(json.dumps(appearance,indent=2)+'\n')
    model='models/dh_course/model.sdf';visual='models/dh_course/meshes/course_visual.obj'
    comparison={'schema_version':1,'frame':'world ENU metres','test_name':'Changed M3 −18° · bounded stereo capture' if variant=='changed' else 'Nominal M3 · bounded stereo capture',
       'variant':variant,'guard_activity':{'mode':'off','note':'shadow receipts must be archived separately; no avoidance controller'},'geometry_identical':variant=='nominal','preloaded_map':{'unchanged':True,'source':{'path':'provenance/preloaded-course_visual.obj','sha256':sha(prior)}},
       'changed_walls':[] if variant=='nominal' else [{'name':'M3','angle_enu_deg':-18,'pivot_enu_xy_m':[.3048,2.12085]}]}
    for tag in ('baseline','actual'):
        comparison[tag]={key:{'path':f'map-assets/{tag}/{member}','sha256':sha(out/'map-assets'/tag/member)} for key,member in [('model',model),('visual',visual)]}
    (out/'map-comparison.json').write_text(json.dumps(comparison,indent=2)+'\n')
    meta={'schema_version':1,'generator_sha256':sha(Path(__file__)),'profile':profile,'variant':variant,'route_screen_source_sha256':sha(route),
       'file_sha256':{str(p.relative_to(out)):sha(p) for p in out.rglob('*') if p.is_file()},
       'qualification':'generated scene only; no flight or avoidance result','what_changed':['M3 visual vertices/normals and physical collision rotate −18 degrees at fixed north tip' if variant=='changed' else 'physical M3 remains nominal','onboard prior, route, sensors and estimator unchanged']}
    (out/'bundle.json').write_text(json.dumps(meta,indent=2)+'\n')
    return meta


def verify_bundle(root,expected=None):
    root=Path(root).resolve()
    if expected and sha(root/'bundle.json')!=expected:raise ValueError('unreviewed bundle digest')
    meta=json.loads((root/'bundle.json').read_text())
    if meta.get('generator_sha256') != sha(Path(__file__)):raise ValueError('scene verifier/generator source differs from pinned bundle')
    for member,wanted in meta['file_sha256'].items():
        relative=Path(member);target=(root/relative).resolve()
        if relative.is_absolute() or not target.is_relative_to(root) or sha(target)!=wanted:raise ValueError('scene file containment/hash mismatch: '+member)
    assets=root/'assets';world=assets/'worlds/dronehacks-synthetic.sdf'
    if ET.parse(world).find('world').get('name')!='dronehacks':raise ValueError('wrong world')
    return {'profile':meta['profile'],'variant':meta['variant'],'world':str(world),'world_sha256':sha(world),
       'appearance_sidecar':str(assets/'appearance.json'),'appearance_sha256':sha(assets/'appearance.json'),
       'course_model_sha256':sha(assets/'models/dh_course/model.sdf'),'fixture_bundle_sha256':sha(root/'bundle.json'),
       'preloaded_map_unchanged':True,'qualification':'source scene receipt only; not flight evidence'}


def archive_bundle(bundle_root,run):
    record=verify_bundle(bundle_root);run=Path(run)
    if (run/'map-comparison.json').exists():raise FileExistsError('scene receipt already exists')
    for name in ('map-assets','provenance'):shutil.copytree(Path(bundle_root)/name,run/name)
    meta=json.loads((Path(bundle_root)/'map-comparison.json').read_text());meta['run_id']=run.name
    meta['actual_scene_verification']={'status':'generated geometry only; compare actual renderer receipt separately','expected':record}
    receipt=run/'renderer-source.json'
    if receipt.exists():
        observed=json.loads(receipt.read_text())
        if observed.get('course_model_sha256')!=record['course_model_sha256'] or observed.get('world_sha256')!=record['world_sha256']:raise ValueError('active renderer does not match selected scene')
        meta['actual_scene_verification']={'status':'verified actual renderer receipt','renderer_receipt_sha256':sha(receipt)}
    (run/'map-comparison.json').write_text(json.dumps(meta,indent=2)+'\n')
    return meta


if __name__=='__main__':
    ap=argparse.ArgumentParser(description=__doc__);sub=ap.add_subparsers(dest='mode',required=True)
    p=sub.add_parser('build');p.add_argument('--source',type=Path,required=True);p.add_argument('--output',type=Path,required=True);p.add_argument('--route',type=Path,required=True);p.add_argument('--variant',choices=('nominal','changed'),required=True)
    p=sub.add_parser('verify');p.add_argument('root',type=Path);p.add_argument('--expected-sha256')
    p=sub.add_parser('archive');p.add_argument('root',type=Path);p.add_argument('run',type=Path)
    a=ap.parse_args()
    if a.mode=='build':print(json.dumps(bundle(a.source,a.output,a.route,a.variant),indent=2))
    elif a.mode=='verify':print(json.dumps(verify_bundle(a.root,a.expected_sha256),indent=2))
    else:print(json.dumps(archive_bundle(a.root,a.run),indent=2))
