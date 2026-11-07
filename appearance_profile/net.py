#!/usr/bin/env python3
"""Opt-in open strand NET visual, derived from CAD bounds; no live commands.

Organizer CAD contains solid thin proxies, not a measured weave. Pitch and
strand diameter are explicit assumptions. All collision bytes stay unchanged.
"""
import argparse
import hashlib
import json
import math
from pathlib import Path
import re
import shutil
import xml.etree.ElementTree as ET
import zipfile

PROFILE='synthetic-neutral-open-net-v2'
CAD_GROUPS={'Net side north':'north','Net side south':'south','Net side east':'east',
            'Net side west':'west','Net roof':'roof'}


def digest(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def cad_panels(archive):
    with zipfile.ZipFile(archive) as z:
        obj=z.read('dronehacks-course-v3.obj').decode()
        readme=z.read('README.txt').decode()
        mtl=z.read('dronehacks-course-v3.mtl').decode()
    vertices=[];faces={};materials={};group=None
    for line in obj.splitlines():
        if line.startswith('v '):vertices.append(tuple(map(float,line.split()[1:4])))
        elif line.startswith('g '):group=line[2:].strip()
        elif line.startswith('usemtl ') and group in CAD_GROUPS:materials[group]=line[7:].strip()
        elif line.startswith('f ') and group in CAD_GROUPS:
            faces.setdefault(group,[]).append([int(r.split('/')[0])-1 for r in line.split()[1:]])
    result={}
    for name in CAD_GROUPS:
        pts=[vertices[i] for face in faces[name] for i in face]
        lo=[min(p[k] for p in pts) for k in range(3)];hi=[max(p[k] for p in pts) for k in range(3)]
        result[name]={'low':lo,'high':hi,'triangle_count':len(faces[name]),
                      'unique_vertex_count':len(set(i for f in faces[name] for i in f)),
                      'material':materials[name]}
    return result,{'zip_sha256':digest(archive),'README':readme,
        'obj_sha256':hashlib.sha256(obj.encode()).hexdigest(),
        'mtl_sha256':hashlib.sha256(mtl.encode()).hexdigest(),
        'interpretation':'five thin solid panel proxies; no strand/pitch geometry in organizer CAD'}


def positions(low,high,pitch,diameter):
    count=max(2,round((high-low-diameter)/pitch)+1)
    return [low+diameter/2+i*(high-low-diameter)/(count-1) for i in range(count)]


def strand_mesh(path,panels,pitch,diameter,sides=6):
    vertices=[];faces=[];groups=[];metadata={}
    def cylinder(a,b):
        face_start=len(faces)
        axis=next(k for k in range(3) if abs(b[k]-a[k])>1e-10)
        uv=[k for k in range(3) if k!=axis];start=len(vertices)
        for center in (a,b):
            for j in range(sides):
                p=list(center);t=2*math.pi*j/sides
                p[uv[0]]+=diameter/2*math.cos(t);p[uv[1]]+=diameter/2*math.sin(t)
                vertices.append(p)
        for j in range(sides):
            k=(j+1)%sides
            faces.extend([(start+j,start+k,start+sides+k),(start+j,start+sides+k,start+sides+j)])
        for j in range(1,sides-1):
            faces.extend([(start,start+j+1,start+j),(start+sides,start+sides+j,start+sides+j+1)])
        if axis==1:
            faces[face_start:]=[(a,c,b) for a,b,c in faces[face_start:]]
    for name,panel in panels.items():
        begin=len(faces);lo,hi=panel['low'],panel['high']
        normal=min(range(3),key=lambda k:hi[k]-lo[k]);uv=[k for k in range(3) if k!=normal]
        center=(lo[normal]+hi[normal])/2
        u=positions(lo[uv[0]],hi[uv[0]],pitch,diameter)
        v=positions(lo[uv[1]],hi[uv[1]],pitch,diameter)
        for constant,run,fixed in [(u,uv[1],uv[0]),(v,uv[0],uv[1])]:
            for c in constant:
                a=[0.,0.,0.];b=[0.,0.,0.]
                a[normal]=b[normal]=center;a[fixed]=b[fixed]=c
                a[run]=lo[run];b[run]=hi[run]
                cylinder(a,b)
        groups.append((name,begin,len(faces)))
        actual=[u[1]-u[0],v[1]-v[0]]
        metadata[name]={**panel,'normal_axis':normal,'center_plane_m':center,
                        'projection_axes':uv,'strand_count':len(u)+len(v),
                        'actual_pitch_m':actual,'nominal_open_area_fraction':
                        (1-diameter/actual[0])*(1-diameter/actual[1]),
                        'triangle_count':len(faces)-begin}
    # A single mesh draw call contains disconnected solid strands and TRUE holes.
    lines=['# OPEN NET VISUAL: CAD panel bounds; assumed weave, not a wall\n',
           'mtllib net_strands.mtl\n']
    lines += ['v '+' '.join(f'{x:.9f}' for x in p)+'\n' for p in vertices]
    normals=[]
    for f in faces:
        a,b,c=[vertices[i] for i in f];u=[b[k]-a[k] for k in range(3)];v=[c[k]-a[k] for k in range(3)]
        n=[u[1]*v[2]-u[2]*v[1],u[2]*v[0]-u[0]*v[2],u[0]*v[1]-u[1]*v[0]]
        length=math.sqrt(sum(x*x for x in n));normals.append([x/length for x in n])
    lines += ['vn '+' '.join(f'{x:.7f}' for x in n)+'\n' for n in normals]
    lines.append('usemtl net_source_dark\n')
    for name,start,end in groups:
        lines.append('g '+name+'\n')
        for i in range(start,end):lines.append('f '+' '.join(f'{v+1}//{i+1}' for v in faces[i])+'\n')
    Path(path).write_text(''.join(lines))
    return metadata,vertices,faces


def collision_bytes(text):
    return re.findall(r'<collision\b.*?</collision>',text,flags=re.S)


def without_net_visuals(text):
    tree=ET.fromstring(text)
    for parent in tree.iter():
        for child in list(parent):
            if child.tag=='visual' and (child.get('name','').startswith('net_')):
                parent.remove(child)
    return ET.tostring(tree)


def generate(source,output,archive,pitch=.0508,diameter=.00127):
    source,output=Path(source).resolve(),Path(output).resolve()
    if source==output or source in output.parents or output in source.parents:
        raise ValueError('new net profile must be a separate sibling root')
    if output.exists():raise FileExistsError('immutable output already exists')
    if not .015<=pitch<=.15 or not .0005<=diameter<=.005 or diameter>=pitch/5:
        raise ValueError('reviewed limits: pitch15..150mm,strand0.5..5mm,at least80% linear openings')
    panels,provenance=cad_panels(archive)
    if any(p.is_symlink() for p in source.rglob('*')):
        raise ValueError('source asset tree must not contain symlinks')
    source_hashes={str(p.relative_to(source)):digest(p) for p in source.rglob('*') if p.is_file()}
    parent=json.loads((source/'appearance.json').read_text())
    if parent.get('schema_version') != 2 or parent.get('appearance_profile') != 'synthetic-nonrepeating-neutral-v1':
        raise ValueError('portable neutral parent profile required')
    expected=parent.get('verified_textured_files_sha256',{})
    if not expected or any(source_hashes.get(name) != value for name,value in expected.items()):
        raise ValueError('parent profile assets differ from recorded hashes')
    relative=Path('models/dh_course/model.sdf');baseline=(source/relative).read_text()
    existing=ET.fromstring(baseline).find('model/link')
    if existing is None:raise ValueError('expected static course link')
    net_names=[n.get('name') for n in existing.findall('visual') if n.get('name','').startswith('net_')]
    if set(net_names)!={'net_north_v','net_east_v','net_west_v'}:
        raise ValueError('input must be reviewed three-pane baseline net, not another variant')
    shutil.copytree(source,output)
    meshdir=output/'models/dh_course/meshes'
    metadata,vertices,faces=strand_mesh(meshdir/'net_open_strands.obj',panels,pitch,diameter)
    (meshdir/'net_strands.mtl').write_text('# Net dark color follows organizer CAD material; strands opaque, holes empty\nnewmtl net_source_dark\nKa 0.09804 0.09804 0.09804\nKd 0.09804 0.09804 0.09804\nKs 0 0 0\nd 1\nNs 0\n')
    visual='''<visual name="net_open_strands_v"><geometry><mesh><uri>model://dh_course/meshes/net_open_strands.obj</uri></mesh></geometry><cast_shadows>false</cast_shadows></visual>'''
    pattern=r'<visual name="net_(?:north|east|west)_v">.*?</visual>'
    matches=list(re.finditer(pattern,baseline,flags=re.S))
    if len(matches)!=3:raise ValueError('baseline net visuals not uniquely matched')
    replacement=re.sub(pattern,lambda m:visual if m.start()==matches[0].start() else '',baseline,flags=re.S)
    assert collision_bytes(baseline)==collision_bytes(replacement)
    assert without_net_visuals(baseline)==without_net_visuals(replacement)
    (output/relative).write_text(replacement)
    manifest={'appearance_profile':PROFILE,'parent_appearance_profile':parent['appearance_profile'],
        'schema_version':2,'asset_root':'.',
        'parent_appearance_sha256':digest(source/'appearance.json'),
        'world_for_renderer':'worlds/dronehacks-synthetic.sdf',
        'synthetic':True,'net_visual_type':'opaque dark cylindrical strands with real geometric holes; no alpha plane',
        'organizer_source':provenance,'panels':metadata,'nominal_pitch_m':pitch,
        'strand_diameter_m':diameter,'cross_section_sides':6,
        'assumptions':['weave pitch and strand diameter are NOT measured; explicit opt-in assumptions',
          'CAD gives panel bounds/material only; Scaniverse cannot validate weave or metric registration',
          'straight taut orthogonal strands, no measured sag/knots/support poles',
          'no shadows, matching prior visual shadow setting; no invented opaque frames or marks'],
        'collision_preservation':'all collision elements byte-identical to parent; solid physical safety envelope retained',
        'collision_sha256':hashlib.sha256(''.join(collision_bytes(baseline)).encode()).hexdigest(),
        'physics_mismatch_disclosure':'net collisions remain solid20mm boxes with6.2m spans, not strand holes; inner faces±3.048m/roof3.048m match intended CAD volume',
        'old_visual_disclosure':'three uniform90%-transparent thin boxes replaced; missing south/roof optical net restored from CAD semantics',
        'mesh_vertex_count':len(vertices),'mesh_triangle_count':len(faces),
        'verified_baseline_files_sha256':source_hashes,
        'qualification':'generator geometry/invariants only; passive Gazebo view/depth/performance and flight remain separate'}
    (output/'net-appearance-profile.json').write_text(json.dumps(manifest,indent=2)+'\n')
    (output/'SYNTHETIC_APPEARANCE.txt').write_text('OPT-IN SYNTHETIC APPEARANCE + OPEN NET VISUAL\nParent neutral pattern retained. Organizer CAD net extents/color; assumed50.8mm weave and1.27mm strands.\nTrue geometric holes, no alpha sheet/opaque wall. All collision bytes unchanged; physics net remains a solid envelope.\n')
    changed={str(p.relative_to(output)) for p in output.rglob('*') if p.is_file()
             and str(p.relative_to(output)) in source_hashes and digest(p)!=source_hashes[str(p.relative_to(output))]}
    assert changed<={str(relative),'appearance.json','SYNTHETIC_APPEARANCE.txt'}
    manifest['verified_textured_files_sha256']={str(p.relative_to(output)):digest(p)
         for p in output.rglob('*') if p.is_file() and p.name!='appearance.json'}
    manifest['changed_existing_files']=sorted(changed|{'appearance.json'})
    (output/'appearance.json').write_text(json.dumps(manifest,indent=2)+'\n')
    assert all(digest(source/f)==h for f,h in source_hashes.items())
    return manifest


if __name__=='__main__':
    ap=argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--source',type=Path,required=True);ap.add_argument('--output',type=Path,required=True)
    ap.add_argument('--organizer-zip',type=Path,required=True)
    ap.add_argument('--pitch-m',type=float,default=.0508);ap.add_argument('--strand-diameter-m',type=float,default=.00127)
    args=ap.parse_args();m=generate(args.source,args.output,args.organizer_zip,args.pitch_m,args.strand_diameter_m)
    print(json.dumps({k:m[k] for k in ['appearance_profile','mesh_vertex_count','mesh_triangle_count','collision_sha256','changed_existing_files']},indent=2))
