#!/usr/bin/env python3
"""Opt-in SYNTHETIC appearance profile; no ROS, simulator or operator commands.

No scan-derived pixels and no claim of hardware qualification. Geometry,
collisions and semantic painted course markings are preserved. Default assets
are read only. Pillow is the only non-stdlib dependency.
"""
import argparse
from collections import defaultdict
import hashlib
import json
import math
from pathlib import Path
import random
import shutil
import xml.etree.ElementTree as ET

PROFILE = 'synthetic-nonrepeating-neutral-v1'
SEMANTIC_PREFIXES = ('Landing pad', 'Launch mark', 'Guideline', 'Approach ring', 'Net ')


def digest(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def geometry_signature(path):
    """Hash exact vertex/normal/group/face geometry, excluding appearance UVs."""
    records = []
    for line in Path(path).read_text().splitlines():
        if line.startswith(('v ', 'vn ', 'g ', 'o ')):
            records.append(line)
        elif line.startswith('f '):
            refs = [r.split('/') for r in line.split()[1:]]
            records.append('f '+' '.join(r[0]+'//'+(r[2] if len(r)>2 else '') for r in refs))
    return hashlib.sha256(('\n'.join(records)+'\n').encode()).hexdigest()


def pattern(path, extent_m, key, seed, pixels_per_m):
    """Unique bounded atlas with unencoded random paint-like patches, in metres."""
    from PIL import Image, ImageDraw
    local_seed = int.from_bytes(hashlib.sha256(f'{seed}:{key}'.encode()).digest()[:8], 'big')
    rng = random.Random(local_seed)
    w,h = [max(16,math.ceil(v*pixels_per_m)) for v in extent_m]
    # Low-frequency neutral shading is nonperiodic, independently seeded per atlas.
    coarse = Image.new('L',(max(2,math.ceil(extent_m[0]/.20)),max(2,math.ceil(extent_m[1]/.20))))
    coarse.putdata([rng.randint(158,196) for _ in range(coarse.width*coarse.height)])
    image = coarse.resize((w,h),Image.Resampling.BICUBIC)
    draw = ImageDraw.Draw(image)
    area = extent_m[0]*extent_m[1]
    features = []
    # Two scales stay useful over the camera's usual working distances. There is
    # no lattice, tile reuse, fiducial code, semantic marker or per-image noise.
    for count, limits in [(max(1,math.ceil(area*80)),(.025,.075)),
                          (max(1,math.ceil(area*22)),(.080,.160))]:
        for _ in range(count):
            size = rng.uniform(*limits)
            aspect = rng.uniform(.55,1.35)
            cx,cy = rng.uniform(0,w),rng.uniform(0,h)
            rx,ry = size*pixels_per_m/2,size*aspect*pixels_per_m/2
            value = rng.choice((64,82,105,214,230))
            if rng.random()<.55:
                draw.ellipse((cx-rx,cy-ry,cx+rx,cy+ry),fill=value)
            else:
                angle = rng.uniform(0,2*math.pi)
                vertices = []
                corners=rng.choice((3,4,5))
                for j in range(corners):
                    t = angle+2*math.pi*j/corners
                    vertices.append((cx+rx*math.cos(t),cy+ry*math.sin(t)))
                draw.polygon(vertices,fill=value)
            features.append(size)
    image.save(path,format='PNG',optimize=False,compress_level=9)
    return {'pixel_size':[w,h],'extent_m':list(extent_m),'pixels_per_m':pixels_per_m,
            'feature_count':len(features),'feature_diameter_m':[min(features),max(features)],
            'gray_range':list(image.getextrema()),'seed_sha256':hashlib.sha256(str(local_seed).encode()).hexdigest(),
            'sha256':digest(path)}


def make_ground_world(world_source,output,seed,pixels_per_m=128):
    """Texture existing ground visual only; keep plane geometry and physics."""
    tree=ET.parse(world_source)
    world=tree.getroot().find('world')
    visual=world.find("./model[@name='ground']/link/visual")
    if visual is None or visual.find('geometry/plane') is None:
        raise ValueError('source world must have its existing named ground plane visual')
    size=list(map(float,visual.findtext('geometry/plane/size').split()))
    directory=output/'models/synthetic_ground/materials/textures'
    directory.mkdir(parents=True)
    info=pattern(directory/'ground.png',size,'world-ground-unique-full-extent',seed,pixels_per_m)
    material=visual.find('material')
    if material is None:material=ET.SubElement(visual,'material')
    if material.find('pbr') is not None:
        raise ValueError('source ground already has PBR; profile requires reviewed plain baseline')
    pbr=ET.SubElement(material,'pbr');metal=ET.SubElement(pbr,'metal')
    ET.SubElement(metal,'albedo_map').text='model://synthetic_ground/materials/textures/ground.png'
    ET.SubElement(metal,'metalness').text='0'
    ET.SubElement(metal,'roughness').text='1'
    destination=output/'worlds/dronehacks-synthetic.sdf'
    destination.parent.mkdir(parents=True)
    tree.write(destination,encoding='utf-8',xml_declaration=True)
    # Entire source world must be identical after removing ONLY this added PBR.
    compare=ET.parse(destination)
    compare_visual=compare.getroot().find("./world/model[@name='ground']/link/visual")
    compare_visual.find('material').remove(compare_visual.find('material/pbr'))
    assert ET.tostring(compare.getroot())==ET.tostring(ET.parse(world_source).getroot())
    info.update({'world_source_sha256':digest(world_source),
                 'world_generated_sha256':digest(destination),'world_relative_path':str(destination.relative_to(output)),
                 'changes':'only existing ground visual material gains neutral albedo; sky,lights,net,physics unchanged',
                 'map_uri':'model://synthetic_ground/materials/textures/ground.png'})
    return info


def generate(source, output, seed=20261107, pixels_per_m=256,world_source=None):
    source,output = Path(source).resolve(),Path(output).resolve()
    if source==output or source in output.parents or output in source.parents:
        raise ValueError('source and opt-in output must be separate sibling trees')
    if output.exists():
        raise FileExistsError('choose a new output directory; existing assets are never overwritten')
    if not 64<=pixels_per_m<=512:
        raise ValueError('pixels_per_m must be64..512')
    model = Path('models/dh_course')
    mesh = model/'meshes/course_visual.obj'
    original_obj = source/mesh
    original_mtl = source/model/'meshes/course.mtl'
    if any(p.is_symlink() for p in source.rglob('*')):
        raise ValueError('source asset tree must not contain symlinks')
    source_files = {str(p.relative_to(source)):digest(p) for p in source.rglob('*') if p.is_file()}
    world_source=Path(world_source) if world_source else Path(__file__).resolve().parents[1]/'gazebo_sim/worlds/dronehacks.sdf'
    lines = original_obj.read_text().splitlines(keepends=True)
    vertices,uv_count,group,material = [],0,'ungrouped',None
    faces,atlases = {},defaultdict(list)
    for index,line in enumerate(lines):
        if line.startswith('v '):
            vertices.append(tuple(map(float,line.split()[1:4])))
        elif line.startswith('vt '):
            uv_count += 1
        elif line.startswith('g '):
            group = line[2:].strip()
        elif line.startswith('usemtl '):
            material = line.split(maxsplit=1)[1].strip()
        elif line.startswith('f ') and not group.startswith(SEMANTIC_PREFIXES):
            refs = [r.split('/') for r in line.split()[1:]]
            ids = [int(r[0])-1 if int(r[0])>0 else len(vertices)+int(r[0]) for r in refs]
            points = [vertices[i] for i in ids]
            a,b = [[points[j][k]-points[0][k] for k in range(3)] for j in (1,2)]
            normal = (a[1]*b[2]-a[2]*b[1],a[2]*b[0]-a[0]*b[2],a[0]*b[1]-a[1]*b[0])
            dominant = max(range(3),key=lambda k:abs(normal[k]))
            if max(abs(v) for v in normal)<1e-16:
                continue
            axes = [(1,2),(0,2),(0,1)][dominant]
            key = (group,dominant,1 if normal[dominant]>=0 else -1,material)
            faces[index] = (key,refs,ids,points,axes)
            atlases[key].extend((p[axes[0]],p[axes[1]]) for p in points)
    if not faces:
        raise ValueError('no eligible fixed course surfaces in input OBJ')
    shutil.copytree(source,output)
    meshes = output/model/'meshes'
    manifest = {'profile':PROFILE,'opt_in':True,'synthetic':True,'seed':seed,
                'appearance_source':'procedural; NOT real scan imagery or calibrated reflectance',
                'qualification':'generated appearance only; no rendered stereo/flight/hardware qualification',
                'geometry_preserved':True,'collisions_preserved':True,
                'semantic_group_exclusions':list(SEMANTIC_PREFIXES),
                'source_files_sha256':source_files,'atlases':{},'textured_faces':len(faces)}
    bounds = {}
    for key,pts in sorted(atlases.items()):
        ident = hashlib.sha256(json.dumps(key).encode()).hexdigest()[:16]
        low = [min(p[k] for p in pts)-.008 for k in range(2)]
        high = [max(p[k] for p in pts)+.008 for k in range(2)]
        # Thin edges retain the same metric scale; a little unused atlas padding
        # avoids extreme aspect ratios without stretching the pattern.
        for k in range(2):
            if high[k]-low[k]<.0625:
                mid=(high[k]+low[k])/2;low[k],high[k]=mid-.03125,mid+.03125
        extent=[high[k]-low[k] for k in range(2)]
        name=f'synthetic_{ident}.png'
        info=pattern(meshes/name,extent,key,seed,pixels_per_m)
        info.update({'group':key[0],'projection_normal_axis':key[1],'normal_sign':key[2],
                     'base_material':key[3],'projection_low_m':low,'filename':name})
        manifest['atlases'][ident]=info
        bounds[key]=(ident,low,extent)
    mtl_blocks={}
    current=None
    for line in original_mtl.read_text().splitlines(keepends=True):
        if line.startswith('newmtl '):
            current=line.split(maxsplit=1)[1].strip();mtl_blocks[current]=[]
        elif current is not None:
            mtl_blocks[current].append(line)
    # Originals remain available, unchanged, for semantic exclusions. New
    # materials preserve original diffuse colors, transparency and shading.
    mtl = original_mtl.read_text()+'\n# OPT-IN SYNTHETIC neutral nonrepeating appearance; not scan-derived\n'
    for key,(ident,_,_) in sorted(bounds.items()):
        mtl+=f'newmtl synthetic_{ident}\n'
        mtl+=''.join(line for line in mtl_blocks[key[3]] if not line.startswith('map_'))
        mtl+=f'map_Kd synthetic_{ident}.png\n'
    new_uvs=[];body=[];active_original=None
    for index,line in enumerate(lines):
        if line.startswith('usemtl '):
            active_original=line.split(maxsplit=1)[1].strip()
        if index not in faces:
            if line.startswith('f '):body.append(f'usemtl {active_original}\n')
            body.append(line)
            continue
        key,refs,ids,points,axes=faces[index]
        ident,low,extent=bounds[key];tokens=[]
        for ref,p in zip(refs,points):
            u,v=[(p[axes[k]]-low[k])/extent[k] for k in range(2)]
            new_uvs.append(f'vt {u:.9f} {v:.9f}\n')
            vn=ref[2] if len(ref)>2 else ''
            tokens.append(f'{ref[0]}/{uv_count+len(new_uvs)}/{vn}')
        body.append(f'usemtl synthetic_{ident}\n')
        body.append('f '+' '.join(tokens)+'\n')
    # Existing UVs appear before source faces. Append new UVs immediately after
    # their last existing declaration, so old positive indices stay valid.
    insert=max((i+1 for i,line in enumerate(body) if line.startswith('vt ')),default=0)
    body[insert:insert]=new_uvs
    (output/mesh).write_text('# OPT-IN '+PROFILE+'; synthetic, not scan-derived\n'+''.join(body))
    (meshes/'course.mtl').write_text(mtl)
    assert geometry_signature(original_obj)==geometry_signature(output/mesh)
    changed={str(p.relative_to(output)) for p in output.rglob('*') if p.is_file()
             and str(p.relative_to(output)) in source_files
             and digest(p)!=source_files[str(p.relative_to(output))]}
    assert changed=={str(mesh),str(model/'meshes/course.mtl')},changed
    manifest['changed_existing_files']=sorted(changed)
    manifest['geometry_signature_sha256']=geometry_signature(original_obj)
    manifest['model_sdf_sha256']=digest(output/model/'model.sdf')
    manifest['outside_ground']=make_ground_world(world_source,output,seed)
    manifest['generated_files_sha256']={str(p.relative_to(output)):digest(p) for p in output.rglob('*')
                                      if p.is_file() and str(p.relative_to(output)) not in source_files}
    (output/'synthetic-appearance-profile.json').write_text(json.dumps(manifest,indent=2)+'\n')
    (output/'SYNTHETIC_APPEARANCE.txt').write_text(
        'OPT-IN GENERATED SYNTHETIC APPEARANCE\nNot a scan, real-course observation, or hardware qualification.\n'
        'Only OBJ UV/material references and MTL diffuse maps differ; source geometry, model.sdf,\n'
        'collisions, cameras, semantic painted markings and grader remain unchanged.\n')
    sidecar={'appearance_profile':PROFILE,'synthetic':True,'seed':seed,
             'schema_version':2,
             'asset_root':'.',
             'source_identity':'verified_baseline_files_sha256',
             'verified_baseline_files_sha256':source_files,
             'verified_textured_files_sha256':{str(p.relative_to(output)):digest(p)
                 for p in output.rglob('*') if p.is_file()},
             'geometry_signature_sha256':manifest['geometry_signature_sha256'],
             'collisions_model_sdf_sha256':manifest['model_sdf_sha256'],
             'existing_files_changed':manifest['changed_existing_files'],
             'world_source':'verified by world_source_sha256',
             'world_source_sha256':digest(world_source),
             'world_for_renderer':manifest['outside_ground']['world_relative_path'],
             'outside_ground':manifest['outside_ground'],
             'qualification':'generator invariants only; rendered stereo/flight not yet qualified',
             'run_archiving':'copy this exact appearance.json into selected run root before trial',
             'default_untouched':True}
    (output/'appearance.json').write_text(json.dumps(sidecar,indent=2)+'\n')
    return manifest


if __name__=='__main__':
    ap=argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--source',type=Path,required=True)
    ap.add_argument('--output',type=Path,required=True)
    ap.add_argument('--seed',type=int,default=20261107)
    ap.add_argument('--pixels-per-m',type=int,default=256)
    ap.add_argument('--world-source',type=Path)
    args=ap.parse_args()
    result=generate(args.source,args.output,args.seed,args.pixels_per_m,args.world_source)
    print(json.dumps({k:result[k] for k in ['profile','synthetic','textured_faces','changed_existing_files','model_sdf_sha256']},indent=2))
    print('atlases:',len(result['atlases']))
