#!/usr/bin/env python3
"""Verify opt-in renderer assets before launching; no simulator/ROS commands."""
import argparse
import hashlib
import json
from pathlib import Path
import xml.etree.ElementTree as ET

PROFILE = 'synthetic-nonrepeating-neutral-v1'


def digest(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def verify(overlay, expected_manifest, world, resource_path, camera):
    overlay, world, camera = Path(overlay).resolve(), Path(world).resolve(), Path(camera)
    sidecar = overlay / 'appearance.json'
    if digest(sidecar) != expected_manifest:
        raise ValueError('unreviewed appearance manifest')
    manifest = json.loads(sidecar.read_text())
    if manifest.get('appearance_profile') != PROFILE or manifest.get('synthetic') is not True:
        raise ValueError('unknown or unlabeled synthetic appearance')
    if manifest.get('schema_version') != 2 or manifest.get('asset_root') != '.':
        raise ValueError('portable schema 2 manifest required')
    members = manifest['verified_textured_files_sha256']
    required = ('models/dh_course/model.sdf', 'models/dh_course/meshes/course_visual.obj',
                'models/dh_course/meshes/course.mtl', 'worlds/dronehacks-synthetic.sdf',
                'models/synthetic_ground/materials/textures/ground.png')
    if not all(name in members for name in required):
        raise ValueError('required asset hash missing')
    for member, expected in members.items():
        relative = Path(member)
        target = (overlay / relative).resolve()
        if relative.is_absolute() or not target.is_relative_to(overlay) or digest(target) != expected:
            raise ValueError('synthetic asset containment or hash mismatch')
    if world != overlay / 'worlds/dronehacks-synthetic.sdf':
        raise ValueError('synthetic world was not selected')
    if Path(resource_path.split(':')[0]).resolve() != overlay / 'models':
        raise ValueError('synthetic models must precede the original models')
    tree = ET.parse(world)
    if tree.find('world') is None or tree.find('world').get('name') != 'dronehacks':
        raise ValueError('expected isolated dronehacks world')
    if not any((x.text or '').startswith('model://synthetic_ground/') for x in tree.iter('albedo_map')):
        raise ValueError('synthetic ground albedo missing')
    if not any(x.text == 'model://dh_course' for x in tree.findall('.//include/uri')):
        raise ValueError('published course include missing')
    if not camera.is_file() or camera.name != 'dh_quad_stereo.sdf':
        raise ValueError('retain the existing qualified split camera/vehicle file')
    return {'schema_version': 1, 'appearance_profile': PROFILE, 'synthetic': True,
            'appearance_sha256': expected_manifest, 'world_sha256': digest(world),
            'camera_sha256': digest(camera),
            'qualification': 'Asset verification only; no stereo/VIO/flight/hardware qualification.'}


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--overlay', type=Path, required=True)
    parser.add_argument('--manifest-sha256', required=True)
    parser.add_argument('--world', type=Path, required=True)
    parser.add_argument('--resource-path', required=True)
    parser.add_argument('--camera', type=Path, required=True)
    args = parser.parse_args()
    print(json.dumps(verify(args.overlay, args.manifest_sha256, args.world,
                            args.resource_path, args.camera), indent=2))
