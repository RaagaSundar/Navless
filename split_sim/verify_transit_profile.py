#!/usr/bin/env python3
"""Read-only verification of the recorded transit recipe; never starts a mission."""
import argparse
import hashlib
import json
from pathlib import Path

HERE=Path(__file__).resolve().parent
PROFILE=HERE/'profiles/passed-centered-window/profile.json'


def digest(path):return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def contained(root, member):
    relative=Path(member)
    path=(root/relative).resolve()
    if relative.is_absolute() or not path.is_relative_to(root.resolve()):
        raise ValueError('profile asset escapes its root')
    return path


def verify(runtime=HERE/'course_runtime', appearance=None, camera=None, profile_path=PROFILE):
    profile=json.loads(Path(profile_path).read_text())
    expected=profile['runtime_files_sha256']
    actual={line.split('  ',1)[1]:line.split('  ',1)[0]
            for line in (Path(runtime)/'sources.sha256').read_text().splitlines()}
    if set(actual)!=set(expected):raise ValueError('runtime member list differs from recorded profile')
    for name,wanted in expected.items():
        path=contained(Path(runtime),name)
        if digest(path)!=actual[name]:raise ValueError('local runtime differs from its own manifest: '+name)
        if name=='window-forward-camera-candidate.json' and profile['config_metadata_only_exceptions']:
            config=json.loads(path.read_text())
            for key in profile['config_metadata_only_exceptions']:config.pop(key,None)
            semantic=hashlib.sha256(json.dumps(config,sort_keys=True,separators=(',',':')).encode()).hexdigest()
            if semantic!=profile['window_config_semantic_sha256']:raise ValueError('window yaw behavior/config changed')
        elif actual[name]!=wanted:raise ValueError('runtime no longer matches recorded transit: '+name)
    if appearance:
        for name,wanted in profile['appearance_visual_files_sha256'].items():
            if digest(contained(Path(appearance),name))!=wanted:raise ValueError('appearance visual bytes differ: '+name)
    if camera and digest(camera)!=profile['camera_sha256']:raise ValueError('camera/vehicle bytes differ')
    return {'profile':profile['profile'],'runtime_matches':True,'appearance_checked':bool(appearance),
            'camera_checked':bool(camera),'environment':profile['environment'],
            'recorded_graded_passed':profile.get('graded_passed',False),'qualification':'One recorded simulation outcome; source equality is not approval to arm or hardware qualification.'}


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--profile-json',type=Path,default=PROFILE)
    parser.add_argument('--runtime',type=Path,default=HERE/'course_runtime')
    parser.add_argument('--appearance',type=Path)
    parser.add_argument('--camera',type=Path)
    args=parser.parse_args()
    try:print(json.dumps(verify(args.runtime,args.appearance,args.camera,args.profile_json),indent=2))
    except (OSError,ValueError,KeyError) as error:parser.exit(1,str(error)+'\n')
