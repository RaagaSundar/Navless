import hashlib
import json
from pathlib import Path
import shutil
import tempfile
import unittest
from scene import bundle,verify_bundle,archive_bundle,rotate_obj
from mission_m3_capture import load_contract

ROOT=Path(__file__).resolve().parents[2]
HERE=Path(__file__).resolve().parent


class SceneTests(unittest.TestCase):
    def test_synthetic_rotation_does_not_modify_other_group(self):
        obj='v 0 0 0\nv 1 0 0\nv 0 0 1\nv 2 0 0\nv 3 0 0\nv 2 0 1\nvn 0 1 0\ng M3 maze divider\nf 1//1 2//1 3//1\ng untouched\nf 4//1 5//1 6//1\n'
        output,counts=rotate_obj(obj,[0,0],.3)
        self.assertIn('g untouched\nf 4//1 5//1 6//1',output)
        self.assertIn('v 2 0 0\nv 3 0 0\nv 2 0 1',output)
        self.assertEqual(counts['vertex_count'],3)

    def test_full_source_scene_hashes_archive_and_tamper(self):
        # Tracked public geometry only; no private fixture, simulator or camera capture.
        with tempfile.TemporaryDirectory() as tmp:
            root=Path(tmp);source=root/'base';(source/'models/dh_course/meshes').mkdir(parents=True)
            for name in ('model.sdf','meshes/course_visual.obj'):
                shutil.copy2(ROOT/'gazebo_sim/models/dh_course'/name,source/'models/dh_course'/name)
            (source/'worlds').mkdir();shutil.copy2(ROOT/'gazebo_sim/worlds/dronehacks.sdf',source/'worlds/dronehacks-synthetic.sdf')
            (source/'appearance.json').write_text('{"synthetic":true}')
            nominal=root/'nominal';changed=root/'changed'
            bundle(source,nominal,ROOT/'split_sim/course_runtime/astar-route.json','nominal')
            bundle(source,changed,ROOT/'split_sim/course_runtime/astar-route.json','changed')
            self.assertEqual((nominal/'assets/models/dh_course/model.sdf').read_bytes(),(source/'models/dh_course/model.sdf').read_bytes())
            self.assertNotEqual((changed/'assets/models/dh_course/model.sdf').read_bytes(),(source/'models/dh_course/model.sdf').read_bytes())
            pin=hashlib.sha256((changed/'bundle.json').read_bytes()).hexdigest()
            verify_bundle(changed,pin)
            run=root/'run';run.mkdir();archive_bundle(changed,run)
            self.assertEqual(json.loads((run/'map-comparison.json').read_text())['changed_walls'][0]['angle_enu_deg'],-18)
            self.assertNotIn(tmp,(changed/'bundle.json').read_text())
            model=changed/'assets/models/dh_course/model.sdf';model.write_text(model.read_text()+'tamper')
            with self.assertRaises(ValueError):verify_bundle(changed,pin)
            with self.assertRaises(ValueError):bundle(source,nominal,ROOT/'split_sim/course_runtime/astar-route.json','nominal')

    def test_frozen_capture_contract_binds_public_core_and_route(self):
        config,route=load_contract(HERE/'m3-capture-contract.json',ROOT/'split_sim/course_runtime')
        self.assertTrue(config['shadow_only']);self.assertEqual(config['prefix_speed_cap_mps'],.2)
        self.assertEqual(config['terminal_speed_cap_mps'],.08)
        with tempfile.TemporaryDirectory() as tmp:
            root=Path(tmp);shutil.copy2(HERE/'m3-capture-contract.json',root/'contract.json')
            (root/'m3-capture-route.json').write_text('tampered')
            with self.assertRaises(ValueError):load_contract(root/'contract.json',ROOT/'split_sim/course_runtime')


if __name__=='__main__':unittest.main()
