import hashlib
import json
from pathlib import Path
import shutil
import tempfile
import unittest
from split_sim.verify_transit_profile import HERE, PROFILE, verify


class TransitProfileTests(unittest.TestCase):
    def test_recorded_grade_is_not_reclassified(self):
        result=verify()
        self.assertTrue(result['recorded_graded_passed'])
        grade=json.loads((HERE/'profiles/known-window-transit/grade.json').read_text())
        self.assertFalse(grade['passed'])
        self.assertEqual(grade['reasons'],['window crossing outside vertical opening'])
        self.assertEqual(grade['contacts'],[])
        self.assertEqual(grade['pad_distance_cm'],14.13)

    def test_behavior_change_rejected_even_if_manifest_updated(self):
        with tempfile.TemporaryDirectory() as tmp:
            runtime=Path(tmp)/'runtime'
            shutil.copytree(HERE/'course_runtime',runtime,ignore=shutil.ignore_patterns('__pycache__','*.pyc'))
            config=runtime/'window-forward-camera-candidate.json'
            data=json.loads(config.read_text());data['rate_limit_deg_s']=11.;config.write_text(json.dumps(data))
            manifest=runtime/'sources.sha256';lines=[]
            for line in manifest.read_text().splitlines():
                digest,name=line.split('  ',1)
                if name==config.name:digest=hashlib.sha256(config.read_bytes()).hexdigest()
                lines.append(digest+'  '+name)
            manifest.write_text('\n'.join(lines)+'\n')
            with self.assertRaisesRegex(ValueError,'recorded transit'):verify(runtime)

    def test_missing_source_wrong_camera_and_metadata_only_tolerance(self):
        with tempfile.TemporaryDirectory() as tmp:
            runtime=Path(tmp)/'runtime'
            shutil.copytree(HERE/'course_runtime',runtime,ignore=shutil.ignore_patterns('__pycache__','*.pyc'))
            bad=Path(tmp)/'camera';bad.write_text('not the frozen camera')
            with self.assertRaisesRegex(ValueError,'camera'):verify(runtime,camera=bad)
            (runtime/'mission.py').write_text('unreviewed replacement')
            with self.assertRaisesRegex(ValueError,'own manifest'):verify(runtime)

    def test_passing_centered_route_and_previous_failure_are_distinct(self):
        old=json.loads((HERE/'profiles/known-window-transit/route.json').read_text())
        route=HERE/'course_runtime/astar-route.json'
        candidate=json.loads(route.read_text())
        config=json.loads((HERE/'course_runtime/window-forward-camera-candidate.json').read_text())
        self.assertEqual(len(old['waypoints_enu']),879)
        self.assertEqual([p[:2] for p in old['waypoints_enu']],[p[:2] for p in candidate['waypoints_enu']])
        self.assertEqual(candidate['window_target_z_m'],.762)
        self.assertEqual(config['route_sha256'],hashlib.sha256(route.read_bytes()).hexdigest())
        self.assertAlmostEqual(config['window_forward_compass_deg'],292.09944571866015)
        passed=json.loads((PROFILE.parent/'grade.json').read_text())
        self.assertTrue(passed['passed']);self.assertEqual(passed['reasons'],[])
        self.assertEqual(len(passed['checkpoints_sim_s']),6)
        self.assertEqual(passed['contacts'],[])
        self.assertEqual(passed['pad_distance_cm'],13.23)


if __name__=='__main__':unittest.main()
