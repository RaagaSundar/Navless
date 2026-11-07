"""Offline temporary fixtures; never invokes ROS or flight code."""
import importlib.util
import json
from pathlib import Path
import tempfile
import unittest

spec=importlib.util.spec_from_file_location('comparison',Path(__file__).with_name('run_comparison.py'))
comparison=importlib.util.module_from_spec(spec);spec.loader.exec_module(comparison)


class ComparisonTests(unittest.TestCase):
    def setUp(self):
        self.temp=tempfile.TemporaryDirectory();self.addCleanup(self.temp.cleanup)
        self.root=Path(self.temp.name)/'nvidia-runs';self.root.mkdir()
        self.old=self.root/'20260101T010000Z-run';self.new=self.root/'20260101T020000Z-run'
        for run in (self.old,self.new):
            (run/'flight').mkdir(parents=True)
            (run/'settings.json').write_text(json.dumps({'height_profile':'range','secret_token':'never copy','private_path':'/private/example'}))
            (run/'flight/track.json').write_text(json.dumps({'samples':[{'t':1,'z':0,'armed':False,'landed':True}]}))

    def test_equal_is_not_proof_of_identical_experiment(self):
        data=comparison.seed(self.new,baseline=self.old)
        self.assertEqual(data['changes'],[])
        self.assertEqual(data['comparison_state'],'no recorded differences')
        self.assertFalse(data['telemetry']['observed_flight'])
        self.assertNotIn('secret_token',json.dumps(data));self.assertNotIn('/private/example',json.dumps(data))

    def test_missing_is_unknown_and_diff_evidenced(self):
        (self.new/'settings.json').write_text(json.dumps({'height_profile':'ev-terrain','yaw_policy':'window-forward'}))
        data=comparison.seed(self.new,baseline=self.old)
        diff={d['field']:d for d in data['changes']}
        self.assertEqual(diff['settings.json:height_profile']['before'],'range')
        self.assertEqual(diff['settings.json:yaw_policy']['before'],'not archived')
        self.assertEqual(diff['settings.json:yaw_policy']['evidence'],['settings.json'])

    def test_airborne_requires_correlated_actual_flags(self):
        for samples,expected in [([{'z':1,'armed':True,'landed':False}],True),([{'z':1,'armed':False,'landed':False}],False),([{'z':1}],False)]:
            (self.new/'flight/track.json').write_text(json.dumps({'samples':samples}))
            self.assertEqual(comparison.seed(self.new)['telemetry']['observed_flight'],expected)

    def test_annotation_identity_and_external_evidence_rejected(self):
        with self.assertRaises(ValueError):comparison.seed(self.new,annotations={'run_id':'other'})
        with self.assertRaises(ValueError):comparison.seed(self.new,annotations={'changes':[{'field':'test','before':0,'after':1,'evidence':['../outside.txt']}]})

    def test_atomic_writer_and_immutable_id(self):
        comparison.write(self.new,name='Named run',baseline=self.old)
        target=self.new/'run-comparison.json';self.assertEqual(json.loads(target.read_text())['run_id'],'nvidia:'+self.new.name)
        target.write_text('{"run_id":"other"}')
        with self.assertRaises(ValueError):comparison.write(self.new)


if __name__=='__main__':unittest.main()
