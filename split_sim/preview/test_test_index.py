"""Portable synthetic inventory tests; no ROS, simulator or genuine run data."""
import json
from pathlib import Path
import tempfile
import time
import unittest
from unittest.mock import patch
import test_index as index

class InventoryTests(unittest.TestCase):
    def setUp(self):
        self.temp=tempfile.TemporaryDirectory();self.root=Path(self.temp.name)
        self.receipts=self.root/'evidence';self.receipts.mkdir()
        self.patches=[patch.object(index,'ROOT',self.root),patch.object(index,'ROOTS',{'fixture':('evidence',2,'Synthetic receipt fixtures')}),patch.object(index,'_CACHE',None),patch.object(index,'_CACHE_AT',0)]
        for p in self.patches:p.start()
    def tearDown(self):
        for p in reversed(self.patches):p.stop()
        self.temp.cleanup()
    def write(self,name,text):
        p=self.receipts/name;p.parent.mkdir(parents=True,exist_ok=True);p.write_text(text);return p
    def test_receipts_and_unknowns_are_not_flights(self):
        self.write('software-tests.txt','Ran 4 tests in 0.1s\n\nOK\n')
        self.write('candidate.json','{"status":"proposed","scope":"not executed"}')
        self.write('capture/summary.json','{"complete":true,"scope":"passive capture only"}')
        self.write('unknown.json','{"source":"fixture"}')
        data=index.inventory(force=True);outcomes={r['source']:r['outcome'] for r in data['records']}
        self.assertEqual(outcomes['fixture/software-tests'],'passed check')
        self.assertEqual(outcomes['fixture/candidate'],'reported proposed')
        self.assertEqual(outcomes['fixture/capture'],'capture complete')
        self.assertEqual(outcomes['fixture/unknown'],'unknown')
        self.assertTrue(all(r['kind']!='flight' for r in data['records']))
    def test_safe_allowlist_no_vendor_secret_binary_or_traversal(self):
        safe=self.write('good/result.json','{"passed":false}')
        for name in ['vendor/result.json','build/result.json','secret.json','.env.json','raw.sqlite','source/code.json']:
            self.write(name,'not evidence')
        outside=self.root/'outside.txt';outside.write_text('outside')
        (self.receipts/'linked.txt').symlink_to(outside)
        (self.receipts/'linkdir').symlink_to(self.root,target_is_directory=True)
        data=index.inventory(force=True)
        self.assertEqual(data['scanned_files'],1)
        self.assertEqual(index.resolve_file('fixture','good/result.json'),safe)
        for value in ['../outside.txt','good/../../outside.txt','linked.txt','.env.json','good\\result.json','/good/result.json']:
            self.assertIsNone(index.resolve_file('fixture',value))
        self.assertIsNone(index.resolve_file('unknown','good/result.json'))
    def test_bounded_result_is_scoped_not_course_pass(self):
        a=self.write('bounded/m3-capture-result.json',json.dumps({'capture_met_declared_goals':True,'ordinary_full_course_grade_unchanged':{'passed':False}}))
        self.assertEqual(index._receipt(a)[0],'bounded goals met')
        b=self.write('physical/m3-physical-clearance.json',json.dumps({'clearance_and_contact_passed':True}))
        self.assertEqual(index._receipt(b)[0],'clearance check passed')
    def test_unverified_named_setup_receipts_remain_not_flights(self):
        self.write('attempt/trial-intent.json',json.dumps({'name':'Synthetic yaw intent','kind':'intent'}))
        self.write('attempt/review.json',json.dumps({'outcome':'Deployment stopped before mission','failure':'Saved predeployment check failed'}))
        with patch.object(index,'ROOTS',{'nvidia-runs':('evidence',2,'Synthetic sessions')}):
            row=index.inventory(force=True)['records'][0]
        self.assertEqual(row['kind'],'setup / preflight attempt')
        self.assertEqual(row['outcome'],'reported setup failure')
        self.assertIn('Synthetic yaw intent',row['title'])
        self.assertNotIn('observed_flight',row)
        self.assertNotIn('course_pass',row)
    def test_mixed_results_preserved(self):
        self.write('group/passed-tests.txt','PASS: first check\n')
        self.write('group/failed-tests.txt','FAILED (failures=1)\n')
        row=index.inventory(force=True)['records'][0]
        self.assertEqual(row['outcome'],'mixed recorded checks')
        self.assertEqual(len(row['outcome_receipts']),2)
    def test_cache_and_original_run_grade_are_preserved(self):
        self.write('unit-tests.txt','PASS: scope\n')
        run={'id':'nvidia:synthetic','run_id':'synthetic','platform_label':'NVIDIA SIM','observed_flight':False,'status':'setup / blocked','files':[]}
        before=time.monotonic();data=index.inventory([run],force=True)
        self.assertLess(time.monotonic()-before,1)
        row=next(r for r in data['records'] if r['id']=='run:nvidia:synthetic')
        self.assertEqual(row['outcome'],'setup / blocked');self.assertEqual(row['kind'],'setup / preflight attempt')
        self.assertEqual(index.inventory([run])['generated_at'],data['generated_at'])

if __name__=='__main__':unittest.main()
