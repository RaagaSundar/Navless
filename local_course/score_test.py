"""Offline regression tests for independent acceptance criteria (no simulator)."""
import copy
import json
from pathlib import Path
import unittest
from local_course.score_course import grade, ROOT


class ScorerTests(unittest.TestCase):
    def fixture(self):
        route=json.loads((ROOT/'gazebo_sim/course/route_centreline_window.json').read_text())['waypoints_enu']
        start=route[0]; points=[[start[0],start[1],.08]]*22+route+[[route[-1][0],route[-1][1],.08]]*35
        samples=[dict(t=i*.1,x=p[0],y=p[1],z=p[2],yaw=1.57079632679,
                      armed=22<=i<22+len(route),landed=not(22<=i<22+len(route)),status_age=.01)
                 for i,p in enumerate(points)]
        return {'samples':samples,'contacts':[],'contact_topics_verified':True}
    def score(self,data): return grade(data,clearance_fn=lambda rows:[.16]*len(rows))
    def test_complete_ordered_route(self):
        result=self.score(self.fixture()); self.assertTrue(result['passed'],result)
    def test_contacts_fail(self):
        d=self.fixture();d['contacts']=[{'t':3,'name':'wall'}]; self.assertFalse(self.score(d)['passed'])
    def test_stale_telemetry_fails(self):
        d=self.fixture();d['samples'][30]['status_age']=3;self.assertFalse(self.score(d)['passed'])
    def test_missing_checkpoint_fails(self):
        d=self.fixture();d['samples']=d['samples'][:22]+d['samples'][-35:]
        self.assertFalse(self.score(d)['passed'])
    def test_armed_finish_fails(self):
        d=self.fixture();d['samples'][-1]['armed']=True;self.assertFalse(self.score(d)['passed'])
    def test_unverified_contacts_fail(self):
        d=self.fixture();d['contact_topics_verified']=False;self.assertFalse(self.score(d)['passed'])
    def test_negative_clearance_fails(self):
        result=grade(self.fixture(),clearance_fn=lambda rows:[-.01]*len(rows));self.assertFalse(result['passed'])
    def test_overflight_fails(self):
        d=self.fixture()
        for s in d['samples'][22:-35]: s['z']=2.0
        self.assertFalse(self.score(d)['passed'])
    def test_intermediate_landing_fails(self):
        d=self.fixture();d['samples'][100]['landed']=True;d['samples'][100]['armed']=False
        self.assertFalse(self.score(d)['passed'])
    def test_reversed_route_fails(self):
        d=self.fixture();poses=[(s['x'],s['y'],s['z']) for s in d['samples'][22:-35]][::-1]
        for s,p in zip(d['samples'][22:-35],poses): s['x'],s['y'],s['z']=p
        self.assertFalse(self.score(d)['passed'])
    def test_clipped_gate_fails(self):
        d=self.fixture()
        for s in d['samples']:
            if s['y']>2 and s['x']< -2: s['x']=-2.98
        self.assertFalse(self.score(d)['passed'])
    def test_pad_partial_containment_fails(self):
        d=self.fixture()
        for s in d['samples'][-35:]:s['x']+=.6
        self.assertFalse(self.score(d)['passed'])
    def test_missing_landing_fails(self):
        d=self.fixture();d['samples'][-1]['landed']=False;self.assertFalse(self.score(d)['passed'])

class ClockTests(unittest.TestCase):
    def test_slow_realtime_is_healthy(self):
        from local_course.score_clock import TelemetryClock
        c=TelemetryClock();c.observe(20000000,20.,100.)
        self.assertAlmostEqual(c.age(20.9),.9)
        c.observe(21000000,21.,110.)  # 10 wall seconds per simulated second
        self.assertAlmostEqual(c.age(21.1),.1)
        self.assertFalse(c.stalled(120.))
    def test_source_dropout_stales(self):
        from local_course.score_clock import TelemetryClock
        c=TelemetryClock();c.observe(20000000,20.,100.)
        c.observe(20000000,23.,110.)  # replayed receipt is not freshness
        self.assertGreater(c.age(23.),2.5)
        self.assertTrue(c.stalled(161.))
    def test_old_source_epoch_is_stale(self):
        from local_course.score_clock import TelemetryClock
        c=TelemetryClock();c.observe(10000000,20.,100.)
        self.assertGreater(c.age(20.),2.5)
    def test_future_source_epoch_is_stale(self):
        from local_course.score_clock import TelemetryClock
        c=TelemetryClock();c.observe(30000000,20.,100.)
        self.assertEqual(c.age(20.),float('inf'))
    def test_timestamp_reversal_stales(self):
        from local_course.score_clock import TelemetryClock
        c=TelemetryClock();c.observe(20000000,20.,100.);c.observe(19000000,21.,101.)
        self.assertEqual(c.age(21.),float('inf'))

if __name__=='__main__':unittest.main()
