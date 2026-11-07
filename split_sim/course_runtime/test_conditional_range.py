import json
from pathlib import Path
from types import SimpleNamespace
import unittest
from conditional_range import WindowRangeHealth


class RangeHealthTests(unittest.TestCase):
    def setUp(self):
        course=json.loads((Path(__file__).parent/'navless/gazebo_sim/course/course_v3.json').read_text())
        self.health=WindowRangeHealth(next(w for w in course['walls'] if 'window' in w))
        self.flags=SimpleNamespace(timestamp=1000000,cs_ev_hgt=True,cs_baro_hgt=True,
            cs_fake_hgt=False,cs_baro_fault=False,fs_bad_acc_vertical=False,cs_rng_fault=False,
            cs_rng_stuck=False,cs_rng_hgt=True,cs_rng_terrain=True,cs_rng_kin_consistent=True)
        self.east,self.north=self.health.center
        self.feed(1.,.82)
        self.assertTrue(self.check(1.)[0])
    def feed(self,t,r): self.health.receive(t,r,'downward_range_link',.02,4.)
    def check(self,t,authority=True):
        return self.health.check(t,self.flags,self.east,self.north,True,authority)
    def step(self):
        self.feed(1.04,.52);self.flags.cs_rng_hgt=self.flags.cs_rng_terrain=self.flags.cs_rng_kin_consistent=False
    def test_real_step_and_recovery(self):
        self.step()
        self.assertEqual(self.check(1.05)[:2],(True,True))
        self.feed(1.42,.83)
        self.flags.cs_rng_kin_consistent=True
        self.assertTrue(self.check(1.43)[0])
        self.flags.cs_rng_hgt=self.flags.cs_rng_terrain=True
        self.assertEqual(self.check(1.44)[:2],(True,False))
    def test_delayed_flags_use_acquisition_aligned_event(self):
        self.feed(1.04,.52)
        self.assertTrue(self.health.check(1.10,self.flags,self.east,self.north,True,True,
            lambda stamp:(self.east,self.north))[0])
        self.feed(1.74,.52);self.flags.timestamp=1700000
        self.flags.cs_rng_hgt=self.flags.cs_rng_terrain=self.flags.cs_rng_kin_consistent=False
        self.assertTrue(self.check(1.75)[1])

    def test_stale_raw_stuck_fault_and_lost_authority(self):
        self.step()
        self.assertFalse(self.check(1.3)[0])
        self.assertFalse(self.check(1.05,False)[0])
        for key in ('cs_fake_hgt','cs_baro_fault','fs_bad_acc_vertical','cs_rng_fault','cs_rng_stuck'):
            setattr(self.flags,key,True);self.assertFalse(self.check(1.05)[0]);setattr(self.flags,key,False)
        for key in ('cs_ev_hgt','cs_baro_hgt'):
            setattr(self.flags,key,False);self.assertFalse(self.check(1.05)[0]);setattr(self.flags,key,True)
    def test_unexplained_range_loss_low_clearance_outside_or_expired(self):
        self.flags.cs_rng_hgt=self.flags.cs_rng_terrain=self.flags.cs_rng_kin_consistent=False
        self.assertFalse(self.check(1.05)[0])
        self.step();self.east+=1.;self.assertFalse(self.check(1.05)[0]);self.east-=1.
        self.assertTrue(self.check(1.05)[0])
        self.feed(1.08,.15);self.assertFalse(self.check(1.09)[0])
        self.flags.timestamp=4000000;self.feed(4.,.52);self.assertFalse(self.check(4.)[0])
