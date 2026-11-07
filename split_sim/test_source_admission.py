import math
import sys
import unittest
from pathlib import Path
from types import SimpleNamespace as NS

here=Path(__file__).resolve().parent
sys.path[:0]=[str(here/'course_runtime'),str(here/'viewer')]
from source_admission import SourceAdmission, LandedBootstrap
from px4_navigation import FusedNavigationGate


class AdmissionTests(unittest.TestCase):
    def test_quarantined_jump_is_never_published_or_used_as_reference(self):
        g=SourceAdmission()
        self.assertTrue(g.accept(1,(0,0,0)))
        self.assertFalse(g.accept(1.1,(.11,0,0)))
        self.assertFalse(g.accept(1.2,(.02,0,0)))
        self.assertTrue(g.accept(1.3,(.03,0,0)))
        self.assertEqual(g.reference,(1.3,(.03,0,0)))
        self.assertFalse(g.accept(1.3,(.03,0,0)))
    def test_persistent_shift_or_long_gap_cannot_rehome(self):
        g=SourceAdmission();g.accept(0,(0,0,0))
        for i in range(1,30):
            self.assertFalse(g.accept(i/10,(.15,0,0)))
        self.assertFalse(g.accept(3,(0,0,0)))
    def test_cold_start_needs_continuous_ground_and_disarmed_sources(self):
        boot = LandedBootstrap()
        status = NS(timestamp=0, ARMING_STATE_DISARMED=1, arming_state=2, failsafe=False)
        land = NS(timestamp=0, landed=True)
        for i in range(30):
            t=i/10
            status.timestamp=land.timestamp=int(t*1e6)
            self.assertFalse(boot.observe(t,t,(0,0,0),status,land))
        status.arming_state=1
        self.assertFalse(boot.observe(5,5,(0,0,0),status,land))
        for i in range(21):
            t=6+i/10
            status.timestamp=land.timestamp=int(t*1e6)
            self.assertEqual(boot.observe(t,t,(0,0,0),status,land), i==20)

    def test_relative_and_geographic_heading_contracts(self):
        p=NS(xy_valid=True,z_valid=True,v_xy_valid=True,v_z_valid=True,
             heading_good_for_control=False,heading_var=.0004,x=0,y=0,z=0,
             vx=0,vy=0,vz=0,heading=1.57,eph=.02,epv=.02,evh=.02,evv=.02,
             heading_reset_counter=1,xy_reset_counter=0,z_reset_counter=0,
             vxy_reset_counter=0,vz_reset_counter=0)
        flags=NS(cs_ev_yaw=True,cs_tilt_align=True,cs_ev_yaw_fault=False,
                 cs_inertial_dead_reckoning=False,cs_rng_hgt=True,cs_ev_pos=True)
        fr=NS(bearing=90,reset=(1,0,0))
        def eval_gate(g,now,t):
            return g.evaluate(p,now,t,(0,0,0),1.57,flags,now,(0,0,0,1.57))[0]
        self.assertEqual(eval_gate(FusedNavigationGate(),1,1),'waiting')
        g=FusedNavigationGate(fr)
        self.assertEqual(eval_gate(g,1,1),'waiting')
        self.assertEqual(eval_gate(g,2,2),'ready')
        self.assertEqual(eval_gate(g,2.4,2),'coast')
        self.assertEqual(eval_gate(g,2.45,2.4),'ready')
        self.assertEqual(eval_gate(g,3.5,2.4),'reject')
        p.heading_var=math.radians(5)**2
        self.assertEqual(eval_gate(g,3.6,3.6),'reject')
        p.heading_var=.0004;flags.cs_ev_yaw=False
        self.assertEqual(eval_gate(g,3.7,3.7),'reject')
