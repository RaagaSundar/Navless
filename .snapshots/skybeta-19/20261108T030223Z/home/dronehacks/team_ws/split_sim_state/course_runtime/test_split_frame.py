import math
import unittest
from types import SimpleNamespace as NS
from frames import launch_odom_to_ned
from split_frame import SplitFrame


class FrameTests(unittest.TestCase):
    def setUp(self):
        self.g = SplitFrame(0)
        self.p = NS(xy_valid=True,z_valid=True,v_xy_valid=True,v_z_valid=True,
                    vx=0,vy=0,vz=0,heading=math.pi/2,xy_reset_counter=0,
                    z_reset_counter=0,heading_reset_counter=1)
        self.flags = NS(cs_tilt_align=True,cs_ev_pos=True,cs_ev_yaw=True,
                        cs_ev_yaw_fault=False,cs_inertial_dead_reckoning=False,timestamp=0)
    def observe(self,t,vo=None,aligned=math.pi/2):
        self.flags.timestamp=int(t*1e6)
        return self.g.observe(t,t if vo is None else vo,(1,0,0,0),self.p,
                              self.flags,False,True,aligned)[0]
    def test_geography_control_and_reset(self):
        for i in range(20): self.assertEqual(self.observe(i/10),'wait')
        self.assertEqual(self.observe(2),'ready')
        self.assertAlmostEqual(launch_odom_to_ned((1,0,0),self.g.bearing)[1],1)
        self.assertEqual(self.g.course_position((1,0,0)),[1,0,0])
        self.assertAlmostEqual(self.g.course_yaw_to_control(0),math.pi/2)
        self.p.heading_reset_counter+=1
        self.assertEqual(self.observe(2.1),'reject')
    def test_no_stale_repeated_faulty_or_unaligned_registration(self):
        for i in range(30): self.observe(i/10,vo=0)
        self.assertIsNone(self.g.bearing)
        for i in range(30): self.observe(i/10,aligned=0)
        self.assertIsNone(self.g.bearing)
        self.flags.cs_ev_yaw=False
        for i in range(30): self.observe(i/10)
        self.assertIsNone(self.g.bearing)
    def test_non_level_quaternion_and_drifting_heading(self):
        for i in range(30): self.observe(i/10,aligned=math.radians(89+i*.09))
        self.assertIsNone(self.g.bearing)
