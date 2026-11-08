import math
from types import SimpleNamespace as NS
import unittest
import numpy as np
from passage import advance, midpoint


class PassageTest(unittest.TestCase):
    def test_left_eye_offset(self):
        tf=NS(translation=NS(x=.1053,y=.0375,z=.0304), rotation=NS(x=-.5,y=.5,z=-.5,w=.5))
        pose=NS(position=NS(x=1,y=2,z=1),orientation=NS(x=0,y=0,z=math.sin(math.pi/4),w=math.cos(math.pi/4)))
        aim=dict(left_px=274,right_px=373,center_px=323.5,bearing_right_deg=.5)
        center=midpoint(aim,[400,0,320],tf,pose)
        np.testing.assert_allclose(center,[.9975,6.1053],atol=1e-6)

    def test_progress_bounds(self):
        pos=np.array([1.,1.])
        goal=np.array([3.,3.])
        self.assertAlmostEqual(np.linalg.norm(advance(pos,pos,goal,.1)-pos),.025)
        self.assertAlmostEqual(np.linalg.norm(advance(goal,pos,goal,.1)-pos),.16)
