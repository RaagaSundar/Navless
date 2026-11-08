import math
from pathlib import Path
import unittest
import xml.etree.ElementTree as ET
import numpy as np
from opening import opening


class FunnelTest(unittest.TestCase):
    def test_clear_gap_and_included_angle(self):
        world = ET.parse(Path(__file__).with_name('funnel.sdf')).find('world')
        tips, angles = [], []
        for name in ('left', 'right'):
            m = world.find(f"model[@name='{name}']")
            x,y,_,_,_,yaw = map(float, m.findtext('pose').split())
            length,width,_ = map(float,m.findtext('link/collision/geometry/box/size').split())
            corners = [(x+u*math.cos(yaw)-v*math.sin(yaw),
                        y+u*math.sin(yaw)+v*math.cos(yaw))
                       for u in (-length/2,length/2) for v in (-width/2,width/2)]
            tips.append(min(corners,key=lambda pt: abs(pt[1])))
            angles.append(yaw)
        self.assertAlmostEqual(math.dist(*tips),1.)
        self.assertAlmostEqual(math.degrees(abs(angles[0]-angles[1])),60.)

    def test_bearing_and_abstention(self):
        img = np.full((400,640),30,np.uint8)
        self.assertIsNone(opening(img,380,320,200))
        img[:,200:300] = 215
        self.assertAlmostEqual(opening(img,380,320,200)['bearing_right_deg'],
                               math.degrees(math.atan2(-70.5,380)))
        img[:,400:500] = 215
        self.assertIsNone(opening(img,380,320,200))
