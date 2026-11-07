import unittest
from types import SimpleNamespace

from mission import landing_outcome, route_land_ready
from navless_nav.route import Carrot


class LandingTests(unittest.TestCase):
    def test_route_complete_and_current_authority(self):
        path=[(0.,0.,.4), (1.,0.,.4)]
        carrot=Carrot(path,.4,.18)
        mission=SimpleNamespace(state='LAND',position=path[-1],arrival_radius=.25,carrot=carrot)
        inputs=['ACTIVE','ready',True,True,False,False]
        self.assertFalse(route_land_ready(mission,*inputs))
        carrot.index=len(path)-1
        carrot.point=path[-1]
        self.assertTrue(route_land_ready(mission,*inputs))
        for i,v in enumerate(['WAIT','coast',False,False,True,True]):
            changed=inputs.copy()
            changed[i]=v
            self.assertFalse(route_land_ready(mission,*changed))
        mission.position=path[0]
        self.assertFalse(route_land_ready(mission,*inputs))

    def test_passive_mode_transition_and_terminal_state(self):
        self.assertIsNone(landing_outcome(.1,None,True,False,True,False))
        self.assertIsNone(landing_outcome(10.,0,True,False,False,True))
        self.assertEqual(landing_outcome(11.,0,False,True,False,True)[1],0)
        for state in [(1.1,None,True,False,True,False),(.1,1,True,False,True,False),
                      (.1,0,True,False,False,False)]:
            self.assertEqual(landing_outcome(*state)[1],1)
