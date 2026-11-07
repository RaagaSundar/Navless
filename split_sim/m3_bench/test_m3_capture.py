"""Finite wrapper algorithm tests only; no publishers or vehicle commands."""
import io,json,unittest,sys
from types import SimpleNamespace
from unittest.mock import patch
from pathlib import Path
from mission_m3_capture import install_course,load_contract

class Fake:
 def __init__(self):self.position=(0,0,1);self.time=1.;self.target=self.position;self.state='ROUTE';self.sent=[];self.arrival_radius=.15;self.calls=0
 def create_subscription(self,*a):pass
 def now_s(self):return self.time
 def send(self,*p):self.sent.append(p)
 def goto(self,s,t):self.state=s;self.target=t
 def start_route(self):pass
 def _state_route(self):self.calls+=1

class CaptureTest(unittest.TestCase):
 def setUp(self):
  self.modules=patch.dict(sys.modules,{'std_msgs':SimpleNamespace(),'std_msgs.msg':SimpleNamespace(String=object)})
  self.modules.start();self.addCleanup(self.modules.stop)
 def config(self):return json.loads(Path(__file__).with_name('m3-capture-contract.json').read_text())
 def test_actual_stop_cannot_resume_before_settle(self):
  log=io.StringIO();Cls=install_course(Fake,self.config(),log);node=Cls();node.carrot=type('C',(),{'index':35})()
  node.m3_velocity=(.1,0,0);node._state_route();self.assertEqual(node.calls,0)
  node.time=2;node.m3_velocity=(0,0,0);node._state_route();self.assertEqual(node.calls,0)
  node.time=3.1;node._state_route();self.assertEqual(node.calls,1);self.assertTrue(node.m3_brake_once)
 def test_unsettled_stop_requests_normal_land(self):
  Cls=install_course(Fake,self.config(),io.StringIO());node=Cls();node.carrot=type('C',(),{'index':35})();node.m3_velocity=(.1,0,0)
  node._state_route();node.time=6.1;node._state_route();self.assertEqual(node.state,'LAND');self.assertIn('did not settle',node.m3_land_reason)
 def test_endpoint_hold_is_distinct_from_course_success(self):
  log=io.StringIO();Cls=install_course(Fake,self.config(),log);node=Cls();node._state_land();self.assertIsNone(node.m3_land_reason)
  node.time=4.1;node._state_land();self.assertIn('not course completion',node.m3_land_reason)
if __name__=='__main__':unittest.main()
