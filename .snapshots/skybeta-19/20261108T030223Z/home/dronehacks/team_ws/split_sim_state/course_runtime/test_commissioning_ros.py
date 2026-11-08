import ast,math,unittest
from pathlib import Path
from types import SimpleNamespace as S
import numpy as np
from commissioning_ros import mono_array,transform_matrix,commissioned_course_type

class CommissionIntegrationTests(unittest.TestCase):
    def test_native_mono_decoder_stride_and_reject_color(self):
        msg=S(encoding='mono8',is_bigendian=False,step=4,width=3,height=2,data=bytes([1,2,3,99,4,5,6,99]))
        np.testing.assert_array_equal(mono_array(msg),[[1,2,3],[4,5,6]])
        msg.encoding='rgb8'
        with self.assertRaises(ValueError):mono_array(msg)
    def test_actual_tf_quaternion_contract(self):
        t=S(rotation=S(w=.5,x=-.5,y=.5,z=-.5),translation=S(x=.1053,y=.0375,z=.0304))
        T=transform_matrix(t);np.testing.assert_allclose(T[:3,:3],[[0,0,1],[-1,0,0],[0,-1,0]])
        np.testing.assert_allclose(T[:3,3],[.1053,.0375,.0304])
    def test_original_takeoff_target_and_start_checks_are_preserved(self):
        class Base:
            def __init__(self):self.target=(.25,0,.88);self.state='TAKEOFF';self.sent=[];self.safe=True
            def start_route(self):self.state='ROUTE' if self.safe else 'HOLD';self.target=None
            def goto(self,state,target):self.state=state;self.target=target
            def vio_ok(self):return True
            def send(self,*goal):self.sent.append(goal)
        C=commissioned_course_type(Base);node=C();node.start_route();self.assertEqual(node.state,'OBSERVE');self.assertEqual(node.target,(.25,0,.88));node._state_observe();self.assertEqual(node.sent,[(.25,0,.88)])
        rejected=C();rejected.safe=False;rejected.start_route();self.assertEqual(rejected.state,'HOLD')
    def test_native_mission_has_no_stale_health_gate_removed(self):
        source=Path(__file__).with_name('mission.py').read_text();ast.parse(source)
        for required in ['self.integrity','self.nav_guard.evaluate','authority lost: stopped all control publication','telemetry stale: stopped all control publication','if route_land_ready','self.command(VehicleCommand.VEHICLE_CMD_NAV_LAND)']:
            self.assertIn(required,source)
        self.assertIn("observation.action=='integrity_stop'",source)
        self.assertIn('interface.commission.close()',source)


class ExactTakeoffGeometryTests(unittest.TestCase):
    def test_config_center_matches_existing_carrot_takeoff_lean(self):
        import json,sys
        base=Path(__file__).parent;sys.path.insert(0,str(base/'navless/navless_nav'))
        from navless_nav.route import Carrot
        route=json.loads((base/'astar-route.json').read_text());cfg=json.loads((base/'active-observation-config.json').read_text())
        point=Carrot(route['waypoints_enu'],.4,.18).advance(cfg['takeoff_lead_m'])
        expected=[route['launch_origin_enu'][i]+point[i]-route['waypoints_enu'][0][i] for i in range(2)]
        np.testing.assert_allclose(expected,cfg['launch_center_enu_xy'],atol=1e-10)

if __name__=='__main__':unittest.main()
