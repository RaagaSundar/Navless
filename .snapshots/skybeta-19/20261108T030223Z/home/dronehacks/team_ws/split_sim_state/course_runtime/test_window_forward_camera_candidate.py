import json,math,unittest
from pathlib import Path
import numpy as np
from window_forward_camera_candidate import WindowForwardLook
from yaw_policy import FirstExteriorLook

R=Path(__file__).resolve().parents[2]
C=Path(__file__).with_name('window-forward-camera-candidate.json')
J=json.loads(C.read_text())
P=Path(__file__).with_name('astar-route.json')

class WindowTests(unittest.TestCase):
    def test_bounded_coast_holds_and_route_can_resume(self):
        look=WindowForwardLook.from_files(P,C)
        self.assertEqual(look.step(-2.515,.914,.4,1.,1.,True,'COAST'),0.)
        self.assertFalse(look.finished)
        self.assertIsNotNone(look.step(-2.515,.914,.4,1.1,1.1,True,'TAKEOFF'))
        look.step(-2.515,.914,.4,1.2,1.2,True,'LAND')
        self.assertTrue(look.finished)
    def test_normal_yaw_conventions_and_wrap(self):
        b=math.radians(J['window_forward_compass_deg']);v=np.array([math.sin(b),math.cos(b)])
        np.testing.assert_allclose(v,J['window_forward_enu_xy'],atol=1e-10)
        self.assertLess(J['window_forward_ned_deg'],0)
        self.assertLess(abs(J['window_forward_ned_deg']+67.90055428134),1e-6)
        self.assertGreater(np.dot(v,J['window_normal_enu_xy']),.999)
        yaw=math.pi/2-b
        np.testing.assert_allclose([math.cos(yaw),math.sin(yaw)],v,atol=1e-10)
    def test_start_lookahead_lock_exit_and_rate(self):
        h=WindowForwardLook.from_files(P,C);base=FirstExteriorLook();p=np.array(json.loads(P.read_text())['waypoints_enu']);s=np.r_[0,np.cumsum(np.linalg.norm(np.diff(p,axis=0),axis=1))];previous=0.;locked=[]
        for i,a in enumerate(np.arange(0,s[-1],.02)):
            pos=[float(np.interp(a,s,p[:,k])) for k in range(3)];t=i*.05
            ref=base.step(pos[0],pos[1],pos[2]-.07,t,t,True)
            y=h.step(pos[0],pos[1],pos[2]-.07,t,t,True,'ROUTE')
            self.assertIsNotNone(y)
            if a<J['activation_arc_m']-.05:self.assertEqual(y,ref)
            self.assertLessEqual(abs(math.atan2(math.sin(y-previous),math.cos(y-previous))),math.radians(.5)+1e-10)
            if J['lock_entry_arc_m']+.02<a<J['lock_exit_arc_m']-.02:locked.append(math.degrees(y))
            previous=y
        self.assertTrue(locked)
        self.assertLess(max(abs(y-J['window_forward_ned_deg']) for y in locked),.1)
        held=h.step(pos[0],pos[1],.2,t+.05,t+.05,True,'LAND')
        self.assertEqual(held,previous)
    def test_center_in_forward_image_not_north(self):
        forward=np.array(J['window_forward_enu_xy']);center=np.array(J['window_center_enu_xy']);pos=center-.8*forward
        psi=math.radians(J['window_forward_enu_yaw_deg']);rot=np.array([[math.cos(psi),-math.sin(psi)],[math.sin(psi),math.cos(psi)]])
        body=(center-pos)@rot
        self.assertGreater(body[0],.79)
        self.assertLess(abs(body[1]),1e-10)
        # Left-eye mount .0375m left makes center project slightly right.
        u=320-381.361274719*(body[1]-.0375)/(body[0]-.1053)
        self.assertGreater(u,320);self.assertLess(u,360)
        self.assertLess((center-pos)[1],.4) # north camera sees target far left, beyond40degHFOV
        north_angle=abs(math.atan2((center-pos)[0],(center-pos)[1]))
        self.assertGreater(north_angle,math.radians(40))

if __name__=='__main__':unittest.main()
