import unittest
from types import SimpleNamespace
from copy import deepcopy
import numpy as np
from roundoff import canonicalize, canonicalize_message

class RoundoffTest(unittest.TestCase):
    def test_roundoff_is_bounded_and_zero_is_unknown(self):
        a=np.zeros((6,6));a[5,5]=-3e-16
        output,info=canonicalize(a.ravel())
        self.assertEqual(output,[0.]*36);self.assertTrue(info['unknown_confidence'])
        self.assertLess(info['max_change'],info['tolerance'])
    def test_psd_projection_does_not_change_real_positive_covariance(self):
        a=np.diag([.01,.02,.03,.001,.002,.003]);out,info=canonicalize(a.ravel())
        np.testing.assert_array_equal(np.array(out).reshape(6,6),a);self.assertEqual(info['status'],'unchanged')
    def test_material_negative_indefinite_asymmetric_sentinel_nan_reject(self):
        cases=[]
        a=np.eye(6)*.01;a[0,0]=-1e-8;cases.append(a)
        a=np.eye(6)*.01;a[0,1]=a[1,0]=.02;cases.append(a)
        a=np.eye(6)*.01;a[0,1]=.001;cases.append(a)
        cases.append(np.eye(6));a=np.eye(6)*.01;a[2,2]=np.nan;cases.append(a)
        for a in cases:
            with self.subTest(a=a):self.assertIsNone(canonicalize(a.ravel())[0])
    def test_pose_clock_frame_and_input_preserved(self):
        covariance=[0.]*36;covariance[35]=-3e-16
        msg=SimpleNamespace(header={'stamp':182112000000,'frame':'odom'},position=[1,2,3],orientation=[0,0,0,1],pose=SimpleNamespace(covariance=covariance),twist=SimpleNamespace(covariance=[0.]*36,velocity=[.1,.2,.3]))
        original=deepcopy(msg);out,report=canonicalize_message(msg)
        self.assertEqual(out.header,msg.header);self.assertEqual(out.position,msg.position);self.assertEqual(out.orientation,msg.orientation);self.assertEqual(out.twist.velocity,msg.twist.velocity)
        self.assertEqual(msg.pose.covariance,original.pose.covariance);self.assertTrue(report['pose']['unknown_confidence'])

class ExistingGateTest(unittest.TestCase):
    def test_original_age_tracking_frame_and_quaternion_checks_still_fail_closed(self):
        import ast,math
        from pathlib import Path
        base=Path(__file__).resolve().parent;source=next(p for p in [base.parent/'viewer/map_server.py',base/'original_map_server_local_source.py',base/'map_server_local_source.py'] if p.exists())
        module=ast.parse(source.read_text());fn=next(n for n in module.body if isinstance(n,ast.FunctionDef) and n.name=='valid_odometry');scope={'math':math};exec(compile(ast.Module(body=[fn],type_ignores=[]),str(source),'exec'),scope);valid=scope['valid_odometry']
        V=lambda:SimpleNamespace(x=0.,y=0.,z=0.)
        covariance=[0.]*36;covariance[35]=-3e-16
        msg=SimpleNamespace(header=SimpleNamespace(stamp=SimpleNamespace(sec=182,nanosec=80000000),frame_id='odom'),child_frame_id='base_link',pose=SimpleNamespace(pose=SimpleNamespace(position=V(),orientation=SimpleNamespace(x=0.,y=0.,z=0.,w=1.)),covariance=covariance),twist=SimpleNamespace(twist=SimpleNamespace(linear=V(),angular=V()),covariance=[0.]*36))
        out,_=canonicalize_message(msg)
        self.assertFalse(valid(msg,182.12,True,.01))
        self.assertTrue(valid(out,182.12,True,.01))
        self.assertFalse(valid(out,182.608,True,.01),'0.528-second acquisition age stays rejected')
        self.assertFalse(valid(out,182.12,False,.01))
        self.assertFalse(valid(out,182.12,True,.3))
        out.header.frame_id='map';self.assertFalse(valid(out,182.12,True,.01));out.header.frame_id='odom'
        out.pose.pose.orientation.w=2.;self.assertFalse(valid(out,182.12,True,.01))

if __name__=='__main__':unittest.main()
