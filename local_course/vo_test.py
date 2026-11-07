"""Pure calibration/TF/covariance tests for camera-only ROS wrapper."""
import math
import unittest
from types import SimpleNamespace as S
import numpy as np
from local_course.vo_core import camera_calibration, camera_to_body, transform_matrix, quaternion_xyzw, StartupReadiness


def info(tx=0):
    return S(width=640,height=400,d=[0.]*5,p=[400.,0.,320.,tx,0.,400.,200.,0.,0.,0.,1.,0.])


class VOTest(unittest.TestCase):
    def test_startup_clock_debounce(self):
        gate=StartupReadiness()
        self.assertFalse(gate.observe(10,0))
        for i in range(4):self.assertFalse(gate.observe(10+i*.1,10.1+i*.1))
        self.assertTrue(gate.observe(10.4,10.5))
        # A repeated image or paused clock cannot seed a new continuous origin.
        self.assertFalse(gate.observe(10.4,10.5))
        for i in range(4):self.assertFalse(gate.observe(11+i*.1,11.1))
        self.assertFalse(gate.observe(12,13))

    def test_calibration_and_rejection(self):
        k,b=camera_calibration(info(),info(-30))
        self.assertAlmostEqual(b,.075)
        self.assertEqual(k[0,0],400)
        for mutation in ('distortion','no_baseline','mismatch'):
            left,right=info(),info(-30)
            if mutation=='distortion':left.d[0]=.1
            if mutation=='no_baseline':right.p[3]=0
            if mutation=='mismatch':right.p[0]=410
            with self.subTest(mutation=mutation):
                with self.assertRaises(ValueError):camera_calibration(left,right)

    def test_camera_body_geometry_and_covariance(self):
        mount=transform_matrix([.1053,.0375,.0304],[-.5,.5,-.5,.5])
        cov=np.diag([.01,.02,.03,.04,.05,.06])
        pose=np.eye(4);pose[2,3]=1
        body,bcov=camera_to_body(pose,cov,mount)
        np.testing.assert_allclose(body[:3,3],[1,0,0],atol=1e-12)
        np.testing.assert_allclose(body[:3,:3],np.eye(3),atol=1e-12)
        self.assertGreater(np.linalg.eigvalsh(bcov).min(),0)
        # Finite-difference check of fixed-world angular lever-arm Jacobian.
        base=np.eye(4);base[:3,:3]=np.array([[0.,-1,0],[1,0,0],[0,0,1]])
        tiny=1e-6
        perturbed=base.copy()
        rx=np.array([[1,0,0],[0,math.cos(tiny),-math.sin(tiny)],[0,math.sin(tiny),math.cos(tiny)]])
        perturbed[:3,:3]=rx@base[:3,:3]
        p0,_=camera_to_body(base,cov,mount);p1,_=camera_to_body(perturbed,cov,mount)
        r=mount[:3,:3];lever=base[:3,:3]@np.linalg.inv(mount)[:3,3]
        expected=r@np.cross([1,0,0],lever)
        np.testing.assert_allclose((p1[:3,3]-p0[:3,3])/tiny,expected,atol=1e-6)
        np.testing.assert_allclose(quaternion_xyzw(np.eye(3)),[0,0,0,1])
        with self.assertRaises(ValueError):camera_to_body(pose,np.zeros((6,6)),mount)


if __name__=='__main__':unittest.main()
