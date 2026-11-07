"""Meaningful failure-state and measured four-view covariance checks."""
import unittest
import cv2
import numpy as np
from local_course.replay_core import StereoVO,_motion_covariance

class ReplayContractTests(unittest.TestCase):
    def setUp(self):
        self.K=np.array([[381.,0,320],[0,381.,200],[0,0,1.]])
        self.image=np.zeros((400,640),np.uint8)
    def test_blank_latches_and_explicit_reset(self):
        v=StereoVO(self.K,.075)
        self.assertFalse(v.process(1,self.image,self.image)['valid'])
        self.assertEqual(v.process(1.1,self.image,self.image)['reason'],'no_features')
        self.assertEqual(v.process(1.2,self.image,self.image)['reason'],'lost_reset_required')
        v.reset();self.assertEqual(v.process(2,self.image,self.image)['reason'],'initialized_no_motion')
    def test_timestamp_gap_and_regression_fail_closed(self):
        for t in (1.,.9,1.3):
            v=StereoVO(self.K,.075);v.process(1,self.image,self.image)
            self.assertEqual(v.process(t,self.image,self.image)['reason'],'timestamp_gap_or_regression')
            self.assertIsNone(v.process(2,self.image,self.image)['T_origin_camera'])
    def test_covariance_scales_with_measured_pixel_residual(self):
        rng=np.random.default_rng(24);X=rng.uniform([-1,-.5,1.],[1,.5,4.],size=(100,3));rv=np.array([.01,.02,-.01]);tv=np.array([.01,.01,.02]);offset=np.array([.075,0,0])
        def proj(x):
            a=(self.K@x.T).T;return a[:,:2]/a[:,2:]
        R=cv2.Rodrigues(rv)[0];cur=(R@X.T).T+tv
        obs=[proj(X),proj(X-offset),proj(cur),proj(cur-offset)]
        noise=[rng.normal(0,1,a.shape) for a in obs]
        c1,i1=_motion_covariance(X,*[a+.1*n for a,n in zip(obs,noise)],self.K,.075,rv,tv)
        c2,i2=_motion_covariance(X,*[a+.2*n for a,n in zip(obs,noise)],self.K,.075,rv,tv)
        self.assertGreater(np.linalg.eigvalsh(c1).min(),0)
        np.testing.assert_allclose(c2,c1*4,rtol=1e-10,atol=1e-12)
    def test_keyframe_recovery_spans_outage_without_reset(self):
        rng=np.random.default_rng(77)
        base=cv2.GaussianBlur(rng.integers(0,256,(400,640),dtype=np.uint8),(3,3),.7)
        def shifted(dx):return cv2.warpAffine(base,np.array([[1.,0,dx],[0,1,0]]),(640,400))
        v=StereoVO(self.K,.075,recovery_horizon_s=.5)
        v.process(1.,shifted(0),shifted(-8.2))
        self.assertFalse(v.process(1.1,self.image,self.image)['valid'])
        r=v.process(1.2,shifted(-1.2),shifted(-9.4))
        self.assertTrue(r['valid'],r['reason'])
        self.assertEqual(r['reason'],'relocalized_same_origin')
        self.assertGreater(np.linalg.norm(r['T_origin_camera'][:3,3]),.001)
    def test_invalid_calibration_rejected(self):
        with self.assertRaises(ValueError):StereoVO(self.K,0)

if __name__=='__main__':unittest.main()
