"""Data-loss/unknown behavior tests; no images or telemetry are fabricated as fixes."""
import unittest
from unittest.mock import patch
import cv2,numpy as np
from visual_quality import VisualQuality

class SafetyInputs(unittest.TestCase):
 def setUp(self):
  self.im=np.zeros((40,64),np.uint8);self.probe=VisualQuality([[40,0,32],[0,40,20],[0,0,1]],.075,np.eye(4));self.gyro=np.array([[1,0,0,0],[1.004,0,0,0],[1.008,0,0,0]])
 def evaluate(self,g=None):return self.probe.evaluate(1,1.008,*([self.im]*4),self.gyro if g is None else g)
 def test_blank_view_is_unknown_without_landmarks(self):
  r=self.evaluate();self.assertEqual(r['state'],'unknown');self.assertEqual(r['landmarks_current_camera'],[])
 def test_missing_imu_cannot_qualify(self):self.assertEqual(self.evaluate([])['reasons'],['missing_or_gapped_imu'])
 def test_nan_imu_cannot_qualify(self):
  g=self.gyro.copy();g[1,2]=np.nan;self.assertEqual(self.evaluate(g)['reasons'],['missing_or_gapped_imu'])
 def test_duplicate_imu_cannot_qualify(self):
  g=self.gyro.copy();g[1,0]=g[0,0];self.assertEqual(self.evaluate(g)['reasons'],['missing_or_gapped_imu'])
 def test_long_frame_gap_cannot_qualify(self):self.assertEqual(self.probe.evaluate(1,1.2,*([self.im]*4),self.gyro)['state'],'unknown')
 def test_opencv_failure_is_unknown(self):
  with patch('visual_quality.cv2.goodFeaturesToTrack',side_effect=cv2.error('test')):
   r=self.evaluate();self.assertEqual(r['state'],'unknown');self.assertFalse(r['landmarks_current_camera'])
if __name__=='__main__':unittest.main()
