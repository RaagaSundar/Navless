"""Pure-Python safety and frame-contract tests; no ROS or flight required."""
import math
import unittest
from types import SimpleNamespace as S
from local_course.bridge_core import Gate, convert, covariance_valid
from local_course.frames import launch_odom_to_ned, ned_to_launch_odom


def sample():
    cov = [0.0]*36
    for i,v in zip((0,7,14,21,28,35), (.01,.02,.03,.04,.05,.06)):
        cov[i] = v
    return S(header=S(frame_id='odom', stamp=S(sec=10,nanosec=0)),
             child_frame_id='base_link',
             pose=S(pose=S(position=S(x=1.,y=2.,z=3.),orientation=S(x=0.,y=0.,z=0.,w=1.)),covariance=cov))


class BridgeTest(unittest.TestCase):
    def test_frames_time_unknown_velocity(self):
        out = convert(sample(),10_010_000_000,3)
        self.assertEqual(out['position'],[2.,1.,-3.])
        self.assertEqual(out['timestamp_sample'],10_000_000)
        self.assertEqual(out['timestamp'],10_010_000)
        self.assertEqual(out['pose_frame'],2)
        self.assertEqual(out['position_variance'],[.02,.01,.03])
        self.assertEqual(out['orientation_variance'],[.05,.04,.06])
        self.assertTrue(all(math.isnan(x) for x in out['velocity']))
        self.assertAlmostEqual(abs(out['q'][0]),math.sqrt(.5))
        self.assertAlmostEqual(abs(out['q'][3]),math.sqrt(.5))

    def test_calibrated_launch_rotations(self):
        self.assertEqual(launch_odom_to_ned([1,2,3],0),[1,-2,-3])
        for bearing in (0,30,90,180,-90):
            xyz=[1.2,-.7,.4]
            out=launch_odom_to_ned(xyz,bearing)
            inverse=ned_to_launch_odom(out,bearing)
            for a,b in zip(xyz,inverse): self.assertAlmostEqual(a,b)
        east=launch_odom_to_ned([1,0,0],90)
        self.assertAlmostEqual(east[0],0)
        self.assertAlmostEqual(east[1],1)
        m=sample()
        for bearing in (0,30,90,-90):
            out=convert(m,10_020_000_000,0,bearing)
            self.assertEqual(out['pose_frame'],1)
            w,x,y,z=out['q']
            yaw=math.atan2(2*(w*z+x*y),1-2*(y*y+z*z))
            self.assertAlmostEqual(yaw,math.radians(bearing))
        m.pose.pose.orientation.w=math.cos(math.pi/12)
        m.pose.pose.orientation.z=math.sin(math.pi/12)
        out=convert(m,10_020_000_000,0,0)
        w,x,y,z=out['q']
        self.assertAlmostEqual(math.atan2(2*(w*z+x*y),1-2*(y*y+z*z)),-math.pi/6)
        m.pose.covariance[1]=m.pose.covariance[6]=.003
        out=convert(m,10_020_000_000,0,45)
        self.assertAlmostEqual(out['position_variance'][0],.018)
        self.assertAlmostEqual(out['position_variance'][1],.012)
        self.assertEqual(out['position_variance'][2],.03)

    def test_fail_closed(self):
        for mutation in ('stale','future','lost','health_age','unmatched','frame','nan','zero_cov','bad_cov','quaternion'):
            m = sample(); now=10_020_000_000; health=True; age=.02; matched=True
            if mutation=='stale': now+=500_000_000
            if mutation=='future': now-=500_000_000
            if mutation=='lost': health=False
            if mutation=='health_age': age=.5
            if mutation=='unmatched': matched=False
            if mutation=='frame': m.header.frame_id='map'
            if mutation=='nan': m.pose.pose.position.x=math.nan
            if mutation=='zero_cov': m.pose.covariance=[0.]*36
            if mutation=='bad_cov': m.pose.covariance[0]=-1
            if mutation=='quaternion': m.pose.pose.orientation.w=0
            with self.subTest(mutation=mutation):
                self.assertFalse(Gate().accept(m,now,health,age,matched))

    def test_duplicate_and_reset(self):
        g=Gate(); m=sample()
        self.assertTrue(g.accept(m,10_020_000_000,True,.01,True))
        self.assertFalse(g.accept(m,10_020_000_000,True,.01,True))
        m.header.stamp.nanosec=30_000_000
        self.assertTrue(g.accept(m,10_040_000_000,True,.01,True))
        self.assertEqual(g.reset,1)
        m.header.stamp.nanosec=50_000_000
        m.pose.pose.position.x=20
        self.assertFalse(g.accept(m,10_060_000_000,True,.01,True))
        self.assertEqual(g.reason,'pose_jump')

    def test_slow_sim_and_wall_stall(self):
        g=Gate(); m=sample()
        self.assertTrue(g.accept(m,10_140_000_000,True,.14,True))
        # At slow RTF a 400ms wall gap is only 80ms of simulation time.
        g.watchdog(10_220_000_000,.4)
        self.assertTrue(g.tracking)
        m.header.stamp.nanosec=80_000_000
        self.assertTrue(g.accept(m,10_220_000_000,True,.14,True))
        self.assertEqual(g.reset,0)
        # Missing source: age exceeds unchanged 250ms simulation limit.
        g.watchdog(10_480_000_000,.6)
        self.assertFalse(g.tracking)
        m.header.stamp.nanosec=350_000_000
        self.assertTrue(g.accept(m,10_490_000_000,True,.14,True))
        self.assertEqual(g.reset,1)
        # Paused simulation clock still has a bounded wall fail-closed path.
        g.watchdog(10_490_000_000,2.01)
        self.assertFalse(g.tracking)

    def test_covariance_non_psd(self):
        c=sample().pose.covariance
        c[1]=c[6]=2
        self.assertFalse(covariance_valid(c))


if __name__ == '__main__':
    unittest.main()
