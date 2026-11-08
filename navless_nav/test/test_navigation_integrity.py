"""Recorded jump magnitudes and pose-integrity boundaries; no ROS or flight process."""
import math
from pathlib import Path
import sys
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from navless_nav.navigation_integrity import PoseIntegrity


class PoseIntegrityTests(unittest.TestCase):
    def packet(self, guard, stamp=1.0, now=None, **changes):
        values = dict(stamp_s=stamp, now_s=stamp if now is None else now,
                      frame_id='odom', child_frame_id='base_link',
                      position=(0., 0., 0.), quaternion=(0., 0., 0., 1.))
        values.update(changes)
        return guard.observe(**values)

    def test_no_pose_is_not_ready(self):
        guard = PoseIntegrity()
        self.assertFalse(guard.fresh(1.0))
        self.assertEqual(guard.reason, 'no_sample')
        self.assertIsNone(guard.fault)

    def test_normal_motion_at_half_and_one_point_five_metres_per_second(self):
        for speed in (.5, 1.5):
            with self.subTest(speed=speed):
                guard = PoseIntegrity()
                for i in range(20):
                    self.assertTrue(self.packet(guard, stamp=1+i*.1, now=1+i*.1+.03,
                                               position=(i*.1*speed, 0., .7)))
                self.assertTrue(guard.fresh(2.95))
                self.assertIsNone(guard.fault)

    def test_recorded_crash_jump_magnitudes_latch_without_using_tracking_state(self):
        for distance, dt in ((1.14, .102), (1.825, .098)):
            with self.subTest(distance=distance):
                guard = PoseIntegrity()
                self.assertTrue(self.packet(guard))
                self.assertFalse(self.packet(guard, stamp=1+dt, position=(distance, 0., 0.)))
                self.assertEqual(guard.fault, 'position_jump')
                self.assertEqual(guard.last_stamp_s, 1.0)
                self.assertFalse(self.packet(guard, stamp=2.0))
                self.assertFalse(guard.fresh(2.0))
                self.assertEqual(guard.reason, 'position_jump')

    def test_arrival_delay_does_not_increase_allowed_motion(self):
        guard = PoseIntegrity(max_age_s=1.0)
        self.assertTrue(self.packet(guard))
        self.assertFalse(self.packet(guard, stamp=1.1, now=1.8, position=(.9, 0., 0.)))
        self.assertEqual(guard.fault, 'position_jump')

    def test_duplicate_does_not_refresh_accepted_source_stamp(self):
        guard = PoseIntegrity()
        self.assertTrue(self.packet(guard))
        self.assertFalse(self.packet(guard, stamp=1.0, now=1.20))
        self.assertEqual(guard.reason, 'duplicate_stamp')
        self.assertEqual(guard.last_stamp_s, 1.0)
        self.assertFalse(guard.fresh(1.30))
        self.assertIsNone(guard.fault)

    def test_source_stale_and_future_packets_never_refresh_continuity(self):
        guard = PoseIntegrity()
        self.assertTrue(self.packet(guard))
        self.assertFalse(self.packet(guard, stamp=1.1, now=1.5, position=(.1, 0., 0.)))
        self.assertEqual(guard.reason, 'stale_sample')
        self.assertFalse(self.packet(guard, stamp=2., now=1.5))
        self.assertEqual(guard.reason, 'future_sample')
        self.assertEqual(guard.last_stamp_s, 1.0)
        self.assertIsNone(guard.fault)
        # A genuine fresh sample can resume continuity across a longer source interval.
        self.assertTrue(self.packet(guard, stamp=2., position=(.5, 0., 0.)))

    def test_nonmonotonic_source_stamp_latches(self):
        guard = PoseIntegrity()
        self.assertTrue(self.packet(guard))
        self.assertFalse(self.packet(guard, stamp=.99, now=1.01))
        self.assertEqual(guard.fault, 'nonmonotonic_stamp')

    def test_initial_invalid_pose_is_not_latched_but_later_invalid_pose_is(self):
        invalids = [dict(position=None), dict(position=(math.nan, 0., 0.)),
                    dict(position=(0., 0.)), dict(quaternion=(0., 0., 0., 0.)),
                    dict(quaternion=(0., 0., 0., 1.1)),
                    dict(quaternion=(0., math.inf, 0., 1.))]
        for values in invalids:
            with self.subTest(values=values):
                guard = PoseIntegrity()
                self.assertFalse(self.packet(guard, **values))
                self.assertIsNone(guard.last_stamp_s)
                self.assertIsNone(guard.fault)
                self.assertTrue(self.packet(guard))
                self.assertFalse(self.packet(guard, stamp=1.1, **values))
                self.assertIsNotNone(guard.fault)

    def test_missing_or_nonfinite_times_rejected(self):
        for changes in [dict(stamp_s=None), dict(stamp_s=math.nan), dict(now_s=math.inf)]:
            with self.subTest(changes=changes):
                guard = PoseIntegrity()
                self.assertFalse(self.packet(guard, **changes))
                self.assertIsNone(guard.last_stamp_s)
                self.assertTrue(self.packet(guard))
                self.assertFalse(self.packet(guard, **changes))
                self.assertEqual(guard.fault, 'invalid_time')

    def test_changed_or_unknown_frames_rejected(self):
        for changes in [dict(frame_id='map'), dict(frame_id=''), dict(frame_id=None),
                        dict(child_frame_id='camera'), dict(child_frame_id='')]:
            with self.subTest(changes=changes):
                guard = PoseIntegrity()
                self.assertFalse(self.packet(guard, **changes))
                self.assertIsNone(guard.fault)
                self.assertTrue(self.packet(guard))
                self.assertFalse(self.packet(guard, stamp=1.1, **changes))
                self.assertEqual(guard.fault, 'frame_mismatch')

    def test_reset_counter_change_latches_including_wraparound(self):
        for before, after in [(0, 1), (255, 0)]:
            guard = PoseIntegrity()
            self.assertTrue(self.packet(guard, reset_counter=before))
            self.assertFalse(self.packet(guard, stamp=1.1, reset_counter=after))
            self.assertEqual(guard.fault, 'reset_counter_changed')

    def test_optional_counter_can_be_learned_without_inventing_a_reset(self):
        guard = PoseIntegrity()
        self.assertTrue(self.packet(guard))
        self.assertTrue(self.packet(guard, stamp=1.1, reset_counter=4))
        self.assertTrue(self.packet(guard, stamp=1.2))
        self.assertFalse(self.packet(guard, stamp=1.3, reset_counter=5))
        self.assertEqual(guard.fault, 'reset_counter_changed')

    def test_quaternion_sign_is_equivalent_but_large_rotation_latches(self):
        guard = PoseIntegrity()
        self.assertTrue(self.packet(guard))
        self.assertTrue(self.packet(guard, stamp=1.1, quaternion=(0., 0., 0., -1.)))
        self.assertFalse(self.packet(guard, stamp=1.2,
                                     quaternion=(0., 0., math.sin(.5), math.cos(.5))))
        self.assertEqual(guard.fault, 'orientation_jump')

    def test_angular_rate_uses_full_quaternion_and_source_interval(self):
        guard = PoseIntegrity()
        self.assertTrue(self.packet(guard))
        angle = .2
        self.assertTrue(self.packet(guard, stamp=1.1,
                                    quaternion=(math.sin(angle/2), 0., 0., math.cos(angle/2))))
        self.assertFalse(self.packet(guard, stamp=1.2,
                                     quaternion=(math.sin(.9), 0., 0., math.cos(.9))))
        self.assertEqual(guard.fault, 'orientation_jump')

    def test_fresh_uses_source_time_and_validates_now(self):
        guard = PoseIntegrity()
        self.assertTrue(self.packet(guard, stamp=1., now=1.2))
        self.assertTrue(guard.fresh(1.24))
        self.assertFalse(guard.fresh(1.26))
        self.assertFalse(guard.fresh(.9))
        self.assertFalse(guard.fresh(math.nan))
        self.assertIsNone(guard.fault)

    def test_configuration_validation(self):
        for values in [dict(max_age_s=0), dict(max_speed_mps=-1),
                       dict(max_yaw_rate_rps=math.inf), dict(position_slack_m=-.1),
                       dict(future_tolerance_s=-.001), dict(frame_id=''),
                       dict(child_frame_id=None)]:
            with self.subTest(values=values):
                with self.assertRaises((TypeError, ValueError)):
                    PoseIntegrity(**values)


if __name__ == '__main__':
    unittest.main()
