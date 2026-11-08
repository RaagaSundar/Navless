"""Offline fusion behavior tests with deliberately synthetic sensor inputs."""
from dataclasses import replace
import math
import unittest

import numpy as np

from fusion import FusionSession, LineSample, MapSegment, SlamSample
from reference import fit_reference


def prior(**changes):
    base = SlamSample(10, 10, (1, .1, .08),
                      ((.01, 0, 0), (0, .01, 0), (0, 0, .01)),
                      'odom', 'epoch-1', True, True)
    return replace(base, **changes)


SEGMENT = MapSegment('straight-1', (0, 0), (3, 0), 'odom', 'epoch-1')


def observation(offset=0, heading=0, **changes):
    # Real geometry fitter is the source of line observations in these tests.
    points = [(x*math.cos(heading)-offset*math.sin(heading),
               x*math.sin(heading)+offset*math.cos(heading))
              for x in np.linspace(.5, 1.5, 20)]
    ref = fit_reference(points, 10, 10)
    return replace(LineSample(ref, ((.0004, 0), (0, .0009)), 'straight-1', True, True), **changes)


def fuse(slam=None, line=None, segment=SEGMENT):
    return FusionSession('odom', 'epoch-1').fuse(
        prior() if slam is None else slam,
        observation() if line is None else line, segment, 10.05)


class FusionTests(unittest.TestCase):
    def test_centred_line_corrects_lateral_drift_and_yaw(self):
        result = fuse()
        self.assertEqual(result.mode, 'fused')
        self.assertLess(abs(result.state[1]), .02)
        self.assertLess(abs(result.state[2]), .03)
        self.assertEqual(result.state[0], 1)
        self.assertTrue(np.all(np.linalg.eigvalsh(result.covariance) > 0))

    def test_measurement_sign_and_nonzero_map_offset(self):
        # Line at world y=.5; line is .1 m left of actual vehicle -> y=.4.
        segment = replace(SEGMENT, start_xy=(0, .5), end_xy=(3, .5))
        result = fuse(prior(state=(1, .48, 0)), observation(offset=.1), segment)
        self.assertEqual(result.mode, 'fused')
        self.assertLess(abs(result.state[1]-.4), .01)

    def test_reversing_map_endpoints_preserves_answer(self):
        a = fuse(line=observation(offset=.03))
        b = fuse(line=observation(offset=.03), segment=replace(
            SEGMENT, start_xy=SEGMENT.end_xy, end_xy=SEGMENT.start_xy))
        np.testing.assert_allclose(a.state, b.state, atol=1e-12)
        np.testing.assert_allclose(a.covariance, b.covariance, atol=1e-12)

    def test_shared_camera_equal_information_not_double_counted(self):
        r = ((.01, 0), (0, .01))
        result = fuse(prior(state=(1, 0, 0)), observation(covariance=r))
        self.assertEqual(result.mode, 'fused')
        self.assertAlmostEqual(result.covariance[1][1], .01)
        self.assertAlmostEqual(result.covariance[2][2], .01)
        # No extra information along the straight line; CI is conservative.
        self.assertAlmostEqual(result.covariance[0][0], .01/.75)

    def test_rotated_line_and_yaw_wrap(self):
        segment = replace(SEGMENT, start_xy=(0, 0), end_xy=(0, 3))
        result = fuse(prior(state=(.1, 1, math.pi/2)), observation(), segment)
        self.assertEqual(result.mode, 'fused')
        self.assertLess(abs(result.state[0]), .02)
        self.assertAlmostEqual(result.state[1], 1)
        wrapped = fuse(prior(state=(1, 0, math.pi-.02)), observation(heading=.02))
        self.assertEqual(wrapped.mode, 'fused')
        self.assertAlmostEqual(wrapped.state[2], math.pi-.02)

    def test_no_line_returns_exact_raw_slam(self):
        result = FusionSession('odom', 'epoch-1').fuse(prior(), None, None, 10.05)
        self.assertEqual(result.mode, 'slam_only')
        self.assertEqual(result.state, prior().state)
        self.assertEqual(result.covariance, prior().covariance)

    def test_bad_slam_never_rescued_by_line(self):
        cases = [prior(tracking=False), prior(covariance_available=False),
                 prior(stamp_s=9), prior(status_stamp_s=9), prior(stamp_s=11),
                 prior(frame='map'), prior(reset_id='epoch-2'),
                 prior(state=(math.nan, 0, 0)), prior(covariance=np.zeros((3, 3))),
                 prior(covariance=((1, 2, 0), (2, 1, 0), (0, 0, 1)))]
        for case in cases:
            with self.subTest(case=case):
                result = fuse(case)
                self.assertEqual(result.mode, 'unavailable')
                self.assertIsNone(result.state)

    def test_line_freshness_sync_association_and_quality_gates(self):
        good = observation()
        cases = [replace(good, association_validated=False),
                 replace(good, reference=None),
                 replace(good, sensors_validated=False), replace(good, segment_id='wrong'),
                 replace(good, reference=replace(good.reference, stamp_s=9)),
                 replace(good, reference=replace(good.reference, stamp_s=10.04)),
                 replace(good, reference=replace(good.reference, stamp_s=11)),
                 replace(good, reference=replace(good.reference, rms_m=.1)),
                 replace(good, covariance=((.001, .01), (.01, .001))),
                 replace(good, covariance=((math.nan, 0), (0, 1)))]
        for case in cases:
            with self.subTest(case=case):
                result = fuse(line=case)
                self.assertEqual(result.mode, 'slam_only')
                self.assertEqual(result.state, prior().state)

    def test_wrong_segment_frame_reset_and_extrapolation(self):
        for segment in (replace(SEGMENT, frame='map'), replace(SEGMENT, reset_id='epoch-2'),
                        replace(SEGMENT, end_xy=(0, 0)),
                        replace(SEGMENT, start_xy=(2, 0)),
                        replace(SEGMENT, start_xy=(0, math.nan))):
            self.assertEqual(fuse(segment=segment).mode, 'slam_only')

    def test_large_disagreement_or_correction_rejected_not_clipped(self):
        self.assertEqual(fuse(line=observation(offset=.5)).reason, 'association_disagreement')
        self.assertEqual(fuse(line=observation(heading=.6)).reason, 'association_disagreement')
        self.assertEqual(fuse(prior(state=(1, .3, 0))).reason, 'correction_too_large')
        tiny = replace(prior(state=(1, .03, 0)), covariance=np.diag([1e-6]*3))
        self.assertEqual(fuse(tiny, observation(covariance=np.diag([1e-6]*2))).reason,
                         'innovation_gate')

    def test_repeated_slam_or_image_cannot_accumulate_information(self):
        session = FusionSession('odom', 'epoch-1')
        first = session.fuse(prior(), observation(), SEGMENT, 10.05)
        self.assertEqual(first.mode, 'fused')
        duplicate = session.fuse(prior(), observation(), SEGMENT, 10.05)
        self.assertEqual(duplicate.reason, 'repeated_or_out_of_order_slam')
        repeated_image = session.fuse(prior(stamp_s=10.01), observation(), SEGMENT, 10.05)
        self.assertEqual(repeated_image.reason, 'repeated_or_out_of_order_line')

    def test_full_covariance_is_supported(self):
        p = ((.02, .001, .001), (.001, .01, -.001), (.001, -.001, .01))
        result = fuse(prior(covariance=p), observation(covariance=((.0004, .0001), (.0001, .0009))))
        self.assertEqual(result.mode, 'fused')
        self.assertTrue(np.all(np.linalg.eigvalsh(result.covariance) > 0))


if __name__ == '__main__':
    unittest.main()
