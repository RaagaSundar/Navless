"""Synthetic geometry checks only; never starts ROS or commands a vehicle."""
import math
import unittest

from reference import fit_reference, floor_points


FORWARD = ((0, 0, 1), (-1, 0, 0), (0, -1, 0))
K = (400, 400, 320, 200)


class ReferenceTests(unittest.TestCase):
    def test_forward_camera_projection_and_lateral_sign(self):
        for y in (-0.15, 0.15):
            # Project a physical line into a forward camera at height 0.9 m.
            pixels = [(320 - 400*y/x, 200 + 400*0.9/x)
                      for x in (1 + i/20 for i in range(21))]
            points = floor_points(pixels, K, FORWARD, (0, 0, 0), 0.9)
            fit = fit_reference(points, 10, 10.05)
            self.assertIsNotNone(fit)
            self.assertAlmostEqual(fit.offset_m, y)
            self.assertAlmostEqual(fit.closest_xy[1], y)
            self.assertAlmostEqual(fit.heading_rad, 0)

    def test_mount_translation(self):
        # Optical ray through (1,-.2,-1) from camera at (.1,.03,.04).
        points = floor_points([(400, 600)], K, FORWARD, (.1, .03, .04), .96)
        self.assertAlmostEqual(points[0][0], 1.1)
        self.assertAlmostEqual(points[0][1], -.17)

    def test_tilted_camera(self):
        a = .2
        c, s = math.cos(a), math.sin(a)
        tilt = ((0, s, c), (-1, 0, 0), (0, -c, s))
        # Generate a ray by inverse rotation from a known floor point.
        p = (1.4, .12, -.9)
        ray = [sum(tilt[i][j]*p[i] for i in range(3)) for j in range(3)]
        pixel = (320 + 400*ray[0]/ray[2], 200 + 400*ray[1]/ray[2])
        projected = floor_points([pixel], K, tilt, (0, 0, 0), .9)[0]
        self.assertAlmostEqual(projected[0], p[0])
        self.assertAlmostEqual(projected[1], p[1])

    def test_horizon_upward_distant_and_nonfinite_pixels(self):
        self.assertEqual(floor_points([(320, 200), (320, 100), (320, 201),
                                       (math.nan, 300)], K, FORWARD, (0, 0, 0), 1), [])

    def test_invalid_calibration(self):
        for r, height in ((FORWARD, 0), (((1, 0, 0), (0, 1, 0), (0, 0, -1)), 1),
                          (((2, 0, 0), (0, 1, 0), (0, 0, 1)), 1)):
            with self.assertRaises(ValueError):
                floor_points([], K, r, (0, 0, 0), height)

    def test_heading_and_endpoint_order(self):
        points = [(1+i/20, .1 + .2*(1+i/20)) for i in range(21)]
        a = fit_reference(points, 1, 1)
        b = fit_reference(points[::-1], 1, 1)
        self.assertAlmostEqual(a.heading_rad, math.atan(.2))
        self.assertAlmostEqual(a.offset_m, .1/math.sqrt(1.04))
        self.assertAlmostEqual(a.offset_m, b.offset_m)

    def test_along_line_translation_unobservable(self):
        a = fit_reference([(i/10, .1) for i in range(20)], 1, 1)
        b = fit_reference([(i/10+2, .1) for i in range(20)], 1, 1)
        self.assertAlmostEqual(a.offset_m, b.offset_m)
        self.assertAlmostEqual(a.heading_rad, b.heading_rad)

    def test_reject_stale_future_and_nonfinite_time(self):
        points = [(i/10, .1) for i in range(20)]
        for stamp, now in ((1, 1.2), (2, 1), (math.nan, 1)):
            self.assertIsNone(fit_reference(points, stamp, now))

    def test_reject_corner_mixed_parallel_lines_short_and_missing(self):
        cases = [[], [(i/1000, 0) for i in range(20)],
                 [(i/20, 0) for i in range(20)] + [(1, i/20) for i in range(20)],
                 [(i/20, y) for y in (-.1, .1) for i in range(20)],
                 [(0, i/20) for i in range(20)], [(math.nan, 0)]*20]
        for points in cases:
            self.assertIsNone(fit_reference(points, 1, 1))


if __name__ == '__main__':
    unittest.main()
