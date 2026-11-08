import importlib.util
import math
from pathlib import Path
import unittest

from integrity import IntegrityGate
from replay_integrity import replay


class IntegrityTests(unittest.TestCase):
    def test_bounded_motion(self):
        gate = IntegrityGate()
        self.assertTrue(gate.observe(0, (0, 0, 0), 0, False)[0])
        self.assertTrue(gate.observe(0.1, (0, 0, 0.05), -0.05, True)[0])

    def test_jump_latches_without_replacing_last_fix(self):
        gate = IntegrityGate()
        gate.observe(0, (0, 0, 0), 0, False)
        self.assertFalse(gate.observe(0.1, (2, 0, 0), 0, True)[0])
        self.assertEqual(gate.previous, (0, (0, 0, 0)))
        self.assertFalse(gate.observe(0.2, (0, 0, 0), 0, False)[0])

    def test_continuous_height_mismatch_latches(self):
        gate = IntegrityGate()
        gate.observe(0, (0, 0, 0), 0, False)
        self.assertTrue(gate.observe(1, (0, 0, 0.4), -0.15, True)[0])
        self.assertFalse(gate.observe(1.2, (0, 0, 0.5), -0.25, True)[0])

    def test_missing_or_invalid_not_accepted(self):
        self.assertFalse(IntegrityGate().observe(0, (math.nan, 0, 0), 0, False)[0])

    @unittest.skipUnless(importlib.util.find_spec("pyulog"), "ULog replay needs pyulog")
    def test_actual_split_flight(self):
        root = Path(__file__).resolve().parents[2]
        log = root / ("local_course/state/nvidia-runs/20261107T200405Z-split-astar-flight/"
                      "px4/log/2026-11-07/20_02_46.ulg")
        if not log.exists():
            self.skipTest("preserved failed-flight ULog absent from deployment")
        result = replay(log)
        self.assertIn("vertical disagreement", result["reason"])
        self.assertAlmostEqual(result["source_s"], 525.2)
        self.assertAlmostEqual(result["arrival_s"], 525.312)
        self.assertLess(result["arrival_s"], 529.288)  # independently recorded first contact


if __name__ == "__main__":
    unittest.main()
