import unittest
from startup import RecorderReadiness


class RecorderTests(unittest.TestCase):
    def test_newer_snapshot_waits_for_clock_and_prior_proof_expires(self):
        gate = RecorderReadiness('run')
        token = dict(run_id='run', status_s=10, truth_s=10, land_s=10)
        future = dict(token, status_s=10.5, truth_s=10.5)
        self.assertEqual(gate.check(token, 10.1), (True, True))
        self.assertEqual(gate.check(future, 10.4), (True, True))
        self.assertEqual(gate.check(future, 10.5), (True, True))
        self.assertEqual(gate.check(future, 11.26), (False, False))

    def test_gap_holds_disarmed_stream_only_and_long_gap_blocks_everything(self):
        gate = RecorderReadiness('run')
        token = dict(run_id='run', status_s=10, truth_s=10, land_s=10)
        self.assertEqual(gate.check(None, 10.3), (False, False))
        self.assertEqual(gate.check(token, 10.7), (True, True))
        self.assertEqual(gate.check(token, 10.8), (True, False))
        self.assertEqual(gate.check(token, 11.21), (False, False))
        self.assertEqual(gate.check(dict(token, run_id='other'), 11.22), (False, False))
