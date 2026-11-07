import unittest
from launch_datum import LaunchDatum


def observe(gate, offset=(0.01,-0.02,-0.06)):
    for i in range(61):
        gate.observe(i/10,offset,(0.,0.,0.),True)


class LaunchDatumTests(unittest.TestCase):
    def test_measured_positions_round_trip(self):
        gate=LaunchDatum()
        observe(gate)
        self.assertTrue(gate.freeze(6.2))
        self.assertEqual(gate.offset,(.01,-.02,-.06))
        target=(.2,.3,-.8)
        for expected,actual in zip(target,gate.from_px4(gate.to_px4(target))):
            self.assertAlmostEqual(expected,actual)
        gate.observe(8,(0.,0.,-10.),(0.,0.,0.),False)
        self.assertEqual(gate.offset,(.01,-.02,-.06))

    def test_preserves_raw_prearm_bound_and_stationarity(self):
        for offset in ((0,0,.16),(.16,0,0)):
            gate=LaunchDatum()
            observe(gate,offset)
            self.assertFalse(gate.freeze(6))
        gate=LaunchDatum()
        observe(gate)
        gate.observe(6.1,(0,0,0),(0,0,0),False)
        self.assertFalse(gate.freeze(6.1))

    def test_stale_short_stepped_windows_fail(self):
        g=LaunchDatum()
        observe(g)
        self.assertFalse(g.freeze(6.6))
        g.observe(7,(0,0,0),(0,0,0),True)
        self.assertFalse(g.freeze(7))
        observe(g)
        g.observe(6.1,(0.01,-0.02,0.02),(0,0,0),True)
        self.assertFalse(g.freeze(6.1))
