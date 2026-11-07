from types import SimpleNamespace
import unittest
from split_height.height_profile import settings, preflight_problem


def flags(**changes):
    values = dict(timestamp=1000000,cs_rng_kin_consistent=True,cs_rng_hgt=True, cs_rng_stuck=False, cs_rng_fault=False,
                  cs_ev_hgt=True, cs_baro_hgt=True, cs_rng_terrain=True,
                  cs_fake_hgt=False, cs_baro_fault=False, fs_bad_acc_vertical=False)
    values.update(changes)
    return SimpleNamespace(**values)


def parameters(profile='ev-terrain'):
    return ''.join(key+'\n'+str(value)+'\n' for key, value in settings(profile).items())


def listener(age='.1'):
    return ''.join('TOPIC: '+topic+'\n timestamp: 1000000 ('+age+' seconds ago)\n'
                   for topic in ('distance_sensor','vehicle_imu','estimator_aid_src_rng_hgt')) + '''
fused: True
cs_rng_fault: False
cs_rng_stuck: False
cs_rng_hgt: True
cs_ev_hgt: True
cs_baro_hgt: True
cs_rng_terrain: True
'''


class HeightTests(unittest.TestCase):
    def test_profiles_preserve_real_range_and_baro(self):
        self.assertEqual(settings('ev-terrain'), {'EKF2_HGT_REF':3,'EKF2_EV_CTRL':11,'EKF2_RNG_CTRL':1,'EKF2_BARO_CTRL':1})
        self.assertEqual(settings('range')['EKF2_HGT_REF'], 2)
        with self.assertRaises(ValueError):settings('invented')

    def test_actual_companion_fixed_height_fault_controls(self):
        import json
        from pathlib import Path
        from split_sim.course_runtime.conditional_range import WindowRangeHealth
        course=json.loads((Path(__file__).resolve().parents[1]/'split_sim/course_runtime/navless/gazebo_sim/course/course_v3.json').read_text())
        guard=WindowRangeHealth(next(w for w in course['walls'] if 'window' in w))
        guard.receive(1.,.82,'downward_range_link',.02,4.)
        def check(value):return guard.check(1.,value,*guard.center,True,True)[0]
        self.assertTrue(check(flags()))
        for name in ('cs_ev_hgt','cs_baro_hgt'):
            self.assertFalse(check(flags(**{name:False})))
        for name in ('cs_rng_stuck','cs_rng_fault','cs_fake_hgt','cs_baro_fault','fs_bad_acc_vertical'):
            self.assertFalse(check(flags(**{name:True})))
        self.assertFalse(check(None))

    def test_actual_parameters_and_timestamps_are_required(self):
        self.assertIsNone(preflight_problem('ev-terrain',listener(),parameters()))
        for text in ('',parameters('range'),parameters().replace('11','9')):
            self.assertIsNotNone(preflight_problem('ev-terrain',listener(),text))
        self.assertIsNotNone(preflight_problem('ev-terrain',listener('.501'),parameters()))
        self.assertIsNotNone(preflight_problem('ev-terrain','',parameters()))
        for field in ('fused: True','cs_rng_hgt: True','cs_ev_hgt: True','cs_baro_hgt: True','cs_rng_terrain: True'):
            self.assertIsNotNone(preflight_problem('ev-terrain',listener().replace(field,''),parameters()))

    def test_sill_is_agl_change_not_launch_frame_height(self):
        # Independent explanatory fixture only; this is NOT an estimator or sensor output.
        launch_height=.85
        ground_before=0.
        sill=.30
        range_before=launch_height-ground_before
        range_over_sill=launch_height-sill
        self.assertAlmostEqual(range_before-range_over_sill,.30)
        self.assertEqual(launch_height,.85)
        self.assertNotEqual(range_over_sill,launch_height)


if __name__ == '__main__':unittest.main()
