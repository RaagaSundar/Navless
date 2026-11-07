import unittest
from course_runtime.startup import fresh_recorder_token
from check_startup_recording import recorder_token


class StartupTests(unittest.TestCase):
    def setUp(self):
        self.truth = {'samples': [{'t': 10.0}], 'contact_topics_verified': True}
        self.rows = [
            dict(kind='status', timestamp=9900000, armed=False, failsafe=False),
            dict(kind='land', timestamp=9800000, landed=True),
        ]

    def test_run_bound_and_expiration(self):
        token = recorder_token('new', self.truth, self.rows)
        self.assertTrue(fresh_recorder_token(token, 'new', 10.1))
        for name, now in [('old', 10.1), ('new', 9.99), ('new', 11), ('new', float('nan'))]:
            self.assertFalse(fresh_recorder_token(token, name, now))

    def test_reject_missing_contacts_stale_or_airborne(self):
        for field, value in [('landed', False), ('timestamp', 8000000)]:
            with self.assertRaises(ValueError):
                recorder_token('new', self.truth, [self.rows[0], dict(self.rows[1], **{field: value})])
        with self.assertRaises(ValueError):
            recorder_token('new', dict(self.truth, contact_topics_verified=False), self.rows)
        with self.assertRaises(ValueError):
            recorder_token('new', self.truth, [dict(self.rows[0], failsafe=True), self.rows[1]])
        with self.assertRaises(ValueError):
            recorder_token('new', self.truth, [self.rows[1]])

    def test_newer_airborne_sample_revokes_ready_snapshot(self):
        rows = self.rows + [dict(self.rows[0], timestamp=10200000, armed=True)]
        with self.assertRaises(ValueError):
            recorder_token('id', self.truth, rows)


if __name__ == '__main__':
    unittest.main()
