import copy
import unittest
from split_setup.doctor import assess


def fixture(name='drone19-split-sim', running=True):
    return {'Name': '/' + name, 'State': {'Running': running}, 'Image': 'sha256:fixture',
            'Config': {'Env': ['ROS_DOMAIN_ID=219', 'ROS_LOCALHOST_ONLY=1', 'SECRET=do-not-print']},
            'HostConfig': {'Privileged': False, 'Devices': [{'PathOnHost': '/dev/dri/renderD128'}]},
            'Mounts': [{'Source': '/private/example', 'Destination': '/split'}]}


class IsolationTests(unittest.TestCase):
    def check(self, items):
        return assess(items, 'drone19-split-sim')

    def test_valid_split_sanitized_and_nonmutating(self):
        item = fixture()
        before = copy.deepcopy(item)
        report = self.check([item])
        self.assertTrue(report['isolation_ok'])
        self.assertNotIn('do-not-print', str(report))
        self.assertNotIn('/private/example', str(report))
        self.assertEqual(item, before)

    def test_local_bench_stopped_is_allowed(self):
        self.assertTrue(self.check([fixture(), fixture('drone19-gazebo-bench', False)])['isolation_ok'])

    def test_collision(self):
        self.assertFalse(self.check([fixture(), fixture('drone19-gazebo-bench')])['isolation_ok'])

    def test_missing_or_stopped(self):
        for items in ([], [fixture(running=False)]):
            self.assertFalse(self.check(items)['isolation_ok'])

    def test_wrong_domain_and_localhost(self):
        for env in ([], ['ROS_DOMAIN_ID=19', 'ROS_LOCALHOST_ONLY=1'],
                    ['ROS_DOMAIN_ID=219', 'ROS_LOCALHOST_ONLY=0']):
            item = fixture(); item['Config']['Env'] = env
            self.assertFalse(self.check([item])['isolation_ok'])

    def test_device_and_privilege(self):
        for patch in ({'Privileged': True}, {'Devices': [{'PathOnHost': '/dev/ttyACM0'}]}):
            item = fixture(); item['HostConfig'].update(patch)
            self.assertFalse(self.check([item])['isolation_ok'])
        item = fixture(); item['Mounts'] = [{'Source': '/dev/bus/usb'}]
        self.assertFalse(self.check([item])['isolation_ok'])

    def test_duplicate_record(self):
        self.assertFalse(self.check([fixture(), fixture()])['isolation_ok'])


if __name__ == '__main__':
    unittest.main()
