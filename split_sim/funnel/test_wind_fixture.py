from pathlib import Path
import json
import tempfile
import unittest
from xml.etree import ElementTree as ET
from wind_fixture import assets, verify


class WindFixtureTests(unittest.TestCase):
    def test_seed_and_model(self):
        a=assets(42)
        self.assertEqual(a,assets(42))
        self.assertNotEqual(a['wind-fixture.json'],assets(43)['wind-fixture.json'])
        profile=json.loads(a['wind-fixture.json'])
        self.assertTrue(.20 <= abs(profile['spawn_enu_m'][1]) <= .34)
        self.assertIn(str(profile['spawn_enu_m'][1]),a['spawn-request.txt'].decode())
        quad=ET.fromstring(a['simulated-quad.sdf'])
        self.assertEqual(quad.findtext("model/link[@name='base_link']/enable_wind"),'true')
        world=ET.fromstring(a['funnel.sdf'])
        self.assertEqual(float(world.findtext('world/wind/linear_velocity').split()[1]),
                         profile['wind_enu_m_s'][1])
        with tempfile.TemporaryDirectory() as d:
            for name,data in a.items():
                (Path(d)/name).write_bytes(data)
            verify(Path(d))
            (Path(d)/'spawn-request.txt').write_text('invalid')
            with self.assertRaises(ValueError):
                verify(Path(d))
