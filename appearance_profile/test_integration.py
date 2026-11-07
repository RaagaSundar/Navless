import argparse
import json
from pathlib import Path
import tempfile
import unittest
import xml.etree.ElementTree as ET

from appearance_profile.generate import generate, digest
from appearance_profile import test_generate
from appearance_profile.verify import verify
from appearance_profile.static_bench import prepare


class IntegrationTests(unittest.TestCase):
    def assets(self, root):
        source = test_generate.SyntheticProfileTests().fixture(root / 'base')
        world = source / 'world.sdf'
        world.write_text(world.read_text().replace('name="test"', 'name="dronehacks"').replace(
            '</world>', '<include><uri>model://dh_course</uri></include></world>'))
        overlay = root / 'profile'
        generate(source, overlay, pixels_per_m=128, world_source=world)
        return source, overlay

    def check(self, source, overlay, expected=None, resource=None, world=None):
        return verify(overlay, expected or digest(overlay / 'appearance.json'),
                      world or overlay / 'worlds/dronehacks-synthetic.sdf',
                      resource or str(overlay / 'models') + ':' + str(source / 'models'),
                      source / 'dh_quad_stereo.sdf')

    def test_explicit_pin_and_asset_tamper_rejected(self):
        with tempfile.TemporaryDirectory() as tmp:
            source, overlay = self.assets(Path(tmp))
            report = self.check(source, overlay)
            self.assertTrue(report['synthetic'])
            self.assertNotIn(tmp, json.dumps(report))
            with self.assertRaises(ValueError):
                self.check(source, overlay, expected='0' * 64)
            asset = overlay / 'models/dh_course/model.sdf'
            asset.write_text(asset.read_text() + 'tampered')
            with self.assertRaises(ValueError):
                self.check(source, overlay)

    def test_wrong_world_resource_order_and_escape_rejected(self):
        with tempfile.TemporaryDirectory() as tmp:
            source, overlay = self.assets(Path(tmp))
            with self.assertRaises(ValueError):
                self.check(source, overlay, world=source / 'world.sdf')
            with self.assertRaises(ValueError):
                self.check(source, overlay, resource=str(source / 'models'))
            sidecar = overlay / 'appearance.json'
            meta = json.loads(sidecar.read_text())
            meta['verified_textured_files_sha256']['../base/world.sdf'] = digest(source / 'world.sdf')
            sidecar.write_text(json.dumps(meta))
            with self.assertRaises(ValueError):
                self.check(source, overlay)

    def test_open_net_profile_uses_same_renderer_verifier(self):
        from appearance_profile import net, test_net
        with tempfile.TemporaryDirectory() as tmp:
            root=Path(tmp)
            source=test_generate.SyntheticProfileTests().fixture(root/'base')
            model=Path(__file__).resolve().parents[1]/'gazebo_sim/models/dh_course/model.sdf'
            (source/'models/dh_course/model.sdf').write_bytes(model.read_bytes())
            world=source/'world.sdf'
            world.write_text(world.read_text().replace('name="test"','name="dronehacks"').replace(
                '</world>','<include><uri>model://dh_course</uri></include></world>'))
            parent=root/'parent'
            generate(source,parent,pixels_per_m=128,world_source=world)
            archive=test_net.archive_fixture(root/'proxy.zip')
            overlay=root/'open'
            net.generate(parent,overlay,archive)
            report=self.check(source,overlay)
            self.assertEqual(report['appearance_profile'],'synthetic-neutral-open-net-v2')
            self.assertNotIn(str(root),json.dumps(report))
            (overlay/'models/dh_course/meshes/net_open_strands.obj').write_text('tampered')
            with self.assertRaises(ValueError):self.check(source,overlay)

    def test_static_world_is_optical_only_and_sources_unchanged(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            world = root / 'world.sdf'
            world.write_text('''<sdf><world name="dronehacks"><gravity>0 0 -9.8</gravity>
<plugin name="gz::sim::systems::Physics"/><plugin name="gz::sim::systems::Sensors"/>
<gui/><include><uri>model://dh_course</uri></include></world></sdf>''')
            camera = root / 'dh_quad_stereo.sdf'
            camera.write_text('''<sdf><model><link>
<sensor name="oak_left" type="camera"><pose>0 .0375 0 0 0 0</pose><camera><image><width>640</width><height>400</height></image></camera></sensor>
<sensor name="oak_right" type="camera"><pose>0 -.0375 0 0 0 0</pose><camera><image><width>640</width><height>400</height></image></camera></sensor>
<sensor name="imu" type="imu"/></link></model></sdf>''')
            pose = root / 'pose.json'; attitude = root / 'attitude.json'
            pose.write_text(json.dumps({'nearest_physical_sample': {'x':1,'y':2,'z':3,'yaw':0}}))
            attitude.write_text(json.dumps({'roll_pitch_yaw_ENU_FLU_rad':[.1,.2,.3]}))
            before = {p.name: digest(p) for p in (world, camera, pose, attitude)}
            output = root / 'out'
            args = argparse.Namespace(output=output, world=world, camera=camera, pose=pose, attitude=attitude)
            prepare(args)
            tree = ET.parse(output / 'world.sdf')
            self.assertIsNone(tree.find('./world/gui'))
            self.assertEqual([p.get('name') for p in tree.findall('./world/plugin')], ['gz::sim::systems::Sensors'])
            rig = tree.find("./world/model[@name='passive_camera_rig']")
            self.assertEqual(rig.findtext('static'), 'true')
            self.assertEqual(rig.findtext('pose'), '1 2 3 0.1 0.2 0.3')
            sensors = rig.findall('link/sensor')
            self.assertEqual([s.get('name') for s in sensors], ['oak_left','oak_right'])
            self.assertEqual(ET.tostring(sensors[0]), ET.tostring(ET.parse(camera).find('.//sensor')))
            self.assertEqual(before, {p.name: digest(p) for p in (world, camera, pose, attitude)})
            self.assertNotIn(tmp, (output / 'provenance.json').read_text())
            with self.assertRaises(FileExistsError):
                prepare(args)


if __name__ == '__main__':
    unittest.main()
