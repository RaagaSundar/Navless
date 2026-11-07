"""Meaningful generator invariants; no Gazebo or flight processes."""
import json
from pathlib import Path
import tempfile
import unittest
from PIL import Image
from appearance_profile.generate import generate,geometry_signature,digest


class SyntheticProfileTests(unittest.TestCase):
    def fixture(self,root):
        mesh=root/'models/dh_course/meshes';mesh.mkdir(parents=True)
        (root/'models/dh_course/model.sdf').write_text('<sdf><model name="dh_course"><static>true</static><link name="course"><collision name="wall"><geometry><box><size>1 .1 1</size></box></geometry></collision></link></model></sdf>')
        (root/'dh_quad_stereo.sdf').write_text('camera calibration must remain byte-identical\n')
        (root/'world.sdf').write_text('<sdf><world name="test"><gravity>0 0 -9.8</gravity><model name="ground"><link name="link"><collision name="c"><geometry><plane><size>2 2</size></plane></geometry></collision><visual name="v"><geometry><plane><size>2 2</size></plane></geometry><material><diffuse>.45 .45 .45 1</diffuse></material></visual></link></model><light name="sun" type="directional"/></world></sdf>')
        (mesh/'course.mtl').write_text('newmtl neutral\nKa 0 0 0\nKd .8 .7 .6\nd 1\n')
        (mesh/'course_visual.obj').write_text('''mtllib course.mtl
v 0 0 0
v 2 0 0
v 2 2 0
v 0 2 0
v 0 0 1
v 2 0 1
vn 0 0 1
vn 0 -1 0
usemtl neutral
g Floor 20 x 20 ft
f 1//1 2//1 3//1
f 1//1 3//1 4//1
g M1 maze divider
f 1//2 5//2 6//2
f 1//2 6//2 2//2
g Landing pad cross
f 1//1 2//1 3//1
''')
        return root

    def test_geometry_cameras_and_collisions_unchanged(self):
        with tempfile.TemporaryDirectory() as tmp:
            root=Path(tmp);src=self.fixture(root/'base');out=root/'profile'
            before={str(p.relative_to(src)):digest(p) for p in src.rglob('*') if p.is_file()}
            manifest=generate(src,out,pixels_per_m=128,world_source=src/'world.sdf')
            mesh='models/dh_course/meshes/course_visual.obj'
            self.assertEqual(geometry_signature(src/mesh),geometry_signature(out/mesh))
            self.assertEqual((src/'models/dh_course/model.sdf').read_bytes(),(out/'models/dh_course/model.sdf').read_bytes())
            self.assertEqual((src/'dh_quad_stereo.sdf').read_bytes(),(out/'dh_quad_stereo.sdf').read_bytes())
            self.assertEqual(before,{str(p.relative_to(src)):digest(p) for p in src.rglob('*') if p.is_file()})
            text=(out/mesh).read_text()
            self.assertIn('g Landing pad cross\nusemtl neutral\nf 1//1 2//1 3//1',text)
            self.assertTrue(manifest['synthetic'])
            self.assertEqual(manifest['textured_faces'],4)
            self.assertEqual(len(manifest['changed_existing_files']),2)
            import xml.etree.ElementTree as ET
            world=ET.parse(out/'worlds/dronehacks-synthetic.sdf')
            self.assertEqual(world.findtext("./world/model[@name='ground']/link/visual/material/pbr/metal/albedo_map"),
                             'model://synthetic_ground/materials/textures/ground.png')
            self.assertEqual(world.findtext('./world/gravity'),'0 0 -9.8')
            self.assertEqual(manifest['outside_ground']['extent_m'],[2.0,2.0])
            sidecar=json.loads((out/'appearance.json').read_text())
            self.assertEqual(sidecar['appearance_profile'],manifest['profile'])
            self.assertEqual(sidecar['asset_root'],'.')
            self.assertNotIn(str(root), json.dumps(sidecar))
            self.assertIn('worlds/dronehacks-synthetic.sdf',sidecar['verified_textured_files_sha256'])
            for line in text.splitlines():
                if line.startswith('vt '):
                    self.assertTrue(all(0<=float(t)<=1 for t in line.split()[1:]))

    def test_deterministic_nonrepeating_metric_patterns(self):
        with tempfile.TemporaryDirectory() as tmp:
            root=Path(tmp);src=self.fixture(root/'base')
            a=generate(src,root/'a',seed=123,pixels_per_m=128,world_source=src/'world.sdf')
            b=generate(src,root/'b',seed=123,pixels_per_m=128,world_source=src/'world.sdf')
            self.assertEqual(a,b)
            self.assertEqual(a['generated_files_sha256'],b['generated_files_sha256'])
            for info in a['atlases'].values():
                self.assertTrue(.025<=info['feature_diameter_m'][0]<=.160)
                self.assertTrue(.025<=info['feature_diameter_m'][1]<=.160)
                self.assertGreater(info['gray_range'][1]-info['gray_range'][0],100)
            floor=next(info for info in a['atlases'].values() if info['group'].startswith('Floor'))
            im=Image.open(root/'a/models/dh_course/meshes'/floor['filename'])
            # Same-size distant regions of the floor must not be reused tiles.
            self.assertNotEqual(im.crop((0,0,64,64)).tobytes(),im.crop((128,128,192,192)).tobytes())
            c=generate(src,root/'c',seed=124,pixels_per_m=128,world_source=src/'world.sdf')
            self.assertNotEqual(a['generated_files_sha256'],c['generated_files_sha256'])

    def test_requires_opt_in_separate_new_directory(self):
        with tempfile.TemporaryDirectory() as tmp:
            root=Path(tmp);src=self.fixture(root/'base')
            with self.assertRaises(ValueError):generate(src,src)
            with self.assertRaises(ValueError):generate(src,src/'nested')
            out=root/'already';out.mkdir()
            with self.assertRaises(FileExistsError):generate(src,out)


if __name__=='__main__':unittest.main()
