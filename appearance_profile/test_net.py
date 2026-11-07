"""Open net geometry and source-preservation tests; no live services."""
import json
from pathlib import Path
import tempfile
import unittest
from appearance_profile.net import cad_panels,collision_bytes,digest,generate,strand_mesh,without_net_visuals

ROOT=Path(__file__).resolve().parents[1]
# Build a small synthetic proxy archive in each test; no external binary fixture.
def archive_fixture(path):
    import zipfile
    groups = {
        'Net side north': ([-3.048,3.048,0], [3.048,3.04927,3.048]),
        'Net side south': ([-3.048,-3.04927,0], [3.048,-3.048,3.048]),
        'Net side east': ([3.048,-3.048,0], [3.04927,3.048,3.048]),
        'Net side west': ([-3.04927,-3.048,0], [-3.048,3.048,3.048]),
        'Net roof': ([-3.048,-3.048,3.048], [3.048,3.048,3.04927])}
    lines=[];offset=0
    faces=((1,2,3),(1,3,4),(5,7,6),(5,8,7),(1,5,6),(1,6,2),
           (2,6,7),(2,7,3),(3,7,8),(3,8,4),(4,8,5),(4,5,1))
    for name,(lo,hi) in groups.items():
        vertices=((lo[0],lo[1],lo[2]),(hi[0],lo[1],lo[2]),(hi[0],hi[1],lo[2]),(lo[0],hi[1],lo[2]),
                  (lo[0],lo[1],hi[2]),(hi[0],lo[1],hi[2]),(hi[0],hi[1],hi[2]),(lo[0],hi[1],hi[2]))
        lines.extend('v '+' '.join(map(str,v)) for v in vertices)
        lines.extend(('g '+name,'usemtl net'))
        lines.extend('f '+' '.join(str(i+offset) for i in f) for f in faces)
        offset+=8
    with zipfile.ZipFile(path,'w') as z:
        z.writestr('dronehacks-course-v3.obj','\n'.join(lines))
        z.writestr('README.txt','SYNTHETIC TEST FIXTURE, NOT ORGANIZER DATA. Units: metres')
        z.writestr('dronehacks-course-v3.mtl','newmtl net\nKd .09804 .09804 .09804\n')
    return path


class OpenNetTests(unittest.TestCase):
    def setUp(self):
        self.archive_dir=tempfile.TemporaryDirectory()
        self.addCleanup(self.archive_dir.cleanup)
        self.archive=archive_fixture(Path(self.archive_dir.name)/'proxy.zip')

    def fixture(self,root):
        (root/'models/dh_course/meshes').mkdir(parents=True)
        (root/'models/dh_course/model.sdf').write_bytes((ROOT/'gazebo_sim/models/dh_course/model.sdf').read_bytes())
        (root/'models/dh_course/meshes/fixed.obj').write_text('unchanged non-net mesh\n')
        (root/'worlds').mkdir();(root/'worlds/dronehacks-synthetic.sdf').write_text('unchanged world, sky, lights and ground\n')
        (root/'appearance.json').write_text(json.dumps({'appearance_profile':'synthetic-nonrepeating-neutral-v1','schema_version':2}))
        (root/'SYNTHETIC_APPEARANCE.txt').write_text('parent label')
        meta=json.loads((root/'appearance.json').read_text())
        meta['verified_textured_files_sha256']={str(p.relative_to(root)):digest(p) for p in root.rglob('*') if p.is_file() and p.name!='appearance.json'}
        (root/'appearance.json').write_text(json.dumps(meta))
        return root

    def test_cad_metric_bounds_are_proxies_not_weave(self):
        panels,p=cad_panels(self.archive)
        self.assertEqual(len(panels),5)
        for n,v in panels.items():
            self.assertEqual(v['triangle_count'],12)
            self.assertEqual(v['unique_vertex_count'],8)
        self.assertAlmostEqual(panels['Net side east']['low'][0],3.048)
        self.assertAlmostEqual(panels['Net roof']['low'][2],3.048)
        self.assertIn('Units: metres',p['README'])

    def test_true_geometric_openings_not_a_plane(self):
        with tempfile.TemporaryDirectory() as tmp:
            panels,_=cad_panels(self.archive)
            meta,verts,faces=strand_mesh(Path(tmp)/'net.obj',panels,.0508,.00127)
            self.assertGreater(len(faces),10000)
            self.assertLess(len(faces),25000)
            for n,m in meta.items():
                self.assertGreater(m['nominal_open_area_fraction'],.95)
                self.assertLess(m['nominal_open_area_fraction'],.952)
            # A normal ray through the middle of a NORTH mesh cell must miss all
            # strand triangles; a ray on a strand must intersect actual geometry.
            m=meta['Net side north'];lo=m['low'];pitch=m['actual_pitch_m']
            first_u=lo[0]+.00127/2;first_v=lo[2]+.00127/2
            hole=(first_u+pitch[0]/2,first_v+pitch[1]/2)
            hit=(first_u,first_v+pitch[1]/2)
            north=[f for f in faces if all(abs(verts[i][1]-m['center_plane_m'])<.002 for i in f)]
            def intersections(q):
                found=0
                for f in north:
                    a,b,c=[verts[i] for i in f]
                    x0,z0=a[0],a[2];ux,uz=b[0]-x0,b[2]-z0;vx,vz=c[0]-x0,c[2]-z0
                    det=ux*vz-uz*vx
                    if abs(det)<1e-15:continue
                    px,pz=q[0]-x0,q[1]-z0
                    u=(px*vz-pz*vx)/det;v=(ux*pz-uz*px)/det
                    found+=int(u>=-1e-9 and v>=-1e-9 and u+v<=1+1e-9)
                return found
            self.assertEqual(intersections(hole),0)
            self.assertGreater(intersections(hit),0)
            # Closed strand components have outward winding (positive volume).
            for start in range(0,len(faces),20):
                volume=0
                for f in faces[start:start+20]:
                    a,b,c=[verts[i] for i in f]
                    cross=[b[1]*c[2]-b[2]*c[1],b[2]*c[0]-b[0]*c[2],b[0]*c[1]-b[1]*c[0]]
                    volume+=sum(a[k]*cross[k] for k in range(3))/6
                self.assertGreater(volume,0)

    def test_collision_and_all_non_net_assets_unchanged_and_deterministic(self):
        with tempfile.TemporaryDirectory() as tmp:
            root=Path(tmp);src=self.fixture(root/'base');before={str(p.relative_to(src)):digest(p) for p in src.rglob('*') if p.is_file()}
            a=generate(src,root/'a',self.archive);b=generate(src,root/'b',self.archive)
            old=(src/'models/dh_course/model.sdf').read_text();new=(root/'a/models/dh_course/model.sdf').read_text()
            self.assertEqual(collision_bytes(old),collision_bytes(new))
            self.assertEqual(without_net_visuals(old),without_net_visuals(new))
            self.assertNotIn('net_north_v',new);self.assertIn('net_open_strands_v',new)
            self.assertEqual(before,{str(p.relative_to(src)):digest(p) for p in src.rglob('*') if p.is_file()})
            self.assertEqual((root/'a/models/dh_course/meshes/net_open_strands.obj').read_bytes(),(root/'b/models/dh_course/meshes/net_open_strands.obj').read_bytes())
            self.assertEqual(a['collision_sha256'],b['collision_sha256'])
            self.assertNotIn(str(root),json.dumps(a))
            self.assertEqual(a['schema_version'],2)
            self.assertEqual(a['parent_appearance_sha256'],digest(src/'appearance.json'))
            self.assertEqual((root/'a/worlds/dronehacks-synthetic.sdf').read_bytes(),(src/'worlds/dronehacks-synthetic.sdf').read_bytes())
            with self.assertRaises(FileExistsError):generate(src,root/'a',self.archive)
            (src/'worlds/dronehacks-synthetic.sdf').write_text('tampered')
            with self.assertRaises(ValueError):generate(src,root/'tampered',self.archive)


if __name__=='__main__':unittest.main()
