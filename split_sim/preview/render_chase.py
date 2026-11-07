# /// script
# requires-python = ">=3.11,<3.14"
# dependencies = ["moderngl==5.13.0", "numpy==2.3.2", "pillow==13.0.0", "trimesh==5.2.0", "pyulog==1.3.0"]
# ///
"""Render a passive third-person replay. Run with uv run render_chase.py RUN_DIR."""
import argparse
import bisect
from collections import defaultdict
import hashlib
import json
import math
import subprocess
import sys
import tempfile
import xml.etree.ElementTree as ET
from pathlib import Path

import moderngl as gl
import numpy as np
import trimesh
from PIL import Image, ImageDraw, ImageFont
from pyulog import ULog

REPO = Path(__file__).resolve().parents[2]
WIDTH, HEIGHT, FPS = 854, 480, 25


def read_obj(path, files, diffuse=None):
    """Build draw batches from positive-indexed generated OBJ triangles and material maps."""
    colors, maps = {}, {}
    positions, normals, uvs, batches = [], [], [], defaultdict(list)
    files.add(path)
    material = None
    with path.open() as obj:
        for line in obj:
            fields = line.split()
            if not fields:
                continue
            kind, *values = fields
            if kind == 'mtllib':
                library = path.parent/' '.join(values)
                files.add(library)
                for row in library.read_text().splitlines():
                    key, *parts = row.split() or ['']
                    if key == 'newmtl':
                        name = ' '.join(parts)
                        colors[name] = [.65,.65,.65,1.]
                    elif key == 'Kd':
                        colors[name][:3] = list(map(float,parts))
                    elif key == 'd':
                        colors[name][3] = float(parts[0])
                    elif key == 'map_Kd':
                        maps[name] = library.parent/' '.join(parts)
            elif kind in ('v','vn','vt'):
                {'v':positions,'vn':normals,'vt':uvs}[kind].append(list(map(float,values)))
            elif kind == 'usemtl':
                material = ' '.join(values)
            elif kind == 'f':
                face = []
                for ref in values:
                    v, uv, n = ref.split('/')
                    face.append(positions[int(v)-1] + normals[int(n)-1] +
                                (uvs[int(uv)-1] if uv else [0.,0.]) +
                                (diffuse or colors[material]))
                vertices = batches[maps.get(material)]
                for i in range(1,len(face)-1):
                    vertices.extend((face[0],face[i],face[i+1]))
    return batches


def pose_matrix(node):
    x, y, z, roll, pitch, yaw = map(float, node.findtext('pose', '0 0 0 0 0 0').split())
    transform = trimesh.transformations.euler_matrix(roll, pitch, yaw)
    transform[:3, 3] = [x, y, z]
    return transform


def primitive_visuals(sdf, *, skip_meshes=False):
    solid, transparent = [], []
    for link in ET.parse(sdf).findall('.//model/link'):
        for visual in link.findall('visual'):
            geo = visual.find('geometry')
            if skip_meshes and geo.find('mesh') is not None:
                continue
            if geo.find('box') is not None:
                shape = trimesh.creation.box(list(map(float, geo.findtext('box/size').split())))
            elif geo.find('cylinder') is not None:
                shape = trimesh.creation.cylinder(float(geo.findtext('cylinder/radius')),
                                                  float(geo.findtext('cylinder/length')), sections=32)
            else:
                raise ValueError(f"Unsupported aircraft geometry in {visual.attrib}")
            shape.apply_transform(pose_matrix(link) @ pose_matrix(visual))
            rgb = list(map(float, visual.findtext('material/diffuse', '.65 .65 .65 1').split()))[:3]
            alpha = 1. - float(visual.findtext('transparency', '0'))
            target = transparent if alpha < 1 else solid
            for triangle, normal in zip(shape.triangles, shape.face_normals):
                for xyz in triangle:
                    target.append([*xyz, *normal, 0., 0., *rgb, alpha])
    return solid, transparent


def unit(vector):
    return vector / np.linalg.norm(vector)


class Scene:
    def __init__(self, assets, sdf, world):
        mesh = assets / 'models/dh_course/meshes'
        self.ctx = gl.create_standalone_context(**({'backend': 'egl'} if sys.platform == 'linux' else {}))
        self.fb = self.ctx.simple_framebuffer((WIDTH, HEIGHT))
        self.target = self.fb
        self.fb.use()
        self.ctx.enable(gl.DEPTH_TEST | gl.BLEND)
        self.program = self.ctx.program(vertex_shader="""
            #version 330
            uniform mat4 model, vp;
            in vec3 xyz, normal;
            in vec2 uv;
            in vec4 color;
            out vec3 n;
            out vec2 tex;
            out vec4 c;
            void main(){
                gl_Position = vp * model * vec4(xyz, 1);
                n = mat3(model) * normal; tex = uv; c = color;
            }
        """, fragment_shader="""
            #version 330
            uniform sampler2D tile;
            uniform bool textured;
            in vec3 n; in vec2 tex; in vec4 c;
            out vec4 frag;
            void main(){
                float diffuse = .70 + .30 * abs(dot(normalize(n),normalize(vec3(.4,-.3,.9))));
                vec3 pigment = textured ? texture(tile,tex).rgb : vec3(1.);
                frag = vec4(sqrt(max(c.rgb * diffuse,vec3(0))) * pigment,c.a);
            }
        """)
        self.files = {sdf, world, mesh/'course_visual.obj', mesh/'course.mtl', mesh.parent/'model.sdf'}
        self.textures = {}
        ground = ET.parse(world).find("./world/model[@name='ground']/link/visual")
        ground_size = list(map(float, ground.findtext('geometry/plane/size').split()))
        ground_color = list(map(float, ground.findtext('material/diffuse').split()))
        ground_pose = pose_matrix(ground)
        ground_rows = []
        for u,v in [(0,0),(1,0),(1,1),(0,0),(1,1),(0,1)]:
            pos = ground_pose @ [(u-.5)*ground_size[0],(v-.5)*ground_size[1],0,1]
            ground_rows.append([*pos[:3],0,0,1,u,v,*ground_color])
        ground_map = ground.findtext('material/pbr/metal/albedo_map')
        if ground_map and not ground_map.startswith('model://'):
            raise ValueError('Unsupported ground texture URI '+ground_map)
        ground_tile = assets/'models'/ground_map.removeprefix('model://') if ground_map else None
        self.surfaces = [(self.vao(ground_rows),self.tile(ground_tile))]
        model_sdf = mesh.parent/'model.sdf'
        for link in ET.parse(model_sdf).findall('./model/link'):
            for visual in link.findall('visual'):
                obj_mesh = visual.find('geometry/mesh')
                if obj_mesh is None:
                    continue
                uri = obj_mesh.findtext('uri')
                obj = assets/'models'/uri[8:] if uri.startswith('model://') else model_sdf.parent/uri
                scale = list(map(float,obj_mesh.findtext('scale','1 1 1').split()))
                transform = pose_matrix(link) @ pose_matrix(visual) @ np.diag([*scale,1.])
                color = visual.findtext('material/diffuse')
                diffuse = list(map(float,color.split())) if color else None
                if visual.get('name') == 'net_open_strands_v':
                    # Resolve subpixel strands with multisampling, without enlarging the mesh.
                    self.target = self.ctx.simple_framebuffer((WIDTH,HEIGHT),samples=min(4,self.ctx.max_samples))
                    self.target.use()
                for tile, rows in read_obj(obj,self.files,diffuse).items():
                    vertices = np.asarray(rows,dtype='f4')
                    if not np.array_equal(transform,np.eye(4)):
                        rotation = transform[:3,:3]
                        vertices[:,:3] = vertices[:,:3] @ rotation.T + transform[:3,3]
                        vertices[:,3:6] = vertices[:,3:6] @ np.linalg.inv(rotation)
                    self.surfaces.append((self.vao(vertices),self.tile(tile)))
        net_opaque, net_blended = primitive_visuals(model_sdf,skip_meshes=True)
        if net_opaque:
            self.surfaces.append((self.vao(net_opaque),None))
        self.net = self.vao(net_blended) if net_blended else None
        self.body, self.props = map(self.vao, primitive_visuals(sdf))
        if self.textures:
            next(iter(self.textures.values())).use()

    def tile(self,path):
        if path is None:
            return None
        if path not in self.textures:
            self.files.add(path)
            with Image.open(path) as image:
                pixels = image.convert('RGB').transpose(Image.Transpose.FLIP_TOP_BOTTOM)
            texture = self.ctx.texture(pixels.size,3,pixels.tobytes(),alignment=1)
            texture.build_mipmaps()
            self.textures[path] = texture
        return self.textures[path]

    def vao(self, rows):
        buf = self.ctx.buffer(np.asarray(rows, dtype='f4').tobytes())
        return self.ctx.vertex_array(self.program, [(buf, '3f 3f 2f 4f', 'xyz','normal','uv','color')])

    def matrix(self, name, mat):
        self.program[name].write(np.array(mat.T, dtype='f4').tobytes())

    def render(self, sample, attitude):
        self.target.use()
        self.target.clear(.75, .78, .82, 1)
        position = np.array([sample[k] for k in ('x','y','z')])
        yaw = sample['yaw']
        heading = np.array([math.cos(yaw), math.sin(yaw), 0.])
        eye = position - 1.65 * heading + [0, 0, 1.10]
        aim = position + .48 * heading + [0, 0, .08]
        forward = unit(aim-eye)
        right = unit(np.cross(forward, [0.,0.,1.]))
        view = np.eye(4)
        view[:3,:3] = [right, np.cross(right,forward), -forward]
        view[:3,3] = -view[:3,:3] @ eye
        # A foreground cutaway keeps the subject visible in narrow course corridors.
        near, far = np.dot(position-eye,forward) - .36, 55.
        f = 1 / math.tan(math.radians(52)/2)
        projection = np.array([[f*HEIGHT/WIDTH,0,0,0],[0,f,0,0],
                               [0,0,-(far+near)/(far-near),-2*far*near/(far-near)],[0,0,-1,0]])
        self.matrix('vp', projection @ view)
        self.matrix('model', np.eye(4))
        for vao, texture in self.surfaces:
            self.program['textured'] = texture is not None
            if texture is not None:
                texture.use()
            vao.render()
        if attitude:
            x,y,z,w = sample['orientation_xyzw']
            model = trimesh.transformations.quaternion_matrix([w,x,y,z])
        else:
            model = trimesh.transformations.euler_matrix(0,0,yaw)
        model[:3,3] = position
        self.matrix('model', model)
        self.program['textured'] = False
        self.body.render()
        self.props.render()
        if self.net is not None:
            self.matrix('model',np.eye(4))
            self.net.render()
        if self.target is not self.fb:
            self.ctx.copy_framebuffer(self.fb,self.target)
        return Image.frombytes('RGB', (WIDTH,HEIGHT), self.fb.read(components=3,alignment=1),'raw','RGB',0,-1)


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def recorded_attitude(sample):
    q = sample.get('orientation_xyzw')
    return isinstance(q,list) and len(q)==4 and np.isfinite(q).all() and .99 <= np.linalg.norm(q) <= 1.01


def nearest(stamps, times):
    j = np.searchsorted(stamps,times).clip(0,len(stamps)-1)
    prev = (j-1).clip(0)
    return np.where(np.abs(stamps[prev]-times) < np.abs(stamps[j]-times),prev,j)


def archived_attitudes(run, samples):
    """Match both independent ULog truth topics to the same unshifted simulation clock."""
    paths = list(run.glob('px4/log/**/*.ulg'))
    if len(paths) != 1:
        return {}
    attitude = 'vehicle_attitude_groundtruth'
    position = 'vehicle_local_position_groundtruth'
    log = ULog(str(paths[0]),message_name_filter_list=[attitude,position])
    if not {attitude,position}.issubset(d.name for d in log.data_list):
        return {}
    a = log.get_dataset(attitude).data
    p = log.get_dataset(position).data
    times = np.array([s['t'] for s in samples])
    ta,tp = [np.asarray(d['timestamp_sample'],dtype=float)*1e-6 for d in (a,p)]
    if any(not len(t) or np.any(np.diff(t) < 0) for t in (ta,tp)):
        return {}
    ai,pi = nearest(ta,times),nearest(tp,times)
    w,x,y,z = (a[f'q[{i}]'][ai] for i in range(4))
    yaw_enu = np.pi/2 - np.arctan2(2*(w*z+x*y),1-2*(y*y+z*z))
    yaw_diff = yaw_enu - [s['yaw'] for s in samples]
    yaw_diff = np.abs(np.arctan2(np.sin(yaw_diff),np.cos(yaw_diff)))
    xyz_enu = np.column_stack((p['y'][pi],p['x'][pi],-p['z'][pi]))
    xyz = np.array([[s[k] for k in ('x','y','z')] for s in samples])
    error = np.linalg.norm(xyz-xyz_enu,axis=1)
    valid = ((abs(ta[ai]-times)<=.025) & (abs(tp[pi]-times)<=.025) &
             (error<=.03) & (yaw_diff<=np.deg2rad(1.0)) & (abs(w*w+x*x+y*y+z*z-1)<.01))
    # q_ENU_FLU = q_ENU_NED * q_NED_FRD * q_FRD_FLU in Hamilton convention.
    q_enu_ned = [0,2**-.5,2**-.5,0]
    q_frd_flu = [0,1,0,0]
    mul = trimesh.transformations.quaternion_multiply
    matched = 0
    for i in np.flatnonzero(valid):
        if not recorded_attitude(samples[i]):
            q = mul(mul(q_enu_ned,[w[i],x[i],y[i],z[i]]),q_frd_flu)
            samples[i]['orientation_xyzw'] = [q[1],q[2],q[3],q[0]]
            matched += 1
    return dict(ulog=str(paths[0].relative_to(run)),sha256=sha(paths[0]),matched_samples=matched,
                time_tolerance_s=.025,position_tolerance_m=.03,yaw_tolerance_deg=1.)


def render(run, assets, world_override=None):
    track = run / 'flight/track.json'
    flyx = not track.is_file() and (run/'course-truth-track.json').is_file()
    if flyx:
        track = run/'course-truth-track.json'
    raw = track.read_bytes()
    data = json.loads(raw)
    samples = data['samples']
    if flyx:
        if data.get('frame') != 'course ENU; yaw is clockwise from north':
            raise ValueError('Unknown FlyX track frame')
        # course_audit.py stores truth_hdg_deg as compass radians.
        for sample in samples:
            sample['yaw'] = math.pi/2-sample['yaw']
    times = [s['t'] for s in samples]
    if (not samples or any(not all(math.isfinite(s[k]) for k in ('t','x','y','z','yaw')) for s in samples)
            or any(t < prev for t,prev in zip(times[1:],times))):
        raise ValueError('Expected a finite chronological Gazebo ENU trajectory with yaw')
    ulog_match = {} if flyx or all(recorded_attitude(s) for s in samples) else archived_attitudes(run,samples)
    attitude = [recorded_attitude(s) for s in samples]
    clip_frames = math.ceil((times[-1]-times[0])*FPS) + 1
    appearance_path = run/'appearance.json'
    appearance = json.loads(appearance_path.read_text()) if appearance_path.is_file() else {}
    if assets is None:
        root = appearance.get('source_asset_path_for_renderer')
        if root:
            assets = Path(root) if Path(root).is_absolute() else run/root
        elif appearance.get('synthetic'):
            raise ValueError('Synthetic run has no archived renderer asset path')
        else:
            assets = REPO/'split_sim/state/assets'
    assets = assets.resolve()
    if world_override is not None:
        world = world_override.resolve(strict=True)
    elif appearance.get('world_for_renderer'):
        world_path = Path(appearance['world_for_renderer'])
        if world_path.is_absolute():
            world_path = world_path.relative_to(appearance['source_asset_path_for_renderer'])
        world = assets/world_path
    elif appearance.get('synthetic'):
        raise ValueError('Synthetic run has no archived world sidecar')
    else:
        world = REPO/'gazebo_sim/worlds/dronehacks.sdf'
    sdf = run/'simulated-vehicle.sdf'
    if not sdf.is_file():
        sdf = assets/'dh_quad_stereo.sdf'
    scene = Scene(assets, sdf, world)
    used_hashes = {str(p.relative_to(assets)) if p.is_relative_to(assets) else p.name:sha(p)
                   for p in scene.files}
    if appearance.get('synthetic'):
        expected = appearance['verified_textured_files_sha256']
        for path in scene.files:
            if path == sdf and sdf.parent == run:
                continue
            rel = str(path.relative_to(assets))
            if used_hashes[rel] != expected.get(rel):
                raise ValueError('Synthetic scene asset hash mismatch: '+rel)
    best = max(samples, key=lambda s:s['z'])['t']
    poster_index = min(clip_frames-1, round((best-times[0])*FPS))
    font = ImageFont.load_default(size=16)
    with tempfile.TemporaryDirectory(dir=run) as temp:
        tmp = Path(temp)
        movie = tmp / 'chase-view.mp4'
        cmd = ['ffmpeg','-hide_banner','-loglevel','error','-f','rawvideo','-pix_fmt','rgb24',
               '-s',f'{WIDTH}x{HEIGHT}','-r',str(FPS),'-i','pipe:0','-an','-c:v','libx264',
               '-threads','2','-preset','veryfast','-crf','20','-pix_fmt','yuv420p','-movflags','+faststart',str(movie)]
        with subprocess.Popen(cmd,stdin=subprocess.PIPE) as proc:
            try:
                for i in range(clip_frames):
                    t = min(times[-1], times[0]+i/FPS)
                    j = bisect.bisect_left(times,t)
                    if j and t-times[j-1] <= times[j]-t:
                        j -= 1
                    if abs(times[j]-t) <= .15:
                        frame = scene.render(samples[j], attitude[j])
                    else:
                        frame = Image.new('RGB',(WIDTH,HEIGHT),'#293038')
                        ImageDraw.Draw(frame).text((WIDTH//2,HEIGHT//2),'No recorded pose',anchor='mm',font=font)
                    draw = ImageDraw.Draw(frame)
                    draw.rectangle((0,HEIGHT-32,WIDTH,HEIGHT),fill='#1c242c')
                    draw.text((12,HEIGHT-24),'FlyX path  /  local course reconstruction' if flyx else 'Chase replay  /  recorded simulator poses',font=font,fill='white')
                    draw.text((WIDTH-12,HEIGHT-24),f'{t:.2f} s sim',font=font,fill='white',anchor='ra')
                    if i == poster_index:
                        frame.save(tmp/'chase-view.jpg',quality=90)
                    proc.stdin.write(frame.tobytes())
            finally:
                proc.stdin.close()
            if proc.wait():
                raise RuntimeError('H.264 encoder failed')
        attitude_note = (
            'Uses recorded quaternion attitude.' if all(attitude) else
            f'Uses recorded quaternion attitude for {sum(attitude)} of {len(samples)} poses. Remaining poses show measured yaw with the aircraft drawn level.' if any(attitude) else
            'Uses recorded position and yaw with the aircraft drawn level.')
        provenance = ("Third-person reconstruction from recorded Gazebo simulator poses and the textured course mesh. "
                      + attitude_note + " Foreground geometry is clipped to keep the aircraft visible.")
        if flyx:
            provenance = ("FlyX trajectory reconstruction in the local course mesh with the local aircraft model. "
                          "Uses recorded position and compass heading with the aircraft drawn level. "
                          "This visualization does not reproduce FlyX camera footage or FlyX surface appearance.")
        meta = dict(provenance=provenance,source=str(track.relative_to(run)),
                    start_sim_s=times[0], end_sim_s=times[-1],fps=FPS,frames=clip_frames,
                    width=WIDTH,height=HEIGHT,duration_s=clip_frames/FPS,
                    source_sha256=hashlib.sha256(raw).hexdigest(), source_samples=len(samples),
                    assets_sha256=used_hashes,measured_attitude_samples=int(sum(attitude)),ulog_attitude_match=ulog_match)
        if appearance.get('net_visual_type'):
            pitch = float(appearance['nominal_pitch_m'])*1000
            diameter = float(appearance['strand_diameter_m'])*1000
            meta['note'] = (f"Net geometry follows the five CAD panel bounds. The weave uses assumed "
                            f"{pitch:g} mm spacing and {diameter:g} mm strands.")
        if appearance.get('appearance_profile'):
            meta['appearance_profile'] = appearance['appearance_profile']
        (tmp/'chase-view.json').write_text(json.dumps(meta,indent=2)+'\n')
        if sha(track) != meta['source_sha256']:
            raise RuntimeError('Track changed during replay rendering')
        for name in ['chase-view.jpg','chase-view.mp4','chase-view.json']:
            (tmp/name).replace(run/name)
        print(f'{run.name}: {clip_frames} frames, {movie.name}',flush=True)


if __name__ == '__main__':
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('run', type=Path)
    p.add_argument('--assets',type=Path,help='Relocated, hash-matched appearance asset tree')
    p.add_argument('--world',type=Path,help='World SDF matching archived appearance; synthetic hash checks still apply')
    args = p.parse_args()
    render(args.run, args.assets, args.world)
