#!/usr/bin/env python3
"""Temporary synthetic fixtures only: no ROS, simulator, flight or hardware access."""
import importlib.util
import json
from pathlib import Path
import sqlite3
import subprocess
import sys
import tempfile
import threading
import urllib.error
import urllib.request
import zlib

HERE = Path(__file__).resolve().parent


def module(name, path):
    spec = importlib.util.spec_from_file_location(name, path)
    result = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(result)
    return result


def smoke(chase=False):
    with tempfile.TemporaryDirectory(prefix='navless-viewer-smoke-') as directory:
        root = Path(directory)
        run = root/'runs/synthetic-fixture'
        (run/'flight').mkdir(parents=True)
        # Explicit synthetic disarmed fixture, never counted as observed flight.
        samples = [dict(t=1+i*.1,x=i*.01,y=0,z=0,yaw=0,armed=False,landed=True)
                   for i in range(6)]
        (run/'flight/track.json').write_text(json.dumps(dict(samples=samples)))
        database = run/'synthetic.sqlite'
        with sqlite3.connect(database) as connection:
            connection.execute('CREATE TABLE samples(kind TEXT, stamp INTEGER, frame TEXT, data BLOB)')
            connection.execute('INSERT INTO samples VALUES(?,?,?,?)',
                               ('left_info',0,'synthetic',json.dumps(dict(width=64,height=48))))
            for i in range(6):
                pixels = bytes((x+i*20)%256 for x in range(64*48))
                connection.execute('INSERT INTO samples VALUES(?,?,?,?)',
                                   ('left',1000000000+i*100000000,'synthetic',zlib.compress(pixels)))
        exporter = module('onboard_export', HERE.parent/'export_drone_video.py')
        metadata = exporter.export(database,run,fps=15)
        assert metadata['status']=='complete' and metadata['source_frame_count']==6
        assert metadata['encoded_frame_count']==9
        # The exporter describes the production schema; label this generated fixture explicitly.
        metadata['source']='SYNTHETIC TEST PATTERN; no camera or flight recording'
        metadata['appearance_profile']='Synthetic smoke fixture'
        (run/'drone-view.json').write_text(json.dumps(metadata))
        if chase:
            assets = root/'assets'
            mesh = assets/'models/dh_course/meshes'
            mesh.mkdir(parents=True)
            (mesh/'course_visual.obj').write_text('# Synthetic smoke scene; no recorded course\n')
            (mesh/'course.mtl').write_text('# Empty synthetic material library\n')
            (mesh.parent/'model.sdf').write_text('<sdf version="1.9"><model name="smoke"/></sdf>')
            vehicle = '<sdf version="1.9"><model name="smoke"><link name="body"><visual name="body"><geometry><box><size>.2 .2 .1</size></box></geometry></visual><visual name="transparent-prop"><transparency>.5</transparency><geometry><box><size>.3 .02 .01</size></box></geometry></visual></link></model></sdf>'
            (assets/'dh_quad_stereo.sdf').write_text(vehicle)
            world = assets/'smoke.sdf'
            world.write_text('<sdf version="1.9"><world name="smoke"><model name="ground"><link name="ground"><visual name="ground"><geometry><plane><size>10 10</size></plane></geometry><material><diffuse>.4 .4 .4 1</diffuse></material></visual></link></model></world></sdf>')
            subprocess.run(['uv','run','--python','3.12',str(HERE/'render_chase.py'),str(run),
                            '--assets',str(assets),'--world',str(world)],check=True)
            assert (run/'chase-view.mp4').stat().st_size>0
        server_module = module('run_viewer',HERE/'server.py')
        server_module.RUNS=root/'runs'
        server_module.NVIDIA_RUNS=root/'nvidia'
        server_module.FLYX_RUNS=root/'flyx'
        http = server_module.ThreadingHTTPServer(('127.0.0.1',0),server_module.Handler)
        thread = threading.Thread(target=http.serve_forever,daemon=True)
        thread.start()
        base=f'http://127.0.0.1:{http.server_port}'
        try:
            with urllib.request.urlopen(base+'/api/runs') as response:
                rows=json.load(response)
            assert len(rows)==1 and rows[0]['observed_flight'] is False
            assert 'drone-view.mp4' in rows[0]['files']
            with urllib.request.urlopen(base+'/') as response:
                assert b'video-sync' in response.read()
            with urllib.request.urlopen(base+'/evidence/synthetic-fixture/drone-view.jpg') as response:
                assert response.headers['Content-Type']=='image/jpeg' and response.read().startswith(b'\xff\xd8')
            try:
                urllib.request.urlopen(urllib.request.Request(base+'/api/runs',data=b'{}',method='POST'))
                raise AssertionError('Write endpoint accepted')
            except urllib.error.HTTPError as error:
                assert error.code==501
            movie=(run/'drone-view.mp4').read_bytes()
            for requested, expected in [('bytes=0-63',movie[:64]),('bytes=-64',movie[-64:]),('bytes=64-',movie[64:])]:
                request=urllib.request.Request(base+'/evidence/synthetic-fixture/drone-view.mp4',headers={'Range':requested})
                with urllib.request.urlopen(request) as response:
                    assert response.status==206 and response.headers['Content-Type']=='video/mp4'
                    assert response.read()==expected
            request=urllib.request.Request(base+'/evidence/synthetic-fixture/drone-view.mp4',headers={'Range':'bytes=999999999-'})
            try:
                urllib.request.urlopen(request)
                raise AssertionError('Invalid range accepted')
            except urllib.error.HTTPError as error:
                assert error.code==416
            (run/'linked.mp4').symlink_to(run/'drone-view.mp4')
            assert run/'linked.mp4' not in server_module.evidence_files(run)
        finally:
            http.shutdown()
            http.server_close()
        print('PASS synthetic SQLite → H.264 MP4/poster/manifest; catalog/disarmed classification; HTTP/UI; byte ranges; symlink exclusion'+('; synthetic OpenGL chase render' if chase else ''))


if __name__=='__main__':
    smoke('--with-chase' in sys.argv[1:])
