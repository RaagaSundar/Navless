#!/usr/bin/env python3
"""Offline raw left-camera video evidence. Never subscribes, commands or edits a DB."""
import argparse
import hashlib
import json
import math
import os
from pathlib import Path
import shutil
import sqlite3
import subprocess
import tempfile
import zlib


def export(database, run_dir, fps=30, start_ns=None, end_ns=None):
    database = Path(database).resolve()
    run_dir = Path(run_dir).resolve()
    if Path(str(database) + '-journal').exists() or Path(str(database) + '-wal').exists():
        raise ValueError('Database has a journal/WAL; finalize recording before export')
    if shutil.disk_usage(run_dir).free < 256 * 1024**2:
        raise ValueError('Less than 256 MiB free; refusing encode')
    before = database.stat()
    source_hash = hashlib.file_digest(database.open('rb'), 'sha256').hexdigest()
    with sqlite3.connect(database.as_uri() + '?mode=ro', uri=True) as db:
        info = db.execute("SELECT data FROM samples WHERE kind='left_info' LIMIT 1").fetchone()
        # This recorder stores CameraInfo as JSON and mono8 frames as zlib bytes.
        calibration = json.loads(info[0]) if info else {}
        width, height = calibration.get('width', 640), calibration.get('height', 400)
        track_path = run_dir / 'flight/track.json'
        if track_path.exists() and start_ns is None and end_ns is None:
            samples = json.loads(track_path.read_text()).get('samples', [])
            if samples:
                start_ns = round(samples[0]['t']*1e9)
                end_ns = round(samples[-1]['t']*1e9)
        lower = start_ns if start_ns is not None else 0
        upper = end_ns if end_ns is not None else 2**63-1
        query = "SELECT stamp,frame,data FROM samples WHERE kind='left' AND stamp>=? AND stamp<=? ORDER BY stamp"
        stamps = [r[0] for r in db.execute(query, (lower, upper))]
        if len(stamps) < 2:
            raise ValueError('Need at least two recorded left-eye frames')
        gaps = [(b-a)/1e9 for a,b in zip(stamps, stamps[1:])]
        nominal = sorted(gaps)[len(gaps)//2]
        end_ns = stamps[-1] + round(nominal * 1e9)
        total = math.ceil((end_ns-stamps[0])/1e9*fps)
        with tempfile.TemporaryDirectory(prefix='.drone-video-', dir=run_dir) as temp:
            temp = Path(temp)
            mp4 = temp / 'drone-view.mp4'
            command = ['ffmpeg','-nostdin','-hide_banner','-loglevel','error','-y','-f','rawvideo','-pixel_format','gray','-video_size',f'{width}x{height}','-framerate',str(fps),'-i','pipe:0','-an','-c:v','libx264','-threads','1','-preset','veryfast','-crf','20','-pix_fmt','yuv420p','-movflags','+faststart',str(mp4)]
            proc = subprocess.Popen(command, stdin=subprocess.PIPE)
            emitted = 0
            try:
                rows = db.execute(query, (lower, upper))
                frame_id = None
                for index,(stamp,frame_id,data) in enumerate(rows):
                    raw = zlib.decompress(data)
                    if len(raw) != width*height:
                        raise ValueError('Recorded image shape differs from mono8 calibration')
                    next_ns = stamps[index+1] if index+1<len(stamps) else end_ns
                    boundary = min(total, math.ceil((next_ns-stamps[0])/1e9*fps))
                    for _ in range(boundary-emitted):
                        proc.stdin.write(raw)
                    emitted = boundary
                proc.stdin.close()
                if proc.wait() != 0:
                    raise RuntimeError('ffmpeg failed')
            except BaseException:
                proc.kill(); proc.wait()
                raise
            after = database.stat()
            if (before.st_size,before.st_mtime_ns)!=(after.st_size,after.st_mtime_ns):
                raise ValueError('Source changed during export; no artifact published')
            probe = json.loads(subprocess.check_output(['ffprobe','-v','error','-count_frames','-show_streams','-of','json',str(mp4)]))['streams'][0]
            if int(probe['nb_read_frames']) != total:
                raise ValueError('Encoded frame count mismatch')
            subprocess.run(['ffmpeg','-nostdin','-hide_banner','-loglevel','error','-y','-ss',str(min(5,total/fps/2)),'-i',str(mp4),'-frames:v','1',str(temp/'drone-view.jpg')],check=True)
            manifest = dict(schema=1,status='complete',source='recorded onboard simulated stereo left eye',appearance_profile='Recorded simulator appearance; profile not established by video encoder',topic='/oak/left/image_raw',kind='left',encoding='mono8',width=width,height=height,frame_id=frame_id,source_database=str(database),source_sha256=source_hash,source_size_bytes=before.st_size,requested_start_ns=start_ns,requested_end_ns=end_ns,track_bounds_source=str(track_path) if track_path.exists() else None,source_frame_count=len(stamps),first_stamp_ns=stamps[0],last_stamp_ns=stamps[-1],source_stamps_ns=stamps,max_source_gap_s=max(gaps),video='drone-view.mp4',poster='drone-view.jpg',fps=fps,encoded_frame_count=total,duration_s=total/fps,timing='Simulation timestamp playback at 1x, quantized to 1/fps; previous raw frame held across gaps; final frame held one median source interval. No interpolation or image enhancement.',hardware_qualification=False,video_sha256=hashlib.file_digest(mp4.open('rb'),'sha256').hexdigest())
            (temp/'drone-view.json').write_text(json.dumps(manifest,indent=2)+'\n')
            # Manifest is the commit marker; readers only advertise complete exports.
            (run_dir/'drone-view.json').unlink(missing_ok=True)
            for name in ('drone-view.mp4','drone-view.jpg','drone-view.json'):
                os.replace(temp/name, run_dir/name)
    return {k:v for k,v in manifest.items() if k!='source_stamps_ns'}


if __name__ == '__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--database',required=True,type=Path)
    parser.add_argument('--run-dir',required=True,type=Path)
    parser.add_argument('--fps',type=int,default=30,choices=(15,30))
    parser.add_argument('--start-ns',type=int)
    parser.add_argument('--end-ns',type=int)
    args=parser.parse_args()
    print(json.dumps(export(args.database,args.run_dir,args.fps,args.start_ns,args.end_ns),indent=2))
