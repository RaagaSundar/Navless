"""Seal and map recorded sensor acquisition intervals to an independent run track."""
import argparse
import hashlib
import json
import os
from pathlib import Path
import sqlite3

def seal(run, source):
    with sqlite3.connect(source.resolve().as_uri()+'?mode=ro',uri=True) as db:
        integrity=db.execute('PRAGMA quick_check').fetchone()[0]
        rows=db.execute('SELECT kind, COUNT(*), MIN(stamp), MAX(stamp) FROM samples GROUP BY kind').fetchall()
    streams={k:(n,lo/1e9,hi/1e9) for k,n,lo,hi in rows}
    if integrity!='ok' or any(k not in streams for k in ('left','right','imu','left_info','right_info')):
        raise ValueError('incomplete source archive or failed SQLite integrity check')
    begin=max(streams[k][1] for k in ('left','right','imu'))
    end=min(streams[k][2] for k in ('left','right','imu'))
    samples=json.loads((run/'flight'/'track.json').read_text())['samples']
    moving=[r['t'] for r in samples if r['armed']]
    covered=not moving or (begin<=min(moving) and end>=max(moving))
    digest=hashlib.sha256()
    with source.open('rb') as f:
        while block:=f.read(1024*1024):digest.update(block)
    data=dict(path=str(source.resolve()),sha256=digest.hexdigest(),integrity=integrity,
              rows=rows,common_interval_sim_s=[begin,end],fully_covers_armed_interval=covered,
              track_interval_sim_s=[samples[0]['t'],samples[-1]['t']] if samples else None,
              armed_interval_sim_s=[min(moving),max(moving)] if moving else None)
    out=run/'camera-imu-capture.json'
    tmp=out.with_suffix('.partial');tmp.write_text(json.dumps(data,indent=2)+'\n');os.replace(tmp,out)
    if not covered:raise ValueError('recording did not span the complete armed interval')
if __name__=='__main__':
    parser=argparse.ArgumentParser()
    parser.add_argument('run',type=Path);parser.add_argument('source',type=Path)
    args=parser.parse_args();seal(args.run,args.source)
