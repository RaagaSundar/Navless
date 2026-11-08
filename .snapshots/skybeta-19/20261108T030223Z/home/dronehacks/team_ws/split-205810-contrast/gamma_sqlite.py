"""Offline fixed pixel-only stereo enhancement. No truth, geometry or ROS input.

Both eyes receive the SAME immutable 256-entry gamma=.5 lookup table for ALL
frames. No per-eye/per-frame fitting, histogram adaptation, image warping,
synthetic features, timestamp modification, CameraInfo or IMU changes.
"""
import argparse
import hashlib
import json
import math
import sqlite3
import zlib
from pathlib import Path

LUT = bytes(round(255*math.sqrt(x/255)) for x in range(256))

def transform_source(source, output):
    if output.exists():
        raise ValueError('Refusing to overwrite output')
    src=sqlite3.connect(f'file:{source.resolve()}?mode=ro',uri=True)
    dst=sqlite3.connect(output)
    dst.execute('CREATE TABLE samples(stamp INTEGER,kind TEXT,frame TEXT,data BLOB, PRIMARY KEY(stamp,kind))')
    counts={}
    for stamp,kind,frame,data in src.execute('SELECT stamp,kind,frame,data FROM samples ORDER BY stamp,kind'):
        if kind in ('left','right'):
            raw=zlib.decompress(data)
            if len(raw)!=640*400:raise ValueError('Unexpected mono shape')
            data=zlib.compress(raw.translate(LUT),1)
        dst.execute('INSERT INTO samples VALUES (?,?,?,?)',(stamp,kind,frame,data))
        counts[kind]=counts.get(kind,0)+1
    dst.commit();dst.close();src.close()
    return {'profile':'fixed-gamma-half','formula':'round(255*sqrt(input/255))',
            'lut_sha256':hashlib.sha256(LUT).hexdigest(),'counts':counts,
            'source':str(source),'output':str(output),'source_sha256':hashlib.file_digest(open(source,'rb'),'sha256').hexdigest()}

if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('source',type=Path);p.add_argument('output',type=Path);a=p.parse_args()
    report=transform_source(a.source,a.output)
    a.output.with_suffix('.manifest.json').write_text(json.dumps(report,indent=2)+'\n')
    print(json.dumps(report,indent=2))
