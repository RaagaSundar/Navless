"""Small reviewed source tree; never extract a user's archive on the host."""
import base64
import json
from pathlib import Path, PurePosixPath
import stat
import sys

LIMIT = 2 * 1024 * 1024
WIRE = 3 * 1024 * 1024


def encode(source):
    files = {}
    total = 0
    for p in sorted(source.rglob('*')) if source.is_dir() else [source]:
        if p.is_symlink():
            raise ValueError(f'symlink not allowed: {p.name}')
        if p.is_dir():
            continue
        name = p.relative_to(source).as_posix() if source.is_dir() else 'main.py'
        if any(part.startswith('.') for part in PurePosixPath(name).parts):
            raise ValueError('use a dedicated source directory with no hidden files')
        if not stat.S_ISREG(p.stat().st_mode) or p.stat().st_size > LIMIT-total:
            raise ValueError('unsupported or oversized file')
        data = p.read_bytes()
        total += len(data)
        files[name] = base64.b64encode(data).decode('ascii')
        if len(files)>128 or total>LIMIT:
            raise ValueError('workspace exceeds 128 files or 2 MiB')
    payload = json.dumps(files).encode()
    if 'main.py' not in files or len(payload)>WIRE:
        raise ValueError('workspace needs main.py and must fit 3 MiB on wire')
    return payload


def unpack(payload, dest):
    if len(payload)>WIRE:
        raise ValueError('bundle too large')
    files = json.loads(payload)
    if not isinstance(files,dict) or 'main.py' not in files or len(files)>128:
        raise ValueError('invalid workspace')
    total = 0
    for name, encoded in files.items():
        rel = PurePosixPath(name)
        if rel.is_absolute() or not rel.parts or rel.as_posix()!=name or any(
            p.startswith('.') or len(p)>200 for p in rel.parts):
            raise ValueError('invalid relative path')
        data = base64.b64decode(encoded, validate=True)
        total += len(data)
        if total>LIMIT:
            raise ValueError('workspace too large')
        p = dest / rel
        p.parent.mkdir(exist_ok=True,parents=True)
        with p.open('xb') as f:
            f.write(data)


if __name__ == '__main__':
    unpack(sys.stdin.buffer.read(WIRE+1),Path.cwd())
