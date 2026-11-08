"""Verify the staged runtime against its locally produced SHA-256 inventory."""
import hashlib
import json
from pathlib import Path

root = Path(__file__).resolve().parent
for name, expected in json.loads((root / 'SHA256.json').read_text()).items():
    path = root / name
    if not path.is_file() or path.is_symlink() or hashlib.sha256(path.read_bytes()).hexdigest() != expected:
        raise SystemExit(f'Package file missing or changed: {name}')
print('Jetson package hashes match.')
