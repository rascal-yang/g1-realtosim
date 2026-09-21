"""Small shared I/O helpers. Paths in manifests are relative to that manifest."""
import hashlib
import json
import os
from pathlib import Path


def read_json(path):
    return json.loads(Path(path).read_text())


def write_json(path, value):
    Path(path).write_text(json.dumps(value, ensure_ascii=False, indent=2) + '\n')


def resolve(value, base):
    return (Path(base) / os.path.expandvars(str(value))).expanduser().resolve()


def sha256(path):
    h = hashlib.sha256()
    with Path(path).open('rb') as f:
        for chunk in iter(lambda: f.read(1024 * 1024), b''):
            h.update(chunk)
    return h.hexdigest()


def fresh_dir(path):
    path = Path(path).resolve()
    path.mkdir(parents=True, exist_ok=False)
    return path
