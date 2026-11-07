"""route.py must find the repo when the package is imported through a symlink.

On the drone, team_build.sh runs `colcon build --symlink-install`. colcon then
symlinks the package directory into team_ws/build/navless_nav/ and puts that
directory on PYTHONPATH, so `navless_nav/route.py` is imported as
team_ws/build/navless_nav/navless_nav/route.py. If REPO_ROOT is taken from that
path without resolving the link, it points at team_ws/build: no route file is
found and the node takes off and hovers instead of flying the course.

This rebuilds that layout in a temporary directory and imports the package
through it, from a working directory that is not the repo (the stack's
competitor window runs in /root/skybeta_ws).
"""
import json
import os
import subprocess
import sys

import pytest

PKG_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))  # navless_nav/
REPO = os.path.dirname(PKG_DIR)


def test_repo_root_survives_colcon_symlink_install(tmp_path):
    build_pkg = tmp_path / 'team_ws' / 'build' / 'navless_nav'
    build_pkg.mkdir(parents=True)
    try:
        os.symlink(os.path.join(PKG_DIR, 'navless_nav'), build_pkg / 'navless_nav',
                   target_is_directory=True)
    except (OSError, NotImplementedError) as exc:     # e.g. Windows without symlink rights
        pytest.skip(f'cannot create a directory symlink here: {exc}')
    probe = (
        'import json, sys\n'
        f'sys.path.insert(0, {str(build_pkg)!r})\n'
        'from navless_nav import route\n'
        'print(json.dumps([route.__file__, route.REPO_ROOT,\n'
        '                  route.resolve_route_path("route_gate_b.json")]))\n'
    )
    elsewhere = tmp_path / 'skybeta_ws'
    elsewhere.mkdir()
    out = subprocess.run([sys.executable, '-I', '-c', probe], cwd=elsewhere,
                         capture_output=True, text=True, timeout=60)
    assert out.returncode == 0, out.stderr
    imported_from, repo_root, route_file = json.loads(out.stdout)
    # it really was imported through the link, as on the drone ...
    assert os.path.realpath(imported_from) != os.path.abspath(imported_from)
    # ... and still finds the repo and its route files
    assert os.path.samefile(repo_root, REPO)
    assert os.path.samefile(route_file, os.path.join(REPO, 'route_gate_b.json'))
