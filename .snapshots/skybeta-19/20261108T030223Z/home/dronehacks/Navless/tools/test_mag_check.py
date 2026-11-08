"""tools/mag_check.py against a fake rclpy and px4_msgs, at 50x speed: one run per verdict.

  python -m pytest -q tools/test_mag_check.py

The fake PX4 feeds estimator flags and attitude by elapsed time. The lens cover maps to vision
fusion stopping from 10.5 s to 16.8 s, and STUB_MODE picks what PX4 does meanwhile:
mag (the magnetometer takes over and yaw jumps 76 deg, as in Gazebo), gyro (heading holds,
drifting 0.3 deg/s), frozen (vision yaw never stops: frozen poses through the stock bridge),
none (no PX4 data at all).
"""
import os
import re
import subprocess
import sys

import pytest

SCRIPT = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'mag_check.py')

FAKE_RCLPY = '''
import math, os, time
from types import SimpleNamespace

_K = float(os.environ.get('MAG_CHECK_TIME_SCALE', '1'))
_MODE = os.environ.get('STUB_MODE', 'mag')
_T0 = [0.0]


class _Node:
    def __init__(self):
        self.subs = {}

    def create_subscription(self, typ, topic, cb, qos):
        self.subs[topic] = cb

    def destroy_node(self):
        pass


def init():
    _T0[0] = time.monotonic()


def ok():
    return True


def create_node(name):
    return _Node()


def shutdown():
    pass


def spin_once(node, timeout_sec=0.0):
    time.sleep(timeout_sec * _K)
    t = (time.monotonic() - _T0[0]) / _K
    if _MODE == 'none':
        return
    covered = 10.5 <= t < 16.8
    ev = (not covered) or _MODE == 'frozen'
    mag = covered and _MODE == 'mag' and t >= 11.07
    node.subs['/fmu/out/estimator_status_flags'](SimpleNamespace(
        cs_ev_pos=ev, cs_ev_yaw=ev, cs_inertial_dead_reckoning=not ev, cs_yaw_align=mag,
        cs_mag_hdg=False, cs_mag_3d=mag, cs_mag_aligned_in_flight=mag, cs_mag_field_disturbed=False))
    yaw = 90.7 - (76.0 if mag else 0.0) + (0.3 * (t - 10.5) if covered and _MODE == 'gyro' else 0.0)
    r = math.radians(yaw)
    node.subs['/fmu/out/vehicle_attitude'](SimpleNamespace(q=[math.cos(r / 2), 0.0, 0.0, math.sin(r / 2)]))
'''

FAKE_QOS = '''
class _Enum:
    def __getattr__(self, name):
        return name


DurabilityPolicy = HistoryPolicy = ReliabilityPolicy = _Enum()


class QoSProfile:
    def __init__(self, **kw):
        self.kw = kw
'''

FAKE_MSGS = '''
class EstimatorStatusFlags:
    pass


class VehicleAttitude:
    pass
'''


@pytest.fixture(scope='module')
def fakes(tmp_path_factory):
    d = tmp_path_factory.mktemp('fakes')
    (d / 'rclpy').mkdir()
    (d / 'px4_msgs').mkdir()
    (d / 'rclpy' / '__init__.py').write_text(FAKE_RCLPY)
    (d / 'rclpy' / 'qos.py').write_text(FAKE_QOS)
    (d / 'px4_msgs' / '__init__.py').write_text('')
    (d / 'px4_msgs' / 'msg.py').write_text(FAKE_MSGS)
    return str(d)


def run(fakes, mode):
    env = dict(os.environ, PYTHONPATH=fakes, MAG_CHECK_TIME_SCALE='0.02', STUB_MODE=mode)
    out = subprocess.run([sys.executable, SCRIPT], env=env, capture_output=True, text=True,
                         timeout=60)
    assert out.returncode == 0, out.stderr
    results = [line for line in out.stdout.splitlines() if line.startswith('RESULT:')]
    assert len(results) == 1, out.stdout
    return out.stdout, results[0]


@pytest.mark.parametrize('mode, verdict', [
    ('mag', 'MAGNETOMETER TOOK OVER'),
    ('gyro', 'did NOT take over'),
    ('frozen', 'INCONCLUSIVE'),
    ('none', 'nothing from PX4'),
])
def test_verdict(fakes, mode, verdict):
    _, result = run(fakes, mode)
    assert verdict in result


def test_takeover_reports_the_heading_jump(fakes):
    out, result = run(fakes, 'mag')
    assert 'yaw moved up to 76.0 deg' in result
    assert 'COVER BOTH LENSES NOW' in out and 'UNCOVER' in out
    assert 'mag_aligned_in_flight' in out                  # the flag change is printed


def test_gyro_hold_reports_the_drift(fakes):
    _, result = run(fakes, 'gyro')
    drift = float(re.search(r'yaw moved up to ([0-9.]+) deg', result).group(1))
    assert 1.5 <= drift <= 1.95        # 0.3 deg/s over the ~6.3 s without vision yaw
