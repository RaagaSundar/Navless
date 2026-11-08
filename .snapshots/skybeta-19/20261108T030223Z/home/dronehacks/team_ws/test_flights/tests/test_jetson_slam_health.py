"""Pure health adapter tests. No ROS, aircraft, network or simulator."""
import importlib.util
import json
from pathlib import Path
from types import SimpleNamespace as NS

import pytest

spec = importlib.util.spec_from_file_location('jetson_health', Path(__file__).parents[1]/'jetson_slam_health.py')
health = importlib.util.module_from_spec(spec)
spec.loader.exec_module(health)


def sample(stamp=100.):
    return NS(header=NS(stamp=NS(sec=int(stamp), nanosec=round((stamp-int(stamp))*1e9)), frame_id='odom'),
              child_frame_id='base_link', pose=NS(pose=NS(position=NS(x=0.,y=0.,z=0.),
              orientation=NS(x=0.,y=0.,z=0.,w=1.)), covariance=[0.]*36))


def report(**kw):
    return json.dumps(dict(accepted=10, rejected=0, sample_age_ms=30., tracking=True) | kw)


def ready():
    gate = health.GateHealth()
    gate.odometry(sample(), 100., 10.)
    gate.report(report(), 10.)
    return gate


def test_requires_both_real_inputs():
    g = health.GateHealth()
    assert not g.healthy(100.,10.)
    g.report(report(), 10.)
    assert not g.healthy(100.,10.)
    g.odometry(sample(), 100.,10.)
    assert g.healthy(100.,10.)


@pytest.mark.parametrize('data', ['broken', 'null', '[]', '{}', report(tracking=False),
    report(quarantined=True), report(landed_bootstrap=False), report(accepted=0),
    report(accepted=True), report(sample_age_ms=-1), report(sample_age_ms=251)])
def test_unhealthy_gate_report(data):
    g = ready()
    g.report(data,10.)
    assert not g.healthy(100.,10.)


def test_frozen_samples_and_gate_reports_expire():
    g = ready()
    g.odometry(sample(),100.2,10.2)
    assert g.sample_received == 10.
    assert not g.healthy(100.26,10.26)
    # Fresh odometry alone does not keep an old gate report or frozen counter alive.
    g.odometry(sample(102.),102.,12.)
    g.report(report(),12.)
    assert not g.healthy(102.,12.)
    g.report(report(accepted=11),12.)
    assert g.healthy(102.,12.)


@pytest.mark.parametrize('bad', ['frame','quaternion','covariance','nan','future'])
def test_invalid_validated_odometry_is_not_health(bad):
    g = ready()
    m = sample(100.01)
    if bad == 'frame': m.header.frame_id='map'
    if bad == 'quaternion': m.pose.pose.orientation.w=0
    if bad == 'covariance':
        for i in (0,7,14,21,28,35): m.pose.covariance[i]=1.
    if bad == 'nan': m.pose.pose.position.x=float('nan')
    if bad == 'future': m=sample(100.2)
    g.odometry(m,100.01,10.01)
    assert not g.healthy(100.01,10.01)


def test_clock_or_gate_restart_requires_new_sample():
    g = ready()
    assert not g.healthy(99.,10.1)
    g = ready()
    g.report(report(accepted=1),10.1)
    assert not g.healthy(100.,10.1)
