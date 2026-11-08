"""Pure synthetic collector tests; no ROS, network, flight or live bench."""
import json
from pathlib import Path
import sys
from types import SimpleNamespace as NS

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from tools.navigation_readiness import (ReadinessCollector, archive_comparison,  # noqa: E402
                                        distribution, header_stamp, localizer_observation,
                                        main, odometry_observation, px4_observation, vio_observation)


def add(collector, *, wall=1.0, ros=10.0, stamp=9.98, stream='raw_odometry', **kwargs):
    return collector.observe(stream, wall_s=wall, ros_s=ros, source_stamp_s=stamp, **kwargs)


def test_percentiles_and_unknown_not_zero():
    assert distribution([]) == dict(count=0, max=None, p50=None, p95=None, p99=None)
    s = distribution([0, 1, 2, 3, 4])
    assert s['p50'] == 2 and s['p95'] == pytest.approx(3.8) and s['p99'] == pytest.approx(3.96)


def test_healthy_samples_source_age_and_wall_receipt_are_distinct():
    c = ReadinessCollector(required=['raw_odometry'])
    add(c)
    event = add(c, wall=1.2, ros=10.1, stamp=10.08)
    assert event['source_gap_s'] == pytest.approx(.1)
    assert event['receipt_gap_wall_s'] == pytest.approx(.2)
    assert event['capture_age_ros_s'] == pytest.approx(.02)
    report = c.summary(wall_s=1.21, ros_s=10.11)
    assert report['observation_healthy']
    assert report['streams']['raw_odometry']['accepted'] == 2


@pytest.mark.parametrize('stamp,ros,reason', [(8, 10, 'future/stale'), (10.02, 10, 'future/stale'),
                                           (None, 10, 'source stamp'), (-1, 10, 'source stamp'),
                                           (9.9, -10, 'invalid clock')])
def test_invalid_or_unknown_times_never_become_healthy(stamp, ros, reason):
    c = ReadinessCollector(required=['raw_odometry'])
    event = add(c, stamp=stamp, ros=ros)
    assert not event['accepted'] and any(reason in r for r in event['reasons'])
    assert not c.summary(wall_s=1.1, ros_s=10.1)['observation_healthy']


def test_missing_topics_and_dropped_final_feed_are_not_healthy():
    c = ReadinessCollector()
    add(c)
    report = c.summary(wall_s=2, ros_s=11)
    assert 'validated_odometry' in report['missing_streams']
    assert not report['streams']['raw_odometry']['current']
    assert not report['observation_healthy']


def test_px4_boot_time_is_never_subtracted_from_ros_time():
    c = ReadinessCollector(required=['px4_local_position'])
    for i in range(2):
        event = add(c, stream='px4_local_position', wall=1+i*.1, ros=1_800_000_000+i*.1,
                    stamp=100+i*.1, source_clock='px4_boot')
        assert event['accepted'] and event['capture_age_ros_s'] is None
    report = c.summary(wall_s=1.11, ros_s=1_800_000_000.11)
    assert report['observation_healthy']
    assert report['streams']['px4_local_position']['capture_age_ros_s']['count'] == 0


def test_frozen_source_receipt_gaps_and_reset_counters_are_recorded():
    c = ReadinessCollector(required=['raw_odometry'])
    add(c, values={'xy_reset_counter': 0})
    e = add(c, wall=1.8, ros=10.1, stamp=9.98, values={'xy_reset_counter': 1})
    assert {'source stamp repeated/regressed', 'wall receipt gap exceeded/regressed',
            'reset/frame change: xy_reset_counter'} <= set(e['reasons'])
    report = c.summary(wall_s=1.9, ros_s=10.2)
    assert report['streams']['raw_odometry']['reset_or_frame_changes'] == 1


def test_stopped_or_regressed_ros_clock_invalidates_observation():
    c = ReadinessCollector(required=['raw_odometry'])
    add(c)
    c.observe_clock(2, 10)
    assert 'ROS clock stopped' in c.summary(wall_s=2.1, ros_s=10)['clock_errors']
    c.observe_clock(2.2, 9)
    assert 'ROS clock regressed/reset' in c.clock_errors


def test_capacity_limit_prevents_unbounded_metric_arrays():
    c = ReadinessCollector(required=['raw_odometry'], sample_limit=2)
    for i in range(4):
        add(c, wall=1+i*.1, ros=10+i*.1, stamp=9.98+i*.1)
    report = c.summary(wall_s=1.4, ros_s=10.4)
    assert report['streams']['raw_odometry']['capture_age_ros_s']['count'] == 2
    assert report['streams']['raw_odometry']['rejection_reasons']['sample capacity exceeded'] == 2


def test_extractors_preserve_actual_fields_and_unknown_source_stamps():
    pose = NS(position=NS(x=1., y=2., z=3.), orientation=NS(x=0., y=0., z=0., w=1.))
    msg = NS(header=NS(frame_id='odom', stamp=NS(sec=10, nanosec=0)), pose=NS(pose=pose), child_frame_id='base_link')
    stamp, values, valid, _ = odometry_observation(msg)
    assert stamp == 10 and valid and values['position'] == [1, 2, 3]
    assert header_stamp(NS()) is None
    assert vio_observation(NS(vo_state=1))[0] is None
    assert not vio_observation(NS(vo_state=2))[2]
    payload = NS(data=json.dumps({'status': 'tracking', 'source_stamp_ns': 10_100_000_000,
                                 'odometry_valid': True, 'correction_epoch': 2}))
    assert localizer_observation(payload)[0] == pytest.approx(10.1)
    assert localizer_observation(payload)[2]
    for change in ({'odometry_valid': None}, {'source_epoch_fault': 'regressed'}):
        data = {**json.loads(payload.data), **change}
        assert not localizer_observation(NS(data=json.dumps(data)))[2], change
    # no exact source stamp: unknown, never a substitute field
    legacy = NS(data=json.dumps({'status': 'tracking', 'stamp': 10.1, 'source_stamp_s': 10.1}))
    assert localizer_observation(legacy)[0] is None
    px4 = NS(timestamp=11_000_000, timestamp_sample=10_900_000, x=1., y=2., z=-1., heading=0.,
             xy_valid=True, z_valid=True, heading_good_for_control=True, dead_reckoning=False,
             xy_reset_counter=0, z_reset_counter=0, heading_reset_counter=1)
    assert px4_observation(px4)[2]
    px4.dead_reckoning = True
    assert not px4_observation(px4)[2]
    del px4.heading_good_for_control
    assert 'heading_good_for_control' in px4_observation(px4)[3]


def test_comparison_records_synthetic_bench_name_variant_and_no_flight(tmp_path):
    baseline = tmp_path/'synthetic-baseline'
    baseline.mkdir()
    run = tmp_path/'20990101T000000000001Z-synthetic-collector-unit-fixture'
    run.mkdir()
    (run/'settings.json').write_text(json.dumps({'observer_variant_claim': 'synthetic vio+depth'}))
    (run/'navigation-events.jsonl').write_text('{}\n')
    (run/'navigation-summary.json').write_text('{"observation_healthy":false}')
    report = archive_comparison(run, name='Synthetic collector metadata unit fixture',
                                variant='synthetic vio+depth', baseline=baseline)
    assert report['kind'] == 'bench' and report['baseline_id'] == 'synthetic-baseline'
    assert not report['telemetry']['observed_flight']
    assert 'operator-labelled' in ' '.join(report['notes'])
    assert 'navigation-summary.json' in report['source_sha256']
    assert json.loads((run/'run-comparison.json').read_text())['run_id'] == run.name


@pytest.mark.parametrize('duration', ['0', '-1', '301', 'nan', 'inf'])
def test_wall_duration_is_bounded_before_ros_or_any_run_is_started(duration, tmp_path):
    with pytest.raises(SystemExit) as stop:
        main(['--output-root', str(tmp_path), '--name', 'Synthetic parser unit fixture',
              '--variant', 'synthetic', '--duration', duration])
    assert stop.value.code == 2
    assert not list(tmp_path.iterdir())


def test_unavailable_topic_is_reported_even_with_otherwise_healthy_samples():
    c = ReadinessCollector(required=['raw_odometry'])
    add(c)
    add(c, wall=1.1, ros=10.1, stamp=10.08)
    summary = c.summary(wall_s=1.11, ros_s=10.11, unavailable={'raw_odometry': 'type unavailable'})
    assert not summary['observation_healthy']
    assert summary['unavailable_topics']['raw_odometry'] == 'type unavailable'
