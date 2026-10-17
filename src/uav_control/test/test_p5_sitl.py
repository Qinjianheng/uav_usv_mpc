"""P5 cannot extend leases, fake takeover, or proceed to F3 after unhealthy F2."""
from copy import deepcopy
from pathlib import Path
import sys

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[3]/'scripts'))


def health():
    return dict(ros_now=100., written=100.,
                control=dict(receipt=100., status='FOLLOW', locked=True, compute=.001),
                navigation=dict(receipt=100., stamp=99.98, valid=True,
                                p=[0., 0., -5.], v=[4., 0., 0.]),
                vehicle=dict(receipt=100., nav_state=14, armed=2),
                prediction=dict(receipt=100., stamp=99.98, observation=99.98, valid=True),
                attitude=dict(receipt=100., q=[1., 0., 0., 0.]))


def test_nominal_follow_is_safe_but_does_not_count_as_minco_takeover():
    from p5_minco_sitl import safety_reason
    assert safety_reason(health(), 100.) is None


@pytest.mark.parametrize('section,patch,reason', [
    ('navigation', {'p': [0., 0., -3.9]}, 'HEIGHT_ERROR_OVER_1M'),
    ('navigation', {'valid': False}, 'NAVIGATION_INVALID'),
    ('prediction', {'observation': 99.8}, 'PREDICTION_INPUT_STALE'),
    ('prediction', {'valid': False}, 'PREDICTION_INPUT_STALE'),
    ('vehicle', {'nav_state': 0}, 'OFFBOARD_OR_ARMING_LOST'),
    ('navigation', {'v': [9., 0., 0.]}, 'VELOCITY_DIVERGENCE'),
    ('attitude', {'q': [.5, .8660254, 0., 0.]}, 'ATTITUDE_DIVERGENCE'),
    ('control', {'status': 'WAITING_FOR_STATE'}, 'NAVIGATION_INVALID'),
])
def test_experiment_aborts_on_real_health_failure(section, patch, reason):
    from p5_minco_sitl import safety_reason
    data = health()
    data[section].update(patch)
    assert safety_reason(data, 100.) == reason


def test_missing_or_stale_health_never_counts_as_healthy():
    from p5_minco_sitl import safety_reason
    assert safety_reason({}, 100.)
    assert safety_reason(health(), 101.) == 'HEALTH_MONITOR_STALE'


def test_f3_requires_completed_healthy_f2_with_sustained_control():
    from p5_minco_sitl import allow_f3
    good = dict(completed=True, healthy=True, minco_fraction=.9,
                longest_minco_seconds=10., observation_seconds=25.,
                diagnostic_coverage=.99)
    assert allow_f3(good)
    for field, value in [('completed', False), ('healthy', False),
                         ('minco_fraction', .2), ('longest_minco_seconds', .4),
                         ('observation_seconds', 10.), ('diagnostic_coverage', .7)]:
        data = deepcopy(good)
        data[field] = value
        assert not allow_f3(data)


def test_sustained_full_cycle_overrun_stops_without_extending_ttl():
    from p5_minco_sitl import publication_stop_reason
    assert publication_stop_reason([{'whole_cycle_time': .13}]*3) == 'SUSTAINED_PLANNER_OVERRUN'
    assert publication_stop_reason([{'whole_cycle_time': .13}, {'whole_cycle_time': .01},
                                    {'whole_cycle_time': .13}]) is None


def test_task_attempt_claim_cannot_automatically_retry(tmp_path, monkeypatch):
    import hashlib
    import json
    import p5_minco_sitl as sitl
    from test_p5_release import evidence
    formal, audit, task = (tmp_path/name for name in ('formal', 'audit', 'task'))
    formal.mkdir()
    audit.mkdir()
    (formal/'raw.json').write_text('{}\n')
    (formal/'SHA256SUMS').write_text(hashlib.sha256(b'{}\n').hexdigest()+'  raw.json\n')
    source = sitl.WORKSPACE/'scripts/p4_sitl_session.py'
    payload = dict(candidate=dict(sitl.CANDIDATE), evidence=evidence(),
                   source_checks=[dict(path='scripts/p4_sitl_session.py',
                                       current_sha256=sitl.sha(source))],
                   formal_directory=str(formal))
    release = audit/'release.json'
    release.write_text(json.dumps(payload))
    digest = hashlib.sha256(release.read_bytes()).hexdigest()
    (audit/'SHA256SUMS').write_text(digest+'  release.json\n')
    monkeypatch.setattr(sitl, 'ROOT', task)
    monkeypatch.setattr(sitl, 'preflight', lambda: {'isolated': True})
    sitl.claim(task/'f2_run', 'f2', release)
    assert json.loads((task/'attempt_f2.json').read_text())['maximum_attempts'] == 1
    with pytest.raises(FileExistsError):
        sitl.claim(task/'different_name', 'f2', release)
    assert not (task/'different_name').exists()


def test_vehicle_status_500ms_cadence_is_healthy_without_relaxing_prediction_ttl():
    from p5_minco_sitl import safety_reason
    data = health()
    data['vehicle']['receipt'] = 99.5
    assert safety_reason(data, 100.) is None
    data['prediction']['observation'] = 99.8
    assert safety_reason(data, 100.) == 'PREDICTION_INPUT_STALE'


def test_longest_control_span_uses_actual_ownership_distribution():
    from p5_minco_sitl import ownership_metrics
    rows = [dict(stamp=i*.05, status='MINCO_FOLLOW') for i in range(161)]
    metrics = ownership_metrics(rows)
    assert metrics['longest_minco_seconds'] == pytest.approx(8.)
    assert metrics['minco_fraction'] == 1.
    assert metrics['diagnostic_coverage'] == 1.


def test_harness_safety_abort_or_missing_authority_cannot_qualify_f3():
    from p5_minco_sitl import execution_completed
    ended = dict(reason='WATCHDOG', monitor_returncode=0, still_alive=[])
    authority = dict(graph_samples=5, discovered_samples=5,
                     all_discovered_single_tracker=True, conflicts=[])
    assert execution_completed('MINCO_OBSERVATION_COMPLETE', ended, authority, False)
    for change in ({'reason': 'CONTROL_AUTHORITY_CONFLICT'}, {'monitor_returncode': 1},
                   {'still_alive': [{'pid': 1}]}):
        assert not execution_completed('MINCO_OBSERVATION_COMPLETE', ended | change,
                                       authority, False)
    assert not execution_completed('MINCO_OBSERVATION_COMPLETE', ended, authority, True)
    assert not execution_completed('MINCO_OBSERVATION_COMPLETE', ended,
                                   authority | {'conflicts': [100.]}, False)
    assert not execution_completed('MINCO_OBSERVATION_COMPLETE', ended,
                                   authority | {'discovered_samples': 0}, False)


@pytest.mark.parametrize('field,value', [
    ('short_horizon', 1.2), ('minco_engine', 'follow_guided_minco'), ('p47_refinement', True),
    ('maximum_input_age', .2), ('p44_ablation', 'A'), ('maximum_horizontal_speed', 7.),
])
def test_modified_config_cannot_inherit_scoped_h16_permission(field, value):
    import yaml
    from p5_minco_sitl import validate_candidate_config, WORKSPACE
    config = yaml.safe_load((WORKSPACE/'src/uav_usv_bringup/config/p5_follow_research.yaml')
                            .read_text())
    params = config['p4_follow_planner_node']['ros__parameters']
    limits = {'maximum_horizontal_speed': 6.2, 'maximum_input_age': .125}
    validate_candidate_config(config, limits)
    params[field] = value
    with pytest.raises(ValueError, match='P5_CONFIG_SCOPE_MISMATCH'):
        validate_candidate_config(config, limits)


def test_source_modified_after_release_cannot_inherit_permission(tmp_path, monkeypatch):
    import hashlib
    import p5_minco_sitl as sitl
    monkeypatch.setattr(sitl, 'WORKSPACE', tmp_path)
    source = tmp_path/'solver.py'
    source.write_text('original')
    digest = hashlib.sha256(b'original').hexdigest()
    release = {'source_checks': [dict(path='solver.py', current_sha256=digest)]}
    sitl.validate_release_sources(release)
    source.write_text('changed')
    with pytest.raises(ValueError, match='AUDITED_SOURCE_CHANGED'):
        sitl.validate_release_sources(release)


def test_cleanup_kills_resistant_owned_process_and_continues_other_groups():
    import signal
    import subprocess
    import p5_minco_sitl as sitl
    resistant = subprocess.Popen([
        sys.executable, '-u', '-c', 'import signal,time;'
        'signal.signal(signal.SIGINT,signal.SIG_IGN);'
        'signal.signal(signal.SIGTERM,signal.SIG_IGN);print("ready");time.sleep(60)'],
        stdout=subprocess.PIPE, start_new_session=True)
    ordinary = subprocess.Popen([sys.executable, '-c', 'import time;time.sleep(60)'],
                                stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
                                start_new_session=True)
    owned = [(resistant, sitl.identity(resistant.pid)), (ordinary, sitl.identity(ordinary.pid))]
    try:
        assert resistant.stdout.readline() == b'ready\n'
        result = sitl.stop_owned(owned, timeouts=(.02, .02, .5))
        assert result['survivors'] == []
        assert resistant.returncode == -signal.SIGKILL
        assert ordinary.poll() is not None
    finally:
        for process, entry in owned:
            if process.poll() is None and sitl.matches(entry):
                import os
                os.killpg(entry['pgid'], signal.SIGKILL)
                process.wait(timeout=2.)


def test_px4_float32_attitude_can_be_written_as_health_json():
    import json
    import numpy as np
    from px4_msgs.msg import VehicleAttitude
    from p5_live_health import attitude_record
    message = VehicleAttitude()
    message.q = np.array([1., .1, .2, .3], dtype=np.float32)
    encoded = json.dumps(attitude_record(message, 100.), allow_nan=False)
    assert json.loads(encoded)['q'] == pytest.approx([1., .1, .2, .3])


@pytest.mark.parametrize('code,written,now,expected', [
    (1, 100., 100.1, 'HEALTH_MONITOR_EXITED'),
    (None, None, 103., 'HEALTH_MONITOR_NO_FIRST_SAMPLE'),
    (None, 100., 100.3, 'HEALTH_MONITOR_STALLED'),
    (None, None, 101., None),
    (None, 100., 100.1, None),
])
def test_monitor_failure_aborts_even_before_follow(code, written, now, expected):
    from p5_minco_sitl import health_monitor_reason
    assert health_monitor_reason(code, written, now, 100.) == expected


def test_missing_real_ack_or_command_evidence_never_proves_minco_execution():
    from p5_minco_sitl import execution_chain_proven
    report = dict(lifecycle=dict(accepted_unique=3, active_unique=3),
                  native_command=dict(period_seconds=dict(count=100)))
    assert execution_chain_proven(report)
    for field in ('accepted_unique', 'active_unique'):
        bad = deepcopy(report)
        bad['lifecycle'][field] = 0
        assert not execution_chain_proven(bad)
    report['native_command']['period_seconds'] = None
    assert not execution_chain_proven(report)


def test_health_written_during_file_read_is_measured_after_read(tmp_path, monkeypatch):
    import json
    import p5_minco_sitl as sitl
    from pathlib import Path
    path = tmp_path/'health.json'
    path.write_text(json.dumps({'written': 100.}))
    current = [100.]
    original_read = Path.read_text

    def concurrent_read(file, *args, **kwargs):
        current[0] = 100.01
        file.write_text(json.dumps({'written': current[0]}))
        return original_read(file, *args, **kwargs)

    monkeypatch.setattr(Path, 'read_text', concurrent_read)
    monkeypatch.setattr(sitl.time, 'time', lambda: current[0])
    snapshot, observed = sitl.read_health_snapshot(path)
    assert observed >= snapshot['written']
    assert sitl.health_monitor_reason(None, snapshot['written'], observed, 99.) is None


def test_snapshot_fix_keeps_future_and_stale_observers_rejected():
    from p5_minco_sitl import health_monitor_reason
    assert health_monitor_reason(None, 100.001, 100., 99.) == 'HEALTH_MONITOR_STALLED'
    assert health_monitor_reason(None, 99.749, 100., 99.) == 'HEALTH_MONITOR_STALLED'


def test_late_component_cleanup_uses_exact_task_scope_and_preserves_other_session(tmp_path):
    import json
    import os
    import signal
    import subprocess
    import p5_minco_sitl as sitl
    scope = dict(UAV_USV_RESEARCH_DIRECTORY=str(tmp_path), ROS_DOMAIN_ID='48',
                 GZ_PARTITION='p5_late_cleanup_test')
    (tmp_path/'environment.json').write_text(json.dumps(scope))
    command = [sys.executable, '-u', '-c', 'import signal,time;'
               'signal.signal(signal.SIGINT,signal.SIG_IGN);'
               'signal.signal(signal.SIGTERM,signal.SIG_IGN);print("ready");time.sleep(60)']
    owned = subprocess.Popen(command, env=dict(os.environ, **scope),
                             stdout=subprocess.PIPE, start_new_session=True)
    other = subprocess.Popen(command, env=dict(os.environ, **(scope | {'GZ_PARTITION': 'other'})),
                             stdout=subprocess.PIPE, start_new_session=True)
    try:
        assert owned.stdout.readline() == other.stdout.readline() == b'ready\n'
        captured = sitl.scoped_processes(tmp_path)
        assert any(v['pid'] == owned.pid for v in captured)
        assert not any(v['pid'] == other.pid for v in captured)
        result = sitl.stop_late_components(tmp_path, timeouts=(.02, .02, .5))
        owned.wait(timeout=2.)
        assert result['survivors'] == []
        assert owned.returncode == -signal.SIGKILL
        assert other.poll() is None
    finally:
        for process in (owned, other):
            if process.poll() is None:
                process.kill()
            process.wait(timeout=2.)


def test_environment_publication_waits_for_complete_scope(tmp_path):
    import json
    from p5_minco_sitl import read_environment_snapshot
    path = tmp_path/'environment.json'
    assert read_environment_snapshot(tmp_path) is None
    for partial in ('', '{"ROS_DOMAIN_ID":', '{}'):
        path.write_text(partial)
        assert read_environment_snapshot(tmp_path) is None
    scope = dict(UAV_USV_RESEARCH_DIRECTORY=str(tmp_path), ROS_DOMAIN_ID='43',
                 GZ_PARTITION='p4_test')
    path.write_text(json.dumps(scope))
    assert read_environment_snapshot(tmp_path) == scope
    path.write_text(json.dumps(scope | {'UAV_USV_RESEARCH_DIRECTORY': '/another/task'}))
    with pytest.raises(ValueError, match='SCOPE_MISMATCH'):
        read_environment_snapshot(tmp_path)


def test_incomplete_cleanup_scope_cannot_report_clean(tmp_path):
    from p5_minco_sitl import scoped_processes
    (tmp_path/'environment.json').write_text('{')
    with pytest.raises(ValueError, match='SCOPE_UNAVAILABLE'):
        scoped_processes(tmp_path)


def test_follow_warmup_uses_common_epoch_and_rejects_the_observed_catchup():
    from p5_live_health import follow_settling
    navigation = [dict(stamp=99.99, p=[4.98, 0., -5.], v=[2., 0., 0.], valid=True),
                  dict(stamp=100.01, p=[5.02, 0., -5.], v=[2., 0., 0.], valid=True)]
    target = dict(stamp=100., source=99.98, p=[10., 0., 0.], v=[2., 0., 0.], valid=True)
    settled = follow_settling(navigation, [target], 100.02)
    assert settled['settled']
    assert settled['horizontal_error'] == pytest.approx(0.)
    assert settled['relative_speed'] == pytest.approx(0.)
    assert settled['epoch'] == target['stamp']
    assert settled['navigation_bracket'] == [99.99, 100.01]
    for row in navigation:
        row.update(p=[-3.26, 0., -5.], v=[6.47, 0., 0.])
    catchup = follow_settling(navigation, [target], 100.02)
    assert not catchup['settled']
    assert catchup['horizontal_error'] == pytest.approx(8.26)
    assert catchup['relative_speed'] == pytest.approx(4.47)


def test_follow_warmup_never_extrapolates_or_accepts_stale_tracking():
    from p5_live_health import follow_settling
    navigation = [dict(stamp=99.99, p=[5., 0., -5.], v=[2., 0., 0.], valid=True),
                  dict(stamp=100.01, p=[5., 0., -5.], v=[2., 0., 0.], valid=True)]
    target = dict(stamp=100., source=99.98, p=[10., 0., 0.], v=[2., 0., 0.], valid=True)
    assert not follow_settling(navigation, [target | {'stamp': 100.011}], 100.02)['settled']
    assert not follow_settling(navigation, [target | {'source': 99.8}], 100.02)['settled']
    assert not follow_settling(navigation, [], 100.02)['settled']


def test_original_follow_must_settle_continuously_before_starting_planner():
    from p5_minco_sitl import settled_since
    assert settled_since(None, {'settled': True}, 100.) == 100.
    assert settled_since(100., {'settled': True}, 102.) == 100.
    assert settled_since(100., {'settled': False}, 103.) is None
    assert settled_since(100., {}, 103.) is None
