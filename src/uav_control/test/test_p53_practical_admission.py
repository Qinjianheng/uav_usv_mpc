"""A turning 5m viewpoint has a legitimate velocity different from the target center."""
from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[3]/'scripts'))
import p53_minco_sitl as entry  # noqa: E402
from uav_control.guidance.follow_admission import (  # noqa: E402
    AdmissionConfig, FollowAdmission, follow_settling,
)


def test_turning_viewpoint_can_enter_while_original_center_velocity_gate_blocks():
    # Target heading rotates +.3rad/s. d/dt(target - 5*heading) = (4,-1.5).
    nav = [dict(stamp=100., valid=True, p=[-5., 0., -5.], v=[4., -1.5, 0.])]
    kf = [dict(stamp=100., source=99.98, valid=True, p=[0., 0., 0.], v=[4., 0., 0.])]
    assert not follow_settling(nav, kf, 100., admission_config=AdmissionConfig())['settled']
    assert follow_settling(nav, kf, 100., admission_config=entry.flight_admission())['settled']


def test_practical_velocity_gate_retains_stability_probes_and_active_deviation_recovery():
    gate = FollowAdmission(entry.flight_admission())
    metrics = dict(horizontal_error=.25, relative_speed=1.5, height_error=.02)
    for i in range(51):
        gate.update(i*.1, metrics, True)
    assert gate.converged(5.) and not gate.allowed(5.)
    for i in range(3):
        gate.probe(5.+i*.2, ('actual-probe', i), True)
    assert gate.allowed(5.4)
    assert gate.update(5.4, metrics, True, active=True) == ''
    bad = dict(metrics, relative_speed=2.6)
    assert gate.update(5.5, bad, True, active=True) == ''
    for now in (5.6, 5.7, 5.8, 5.9):
        gate.update(now, bad, True, active=True)
    assert gate.update(6.01, bad, True, active=True) == 'PERFORMANCE_DEVIATION'


def test_practical_entry_still_rejects_position_height_or_illegal_context():
    for patch, legal in ((dict(horizontal_error=1.01), True),
                         (dict(height_error=.101), True), ({}, False)):
        gate = FollowAdmission(entry.flight_admission())
        metrics = dict(horizontal_error=.25, relative_speed=1.5, height_error=.02)
        metrics.update(patch)
        for i in range(61):
            gate.update(i*.1, metrics, legal)
        assert not gate.allowed(6.) and not gate.converged(6.)


def test_execution_audit_reports_loaded_admission_instead_of_default(tmp_path):
    import yaml
    from p51_execution_audit import audit
    config = tmp_path/'config'
    config.mkdir()
    (config/'flight.yaml').write_text(yaml.safe_dump(dict(trajectory_tracker_node=dict(
        ros__parameters=dict(follow_admission_relative_enter=1.8,
                             follow_admission_relative_exit=2.5)))))
    report = audit(tmp_path, [], None, 'f2')
    assert report['thresholds']['admission']['relative_enter'] == 1.8
    assert report['thresholds']['admission']['relative_exit'] == 2.5
