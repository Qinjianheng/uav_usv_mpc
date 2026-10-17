"""Fresh P5.2 protocol regressions; no ROS nodes or simulation are started."""
from dataclasses import replace
from types import SimpleNamespace

import pytest

from test_p51_follow_admission import planner_fixture
from test_follow_sitl_execution import candidate, ctx
from uav_control.guidance.follow_admission import AdmissionConfig, FollowAdmission, follow_settling


def common(error=.6, relative=.3, height=.06):
    nav = [dict(valid=True, stamp=100., p=[error, 0., -5.+height], v=[1.+relative, 0., 0.])]
    kf = [dict(valid=True, stamp=100., source=100., p=[5., 0., 0.], v=[1., 0., 0.])]
    return nav, kf


@pytest.mark.parametrize('config,expected', [
    (AdmissionConfig(horizontal_enter=.5, relative_enter=.2, height_enter=.05), False),
    (AdmissionConfig(), True),
])
def test_shared_settling_uses_the_configured_entry_thresholds(config, expected):
    nav, kf = common()
    assert follow_settling(nav, kf, 100.01, admission_config=config)['settled'] == expected


@pytest.mark.parametrize('patch', [
    {'valid': False}, {'source': 100.005}, {'p': [float('nan'), 0., 0.]},
])
def test_invalid_latest_kf_cannot_silently_reuse_older_healthy_history(patch):
    nav, kf = common()
    kf.append(dict(kf[0], **patch))
    assert not follow_settling(nav, kf, 100.01)['settled']


def test_stationary_convention_and_direction_reversal_do_not_fake_stability():
    nav, kf = common(0., 0., 0.)
    nav[0]['v'] = kf[0]['v'] = [0., 0., 0.]
    r = follow_settling(nav, kf, 100.01)
    assert r['settled'] and r['desired_position'] == [0., 0., -5.]
    kf[0]['v'] = nav[0]['v'] = [-1., 0., 0.]
    r = follow_settling(nav, kf, 100.01)
    assert r['desired_position'] == [10., 0., -5.] and not r['settled']


def tracker_fixture():
    from uav_control.control.trajectory_tracker_node import TrajectoryTrackerNode
    from uav_control.guidance.follow_minco_execution import NominalFollowReceiver
    n = object.__new__(TrajectoryTrackerNode)
    n.follow_receiver = NominalFollowReceiver(admission=FollowAdmission())
    n.follow_receiver.active, n.follow_receiver.deadline = candidate(), 100.49
    n.follow_admission_identity, n.follow_minco_requested = ('receiver', 0, 1), True
    n.follow_navigation_history, n.follow_target_history = common(0., 0., 0.)
    n.flight_guidance = SimpleNamespace(follow_distance=5., flight_altitude=-5.)
    n.vehicle_status_stamp, n.vehicle_status_timeout = 100., .75
    n.follow_ack_pub = SimpleNamespace(publish=lambda _: None)
    n._follow_context = lambda: ctx(100.06)
    return n


@pytest.mark.parametrize('changed', [dict(generation=1), dict(mission_id=2),
                                     dict(receiver_boot_id='new-boot')])
def test_identity_change_revokes_old_execution_in_the_observing_callback(changed):
    n = tracker_fixture()
    context = replace(ctx(100.06), **changed)
    n._follow_context = lambda: context
    n._update_follow_admission(context)
    assert n.follow_receiver.active is None
    assert n.follow_receiver.admission.state == 'FOLLOW_APPROACH'


def heartbeat(n, boot, generation, mission, epoch):
    from uav_usv_interfaces.msg import FollowReceiverState
    from uav_control.controllers.follow_transport import stamp
    m = FollowReceiverState()
    m.receiver, m.receiver_boot_id = 'trajectory_tracker_node', boot
    m.clock_generation, m.mission_id, m.stamp = generation, mission, stamp(epoch)
    n.epoch_callback(m)


def test_retired_boot_and_mission_generation_cannot_return_after_new_or_invalid_heartbeat():
    n = planner_fixture()
    heartbeat(n, 'receiver', 0, 1, 100.05)
    heartbeat(n, 'new-boot', 0, 2, 100.1)
    heartbeat(n, 'receiver', 0, 1, 100.11)
    assert n.receiver_epoch[0][0] == 'new-boot'
    heartbeat(n, 'new-boot', 1, 2, 100.115)
    heartbeat(n, 'new-boot', 0, 2, 101.)  # rejected future sample must not erase high-water
    heartbeat(n, 'new-boot', 0, 2, 100.119)
    assert n.receiver_epoch is None or n.receiver_epoch[0][1] == 1


def test_entry_limit_has_an_authoritative_reason():
    from uav_control.guidance.follow_minco_execution import NominalFollowReceiver
    gate = FollowAdmission()
    gate.entries = gate.config.maximum_entries
    receiver = NominalFollowReceiver(admission=gate)
    ack = receiver.propose(candidate(), ctx())
    assert 'FOLLOW_ENTRY_LIMIT_REACHED' in ack.reasons


@pytest.mark.parametrize('active', [False, True])
def test_explicit_kf_loss_between_control_ticks_resets_stability_or_revokes(active):
    from uav_usv_interfaces.msg import TargetState
    n = tracker_fixture()
    gate = n.follow_receiver.admission
    if not active:
        n.follow_receiver.active = None
        gate.stable_since = 95.
    n.expected_frame_id, n.maximum_state_age = 'local_ned', .125
    n.latest_kf_message, n.latest_target_state = None, None
    n._ros_seconds = lambda: 100.06
    n.target_state_callback(TargetState())
    assert gate.stable_since is None
    assert n.follow_receiver.active is None
    if active:
        assert gate.loss['reason'] == 'EXECUTION_CONTEXT_INVALID'


def test_authoritative_entry_limit_stops_redundant_shadow_probes():
    n = planner_fixture()
    n.follow_admission_required = True
    n.receiver_status = dict(stamp=100.1, state='FOLLOW_STABLE', active=0, pending=0,
                             allowed=False, inhibition='FOLLOW_ENTRY_LIMIT_REACHED')
    assert n._execution_publication_policy(100.12) == 'FOLLOW_ENTRY_LIMIT_REACHED'


def test_health_monitor_loads_the_same_geometry_and_entry_parameters(tmp_path):
    import sys
    from pathlib import Path
    import yaml
    sys.path.insert(0, str(Path(__file__).resolve().parents[3]/'scripts'))
    from p5_live_health import geometry_config
    (tmp_path/'config').mkdir()
    params = dict(follow_distance=7., flight_altitude=-6., follow_admission_horizontal_enter=.4,
                  follow_admission_relative_enter=.2, follow_admission_height_enter=.04)
    (tmp_path/'config/flight.yaml').write_text(yaml.safe_dump({
        'trajectory_tracker_node': {'ros__parameters': params}}))
    distance, altitude, config = geometry_config(tmp_path)
    assert (distance, altitude) == (7., -6.)
    assert (config.horizontal_enter, config.relative_enter, config.height_enter) == (.4, .2, .04)


def test_parent_prefix_loss_cancels_its_future_child_and_enters_recovery_immediately():
    from uav_control.guidance.follow_minco_execution import NominalFollowReceiver
    r = NominalFollowReceiver()
    old = candidate()
    r.propose(old, ctx())
    r.tick(ctx(100.04), True)
    child = replace(candidate(2, 100.15, 1), xyz=((.11, 0., -5.), *old.xyz[1:]))
    assert r.propose(child, ctx(100.06)).state == 'ACCEPTED'
    r.admission = FollowAdmission()
    r.tick(ctx(100.08), {1: False, 2: True})
    assert r.active is None and r.pending is None
    assert r.admission.state == 'FOLLOW_RECOVERY'
    assert r.admission.loss['plan_id'] == 1
