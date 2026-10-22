"""B qualification needs independent, causal native physical response evidence."""
from copy import deepcopy
import importlib.util
import json
from pathlib import Path

import pytest
from rclpy.time import Time
from std_msgs.msg import String
from uav_usv_interfaces.msg import NativeAttitude, UavState

ROOT = Path(__file__).resolve().parents[3]


def module():
    spec = importlib.util.spec_from_file_location(
        'p763_b_validation', ROOT/'scripts/p761_validation.py')
    result = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(result)
    return result


def ros_stamp(value):
    return Time(nanoseconds=round(value*1e9)).to_msg()


def physical_fixture():
    gate = module().BPhysicalAcceptance()
    reference = dict(stamp=10.02, position=[5., 0., -4.], velocity=[4., 0., 0.],
                     acceleration=[0., 0., 0.])
    old = dict(stamp=9.99, rates=[.2, 0., 0.], thrust=-.5,
               reference=dict(reference, stamp=9.98))
    new = dict(stamp=10.04, rates=[1.2, 0., 0.], thrust=-.9, reference=reference)
    runtime = dict(schema_version=2, phase='FOLLOW_ACTIVE', guidance_owner='MINCO_FOLLOW',
                   output_history=[old, new])
    navigation = UavState(stamp=ros_stamp(10.02), valid=True, frame_id='local_ned',
                          px4_timestamp_sample=10020000, native_timestamp_sample=10020000)
    navigation.position.x, navigation.position.z, navigation.velocity.x = 5., -4., 4.
    pose = NativeAttitude(stamp=ros_stamp(10.02), valid=True,
                          quaternion_wxyz=[1., 0., 0., 0.], angular_velocity_frd=[.2, 0., 0.],
                          orientation_variance=[1e-6]*3, px4_timestamp_sample=10020000,
                          native_timestamp_sample=10020000)
    collective = dict(stamp=10.025, native_timestamp=10025000, px4_timestamp=10025000,
                      xyz=[0., 0., -.5])
    return gate, runtime, navigation, pose, collective


def feed(gate, runtime, navigation, pose, collective):
    for name, message in (
        ('paper', String(data=json.dumps(runtime))), ('navigation', navigation), ('pose', pose),
        ('native_thrust', String(data=json.dumps(collective))),
    ):
        gate.observe(name, message, 10.05)


def test_native_response_pairs_to_causal_historical_output_not_newest_command():
    gate, runtime, navigation, pose, collective = physical_fixture()
    feed(gate, runtime, navigation, pose, collective)
    passed, evidence = gate.evaluate(10.05)
    assert passed
    assert evidence['rate_command_stamp'] == pytest.approx(9.99)
    assert evidence['collective_command_stamp'] == pytest.approx(9.99)
    assert evidence['navigation_stamp'] == pytest.approx(10.02)
    assert evidence['position_error'] == pytest.approx(0.)
    assert evidence['collective_error'] == pytest.approx(0.)
    assert evidence['rate_error'] == pytest.approx([0., 0., 0.])
    assert gate.metrics()['count'] == 1
    assert gate.metrics()['rate_rmse'] == pytest.approx([0., 0., 0.])


@pytest.mark.parametrize('mutation', [
    'missing_thrust', 'old_thrust', 'future_only_command', 'old_command', 'bad_rate',
    'bad_collective', 'saturated_collective', 'saturated_rate', 'bad_position',
    'bad_velocity', 'bad_acceleration', 'wrong_reference_epoch', 'missing_native_identity',
    'missing_pose', 'old_pose',
])
def test_missing_stale_incorrect_or_saturated_execution_cannot_qualify(mutation):
    gate, runtime, navigation, pose, collective = physical_fixture()
    if mutation == 'missing_thrust':
        collective = None
    elif mutation == 'old_thrust':
        collective['stamp'] = 9.8
    elif mutation == 'future_only_command':
        runtime['output_history'] = runtime['output_history'][1:]
    elif mutation == 'old_command':
        runtime['output_history'][0]['stamp'] = 9.8
    elif mutation == 'bad_rate':
        pose.angular_velocity_frd = [.6, 0., 0.]
    elif mutation == 'bad_collective':
        collective['xyz'][2] = -.65
    elif mutation == 'saturated_collective':
        runtime['output_history'][0]['thrust'] = collective['xyz'][2] = -1.
    elif mutation == 'saturated_rate':
        runtime['output_history'][0]['rates'] = [1.5, 0., 0.]
        pose.angular_velocity_frd = [1.5, 0., 0.]
    elif mutation == 'bad_position':
        navigation.position.x += 1.01
    elif mutation == 'bad_velocity':
        navigation.velocity.x += 1.01
    elif mutation == 'bad_acceleration':
        navigation.acceleration.x = 1.51
    elif mutation == 'wrong_reference_epoch':
        runtime['output_history'][1]['reference']['stamp'] = 10.021
    elif mutation == 'missing_native_identity':
        navigation.native_timestamp_sample = 0
    elif mutation == 'missing_pose':
        pose = None
    elif mutation == 'old_pose':
        pose.stamp = ros_stamp(9.8)
    gate.observe('paper', String(data=json.dumps(runtime)), 10.05)
    gate.observe('navigation', navigation, 10.05)
    if pose is not None:
        gate.observe('pose', pose, 10.05)
    if collective is not None:
        gate.observe('native_thrust', String(data=json.dumps(collective)), 10.05)
    assert not gate.evaluate(10.05)[0]
    assert gate.metrics()['count'] == 0


def test_repeated_native_sample_or_stale_held_data_cannot_accumulate_25_seconds():
    gate, runtime, navigation, pose, collective = physical_fixture()
    feed(gate, runtime, navigation, pose, collective)
    assert gate.evaluate(10.05)[0]
    assert not gate.evaluate(10.10)[0]
    assert gate.metrics()['count'] == 0
    assert not gate.evaluate(10.25)[0]


def test_a_broken_physical_window_discards_prior_passed_samples():
    gate, runtime, navigation, pose, collective = physical_fixture()
    feed(gate, runtime, navigation, pose, collective)
    assert gate.evaluate(10.05)[0]
    pose.stamp = navigation.stamp = ros_stamp(10.07)
    pose.angular_velocity_frd = [.7, 0., 0.]
    pose.native_timestamp_sample = navigation.native_timestamp_sample = 10070000
    later = deepcopy(runtime['output_history'][1])
    later.update(stamp=10.09, reference=dict(later['reference'], stamp=10.07))
    runtime['output_history'].append(later)
    collective.update(stamp=10.075, xyz=[0., 0., -.9])
    for name, message in [('paper', String(data=json.dumps(runtime))),
                          ('navigation', navigation), ('pose', pose),
                          ('native_thrust', String(data=json.dumps(collective)))]:
        gate.observe(name, message, 10.10)
    assert not gate.evaluate(10.10)[0]
    assert gate.metrics()['count'] == 0


def test_graph_requires_sole_named_tracker_and_no_velocity_setpoint_publisher():
    graph = {'/perception/target_detection': ['yolo_target_detector'],
             '/tracking/target_state': ['spatial_dkf_node'],
             '/fmu/in/offboard_control_mode': ['trajectory_tracker_node'],
             '/fmu/in/vehicle_rates_setpoint': ['trajectory_tracker_node'],
             '/fmu/in/trajectory_setpoint': []}
    validation = module()
    assert validation.bodyrate_graph_qualified(graph)
    for topic, names in [('/fmu/in/offboard_control_mode', []),
                         ('/fmu/in/vehicle_rates_setpoint', ['rogue']),
                         ('/fmu/in/trajectory_setpoint', ['trajectory_tracker_node'])]:
        assert not validation.bodyrate_graph_qualified(dict(graph, **{topic: names}))


def test_complete_window_requires_many_distinct_native_samples_and_reports_observed_errors():
    gate = module().BPhysicalAcceptance()
    for index in range(501):
        epoch = 100.+index*.05
        reference = dict(stamp=epoch, position=[5.+index*.2, 0., -4.],
                         velocity=[4., 0., 0.], acceleration=[0., 0., 0.])
        commands = [dict(stamp=epoch-.01, rates=[0., 0., 0.], thrust=-.5,
                         reference=dict(reference, stamp=epoch-.02)),
                    dict(stamp=epoch+.01, rates=[0., 0., 0.], thrust=-.5, reference=reference)]
        gate.observe('paper', String(data=json.dumps(dict(
            schema_version=2, phase='FOLLOW_ACTIVE', guidance_owner='MINCO_FOLLOW',
            output_history=commands))), epoch+.02)
        nav = UavState(stamp=ros_stamp(epoch), valid=True, frame_id='local_ned',
                       px4_timestamp_sample=round(epoch*1e6),
                       native_timestamp_sample=round(epoch*1e6))
        nav.position.x, nav.position.z, nav.velocity.x = 5.+index*.2, -4., 4.
        pose = NativeAttitude(stamp=ros_stamp(epoch), valid=True,
                              quaternion_wxyz=[1., 0., 0., 0.],
                              angular_velocity_frd=[0., 0., 0.], orientation_variance=[1e-6]*3,
                              px4_timestamp_sample=round(epoch*1e6),
                              native_timestamp_sample=round(epoch*1e6))
        gate.observe('navigation', nav, epoch+.02)
        gate.observe('pose', pose, epoch+.02)
        gate.observe('native_thrust', String(data=json.dumps(dict(
            stamp=epoch+.005, native_timestamp=round((epoch+.005)*1e6),
            px4_timestamp=round((epoch+.005)*1e6), xyz=[0., 0., -.5]))), epoch+.02)
        assert gate.evaluate(epoch+.02)[0]
        if index == 449:
            assert not gate.qualified_window()
    assert gate.qualified_window()
    assert gate.metrics()['count'] == 501
    assert gate.metrics()['source_seconds'] == pytest.approx(25.)
    assert gate.metrics()['position_error_max'] == pytest.approx(0.)
    assert gate.metrics()['rate_rmse'] == pytest.approx([0., 0., 0.])
