"""P7 truth must stay in one NED frame, time bracket and reset epoch."""


import pytest


def load_script():
    from uav_control.analysis import frame_reference
    return frame_reference


def sample(stamp=10.0, target_enu=(2.0, 6.0, 0.0),
           model_enu=(1.0, 3.0, 0.0), px4=(3.1, 1.2, -0.1), gap=0.01):
    row = {
        'measurement_stamp': str(stamp),
        'gazebo_entity_available': 'True',
        'gazebo_model_available': 'True',
        'truth_available': 'True',
        'px4_position_reset_crossed': 'False',
        'px4_attitude_reset_crossed': 'False',
        'model_px4_roll_residual': '0.0',
        'model_px4_pitch_residual': '0.0',
        'model_px4_yaw_residual': '0.0',
        'target_reference_z_offset': '0.42',
        'px4_xy_reset_counter': '5',
        'px4_z_reset_counter': '2',
        'px4_heading_reset_counter': '1',
        'px4_quat_reset_counter': '1',
        'image_clock_reset_count': '0',
        'entity_clock_reset_count_at_write': '0',
        'px4_clock_reset_count_at_write': '0',
        'truth_velocity_x': '0.0',
        'truth_velocity_y': '4.0',
        'truth_velocity_z': '0.0',
    }
    for name in ('entity', 'model', 'px4_position', 'px4_attitude'):
        row[f'{name}_status'] = 'INTERPOLATED'
        row[f'{name}_left_ros_stamp'] = str(stamp - gap)
        row[f'{name}_right_ros_stamp'] = str(stamp + gap)
        suffix = ('sim_stamp' if name in ('entity', 'model')
                  else 'source_stamp')
        row[f'{name}_left_{suffix}'] = str(stamp - gap)
        row[f'{name}_right_{suffix}'] = str(stamp + gap)
    for axis, value in zip('xyz', target_enu):
        row[f'gazebo_entity_center_{axis}'] = str((
            target_enu[1], target_enu[0], -target_enu[2]
        )['xyz'.index(axis)])
    for axis, value in zip('xyz', model_enu):
        row[f'gazebo_model_world_{axis}'] = str(value)
    for axis, value in zip('xyz', px4):
        row[f'px4_position_independent_{axis}'] = str(value)
    for axis, value in zip('wxyz', (1.0, 0.0, 0.0, 0.0)):
        row[f'px4_attitude_independent_{axis}'] = str(value)
    return row


def test_relative_truth_and_enu_ned_velocity():
    record = load_script().build_record(sample(), max_alignment_gap=0.05)
    assert record['frame_valid']
    assert record['gazebo_target_position_enu'] == [2.0, 6.0, 0.0]
    assert record['target_relative_truth_ned'] == pytest.approx(
        [3.0, 1.0, 0.42])
    assert record['target_truth_px4_ned'] == pytest.approx(
        [6.1, 2.2, 0.32])
    assert record['target_truth_velocity_px4_ned'] == [0.0, 4.0, 0.0]


def test_rejects_large_gap_and_extrapolation():
    module = load_script()
    assert not module.build_record(
        sample(gap=0.06), max_alignment_gap=0.05)['frame_valid']
    row = sample()
    row['entity_status'] = 'AFTER_HISTORY'
    assert not module.build_record(
        row, max_alignment_gap=0.05)['frame_valid']


def test_epoch_changes_on_reset_or_time_reversal():
    module = load_script()
    rows = [sample(10.0), sample(10.1), sample(10.2), sample(10.15)]
    rows[2]['px4_xy_reset_counter'] = '6'
    records = module.convert_rows(rows, max_alignment_gap=0.05)
    assert [row['reset_epoch'] for row in records] == [0, 0, 1, 2]
    assert not records[3]['frame_valid']


def test_pose_brackets_reset_crossing_and_model_attitude():
    module = load_script()
    row = sample()
    row['model_px4_yaw_residual'] = '1.5707963267948966'
    record = module.build_record(row)
    assert record['frame_valid']
    assert record['gazebo_uav_attitude_ned_frd_wxyz'] == pytest.approx(
        [2 ** -0.5, 0.0, 0.0, 2 ** -0.5])
    assert record['source_query']['entity']['left_source_stamp'] == 9.99

    row['px4_position_reset_crossed'] = 'True'
    assert not module.build_record(row)['frame_valid']
    row['px4_position_reset_crossed'] = 'False'
    row['model_left_ros_stamp'] = '10.01'
    assert not module.build_record(row)['frame_valid']
