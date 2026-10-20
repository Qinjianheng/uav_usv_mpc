"""Launch the process-isolated UAV-USV interception pipeline."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, SetEnvironmentVariable
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def audit_p76(context):
    """Reject inconsistent or unqualified P76 before starting any flight node."""
    if LaunchConfiguration('p76_enabled').perform(context) != 'true':
        return []
    import sys
    from pathlib import Path
    workspace = Path(os.environ.get('UAV_USV_WS', '/home/qin/data/uav_usv_mpc'))
    sys.path.insert(0, str(workspace/'scripts'))
    from p76_config_audit import audit
    from uav_control.common.p76_contract import load_contract
    contract = Path(LaunchConfiguration('p76_contract').perform(context))
    profile = Path(LaunchConfiguration('config_file').perform(context))
    values = load_contract(contract)
    if LaunchConfiguration('p76_runtime_python').perform(context) != values['detector'][
            'runtime_python']:
        raise ValueError('P76_DETECTOR_RUNTIME_MISMATCH')
    if LaunchConfiguration('prediction_model').perform(context) != 'bctra':
        raise ValueError('P76_BCTRA_REQUIRED')
    audit(contract, profile, Path(os.environ.get('UAV_USV_P76_MODEL_ROOT', '')), True)
    return []


def generate_launch_description():
    """Return one launch graph with a single PX4 command owner."""
    package_share = get_package_share_directory('uav_usv_bringup')
    default_config = os.path.join(package_share, 'config', 'baseline.yaml')
    workspace = os.environ.get('UAV_USV_WS', '/home/qin/data/uav_usv_mpc')
    default_log_directory = os.path.join(
        workspace,
        'data',
        'experiments',
        'current',
    )
    enable_evaluator = LaunchConfiguration('enable_evaluator')
    config_file = LaunchConfiguration('config_file')
    log_directory = LaunchConfiguration('log_directory')
    enable_shadow_perception = LaunchConfiguration(
        'enable_shadow_perception'
    )
    enable_down_camera = LaunchConfiguration('enable_down_camera')
    front_depth_model = LaunchConfiguration('front_depth_model')
    tof_config_file = LaunchConfiguration('tof_config_file')

    return LaunchDescription([
        # Online KF/MINCO matrices are small. A BLAS worker pool was consuming
        # several cores and starving image delivery despite ~3 ms localization.
        # Set limits before child processes import their numerical libraries.
        SetEnvironmentVariable('OPENBLAS_NUM_THREADS', '1'),
        SetEnvironmentVariable('OMP_NUM_THREADS', '1'),
        SetEnvironmentVariable('MKL_NUM_THREADS', '1'),
        SetEnvironmentVariable('BLIS_NUM_THREADS', '1'),
        DeclareLaunchArgument(
            'config_file',
            default_value=default_config,
            description='Shared modular interception parameter YAML.',
        ),
        DeclareLaunchArgument('enable_follow_minco', default_value='true',
                              choices=['true', 'false']),
        DeclareLaunchArgument('p76_enabled', default_value='false', choices=['true', 'false']),
        DeclareLaunchArgument('p76_contract', default_value=os.path.join(
            package_share, 'config/p76_contract.yaml')),
        DeclareLaunchArgument('p76_runtime_python', default_value=os.path.join(
            workspace, 'data/experiments/20261020_p76_runtime_01/venv/bin/python')),
        DeclareLaunchArgument('enable_follow_planner', default_value='true',
                              choices=['true', 'false']),
        DeclareLaunchArgument('enable_intercept_planner', default_value='true',
                              choices=['true', 'false'],
                              description='Legacy terminal planner; unused by P7 IBVS/PNG.'),
        DeclareLaunchArgument('prediction_model', default_value='imm', choices=['bctra', 'imm']),
        DeclareLaunchArgument(
            'log_directory',
            default_value=default_log_directory,
            description='Directory for evaluator run artifacts.',
        ),
        DeclareLaunchArgument(
            'enable_shadow_perception',
            default_value='false',
            description=(
                'Run optional shadow prediction and image diagnostics.'
            ),
        ),
        DeclareLaunchArgument(
            'enable_evaluator', default_value='true',
            description='Record evaluation-only truth; never affects control.',
        ),
        DeclareLaunchArgument(
            'enable_down_camera', default_value='false',
            choices=['true', 'false'],
            description='Bridge and diagnose an explicitly enabled down sensor.',
        ),
        DeclareLaunchArgument(
            'front_depth_model', default_value='ideal', choices=['tof', 'ideal'],
            description='Functional uncalibrated ToF model, or ideal baseline depth.',
        ),
        DeclareLaunchArgument(
            'tof_config_file',
            default_value=os.path.join(package_share, 'config', 'front_tof_simulation.yaml'),
            description='Explicit functional ToF range/noise test assumptions.',
        ),
        OpaqueFunction(function=audit_p76),
        Node(package='uav_control', executable='p4_follow_planner_node',
             name='p4_follow_planner_node', output='screen',
             condition=IfCondition(PythonExpression([
                 "'", LaunchConfiguration('enable_follow_minco'), "' == 'true' and '",
                 LaunchConfiguration('enable_follow_planner'), "' == 'true'",
             ])),
             parameters=[os.path.join(package_share, 'config/follow_minco.yaml'),
                         config_file,
                         {'log_directory': log_directory}]),
        Node(
            package='uav_control',
            executable='moving_target',
            name='moving_target',
            output='screen',
            parameters=[config_file],
        ),
        # Mutually exclusive main publishers; P7 explicitly selects KF/BCTRA.
        Node(package='uav_control', executable='target_predictor_node',
             name='target_predictor_node', output='screen',
             condition=IfCondition(PythonExpression([
                 "'", LaunchConfiguration('prediction_model'), "' == 'bctra'"])),
             parameters=[config_file,
                         {'prediction_topic': '/planning/target_prediction'}]),
        Node(package='uav_control', executable='imm_target_predictor_node',
             name='imm_target_predictor_node', output='screen',
             condition=IfCondition(PythonExpression([
                 "'", LaunchConfiguration('prediction_model'), "' == 'imm'"])),
             parameters=[config_file,
                         {'prediction_topic': '/planning/target_prediction'}]),
        Node(
            package='uav_control',
            executable='intercept_planner_node',
            name='intercept_planner_node',
            output='screen',
            condition=IfCondition(LaunchConfiguration('enable_intercept_planner')),
            parameters=[config_file],
        ),
        Node(
            package='uav_control',
            executable='trajectory_tracker_node',
            name='trajectory_tracker_node',
            output='screen',
            parameters=[config_file, {'follow_minco_enabled': ParameterValue(
                LaunchConfiguration('enable_follow_minco'), value_type=bool)}],
        ),
        Node(
            package='uav_control',
            executable='mission_manager_node',
            name='mission_manager_node',
            output='screen',
            parameters=[config_file],
        ),
        Node(
            package='uav_control',
            executable='intercept_evaluator_node',
            name='intercept_evaluator_node',
            output='screen',
            condition=IfCondition(enable_evaluator),
            parameters=[
                config_file,
                {'log_directory': log_directory},
            ],
        ),
        Node(
            package='uav_control',
            executable='target_kalman_filter',
            name='target_kalman_filter',
            output='screen',
            parameters=[config_file],
            condition=UnlessCondition(LaunchConfiguration('p76_enabled')),
        ),
        Node(package='uav_control', executable='yolo_target_detector',
             name='yolo_target_detector', output='screen', parameters=[config_file],
             prefix=LaunchConfiguration('p76_runtime_python'),
             additional_env={'PROTOCOL_BUFFERS_PYTHON_IMPLEMENTATION': 'python'},
             condition=IfCondition(LaunchConfiguration('p76_enabled'))),
        Node(package='uav_control', executable='spatial_dkf_node',
             name='spatial_dkf_node', output='screen', parameters=[config_file],
             condition=IfCondition(LaunchConfiguration('p76_enabled'))),
        Node(package='uav_control', executable='image_dkf_node',
             name='image_dkf_node', output='screen', parameters=[config_file],
             condition=IfCondition(LaunchConfiguration('p76_enabled'))),
        Node(
            package='ros_gz_image',
            executable='image_bridge',
            name='dual_tof_image_bridge',
            output='screen',
            arguments=[
                '/uav/camera/front/image',
                '/uav/camera/front/depth_image',
            ],
            remappings=[
                ('/uav/camera/front/image', '/camera/front/image_raw'),
                (
                    '/uav/camera/front/depth_image',
                    PythonExpression([
                        "'/camera/front/depth/ideal' if '", front_depth_model,
                        "' == 'tof' else '/camera/front/depth/image_raw'",
                    ]),
                ),
            ],
        ),
        Node(
            package='uav_control', executable='tof_depth_node',
            name='front_tof_depth_model', output='screen',
            parameters=[tof_config_file],
            condition=IfCondition(PythonExpression([
                "'", front_depth_model, "' == 'tof'",
            ])),
        ),
        Node(
            package='ros_gz_image', executable='image_bridge',
            name='down_camera_image_bridge', output='screen',
            condition=IfCondition(enable_down_camera),
            arguments=[
                '/uav/camera/down/image', '/uav/camera/down/depth_image',
            ],
            remappings=[
                ('/uav/camera/down/image', '/camera/down/image_raw'),
                ('/uav/camera/down/depth_image', '/camera/down/depth/image_raw'),
            ],
        ),
        Node(
            package='uav_control',
            executable='target_bearing_node',
            name='target_bearing_node',
            output='screen',
            parameters=[config_file],
        ),
        Node(
            package='uav_control',
            executable='front_tof_monitor',
            name='front_tof_monitor',
            output='screen',
            parameters=[config_file, {'depth_input_ros': PythonExpression([
                "'", front_depth_model, "' == 'tof'",
            ])}],
            condition=IfCondition(PythonExpression([
                "'", enable_shadow_perception, "' == 'true' or '",
                LaunchConfiguration('p76_enabled'), "' == 'true'"])),
        ),
        Node(
            package='uav_control',
            executable='rgbd_target_localizer',
            name='rgbd_target_localizer',
            output='screen',
            parameters=[config_file, tof_config_file, {
                'sphere_fit_mode': front_depth_model,
            }],
        ),
        Node(
            package='uav_control',
            executable='front_tof_monitor',
            name='down_tof_monitor',
            output='screen',
            parameters=[
                config_file,
                {
                    'camera_name': 'down',
                    'diagnostic_prefix': '/perception/down',
                    'camera_frame_id': 'down_camera_optical_frame',
                    'color_gazebo_topic': '/uav/camera/down/image',
                    'depth_gazebo_topic': '/uav/camera/down/depth_image',
                    'color_ros_topic': '/camera/down/image_raw',
                    'depth_ros_topic': '/camera/down/depth/image_raw',
                    'camera_pitch_down': 1.57079632679,
                    'target_visual_height_offset': 0.42,
                    'publish_gazebo_rtf': False,
                },
            ],
            condition=IfCondition(PythonExpression([
                "'", enable_shadow_perception, "' == 'true' and '",
                enable_down_camera, "' == 'true'",
            ])),
        ),
        Node(
            package='uav_control',
            executable='dual_tof_selector',
            name='dual_tof_selector',
            output='screen',
            parameters=[config_file],
            condition=IfCondition(enable_shadow_perception),
        ),
    ])
