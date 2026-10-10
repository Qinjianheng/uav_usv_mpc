"""One selectable research observer, optionally beside the original flight stack."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    """Reuse existing diagnostic switches; keep a single observer outside flight callbacks."""
    share = get_package_share_directory('uav_usv_bringup')
    workspace = os.environ.get('UAV_USV_WS', '/home/qin/data/uav_usv_mpc')
    experiment = os.environ.get('UAV_USV_RESEARCH_DIRECTORY', os.path.join(
        workspace, 'data', 'experiments', '20261008_p3_follow', 'f1'))
    defaults = {
        'config_file': os.path.join(share, 'config', 'baseline.yaml'),
        'log_directory': os.path.join(experiment, 'evaluator'),
        'enable_evaluator': 'true', 'enable_shadow_perception': 'true',
        'enable_down_camera': 'false', 'front_depth_model': 'ideal',
        'start_flight_stack': 'true',
        'lightweight': os.environ.get('UAV_USV_RESEARCH_LIGHTWEIGHT', 'true'),
        'research_mode': os.environ.get('UAV_USV_RESEARCH_MODE', 'greedy_minco'),
        'research_config_file': os.environ.get('UAV_USV_RESEARCH_CONFIG_FILE',
                                               os.path.join(share, 'config',
                                                            'follow_research.yaml')),
        'research_log_directory': os.path.join(experiment, 'shadow'),
    }
    original = ('config_file', 'log_directory', 'enable_evaluator',
                'enable_shadow_perception', 'enable_down_camera', 'front_depth_model')
    arguments = {name: LaunchConfiguration(name) for name in original}
    arguments['enable_shadow_perception'] = PythonExpression([
        "'false' if '", LaunchConfiguration('lightweight'), "'.lower() == 'true' else '",
        LaunchConfiguration('enable_shadow_perception'), "'",
    ])
    return LaunchDescription([
        *(SetEnvironmentVariable(name, '1') for name in (
            'OPENBLAS_NUM_THREADS', 'OMP_NUM_THREADS', 'MKL_NUM_THREADS', 'BLIS_NUM_THREADS')),
        *(DeclareLaunchArgument(name, default_value=value) for name, value in defaults.items()),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(share, 'launch',
                                                       'modular_intercept.launch.py')),
            condition=IfCondition(LaunchConfiguration('start_flight_stack')),
            launch_arguments=arguments.items()),
        Node(package='uav_control', executable='follow_research_shadow_node',
             name='follow_research_shadow_node', output='screen',
             parameters=[LaunchConfiguration('research_config_file'), {
                 'research_mode': LaunchConfiguration('research_mode'),
                 'log_directory': LaunchConfiguration('research_log_directory')}]),
    ])
