"""Run the original modular flight stack with an independent research observer."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    """Forward original flight arguments and keep all new outputs in an experiment directory."""
    share = get_package_share_directory('uav_usv_bringup')
    workspace = os.environ.get('UAV_USV_WS', '/home/qin/data/uav_usv_mpc')
    experiment = os.path.join(workspace, 'data', 'experiments', '20261008_p2_mpc', 'f1')
    defaults = {
        'config_file': os.path.join(share, 'config', 'baseline.yaml'),
        'log_directory': os.path.join(experiment, 'evaluator'),
        'enable_evaluator': 'true',
        'enable_shadow_perception': 'true',
        'enable_down_camera': 'false',
        'front_depth_model': 'ideal',
        'shadow_config_file': os.path.join(share, 'config', 'mpc_seed_shadow.yaml'),
        'shadow_log_directory': os.path.join(experiment, 'shadow'),
    }
    flight_arguments = {key: LaunchConfiguration(key) for key in defaults
                        if not key.startswith('shadow_')}
    return LaunchDescription([
        *(DeclareLaunchArgument(key, default_value=value) for key, value in defaults.items()),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(share, 'launch',
                                                       'modular_intercept.launch.py')),
            launch_arguments=flight_arguments.items()),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(share, 'launch',
                                                       'mpc_seed_shadow.launch.py')),
            launch_arguments={
                'config_file': LaunchConfiguration('shadow_config_file'),
                'log_directory': LaunchConfiguration('shadow_log_directory'),
            }.items()),
    ])
