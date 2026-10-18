"""P7 KF/BCTRA and R11 MINCO FOLLOW, with one Tracker and one main predictor."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    share = get_package_share_directory('uav_usv_bringup')
    workspace = os.environ.get('UAV_USV_WS', '/home/qin/data/uav_usv_mpc')
    defaults = dict(
        config_file=os.path.join(share, 'config/p7_guidance.yaml'),
        log_directory=os.path.join(workspace, 'data/experiments/p7_manual'),
        prediction_model='bctra', enable_follow_minco='true', enable_follow_planner='true',
        enable_intercept_planner='false',
        enable_evaluator='true', enable_shadow_perception='false')
    return LaunchDescription([
        *(DeclareLaunchArgument(key, default_value=value) for key, value in defaults.items()),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(os.path.join(
            share, 'launch/modular_intercept.launch.py')),
            launch_arguments={key: LaunchConfiguration(key) for key in defaults}.items()),
    ])
