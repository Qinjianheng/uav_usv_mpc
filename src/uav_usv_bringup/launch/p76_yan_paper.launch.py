"""Explicit, qualification-gated P76 graph; the default P7 entry is preserved."""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    """One shared YOLO, mutually exclusive DKFs and the existing sole Tracker."""
    share = get_package_share_directory('uav_usv_bringup')
    defaults = dict(config_file=os.path.join(share, 'config/p76_yan_paper.yaml'),
                    p76_contract=os.path.join(share, 'config/p76_contract.yaml'),
                    log_directory=os.path.join(os.environ.get(
                        'UAV_USV_WS', '/home/qin/data/uav_usv_mpc'), 'data/experiments/p76_current'))
    return LaunchDescription([
        *(DeclareLaunchArgument(key, default_value=value) for key, value in defaults.items()),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(os.path.join(
            share, 'launch/modular_intercept.launch.py')), launch_arguments=dict(
                **{key: LaunchConfiguration(key) for key in defaults}, p76_enabled='true',
                prediction_model='bctra', enable_follow_minco='true',
                enable_intercept_planner='false').items()),
    ])
