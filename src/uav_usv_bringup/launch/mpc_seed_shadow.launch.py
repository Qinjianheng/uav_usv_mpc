"""Launch only the independent research MPC observer beside an existing flight stack."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """Keep flight startup with uav_lab and publish shadow results only in research topics."""
    package_share = get_package_share_directory('uav_usv_bringup')
    workspace = os.environ.get('UAV_USV_WS', '/home/qin/data/uav_usv_mpc')
    return LaunchDescription([
        SetEnvironmentVariable('OPENBLAS_NUM_THREADS', '1'),
        SetEnvironmentVariable('OMP_NUM_THREADS', '1'),
        SetEnvironmentVariable('MKL_NUM_THREADS', '1'),
        SetEnvironmentVariable('BLIS_NUM_THREADS', '1'),
        DeclareLaunchArgument('config_file', default_value=os.path.join(
            package_share, 'config', 'mpc_seed_shadow.yaml')),
        DeclareLaunchArgument('log_directory', default_value=os.path.join(
            workspace, 'data', 'experiments', 'mpc_shadow')),
        Node(package='uav_control', executable='follow_mpc_shadow_node',
             name='follow_mpc_shadow_node', output='screen',
             parameters=[LaunchConfiguration('config_file'),
                         {'log_directory': LaunchConfiguration('log_directory')}]),
    ])
