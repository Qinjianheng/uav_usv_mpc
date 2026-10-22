"""Independent native-feedback calibration; no mission, Tracker or target estimator."""
import os

from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node


def generate_launch_description():
    """A task-owned isolated runner is the sole PX4 controller for this graph."""
    workspace = os.environ['UAV_USV_WS']
    command = ['python3', os.path.join(workspace, 'scripts/p76_bodyrate_calibration.py'),
               '--output', os.environ['UAV_USV_RESEARCH_DIRECTORY'],
               '--stage', os.environ['UAV_USV_P761_STAGE']]
    if os.environ.get('UAV_USV_P761_PRIOR_A1'):
        command += ['--prior-a1', os.environ['UAV_USV_P761_PRIOR_A1']]
    return LaunchDescription([
        Node(package='uav_control', executable='rgbd_target_localizer',
             name='rgbd_target_localizer', output='screen',
             parameters=[os.environ['UAV_USV_EXPERIMENT_CONFIG_FILE']]),
        ExecuteProcess(cmd=command, output='screen'),
    ])
