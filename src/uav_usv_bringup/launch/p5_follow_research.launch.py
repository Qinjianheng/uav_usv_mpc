"""Narrow P5 research entry; historical engines remain available through old launch files."""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    share = get_package_share_directory('uav_usv_bringup')
    experiment = os.environ.get('UAV_USV_RESEARCH_DIRECTORY', '/tmp/p5_shadow')
    defaults = dict(config_file=os.path.join(share, 'config/baseline.yaml'),
                    log_directory=os.path.join(experiment, 'evaluator'),
                    enable_planner='true', enable_execution='false')
    return LaunchDescription([
        DeclareLaunchArgument('engine', default_value='p47_follow_guided',
                              choices=['p47_follow_guided', 'p44_adaptive']),
        *(DeclareLaunchArgument(k, default_value=v) for k, v in defaults.items()),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(os.path.join(
            share, 'launch/modular_intercept.launch.py')), launch_arguments={
                'config_file': LaunchConfiguration('config_file'),
                'log_directory': LaunchConfiguration('log_directory'),
                'enable_follow_planner': 'false',
                'enable_shadow_perception': 'false',
                'enable_down_camera': 'false',
                'enable_follow_minco': LaunchConfiguration('enable_execution')}.items()),
        Node(package='uav_control', executable='p4_follow_planner_node',
             name='p4_follow_planner_node', output='screen',
             condition=IfCondition(LaunchConfiguration('enable_planner')),
             parameters=[os.path.join(share, 'config/p5_follow_research.yaml'), dict(
                 minco_engine=LaunchConfiguration('engine'),
                 follow_execution_enabled=ParameterValue(
                     LaunchConfiguration('enable_execution'), value_type=bool),
                 log_directory=os.path.join(experiment, 'shadow'))]),
    ])
