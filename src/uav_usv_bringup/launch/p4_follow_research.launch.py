"""Opt-in P4 shadow/receiver graph beside the unchanged original flight stack."""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """No default control takeover; all actual flight configuration is supplied separately."""
    share = get_package_share_directory('uav_usv_bringup')
    workspace = os.environ.get('UAV_USV_WS', '/home/qin/data/uav_usv_mpc')
    experiment = os.environ.get('UAV_USV_RESEARCH_DIRECTORY', os.path.join(
        workspace, 'data/experiments/p4_shadow'))
    defaults = dict(config_file=os.path.join(share, 'config/baseline.yaml'),
                    log_directory=os.path.join(experiment, 'evaluator'),
                    enable_evaluator='true', enable_shadow_perception='false',
                    enable_down_camera='false', front_depth_model='ideal',
                    enable_planner=os.environ.get('UAV_USV_P4_SHADOW', 'false'),
                    research_config_file=os.environ.get('UAV_USV_RESEARCH_CONFIG_FILE',
                        os.path.join(share, 'config/p4_follow_research.yaml')))
    original = ('config_file', 'log_directory', 'enable_evaluator', 'enable_shadow_perception',
                'enable_down_camera', 'front_depth_model')
    return LaunchDescription([
        *(DeclareLaunchArgument(name, default_value=value) for name, value in defaults.items()),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(os.path.join(
            share, 'launch/modular_intercept.launch.py')),
            launch_arguments={**{k: LaunchConfiguration(k) for k in original},
                              'enable_follow_planner': 'false',
                              'enable_follow_minco': os.environ.get(
                                  'UAV_USV_MINCO_EXECUTE', 'false')}.items()),
        Node(package='uav_control', executable='p4_follow_planner_node',
             name='p4_follow_planner_node', output='screen',
             condition=IfCondition(LaunchConfiguration('enable_planner')),
             parameters=[LaunchConfiguration('research_config_file'),
                         {'log_directory': os.path.join(experiment, 'shadow')}]),
    ])
