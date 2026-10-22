"""Separately authorized P765 unarmed capture; no PX4 input publishers."""
import os

from launch import LaunchDescription
from launch.actions import ExecuteProcess, SetEnvironmentVariable
from launch_ros.actions import Node


def generate_launch_description():
    workspace = os.environ['UAV_USV_WS']
    profile = os.environ['UAV_USV_EXPERIMENT_CONFIG_FILE']
    from uav_control.common.p76_contract import load_contract
    c = load_contract(os.environ['UAV_USV_P76_CONTRACT'])
    return LaunchDescription([
        SetEnvironmentVariable('OPENBLAS_NUM_THREADS', '1'),
        SetEnvironmentVariable('OMP_NUM_THREADS', '1'),
        Node(package='uav_control', executable='moving_target', name='moving_target',
             output='screen', parameters=[profile]),
        Node(package='ros_gz_image', executable='image_bridge', name='p765_rgbd_bridge',
             output='screen',
             arguments=['/uav/camera/front/image', '/uav/camera/front/depth_image'],
             remappings=[('/uav/camera/front/image', '/camera/front/image_raw'),
                         ('/uav/camera/front/depth_image', '/camera/front/depth/image_raw')]),
        Node(package='uav_control', executable='rgbd_target_localizer',
             name='rgbd_target_localizer', output='screen',
             parameters=[profile, {'sphere_fit_diagnostics_enabled': True}]),
        Node(package='uav_control', executable='yolo_target_detector', name='yolo_target_detector',
             output='screen', parameters=[profile], prefix=c['detector']['runtime_python'],
             additional_env={'PROTOCOL_BUFFERS_PYTHON_IMPLEMENTATION': 'python'}),
        ExecuteProcess(cmd=['python3', workspace+'/scripts/p765_resource_monitor.py',
                            '--output', os.path.join(os.environ['UAV_USV_RESEARCH_DIRECTORY'],
                                                     'process_resources.jsonl'),
                            '--duration', '35'], output='screen'),
        ExecuteProcess(cmd=['python3', workspace+'/scripts/p765_roi_capture.py',
                            '--output', os.path.join(os.environ['UAV_USV_RESEARCH_DIRECTORY'], 'roi'),
                            '--duration', '30', '--max-samples', '60', '--max-per-stratum', '10',
                            '--max-bytes', '67108864', '--capacity', '24'], output='screen'),
        ExecuteProcess(cmd=['python3', workspace+'/scripts/p764_diagnostic_collect.py',
                            '--output', os.environ['UAV_USV_RESEARCH_DIRECTORY'],
                            '--duration', '30', '--ros-args', '--params-file', profile],
                       output='screen'),
    ])
