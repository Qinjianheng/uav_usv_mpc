"""Diagnostic-only graph: one bounded disarmed probe, no mission or flight controller."""
import os

from launch import LaunchDescription
from launch.actions import ExecuteProcess


def generate_launch_description():
    """The parent task owns the output directory, watchdog, simulator and grant."""
    workspace = os.environ['UAV_USV_WS']
    return LaunchDescription([ExecuteProcess(cmd=[
        'python3', os.path.join(workspace, 'scripts/p76_px4_probe.py'),
        '--output', os.environ['UAV_USV_RESEARCH_DIRECTORY']], output='screen')])
