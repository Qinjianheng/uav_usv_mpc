"""Nominal execution requires a live isolated session, independently of default request."""
import importlib.util
from pathlib import Path

import pytest


def test_ordinary_environment_never_grants_nominal_execution():
    from uav_control.guidance.follow_sitl_gate import require_isolated_sitl
    with pytest.raises(RuntimeError, match='SITL_EXECUTION_NOT_GRANTED'):
        require_isolated_sitl({})


def test_stale_or_mismatched_grant_is_rejected(tmp_path):
    from uav_control.guidance.follow_sitl_gate import require_isolated_sitl
    grant = tmp_path/'grant.json'
    grant.write_text('{"domain": 43, "partition": "other"}')
    with pytest.raises(RuntimeError, match='SITL_EXECUTION_NOT_GRANTED'):
        require_isolated_sitl({'UAV_USV_SITL_GRANT': str(grant),
                               'ROS_DOMAIN_ID': '43', 'GZ_PARTITION': 'task'})


def test_default_launch_resolves_log_directory_before_planner(tmp_path):
    import launch.logging
    launch.logging.launch_config.log_dir = str(tmp_path)
    from launch import LaunchContext
    from launch.actions import DeclareLaunchArgument
    from launch_ros.actions import Node
    path = Path(__file__).resolve().parents[2]/'uav_usv_bringup/launch/modular_intercept.launch.py'
    spec = importlib.util.spec_from_file_location('p45_launch', path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    context = LaunchContext()
    for entity in module.generate_launch_description().entities:
        if isinstance(entity, DeclareLaunchArgument):
            entity.execute(context)
        if isinstance(entity, Node):
            assert 'log_directory' in context.launch_configurations
            break
