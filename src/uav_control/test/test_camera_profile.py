"""Camera profiles must disable rendering without changing flight geometry."""

import ast
import importlib.util
import os
from pathlib import Path
import xml.etree.ElementTree as ET

import pytest


WORKSPACE = Path(__file__).resolve().parents[3]
MODEL = WORKSPACE / 'src/uav_usv_bringup/models/x500_mono_cam'
PREPARER = WORKSPACE / 'scripts/prepare_gz_camera_model.py'
LAUNCH = WORKSPACE / 'src/uav_usv_bringup/launch/modular_intercept.launch.py'


@pytest.fixture(scope='module', autouse=True)
def launch_log_directory(tmp_path_factory):
    """ROS launch checks must not write to the user's global ROS directory."""
    with pytest.MonkeyPatch.context() as patch:
        patch.setenv('ROS_LOG_DIR', os.environ.get(
            'ROS_LOG_DIR', str(tmp_path_factory.mktemp('camera_launch_logs')),
        ))
        yield


def prepare_model(source, output, enabled):
    assert PREPARER.is_file(), 'camera profile preparation is missing'
    spec = importlib.util.spec_from_file_location('camera_preparer', PREPARER)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    module.prepare(source, output, enable_down_camera=enabled)


def test_disabled_camera_removes_only_sensor_preserving_body_and_front(tmp_path):
    before = (MODEL / 'model.sdf').read_bytes()
    output = tmp_path / 'x500_mono_cam'
    prepare_model(MODEL, output, False)
    expected = ET.fromstring(before)
    down = expected.find(".//link[@name='down_camera_link']")
    down.remove(down.find("sensor[@name='down_tof_camera']"))
    actual = ET.parse(output / 'model.sdf').getroot()
    assert ET.tostring(actual) == ET.tostring(expected)
    assert actual.find(".//sensor[@name='front_tof_camera']") is not None
    assert actual.find(".//sensor[@name='down_tof_camera']") is None
    assert (MODEL / 'model.sdf').read_bytes() == before


def test_enabling_camera_restores_original_model_bytes(tmp_path):
    output = tmp_path / 'x500_mono_cam'
    prepare_model(MODEL, output, True)
    for name in ('model.sdf', 'model.config'):
        assert (output / name).read_bytes() == (MODEL / name).read_bytes()


@pytest.mark.parametrize('relative', ['.', 'generated'])
def test_profile_cannot_overwrite_or_generate_inside_source(relative):
    with pytest.raises(ValueError, match='separate'):
        prepare_model(MODEL, MODEL / relative, False)


@pytest.mark.parametrize('shadow', ['true', 'false'])
@pytest.mark.parametrize('down', ['true', 'false'])
def test_camera_launch_conditions_preserve_front_and_gate_down(shadow, down, tmp_path):
    from launch import LaunchContext
    from launch.conditions import IfCondition
    from launch.substitutions import LaunchConfiguration, PythonExpression

    tree = ast.parse(LAUNCH.read_text())
    nodes = {}
    declarations = {}
    for call in ast.walk(tree):
        if not isinstance(call, ast.Call) or not isinstance(call.func, ast.Name):
            continue
        kwargs = {item.arg: item.value for item in call.keywords}
        if call.func.id == 'Node':
            nodes[kwargs['name'].value] = kwargs
        elif call.func.id == 'DeclareLaunchArgument':
            declarations[call.args[0].value] = kwargs
    assert 'enable_down_camera' in declarations, 'down camera toggle is missing'
    assert declarations['enable_down_camera']['default_value'].value == 'false'
    front = nodes['dual_tof_image_bridge']
    assert 'condition' not in front
    front_topics = ast.literal_eval(front['arguments'])
    assert front_topics == [
        '/uav/camera/front/image', '/uav/camera/front/depth_image',
    ]
    assert 'condition' not in nodes['rgbd_target_localizer']
    assert 'condition' not in nodes['target_bearing_node']
    context = LaunchContext()
    context.launch_configurations.update(
        enable_shadow_perception=shadow, enable_down_camera=down,
    )
    namespace = {
        'IfCondition': IfCondition, 'PythonExpression': PythonExpression,
        'enable_shadow_perception': LaunchConfiguration('enable_shadow_perception'),
        'enable_down_camera': LaunchConfiguration('enable_down_camera'),
    }
    for name, expected in (
        ('down_camera_image_bridge', down == 'true'),
        ('down_tof_monitor', shadow == 'true' and down == 'true'),
    ):
        expression = ast.Expression(nodes[name]['condition'])
        condition = eval(compile(expression, str(LAUNCH), 'eval'), namespace)
        assert condition.evaluate(context) == expected


@pytest.mark.parametrize('mode', ['tof', 'ideal'])
def test_depth_model_has_single_output_owner_and_explicit_ideal_fallback(mode, tmp_path):
    from launch import LaunchContext
    from launch.conditions import IfCondition
    from launch.substitutions import LaunchConfiguration, PythonExpression
    context = LaunchContext()
    context.launch_configurations['front_depth_model'] = mode
    calls = [call for call in ast.walk(ast.parse(LAUNCH.read_text()))
             if isinstance(call, ast.Call) and isinstance(call.func, ast.Name)
             and call.func.id == 'Node']
    nodes = [{k.arg: k.value for k in call.keywords} for call in calls]
    names = {kwargs['name'].value: kwargs for kwargs in nodes}
    scope = {'front_depth_model': LaunchConfiguration('front_depth_model'),
             'PythonExpression': PythonExpression, 'IfCondition': IfCondition}
    remap = names['dual_tof_image_bridge']['remappings'].elts[1].elts[1]
    destination = eval(compile(ast.Expression(remap), str(LAUNCH), 'eval'), scope)
    assert context.perform_substitution(destination) == (
        '/camera/front/depth/ideal' if mode == 'tof' else '/camera/front/depth/image_raw'
    )
    expression = ast.Expression(names['front_tof_depth_model']['condition'])
    condition = eval(compile(expression, str(LAUNCH), 'eval'), scope)
    assert condition.evaluate(context) == (mode == 'tof')
