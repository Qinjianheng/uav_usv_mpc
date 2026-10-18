from pathlib import Path


LAUNCH_FILE = (
    Path(__file__).parents[2]
    / 'uav_usv_bringup'
    / 'launch'
    / 'modular_intercept.launch.py'
)
START_SCRIPT = Path(__file__).parents[3] / 'scripts' / 'start_px4_ros2.sh'


def test_modular_launch_contains_five_pipeline_processes():
    source = LAUNCH_FILE.read_text(encoding='utf-8')

    for node_name in (
        'imm_target_predictor_node',
        'intercept_planner_node',
        'trajectory_tracker_node',
        'mission_manager_node',
        'intercept_evaluator_node',
    ):
        assert f"executable='{node_name}'" in source
        assert source.count(f"name='{node_name}'") == 1

    assert source.count("executable='imm_target_predictor_node'") == 1
    assert source.count("executable='target_predictor_node'") == 1
    assert "name='shadow_target_predictor_node'" not in source


def test_modular_launch_has_only_new_tracker_as_px4_control_owner():
    source = LAUNCH_FILE.read_text(encoding='utf-8')

    assert "executable='trajectory_impact_sim'" not in source
    assert "executable='predictive_intercept'" not in source
    assert "executable='pure_pursuit'" not in source


def test_lab_script_defaults_to_modular_launch_with_legacy_override():
    source = START_SCRIPT.read_text(encoding='utf-8')

    assert 'UAV_USV_EXPERIMENT_LAUNCH' in source
    assert 'modular_intercept.launch.py' in source
