"""Keep the qualified planner/receiver yaw and reference configuration intact."""
from pathlib import Path

import pytest
import yaml

from test_p6_sitl_driver import bundle_inputs as _bundle_inputs


@pytest.fixture
def bundle_inputs(tmp_path, monkeypatch):
    return _bundle_inputs.__wrapped__(tmp_path, monkeypatch)


def inputs(bundle_inputs, tracker_mode='rear_point', planner_mode='rear_point'):
    driver, root, flight_path, research_path = bundle_inputs
    flight = yaml.safe_load(flight_path.read_text())
    research = yaml.safe_load(research_path.read_text())
    tracker = flight['trajectory_tracker_node']['ros__parameters']
    planner = research['p4_follow_planner_node']['ros__parameters']
    tracker.update(follow_reference_mode=tracker_mode, follow_research_yaw_rate=2.5,
                   max_observation_yaw_rate=2.5)
    planner.update(follow_reference_mode=planner_mode, maximum_yaw_rate=2.5)
    flight_path.write_text(yaml.safe_dump(flight))
    research_path.write_text(yaml.safe_dump(research))
    return driver, root, flight_path, research_path, research


def test_new_case_preserves_explicit_qualified_yaw_and_reference_parameters(bundle_inputs):
    driver, root, flight, research, _ = inputs(bundle_inputs)
    case = driver.prepare_case(root/'turn', 'figure_eight', 'B', flight, research, role='baseline')
    p = yaml.safe_load((case/'config/flight.yaml').read_text())
    assert p['trajectory_tracker_node']['ros__parameters']['follow_research_yaw_rate'] == 2.5
    assert p['trajectory_tracker_node']['ros__parameters']['follow_reference_mode'] == 'rear_point'


def test_reference_disagreement_cannot_be_qualified_as_a_shared_case(bundle_inputs):
    driver, root, flight, research, _ = inputs(bundle_inputs, tracker_mode='target')
    with pytest.raises(ValueError, match='REFERENCE_MODE_MISMATCH'):
        driver.prepare_case(root/'bad', 'figure_eight', 'B', flight, research, role='baseline')
    assert not (root/'bad').exists()


def test_actual_harness_condition_uses_the_qualified_yaw_not_a_fixed_one(bundle_inputs):
    driver, root, flight, _, research = inputs(bundle_inputs)
    helper = getattr(driver.harness, 'execution_condition', None)
    assert callable(helper), 'The actual harness must derive limits from audited research'
    condition = helper(Path('flight'), 'f3', research)
    assert condition[4:7] == (3., 2.5, 1.6)
    output = root/'prepared_runtime'
    driver.harness.prepare(output, condition, overlay_config=flight)
    loaded = yaml.safe_load((output/'config/flight.yaml').read_text())
    assert loaded['trajectory_tracker_node']['ros__parameters']['follow_research_yaw_rate'] == 2.5
