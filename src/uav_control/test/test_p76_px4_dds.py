"""PX4 feedback patch preserves prior settings and never changes command inputs."""
import importlib.util
from pathlib import Path

import pytest
import yaml


def test_dds_patch_is_idempotent_and_preserves_native_inputs():
    path = Path(__file__).resolve().parents[3]/'scripts/p76_px4_dds.py'
    spec = importlib.util.spec_from_file_location('p76_dds', path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    text = ('# keep user comment\npublications:\n  - topic: /fmu/out/vehicle_odometry\n'
            '    type: px4_msgs::msg::VehicleOdometry\n    rate_limit: 100.\n'
            'subscriptions:\n  - topic: /fmu/in/vehicle_rates_setpoint\n'
            '    type: px4_msgs::msg::VehicleRatesSetpoint\n')
    result = module.augment(text)
    assert '# keep user comment' in result
    assert module.augment(result) == result
    before, after = yaml.safe_load(text), yaml.safe_load(result)
    assert after['subscriptions'] == before['subscriptions']
    publications = {entry['topic']: entry for entry in after['publications']}
    assert publications['/fmu/out/vehicle_odometry']['rate_limit'] == 100.
    for topic, (kind, rate) in module.REQUIRED.items():
        assert publications[topic] == dict(topic=topic, type='px4_msgs::msg::'+kind,
                                           rate_limit=rate)
    with pytest.raises(ValueError, match='CONFLICT'):
        module.augment(result.replace('rate_limit: 200.', 'rate_limit: 1.'))
