#!/usr/bin/env python3
"""Add explicit P76 feedback publications without overwriting unrelated PX4 changes."""
import argparse
import difflib
from pathlib import Path

import yaml

REQUIRED = {
    '/fmu/out/vehicle_angular_velocity': ('VehicleAngularVelocity', 100.),
    '/fmu/out/vehicle_thrust_setpoint': ('VehicleThrustSetpoint', 200.),
    '/fmu/out/vehicle_rates_setpoint': ('VehicleRatesSetpoint', 200.),
    '/fmu/out/actuator_motors': ('ActuatorMotors', 100.),
    '/fmu/out/vehicle_acceleration': ('VehicleAcceleration', 100.),
}


def augment(text):
    """Preserve existing entries/comments; reject conflicting feedback types or rates."""
    values = yaml.safe_load(text)
    publications = {entry['topic']: entry for entry in values['publications']}
    additions = []
    for topic, (kind, rate) in REQUIRED.items():
        entry = publications.get(topic)
        if entry:
            if entry['type'] != 'px4_msgs::msg::'+kind or entry.get('rate_limit', 0) < rate:
                raise ValueError('P76_DDS_CONFLICT:'+topic)
        else:
            additions.append(f'  - topic: {topic}\n    type: px4_msgs::msg::{kind}\n'
                             f'    rate_limit: {rate}\n')
    marker = 'publications:\n'
    if text.count(marker) != 1:
        raise ValueError('P76_DDS_PUBLICATION_SECTION')
    return text.replace(marker, marker+'\n  # P76 SITL body-rate feedback and calibration.\n'
                        + '\n'.join(additions)+'\n', 1) if additions else text


def main():
    """Generate a reproducible patch; application requires the explicit --apply argument."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--px4-root', type=Path, required=True)
    parser.add_argument('--patch', type=Path, required=True)
    parser.add_argument('--apply', action='store_true')
    options = parser.parse_args()
    relative = 'src/modules/uxrce_dds_client/dds_topics.yaml'
    path = options.px4_root/relative
    before = path.read_text()
    after = augment(before)
    patch = ''.join(difflib.unified_diff(
        before.splitlines(True), after.splitlines(True),
        fromfile='a/'+relative, tofile='b/'+relative))
    if patch:
        options.patch.parent.mkdir(parents=True, exist_ok=True)
        options.patch.write_text(patch)
    if options.apply and after != before:
        path.write_text(after)
    print('P76_DDS_APPLIED' if options.apply else 'P76_DDS_PATCH_PREPARED')


if __name__ == '__main__':
    main()
