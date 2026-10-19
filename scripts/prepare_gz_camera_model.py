#!/usr/bin/env python3
"""Create a session camera model without rewriting the source SDF."""

import argparse
import math
from pathlib import Path
import shutil
import xml.etree.ElementTree as ET

import yaml


def prepare(source, output, *, enable_down_camera=False, front_camera_pitch_down=None):
    """Keep body geometry and front optics; optionally omit the down sensor."""
    source, output = Path(source).resolve(), Path(output).resolve()
    if output == source or source in output.parents:
        raise ValueError('output must be a separate model copy')
    tree = ET.parse(source / 'model.sdf')
    down = tree.find(".//link[@name='down_camera_link']")
    sensor = None if down is None else down.find(
        "sensor[@name='down_tof_camera']",
    )
    if sensor is None or tree.find(
        ".//sensor[@name='front_tof_camera']",
    ) is None:
        raise ValueError('expected the original front and down sensors')
    if front_camera_pitch_down is not None:
        if (not math.isfinite(front_camera_pitch_down)
                or not 0 < front_camera_pitch_down < math.pi/2):
            raise ValueError('invalid front camera pitch')
        pose = tree.find(".//link[@name='front_camera_link']/pose")
        values = pose.text.split()
        values[4] = repr(float(front_camera_pitch_down))
        pose.text = ' '.join(values)
    shutil.copytree(source, output)
    if not enable_down_camera:
        down.remove(sensor)
    if not enable_down_camera or front_camera_pitch_down is not None:
        tree.write(output / 'model.sdf', encoding='utf-8', xml_declaration=True)


def main():
    """Prepare only a new model directory, with no simulator side effects."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--source', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--enable-down-camera', choices=('true', 'false'),
                        default='false')
    parser.add_argument('--flight-config', type=Path)
    args = parser.parse_args()
    pitch = None
    if args.flight_config is not None:
        config = yaml.safe_load(args.flight_config.read_text())
        pitch = config['rgbd_target_localizer']['ros__parameters']['camera_pitch_down']
    prepare(args.source, args.output,
            enable_down_camera=args.enable_down_camera == 'true', front_camera_pitch_down=pitch)


if __name__ == '__main__':
    main()
