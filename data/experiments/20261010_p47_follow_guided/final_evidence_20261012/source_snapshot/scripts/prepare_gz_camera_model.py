#!/usr/bin/env python3
"""Create a session camera model without rewriting the source SDF."""

import argparse
from pathlib import Path
import shutil
import xml.etree.ElementTree as ET


def prepare(source, output, *, enable_down_camera=False):
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
    shutil.copytree(source, output)
    if not enable_down_camera:
        down.remove(sensor)
        tree.write(output / 'model.sdf', encoding='utf-8', xml_declaration=True)


def main():
    """Prepare only a new model directory, with no simulator side effects."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--source', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--enable-down-camera', choices=('true', 'false'),
                        default='false')
    args = parser.parse_args()
    prepare(args.source, args.output,
            enable_down_camera=args.enable_down_camera == 'true')


if __name__ == '__main__':
    main()
