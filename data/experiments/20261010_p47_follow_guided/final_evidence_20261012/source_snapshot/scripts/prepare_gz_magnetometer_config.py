#!/usr/bin/env python3
"""Copy the native server config with explicit ENU/gauss magnetometer input."""

import argparse
from pathlib import Path
import xml.etree.ElementTree as ET


def prepare(source, output):
    """Preserve every existing system and select the matching bridge mode."""
    tree = ET.parse(source)
    selected = [p for p in tree.getroot().findall('plugins/plugin')
                if p.get('name') == 'gz::sim::systems::Magnetometer']
    if len(selected) != 1:
        raise ValueError('expected exactly one native Magnetometer system')
    for key, value in [('use_earth_frame_ned', 'false'), ('use_units_gauss', 'true')]:
        element = selected[0].find(key)
        if element is None:
            element = ET.SubElement(selected[0], key)
        element.text = value
    ET.indent(tree, space='  ')
    tree.write(output, encoding='utf-8', xml_declaration=True)


def main():
    """Generate configuration only; never launch a simulator or alter PX4."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--source', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    if args.source.resolve() == args.output.resolve():
        parser.error('output must be a separate configuration copy')
    prepare(args.source, args.output)


if __name__ == '__main__':
    main()
