#!/usr/bin/env python3
"""Audit immutable P764 frame lineage without starting ROS, Gazebo or PX4."""
import argparse
import hashlib
import json
from pathlib import Path

from uav_control.perception.p765_geometry import analyze_history


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--input', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    result = analyze_history(args.input)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    lineages = result.pop('observations')
    lineage_path = args.output.parent/'histlineage.jsonl'
    with lineage_path.open('x') as stream:
        for record in lineages:
            stream.write(json.dumps(record, sort_keys=True, allow_nan=False)+'\n')
    result['lineage_file'] = dict(
        path=str(lineage_path.resolve()),
        sha256=hashlib.sha256(lineage_path.read_bytes()).hexdigest())
    with args.output.open('x') as stream:
        json.dump(result, stream, indent=2, sort_keys=True, allow_nan=False)
    print(json.dumps(result, indent=2, sort_keys=True, allow_nan=False))


if __name__ == '__main__':
    main()
