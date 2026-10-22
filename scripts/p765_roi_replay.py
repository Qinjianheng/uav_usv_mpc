#!/usr/bin/env python3
"""Replay lossless ROI depth with source pixel mapping and original strict fit contract."""
import argparse
import json
from pathlib import Path

from uav_control.perception.p765_roi_capture import replay_roi, runtime_identity


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--input', required=True, type=Path)
    parser.add_argument('--output', required=True, type=Path)
    args = parser.parse_args()
    result = dict(runtime_identity=runtime_identity(),
                  samples=[replay_roi(path) for path in sorted(args.input.glob('roi_*.json'))],
                  qualification='OFFLINE_ONLY_NO_NEW_REAL_OBSERVATIONS')
    with args.output.open('x') as stream:
        json.dump(result, stream, allow_nan=False, indent=2, sort_keys=True)


if __name__ == '__main__':
    main()
