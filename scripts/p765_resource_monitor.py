#!/usr/bin/env python3
"""One-Hz read-only /proc samples for processes in the separately authorized capture scope."""
import argparse
import json
import os
from pathlib import Path
import time


def sample(root, *, proc=Path('/proc')):
    """Match the exact inherited run directory; never identify or signal by fuzzy name."""
    marker = ('UAV_USV_RESEARCH_DIRECTORY='+str(root.resolve())).encode()
    ticks, page = os.sysconf('SC_CLK_TCK'), os.sysconf('SC_PAGE_SIZE')
    rows = []
    for path in proc.iterdir():
        if not path.name.isdigit():
            continue
        try:
            if marker not in (path/'environ').read_bytes().split(b'\0'):
                continue
            raw = (path/'stat').read_text()
            fields = raw[raw.rfind(')')+2:].split()
            rows.append(dict(pid=int(path.name), start_ticks=int(fields[19]),
                             comm=(path/'comm').read_text().strip(),
                             command=(path/'cmdline').read_bytes().decode(
                                 errors='replace').replace('\0', ' ').strip(),
                             cpu_seconds=(int(fields[11])+int(fields[12]))/ticks,
                             rss_bytes=int(fields[21])*page, threads=int(fields[17])))
        except (OSError, ValueError, IndexError):
            continue
    cpu = {}
    if (proc/'stat').is_file():
        for line in (proc/'stat').read_text().splitlines():
            parts = line.split()
            if parts and (parts[0] == 'cpu' or parts[0][3:].isdigit()):
                cpu[parts[0]] = [int(value) for value in parts[1:]]
    return dict(monotonic=time.monotonic(), wall_stamp=time.time(), processes=rows,
                cpu_ticks_per_core=cpu, clock_ticks_per_second=ticks,
                cpu_scope='Host per-core counters; process attribution limited to exact run env')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--duration', type=float, default=35.)
    args = parser.parse_args()
    if not 1 <= args.duration <= 45:
        raise ValueError('P765_RESOURCE_DURATION')
    root = Path(os.environ['UAV_USV_RESEARCH_DIRECTORY'])
    if not args.output.resolve().is_relative_to(root.resolve()):
        raise ValueError('P765_RESOURCE_OUTPUT_SCOPE')
    began = time.monotonic()
    with args.output.open('x') as stream:
        while time.monotonic()-began < args.duration:
            stream.write(json.dumps(sample(root))+'\n')
            stream.flush()
            time.sleep(1.)


if __name__ == '__main__':
    main()
