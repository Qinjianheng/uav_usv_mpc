#!/usr/bin/env python3
"""Capture authorized launcher terminal stdout without changing its commands or PID groups."""
import os
from pathlib import Path
import re
import shlex
import sys


if __name__ == '__main__':
    args = sys.argv[1:]
    root = Path(os.environ['UAV_USV_TERMINAL_LOG_ROOT']).resolve()
    title = next((v.split('=', 1)[1] for v in args if v.startswith('--title=')), 'component')
    name = re.sub('[^A-Za-z0-9_.-]', '_', title)
    if len(args) < 3 or args[-2] != '-lc':
        raise SystemExit('Unsupported launcher terminal invocation')
    log = shlex.quote(str(root/(name+'.txt')))
    args[-1] = 'exec > >(tee -a '+log+') 2>&1; '+args[-1]
    os.execv('/usr/bin/gnome-terminal', ['gnome-terminal', *args])
