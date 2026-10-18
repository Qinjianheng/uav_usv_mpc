#!/usr/bin/env python3
"""Own a local SITL heartbeat for exactly the lifetime of one launch command."""

import os
from pathlib import Path
import signal
import subprocess
import sys
import time


def stop_process(process):
    """Stop and reap this runner's child, including its launch descendants."""
    if process is None:
        return
    # Children have their own groups so terminal SIGINT reaches the supervisor
    # once. Forwarding it cannot interrupt ROS launch twice.
    try:
        os.killpg(process.pid, signal.SIGTERM)
    except ProcessLookupError:
        pass
    try:
        process.wait(timeout=5.)
    except subprocess.TimeoutExpired:
        try:
            os.killpg(process.pid, signal.SIGKILL)
        except ProcessLookupError:
            pass
        process.wait()


def run(command, *, heartbeat_command=None):
    """Keep the caller's console independent of the experiment's GCS lifetime."""
    heartbeat_command = heartbeat_command or [
        sys.executable, str(Path(__file__).with_name('p45_gcs_heartbeat.py'))]
    stopped = None

    def request_stop(signum, _frame):
        nonlocal stopped
        stopped = signum

    previous = {sig: signal.signal(sig, request_stop)
                for sig in (signal.SIGINT, signal.SIGTERM, signal.SIGHUP)}
    heartbeat = experiment = None
    try:
        heartbeat = subprocess.Popen(heartbeat_command, start_new_session=True)
        experiment = subprocess.Popen(command, start_new_session=True)
        while stopped is None:
            result = experiment.poll()
            if result is not None:
                return result if result >= 0 else 128 - result
            if heartbeat.poll() is not None:
                print('Local SITL GCS heartbeat exited; stopping its experiment.', file=sys.stderr)
                return 1
            time.sleep(.05)
        return 128 + stopped
    finally:
        stop_process(experiment)
        stop_process(heartbeat)
        for sig, handler in previous.items():
            signal.signal(sig, handler)


def main():
    command = sys.argv[1:]
    if command[:1] == ['--']:
        command = command[1:]
    if not command:
        print('Usage: run_with_sitl_gcs.py -- COMMAND [ARG ...]', file=sys.stderr)
        return 2
    return run(command)


if __name__ == '__main__':
    sys.exit(main())
