#!/usr/bin/env python3
"""Bounded STOP/CONT only for exact child identities owned by the isolated trial."""
import json
import os
from pathlib import Path
import signal
import time

from p4_sitl_session import identity


class FaultInjector:
    """No forged prediction, ACK or expiry: suspend real producers and record causal epochs."""

    schedule = ((15., .6, ('target_predictor_node',)),
                (25., .8, ('p4_follow_planner_node',)),
                (35., .6, ('rgbd_target_localizer', 'target_bearing_node')),
                (45., .18, ('p4_follow_planner_node',)))

    def __init__(self, root):
        self.root, self.done, self.stopped, self.events = root, set(), [], []

    def record(self, **event):
        """Write a complete append-only event ledger after each signal."""
        self.events.append(dict(stamp=time.time(), **event))
        (self.root/'fault_injections.json').write_text(json.dumps(self.events, indent=2))

    def resume(self, force=False):
        """Resume exact identities, including in the session's finally block."""
        for item in list(self.stopped):
            entry, deadline = item
            if force or time.monotonic() >= deadline:
                try:
                    if identity(entry['pid']) == entry:
                        os.kill(entry['pid'], signal.SIGCONT)
                        self.record(signal='SIGCONT', identity=entry)
                except OSError:
                    pass
                self.stopped.remove(item)

    def tick(self, elapsed, identities):
        """Never signal the tracker, DDS, PX4, or unrelated process groups."""
        self.resume()
        owner = identities.get('experiment')
        if owner is None:
            return
        for index, (start, duration, names) in enumerate(self.schedule):
            if elapsed < start or index in self.done:
                continue
            self.done.add(index)
            for process in Path('/proc').iterdir():
                if not process.name.isdigit():
                    continue
                try:
                    entry = identity(int(process.name))
                    argv = (process/'cmdline').read_bytes().decode().split('\0')
                    if entry['pgid'] != owner['pgid'] or not any(
                            Path(v).name in names for v in argv if v):
                        continue
                    if identity(entry['pid']) == entry:
                        os.kill(entry['pid'], signal.SIGSTOP)
                        self.stopped.append((entry, time.monotonic()+duration))
                        self.record(signal='SIGSTOP', identity=entry, phase=index,
                                    planned_duration=duration, elapsed=elapsed)
                except (OSError, ValueError):
                    pass
