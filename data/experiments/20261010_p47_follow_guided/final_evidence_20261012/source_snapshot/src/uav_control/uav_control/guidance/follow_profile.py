"""Opt-in per-cycle monotonic exclusive timings; cross-process latency is separate."""
from collections import Counter
from contextlib import contextmanager
from contextvars import ContextVar
from functools import wraps
import time

_ACTIVE = ContextVar('follow_cycle_profile', default=None)


def current():
    """Return only this thread/context's explicit recorder."""
    return _ACTIVE.get()


class CycleProfile:
    """Nested stage time is charged once to its innermost owner."""

    def __init__(self, cycle_id, clock=time.perf_counter):
        """Accept a monotonic clock; no epoch ages use this clock."""
        self.cycle_id, self.clock = cycle_id, clock
        self.seconds, self.calls, self.counts = Counter(), Counter(), Counter()
        self.stack = []
        self.total = 0.

    def __enter__(self):
        self.started = self.clock()
        self.token = _ACTIVE.set(self)
        return self

    def __exit__(self, *_):
        self.total = self.clock()-self.started
        _ACTIVE.reset(self.token)

    def report(self):
        """Return nonoverlapping totals; unmeasured producer/ACK time stays absent."""
        return dict(cycle_id=self.cycle_id, total_seconds=self.total,
                    stages={key: dict(seconds=value, calls=self.calls[key])
                            for key, value in self.seconds.items()}, counts=dict(self.counts),
                    unattributed_seconds=max(0., self.total-sum(self.seconds.values())))


@contextmanager
def stage(name):
    """Read one ContextVar without reading the clock when recording is disabled."""
    recorder = current()
    if recorder is None:
        yield
        return
    entry = [recorder.clock(), 0.]
    recorder.stack.append(entry)
    try:
        yield
    finally:
        elapsed = recorder.clock()-entry[0]
        recorder.stack.pop()
        recorder.seconds[name] += elapsed-entry[1]
        recorder.calls[name] += 1
        if recorder.stack:
            recorder.stack[-1][1] += elapsed


def count(name, amount=1):
    """Count work, not an implied safety proof or a cross-process duration."""
    recorder = current()
    if recorder is not None:
        recorder.counts[name] += amount


def profiled(name):
    """Decorate a mathematical operation without changing its argument/result contract."""
    def decorate(function):
        @wraps(function)
        def wrapper(*args, **kwargs):
            if current() is None:
                return function(*args, **kwargs)
            with stage(name):
                return function(*args, **kwargs)
        return wrapper
    return decorate
