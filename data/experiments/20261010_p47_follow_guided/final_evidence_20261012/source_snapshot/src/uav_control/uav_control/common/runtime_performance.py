"""Small bounded runtime-rate utilities shared by diagnostic nodes."""

from collections import deque
import math


class RateMeter:
    """Measure recent event frequency without retaining an unbounded history."""

    def __init__(self, window_seconds=1.0):
        self.window_seconds = max(float(window_seconds), 1e-3)
        self._stamps = deque()

    def observe(self, stamp):
        stamp = float(stamp)
        if not math.isfinite(stamp):
            return
        self._stamps.append(stamp)
        self._prune(stamp)

    def _prune(self, now):
        cutoff = float(now) - self.window_seconds
        while self._stamps and self._stamps[0] < cutoff:
            self._stamps.popleft()

    def rate(self, now):
        self._prune(now)
        if len(self._stamps) < 2:
            return 0.0
        elapsed = self._stamps[-1] - self._stamps[0]
        return (
            (len(self._stamps) - 1) / elapsed
            if elapsed > 1e-9 else 0.0
        )
