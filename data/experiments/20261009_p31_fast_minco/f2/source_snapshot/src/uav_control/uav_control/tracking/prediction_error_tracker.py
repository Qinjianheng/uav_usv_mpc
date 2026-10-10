"""Match delayed target predictions with later ground-truth positions."""

import math
from collections import deque
from dataclasses import dataclass


@dataclass(frozen=True)
class PredictionEvaluation:
    """A prediction paired with truth after its horizon has elapsed."""

    position: tuple
    error: float
    age: float


class PredictionErrorTracker:
    """Queue multi-horizon predictions and evaluate them when they mature."""

    def __init__(self, horizons=(0.5, 1.0, 2.0)):
        """Configure the positive prediction horizons in seconds."""
        self.horizons = tuple(float(value) for value in horizons)
        if not self.horizons or not all(
            math.isfinite(value) and value > 0.0 for value in self.horizons
        ):
            raise ValueError('prediction horizons must be positive and finite')
        self._queues = {}

    @staticmethod
    def _vector(position):
        vector = tuple(float(value) for value in position)
        if (
            len(vector) != 3
            or not all(math.isfinite(value) for value in vector)
        ):
            raise ValueError('prediction position must contain three values')
        return vector

    def add(self, model, timestamp, predictions):
        """Queue one prediction for every configured horizon."""
        timestamp = float(timestamp)
        if not math.isfinite(timestamp):
            raise ValueError('prediction timestamp must be finite')
        model = str(model)
        for horizon in self.horizons:
            if horizon not in predictions:
                continue
            position = self._vector(predictions[horizon])
            queue = self._queues.setdefault((model, horizon), deque())
            queue.append((timestamp, timestamp + horizon, position))

    def evaluate(self, model, horizon, timestamp, actual_position):
        """Return the newest matured prediction for a model and horizon."""
        queue = self._queues.get((str(model), float(horizon)))
        if not queue:
            return None
        timestamp = float(timestamp)
        actual = self._vector(actual_position)
        matured = None
        while queue and queue[0][1] <= timestamp + 1.0e-9:
            matured = queue.popleft()
        if matured is None:
            return None
        origin_time, _, position = matured
        error = math.sqrt(sum(
            (predicted - truth) ** 2
            for predicted, truth in zip(position, actual)
        ))
        return PredictionEvaluation(
            position=position,
            error=error,
            age=timestamp - origin_time,
        )

    def reset(self):
        """Discard every pending prediction."""
        self._queues.clear()
