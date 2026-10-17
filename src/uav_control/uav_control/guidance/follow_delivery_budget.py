"""Scheduling-only delivery contracts; input freshness and receiver authority stay independent."""
from collections import deque
from dataclasses import dataclass
import math

import numpy as np


def tail_reserve(samples, now, prior, extra=0.):
    """Age measured producer/receiver tails without treating a failed delivery as free."""
    values = [(stamp, value) for stamp, value in samples
              if math.isfinite(stamp) and math.isfinite(value) and value > 0 and stamp <= now]
    if not values:
        return prior
    age = max(0., now-max(stamp for stamp, _ in values))
    scale = 2.**(-max(0., age-.5)/.5)
    return max(prior, 1.2*float(np.percentile([v for _, v in values], 90))*scale+extra)


@dataclass(frozen=True)
class DeliveryPlan:
    """One immutable numerical path and deadlines in original ROS acquisition epochs."""

    identity: tuple
    path: str
    created_ros: float
    input_until: float
    start: float
    parent_id: int
    parent_deadline: float
    worker_until: float
    wire_until: float
    accept_until: float
    worker_hint: float
    available_worker: float
    queue_reserve: float
    publication_reserve: float
    receiver_reserve: float

    def accepts(self, path, finished_ros):
        """Reject late or different-path results without granting execution permission."""
        return path == self.path and self.created_ros <= finished_ros < self.worker_until


def make_plan(identity, now, input_until, start, parent_id, parent_deadline, costs,
              queue=.020, publication=.010, receiver=.020):
    """Budget the entire remaining chain before selecting one feasible numerical path."""
    times = (now, input_until, start, parent_deadline, queue, publication, receiver)
    if (not all(math.isfinite(v) for v in times) or min(queue, publication, receiver) < 0
            or now <= 0 or input_until <= now or start <= now
            or any(not math.isfinite(v) or v <= 0 for v in costs.values())):
        raise ValueError('INVALID_DELIVERY_BUDGET')
    if parent_id and start >= parent_deadline-.05:
        raise ValueError('NO_SUCCESSOR_ACTIVATION_MARGIN')
    accept_until = min(input_until, start)
    wire_until = accept_until-receiver
    worker_until = wire_until-publication
    available = worker_until-now-queue
    path = next((p for p in ('follow', 'fallback') if p in costs and costs[p] < available), None)
    if path is None:
        raise ValueError('SUCCESSOR_AWAITING_FRESH_INPUT')
    return DeliveryPlan(tuple(identity), path, now, input_until, start, parent_id,
                        parent_deadline, worker_until, wire_until, accept_until,
                        costs[path], available, queue, publication, receiver)


class DeliveryCosts:
    """All completed work, including failed/censored work, estimates delivery risk only."""

    priors = {'follow': .015, 'fallback': .012}

    def __init__(self):
        """Bound timing history separately from legacy successful feasibility hints."""
        self.samples = {p: deque(maxlen=16) for p in self.priors}
        self.last_stamp = None

    def advance(self, stamp):
        """Do not reuse future-clock populations after a reset."""
        if not math.isfinite(stamp):
            raise ValueError('INVALID_COST_EPOCH')
        if self.last_stamp is not None and stamp < self.last_stamp:
            for values in self.samples.values():
                values.clear()
        self.last_stamp = stamp

    def observe(self, path, wall, observation_stamp, valid, status, cpu=None):
        """Retain deadline failure as a consumed-cost lower bound, never a free sample."""
        if path not in self.priors or not math.isfinite(wall) or wall <= 0:
            raise ValueError('INVALID_DELIVERY_COST')
        self.advance(observation_stamp)
        self.samples[path].append(dict(wall=wall, cpu=cpu, stamp=observation_stamp,
                                       valid=bool(valid), status=status))

    def report(self, observation_stamp):
        """Expose both populations; the percentile is a hint, never a completion guarantee."""
        self.advance(observation_stamp)
        result = {}
        for path, samples in self.samples.items():
            age = max(0., observation_stamp-samples[-1]['stamp']) if samples else 0.
            scale = 2.**(-max(0., age-.5)/.5)
            measured = float(np.percentile([s['wall'] for s in samples], 90)) if samples else 0.
            result[path] = dict(
                count=len(samples),
                successful_count=sum(s['valid'] for s in samples),
                failed_count=sum(not s['valid'] for s in samples),
                censored_count=sum('DEADLINE' in s['status'] for s in samples),
                p90_wall=measured, hint=max(self.priors[path], measured*1.2*scale),
                estimate_is_guarantee=False)
        return result

    def hints(self, observation_stamp):
        """Only this timing population feeds the locked-path dispatcher."""
        return {path: value['hint'] for path, value in self.report(observation_stamp).items()}
