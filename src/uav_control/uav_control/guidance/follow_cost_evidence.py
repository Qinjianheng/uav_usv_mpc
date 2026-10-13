"""Scheduling hints backed by recent successful work; never execution permission."""
from collections import deque
import math
from statistics import median


class CostEvidence:
    """Protect a rolling cost median for500ms, then age it with a500ms half-life."""

    priors = {'follow': .015, 'fallback': .012}

    def __init__(self):
        """Start with warm priors and no measured cost population."""
        self.samples = {key: deque(maxlen=8) for key in self.priors}
        self.last_stamp = None

    def _advance(self, observation_stamp):
        if self.last_stamp is not None and observation_stamp < self.last_stamp:
            for samples in self.samples.values():
                samples.clear()
        self.last_stamp = observation_stamp

    def observe(self, path, cost, observation_stamp):
        """Use full successful worker time and its original acquired-image epoch."""
        if (path not in self.priors or not math.isfinite(cost) or cost <= 0
                or not math.isfinite(observation_stamp) or observation_stamp <= 0):
            raise ValueError('INVALID_COST_EVIDENCE')
        self._advance(observation_stamp)
        self.samples[path].append((observation_stamp, cost))

    def floor(self, path, observation_stamp):
        """Age only scheduling evidence; input stamps, TTL and leases remain untouched."""
        if not math.isfinite(observation_stamp):
            return self.priors[path]
        self._advance(observation_stamp)
        samples = self.samples[path]
        if not samples:
            return self.priors[path]
        age = max(0., observation_stamp-samples[-1][0])
        scale = 2.**(-max(0., age-.5)/.5)
        return max(self.priors[path], 1.2*median(value for _, value in samples)*scale)

    def report(self, observation_stamp):
        """Expose the actual timing population separately from any feasibility evidence."""
        return {path: dict(count=len(samples),
                           latest_observation_stamp=samples[-1][0] if samples else None,
                           median_seconds=median(v for _, v in samples) if samples else None,
                           floor_seconds=self.floor(path, observation_stamp))
                for path, samples in self.samples.items()}


def evidence_for(solver):
    """Lazily attach pure timing history, including lightweight runner test doubles."""
    if not hasattr(solver, '_cost_evidence'):
        solver._cost_evidence = CostEvidence()
    return solver._cost_evidence


def protect_costs(solver, observation_stamp, age_paths=()):
    """Recover skipped paths from spikes without erasing recent measured cost."""
    evidence = evidence_for(solver)
    for path in ('follow', 'fallback'):
        name = path+'_estimate'
        if not hasattr(solver, name):
            continue
        value = getattr(solver, name)*(.8 if path in age_paths else 1.)
        setattr(solver, name, max(value, evidence.floor(path, observation_stamp)))
