"""Bounded acquisition-time delayed CV Kalman updates with exact chronological replay."""
from dataclasses import dataclass
import math

import numpy as np


@dataclass(frozen=True)
class DelayedEstimate:
    """Predicted epoch and original measurement authority remain separate."""

    stamp: float
    source_stamp: float
    state: np.ndarray
    covariance: np.ndarray
    valid: bool
    reason: str


class DelayedSpatialKalman:
    """Six-state engineering reduction using the delayed-update/replay principle of[28]."""

    def __init__(self, acceleration_std=.5, initial_velocity_std=5., history_seconds=1.,
                 maximum_source_age=.125, dimensions=3):
        self.dimensions = dimensions
        self.acceleration_std = float(acceleration_std)
        self.initial_velocity_std = float(initial_velocity_std)
        self.history_seconds = float(history_seconds)
        self.maximum_source_age = float(maximum_source_age)
        if not (maximum_source_age == .125 and history_seconds >= maximum_source_age
                and acceleration_std > 0 and initial_velocity_std > 0):
            raise ValueError('DKF_CONFIGURATION')
        self.reset()

    def reset(self):
        """Remove every prior and covariance after a clock/frame discontinuity."""
        self.anchor = None
        self.measurements = []
        self.checkpoints = []
        self.source_stamp = 0.
        self.last_rejection = 'UNINITIALIZED'

    def _propagate(self, state, covariance, dt):
        if dt < -1e-9:
            raise ValueError('DKF_NEGATIVE_TIME')
        dt = max(0., dt)
        n = self.dimensions
        identity = np.eye(n)
        transition = np.block([[identity, dt*identity], [np.zeros((n, n)), identity]])
        # Continuous white-acceleration spectral density makes split replay
        # intervals consistent; it is an explicit engineering process model.
        noise = self.acceleration_std**2*np.block([
            [dt**3/3*identity, dt**2/2*identity], [dt**2/2*identity, dt*identity]])
        return transition@state, transition@covariance@transition.T+noise

    def update(self, measurement, covariance, stamp, now):
        """Correct a legal historical acquisition, then replay all later stored measurements."""
        n = self.dimensions
        z, r = np.asarray(measurement, dtype=float), np.asarray(covariance, dtype=float)
        if (not all(math.isfinite(v) for v in (stamp, now)) or not 0 < stamp <= now
                or now-stamp > self.maximum_source_age+1e-9):
            self.last_rejection = 'STALE_MEASUREMENT'
            return False
        if (z.shape != (n,) or r.shape != (n, n) or not np.isfinite(z).all()
                or not np.isfinite(r).all() or not np.allclose(r, r.T)
                or np.linalg.eigvalsh(r).min() <= 0):
            self.last_rejection = 'INVALID_COVARIANCE_OR_MEASUREMENT'
            return False
        if self.anchor is None:
            x = np.r_[z, np.zeros(n)]
            p = np.zeros((2*n, 2*n))
            p[:n, :n], p[n:, n:] = r, np.eye(n)*self.initial_velocity_std**2
            self.anchor = (stamp, x, p)
            self.checkpoints = [self.anchor]
        else:
            if stamp <= self.anchor[0] or any(t == stamp for t, _, _ in self.measurements):
                self.last_rejection = 'DUPLICATE_OR_BEFORE_HISTORY'
                return False
            self.measurements.append((stamp, z.copy(), r.copy()))
            self.measurements.sort(key=lambda item: item[0])
            t, x, p = self.anchor
            checkpoints = [(t, x.copy(), p.copy())]
            observation = np.c_[np.eye(n), np.zeros((n, n))]
            identity = np.eye(2*n)
            for epoch, value, noise in self.measurements:
                x, p = self._propagate(x, p, epoch-t)
                innovation = observation@p@observation.T+noise
                gain = np.linalg.solve(innovation, observation@p).T
                x = x+gain@(value-observation@x)
                residual = identity-gain@observation
                p = residual@p@residual.T+gain@noise@gain.T
                p = .5*(p+p.T)
                t = epoch
                checkpoints.append((t, x.copy(), p.copy()))
            self.checkpoints = checkpoints
            retained = [row for row in checkpoints if row[0] <= now-self.history_seconds]
            if retained:
                self.anchor = retained[-1]
                self.measurements = [row for row in self.measurements if row[0] > self.anchor[0]]
                self.checkpoints = [row for row in checkpoints if row[0] >= self.anchor[0]]
        self.source_stamp = max(self.source_stamp, stamp)
        self.last_rejection = ''
        return True

    def predict(self, now):
        """Predict state without replacing or refreshing the newest real source epoch."""
        if not self.checkpoints:
            return DelayedEstimate(now, 0., np.zeros(2*self.dimensions),
                                   np.zeros((2*self.dimensions, 2*self.dimensions)), False,
                                   'UNINITIALIZED')
        t, x, p = self.checkpoints[-1]
        if not math.isfinite(now) or now < t:
            return DelayedEstimate(now, self.source_stamp, x.copy(), p.copy(), False,
                                   'PREDICTION_BEFORE_MEASUREMENT')
        x, p = self._propagate(x, p, now-t)
        valid = 0 <= now-self.source_stamp <= self.maximum_source_age+1e-9
        return DelayedEstimate(now, self.source_stamp, x, p, valid,
                               'VALID' if valid else 'SOURCE_STALE')
