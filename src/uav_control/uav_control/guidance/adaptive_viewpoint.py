"""
Transport-free bounded side/rear observation-point selection.

This module chooses reference parameters; it has no vehicle command authority.
Existing follow_reference.viewpoint_state remains the sole kinematic formula.
"""

from dataclasses import asdict, dataclass
import hashlib
import json
import math
from numbers import Real

import numpy as np


@dataclass(frozen=True)
class AdaptiveViewpointConfig:
    minimum_distance: float = 4.5
    maximum_distance: float = 6.
    preferred_distance: float = 5.
    maximum_beta: float = math.radians(20.)
    candidate_distances: tuple = (4.5, 5., 5.5, 6.)
    candidate_betas: tuple = tuple(math.radians(x) for x in (-20., -10., 0., 10., 20.))
    maximum_distance_rate: float = .15
    maximum_distance_acceleration: float = .4
    maximum_beta_rate: float = .05
    maximum_beta_acceleration: float = .3
    risk_weight: float = 1.
    follow_error_weight: float = .2
    smooth_weight: float = .2
    switch_weight: float = .05
    acceleration_weight: float = .02
    jerk_weight: float = .005
    hysteresis_cost: float = .1
    minimum_dwell_time: float = .5
    maximum_dt: float = 1.
    maximum_forecast_time: float = 4.
    version: str = 'p6.adaptive_viewpoint.v1'

    def fingerprint(self):
        encoded = json.dumps(asdict(self), sort_keys=True, separators=(',', ':'),
                             allow_nan=False).encode('utf-8')
        return hashlib.sha256(encoded).hexdigest()


@dataclass(frozen=True)
class ViewpointState:
    """Six explicit reference coefficients, metres/radians and SI derivatives."""

    d: float = 5.
    beta: float = 0.
    d_dot: float = 0.
    d_ddot: float = 0.
    beta_dot: float = 0.
    beta_ddot: float = 0.

    def reference_kwargs(self):
        """Map directly to the established follow_reference.viewpoint_state API."""
        return dict(distance=self.d, beta=self.beta, distance_rate=self.d_dot,
                    distance_acceleration=self.d_ddot, beta_rate=self.beta_dot,
                    beta_acceleration=self.beta_ddot)

    def as_tuple(self):
        """Return d,beta,d_dot,d_ddot,beta_dot,beta_ddot for request evidence."""
        return (self.d, self.beta, self.d_dot, self.d_ddot, self.beta_dot, self.beta_ddot)

    def at_time(self, time):
        """
        Analytic constant-acceleration propagation for differentiation tests.

        This algebraic operation alone has no domain safety guarantee. Online
        callers must use forecast(), which validates the entire propagation span.
        """
        if (not math.isfinite(time) or not np.all(np.isfinite(self.as_tuple()))):
            raise ValueError('INVALID_VIEWPOINT_PROPAGATION')
        return ViewpointState(
            self.d + self.d_dot*time + .5*self.d_ddot*time*time,
            self.beta + self.beta_dot*time + .5*self.beta_ddot*time*time,
            self.d_dot + self.d_ddot*time, self.d_ddot,
            self.beta_dot + self.beta_ddot*time, self.beta_ddot)

    def forecast(self, time, config=AdaptiveViewpointConfig()):
        """
        Fail closed if the analytic reference leaves any bound before ``time``.

        Interior extrema are checked, so a return into the domain at the endpoint
        does not hide an excursion. No clipping alters the reference derivatives.
        A planner must reject/reselect a reference that cannot span its horizon.
        """
        _validate_config(config)
        if not math.isfinite(time) or not 0. <= time <= config.maximum_forecast_time:
            raise ValueError('INVALID_VIEWPOINT_FORECAST_TIME')
        axes = ((self.d, self.d_dot, self.d_ddot, config.minimum_distance,
                 config.maximum_distance, config.maximum_distance_rate,
                 config.maximum_distance_acceleration),
                (self.beta, self.beta_dot, self.beta_ddot, -config.maximum_beta,
                 config.maximum_beta, config.maximum_beta_rate,
                 config.maximum_beta_acceleration))
        for value, rate, acceleration, lower, upper, vmax, amax in axes:
            times = [0., time]
            if acceleration != 0. and 0. < -rate/acceleration < time:
                times.append(-rate/acceleration)
            positions = [value + rate*t + .5*acceleration*t*t for t in times]
            if (not np.all(np.isfinite((value, rate, acceleration)))
                    or min(positions) < lower - 1e-12 or max(positions) > upper + 1e-12
                    or max(abs(rate), abs(rate + acceleration*time)) > vmax + 1e-12
                    or abs(acceleration) > amax + 1e-12):
                raise ValueError('VIEWPOINT_FORECAST_OUT_OF_BOUNDS')
        return self.at_time(time)


@dataclass(frozen=True)
class AdaptiveViewpointReport:
    valid: bool
    reason: str
    state: ViewpointState
    target: tuple
    switched: bool = False
    candidate_index: int = 0
    scores: tuple = ()
    config_fingerprint: str = ''


def _validate_config(config):
    if not isinstance(config, AdaptiveViewpointConfig):
        raise ValueError('INVALID_ADAPTIVE_VIEWPOINT_CONFIG')
    scalars = [v for k, v in asdict(config).items()
               if k not in ('candidate_distances', 'candidate_betas', 'version')]
    try:
        distances = np.asarray(config.candidate_distances, dtype=float)
        betas = np.asarray(config.candidate_betas, dtype=float)
        valid = (all(not isinstance(x, (bool, str)) for x in scalars)
                 and np.all(np.isfinite(scalars))
                 and 0. < config.minimum_distance <= config.preferred_distance
                 <= config.maximum_distance
                 and 0. < config.maximum_beta < math.pi/2
                 and min(config.maximum_distance_rate, config.maximum_distance_acceleration,
                         config.maximum_beta_rate, config.maximum_beta_acceleration,
                         config.maximum_dt, config.maximum_forecast_time) > 0.
                 and min(config.risk_weight, config.follow_error_weight, config.smooth_weight,
                         config.switch_weight, config.acceleration_weight, config.jerk_weight,
                         config.hysteresis_cost, config.minimum_dwell_time) >= 0.
                 and distances.ndim == 1 and betas.ndim == 1 and len(distances) > 0
                 and len(betas) > 0 and np.all(np.isfinite(distances))
                 and np.all(np.isfinite(betas))
                 and np.all(distances >= config.minimum_distance)
                 and np.all(distances <= config.maximum_distance)
                 and np.all(np.abs(betas) <= config.maximum_beta)
                 and isinstance(config.version, str) and bool(config.version))
    except (TypeError, ValueError):
        valid = False
    if not valid:
        raise ValueError('INVALID_ADAPTIVE_VIEWPOINT_CONFIG')


def _polynomial_extrema(coefficients, derivative=0):
    polynomial = np.polynomial.polynomial
    coefficients = polynomial.polyder(coefficients, derivative)
    values = [polynomial.polyval(0., coefficients), polynomial.polyval(1., coefficients)]
    roots = polynomial.polyroots(polynomial.polyder(coefficients))
    for root in roots:
        if abs(root.imag) < 1e-8 and 0. < root.real < 1.:
            values.append(polynomial.polyval(root.real, coefficients))
    return min(values), max(values)


@dataclass(frozen=True)
class ViewpointProfile:
    """
    Persistent normalized quintic reference, including its true elapsed time.

    Coefficients are two rows (distance,beta) in ascending powers of s=t/T.
    Rest-to-rest segments are C2 at joins and remain constant after completion.
    This exact representation can be independently reconstructed by a receiver.
    """

    coefficients: tuple
    duration: float
    elapsed: float = 0.

    def as_tuple(self):
        """Serialize duration,elapsed,6 distance coefficients,6 beta coefficients."""
        return (self.duration, self.elapsed, *self.coefficients[0], *self.coefficients[1])

    @classmethod
    def from_tuple(cls, values):
        array = np.asarray(values, dtype=float)
        if (array.shape != (14,) or not np.all(np.isfinite(array)) or array[0] < 0.
                or not 0. <= array[1] <= array[0]):
            raise ValueError('INVALID_VIEWPOINT_PROFILE')
        if array[0] == 0. and np.any(np.abs(np.r_[array[3:8], array[9:14]]) > 1e-12):
            raise ValueError('INVALID_VIEWPOINT_PROFILE')
        return cls((tuple(array[2:8]), tuple(array[8:14])), float(array[0]), float(array[1]))

    def validate(self, config):
        """Check all extrema analytically, including interior rate/acceleration peaks."""
        _validate_config(config)
        ViewpointProfile.from_tuple(self.as_tuple())
        if self.duration == 0.:
            self.forecast(0.).forecast(0., config)
            return self
        for coefficients, lower, upper, vmax, amax in (
                (self.coefficients[0], config.minimum_distance, config.maximum_distance,
                 config.maximum_distance_rate, config.maximum_distance_acceleration),
                (self.coefficients[1], -config.maximum_beta, config.maximum_beta,
                 config.maximum_beta_rate, config.maximum_beta_acceleration)):
            low, high = _polynomial_extrema(coefficients)
            vlow, vhigh = _polynomial_extrema(coefficients, 1)
            alow, ahigh = _polynomial_extrema(coefficients, 2)
            if (low < lower-1e-10 or high > upper+1e-10
                    or max(abs(vlow), abs(vhigh))/self.duration > vmax+1e-10
                    or max(abs(alow), abs(ahigh))/self.duration**2 > amax+1e-10):
                raise ValueError('VIEWPOINT_PROFILE_OUT_OF_BOUNDS')
            # The post-profile hold must join with zero velocity/acceleration.
            for derivative in (1, 2):
                end = np.polynomial.polynomial.polyval(
                    1., np.polynomial.polynomial.polyder(coefficients, derivative))
                if abs(end)/self.duration**derivative > 1e-9:
                    raise ValueError('VIEWPOINT_PROFILE_NONZERO_TERMINAL_DERIVATIVE')
        return self

    def forecast(self, time):
        if isinstance(time, (bool, str)) or not math.isfinite(time) or time < 0.:
            raise ValueError('INVALID_VIEWPOINT_FORECAST_TIME')
        if self.duration == 0.:
            return ViewpointState(self.coefficients[0][0], self.coefficients[1][0])
        relative = self.elapsed + time
        s = min(1., relative/self.duration)
        values = []
        for coefficients in self.coefficients:
            position = float(np.polynomial.polynomial.polyval(s, coefficients))
            if relative >= self.duration:
                rate, acceleration = 0., 0.
            else:
                rate = float(np.polynomial.polynomial.polyval(
                    s, np.polynomial.polynomial.polyder(coefficients)))/self.duration
                acceleration = float(np.polynomial.polynomial.polyval(
                    s, np.polynomial.polynomial.polyder(coefficients, 2)))/self.duration**2
            values.append((position, rate, acceleration))
        d, b = values
        return ViewpointState(d[0], b[0], d[1], d[2], b[1], b[2])


def _make_profile(state, target, config):
    pairs = ((state.d, state.d_dot, state.d_ddot, target[0], config.maximum_distance_rate,
              config.maximum_distance_acceleration),
             (state.beta, state.beta_dot, state.beta_ddot, target[1], config.maximum_beta_rate,
              config.maximum_beta_acceleration))
    baseline = max([.5] + [max(1.9*abs(goal-x)/vmax, math.sqrt(6.*abs(goal-x)/amax))
                           for x, v, a, goal, vmax, amax in pairs])
    for factor in (1., .8, 1.3, .6, 1.8, .4, 2.5, .25, 4.):
        duration, rows = baseline*factor, []
        for x, velocity, acceleration, goal, _, _ in pairs:
            delta, v, a = goal-x, velocity*duration, acceleration*duration**2
            rows.append((x, v, .5*a, 10*delta-6*v-1.5*a,
                         -15*delta+8*v+1.5*a, 6*delta-3*v-.5*a))
        profile = ViewpointProfile(tuple(rows), duration)
        try:
            return profile.validate(config)
        except ValueError:
            continue
    raise ValueError('NO_BOUNDED_VIEWPOINT_TRANSITION')


def profile_state(profile, time, config=None):
    """
    Reproduce a serialized ascending-power profile at elapsed+relative time.

    Accept a profile object or the explicit dict produced by reference_profile().
    Optional config validates the whole curve before sampling. The caller owns
    epoch alignment: time must be the true offset from the last select epoch.
    """
    try:
        if isinstance(profile, dict):
            if (len(profile['distance_coefficients']) != 6
                    or len(profile['beta_coefficients']) != 6):
                raise ValueError('INVALID_VIEWPOINT_PROFILE')
            values = (profile['duration'], profile['elapsed'],
                      *profile['distance_coefficients'], *profile['beta_coefficients'])
        elif isinstance(profile, ViewpointProfile):
            values = profile.as_tuple()
        else:
            if len(profile) == 4 and (len(profile[2]) != 6 or len(profile[3]) != 6):
                raise ValueError('INVALID_VIEWPOINT_PROFILE')
            values = ((profile[0], profile[1], *profile[2], *profile[3])
                      if len(profile) == 4 else profile)
        restored = ViewpointProfile.from_tuple(values)
        if config is not None:
            restored.validate(config)
        return restored.forecast(time)
    except (TypeError, KeyError, OverflowError) as error:
        raise ValueError('INVALID_VIEWPOINT_PROFILE') from error


class AdaptiveViewpointSelector:
    """
    Stateful candidate preference with bounded slew, dwell and hysteresis.

    Candidate scores are supplied by a planner's independent robust evaluation.
    Infinity marks infeasible; no feasible candidate returns explicit failure and
    preserves the existing action while its real time advances. A valid reference
    remains subject to all original
    MINCO/receiver constraints; selection itself is never a vehicle safety proof.
    """

    def __init__(self, config=AdaptiveViewpointConfig(), initial_state=ViewpointState()):
        _validate_config(config)
        try:
            initial_state.forecast(0., config)
            axes = ((initial_state.d, initial_state.d_dot, config.minimum_distance,
                     config.maximum_distance, config.maximum_distance_acceleration),
                    (initial_state.beta, initial_state.beta_dot, -config.maximum_beta,
                     config.maximum_beta, config.maximum_beta_acceleration))
            for x, v, lower, upper, amax in axes:
                if (x - min(v, 0.)**2/(2*amax) < lower - 1e-12
                        or x + max(v, 0.)**2/(2*amax) > upper + 1e-12):
                    raise ValueError('INITIAL_STATE_WITHOUT_BRAKING_ROOM')
        except (ValueError, AttributeError, TypeError) as error:
            raise ValueError('INVALID_INITIAL_VIEWPOINT') from error
        self.config, self.state = config, initial_state
        self.candidates = tuple((float(d), float(b)) for d in config.candidate_distances
                                for b in config.candidate_betas)
        grid = np.asarray(self.candidates)
        initial_error = ((grid[:, 0]-initial_state.d)**2 + config.preferred_distance**2
                         * (grid[:, 1]-initial_state.beta)**2)
        self._index = int(np.argmin(initial_error))
        self._since_switch = config.minimum_dwell_time
        self._fingerprint = config.fingerprint()
        self.profile = _make_profile(initial_state, self.candidates[self._index], config)
        derivatives = (initial_state.d_dot, initial_state.d_ddot,
                       initial_state.beta_dot, initial_state.beta_ddot)
        if all(abs(x) < 1e-12 for x in derivatives):
            if self.candidates[self._index] == (initial_state.d, initial_state.beta):
                self.profile = ViewpointProfile(self.profile.coefficients,
                                                self.profile.duration, self.profile.duration)

    def forecast(self, time):
        """Sample the same bounded C2 profile across and beyond the planning window."""
        if not math.isfinite(time) or not 0. <= time <= self.config.maximum_forecast_time:
            raise ValueError('INVALID_VIEWPOINT_FORECAST_TIME')
        return self.profile.forecast(time)

    def reference_profile(self):
        """Return immutable-request friendly metadata at the last real select epoch."""
        return dict(duration=self.profile.duration, elapsed=self.profile.elapsed,
                    distance_coefficients=self.profile.coefficients[0],
                    beta_coefficients=self.profile.coefficients[1])

    def _report(self, valid, reason, switched=False, scores=()):
        return AdaptiveViewpointReport(valid, reason, self.state, self.candidates[self._index],
                                       switched, self._index, tuple(scores), self._fingerprint)

    def select(self, dt, candidate_risks, *, candidate_errors=None,
               candidate_accelerations=None, candidate_jerks=None):
        """
        Advance selection using nonnegative parallel per-candidate metrics.

        Risk and optional metrics share candidate ordering. Acceleration and jerk
        metrics are squared-norm or other declared nonnegative penalties. Follow
        error always includes deviation from the preferred rear observation point.
        """
        if (not isinstance(dt, Real) or isinstance(dt, bool) or not math.isfinite(dt)
                or not 0. < dt <= self.config.maximum_dt):
            return self._report(False, 'INVALID_SELECTION_DT')
        # First advance the previously chosen action to the current real epoch.
        # New information cannot retrospectively change the preceding interval.
        self.profile = ViewpointProfile(self.profile.coefficients, self.profile.duration,
                                        min(self.profile.duration, self.profile.elapsed+dt))
        self.state = self.profile.forecast(0.)
        self._since_switch += dt
        try:
            risks = np.asarray(candidate_risks, dtype=float)
            if risks.shape != (len(self.candidates),) or np.any(np.isnan(risks)):
                raise ValueError('INVALID_CANDIDATE_METRICS')
            if np.any(risks < 0.):
                raise ValueError('INVALID_CANDIDATE_METRICS')
            optional = []
            for metric in (candidate_errors, candidate_accelerations, candidate_jerks):
                values = (np.zeros(len(risks)) if metric is None
                          else np.asarray(metric, dtype=float))
                if (values.shape != risks.shape or not np.all(np.isfinite(values))
                        or np.any(values < 0.)):
                    raise ValueError('INVALID_CANDIDATE_METRICS')
                optional.append(values)
        except (ValueError, TypeError):
            return self._report(False, 'INVALID_CANDIDATE_METRICS')
        if not np.any(np.isfinite(risks)):
            return self._report(False, 'NO_FEASIBLE_VIEWPOINT')
        config, grid = self.config, np.asarray(self.candidates)
        d, beta = grid.T
        preferred = ((d-config.preferred_distance)**2
                     + 2*d*config.preferred_distance*(1-np.cos(beta)))
        smooth = (d-self.state.d)**2 + config.preferred_distance**2*(beta-self.state.beta)**2
        switched_penalty = np.arange(len(grid)) != self._index
        weighted_risks = config.risk_weight*np.where(np.isfinite(risks), risks, 0.)
        scores = (weighted_risks + config.follow_error_weight*(preferred+optional[0])
                  + config.smooth_weight*smooth + config.switch_weight*switched_penalty
                  + config.acceleration_weight*optional[1] + config.jerk_weight*optional[2])
        scores[~np.isfinite(risks)] = math.inf
        best = int(np.argmin(scores))
        elapsed = self._since_switch
        incumbent_safe = math.isfinite(scores[self._index])
        moving = self.profile.elapsed < self.profile.duration
        if moving and not incumbent_safe:
            return self._report(False, 'ACTIVE_VIEWPOINT_BECAME_INFEASIBLE', scores=scores)
        if (best != self._index and incumbent_safe
                and (elapsed < config.minimum_dwell_time
                     or moving
                     or scores[best] + config.hysteresis_cost >= scores[self._index])):
            best = self._index
        selected = grid[best]
        try:
            profile = (self.profile if best == self._index
                       else _make_profile(self.state, selected, config))
            state = profile.forecast(0.)
        except ValueError as error:
            return self._report(False, str(error), scores=scores)
        switched = best != self._index
        self._index = best
        self._since_switch = 0. if switched else elapsed
        self.state = state
        self.profile = profile
        return self._report(True, 'OK', switched, scores)
