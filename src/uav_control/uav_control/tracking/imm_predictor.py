"""
Raw-position CV/CTRV/CTRA interacting multiple-model EKF, without ROS.

The common horizontal state is [x, y, vx, vy, omega, a_long, alpha].
Transition entry M[i,j] means P(next model=j | current model=i).
Angles are never mixed: heading is represented by Cartesian velocity.
"""

from dataclasses import dataclass, field
import math

import numpy as np


MODEL_NAMES = ('CV', 'CTRV', 'CTRA')
_IDENTITY = np.eye(7)
_QUADRATURE_NODES, _QUADRATURE_WEIGHTS = np.polynomial.legendre.leggauss(12)
_QUADRATURE_NODES = (_QUADRATURE_NODES + 1.) / 2.
_QUADRATURE_WEIGHTS = _QUADRATURE_WEIGHTS / 2.
_NOISE_NODES, _NOISE_WEIGHTS = np.polynomial.legendre.leggauss(4)
_NOISE_NODES = (_NOISE_NODES + 1.) / 2.
_NOISE_WEIGHTS = _NOISE_WEIGHTS / 2.


@dataclass(frozen=True)
class ImmConfig:
    """
    Process noise is spectral standard deviation, not an error bound.

    Horizontal acceleration noise: m/s^2/sqrt(Hz); turn-rate random walk:
    rad/s/sqrt(s); longitudinal/angular jerk: m/s^2/sqrt(s), rad/s^2/sqrt(s).
    These defaults are engineering priors and require independent calibration.
    """

    transition_matrix: object = field(default_factory=lambda: np.array(
        ((.96, .025, .015), (.025, .95, .025), (.015, .035, .95))))
    initial_probabilities: object = (.5, .3, .2)
    acceleration_noise: object = (.7, .5, .35)
    turn_rate_noise: object = (0., .18, .08)
    longitudinal_jerk_noise: float = .6
    turn_jerk_noise: float = .3
    initial_velocity_std: float = 3.
    initial_turn_rate_std: float = .35
    initial_acceleration_std: float = 1.
    initial_turn_acceleration_std: float = .5
    maximum_turn_rate: float = .7
    maximum_longitudinal_acceleration: float = 2.
    maximum_turn_acceleration: float = 1.5
    maximum_horizontal_speed: float = 5.
    covariance_floor: float = 1e-9
    max_update_gap: float = .5
    vertical_acceleration_noise: float = .5
    vertical_velocity_decay_time: float = .75
    maximum_vertical_displacement: float = .5
    maximum_vertical_speed: float = 1.

    def __post_init__(self):
        matrix = np.asarray(self.transition_matrix, dtype=float)
        probabilities = np.asarray(self.initial_probabilities, dtype=float)
        if (matrix.shape != (3, 3) or not np.isfinite(matrix).all()
                or (matrix < 0.).any()
                or not np.allclose(matrix.sum(axis=1), 1., atol=1e-12, rtol=0.)):
            raise ValueError('transition_matrix must have three stochastic rows')
        if (probabilities.shape != (3,) or not np.isfinite(probabilities).all()
                or (probabilities < 0.).any() or probabilities.sum() <= 0.):
            raise ValueError('initial_probabilities must be nonnegative with positive sum')
        for name in ('acceleration_noise', 'turn_rate_noise'):
            values = np.asarray(getattr(self, name), dtype=float)
            if values.shape != (3,) or not np.isfinite(values).all() or (values < 0.).any():
                raise ValueError(f'{name} must contain three nonnegative finite values')
            object.__setattr__(self, name, tuple(values))
        nonnegative = ('longitudinal_jerk_noise', 'turn_jerk_noise',
                       'vertical_acceleration_noise')
        positive = ('initial_velocity_std', 'initial_turn_rate_std',
                    'initial_acceleration_std', 'initial_turn_acceleration_std',
                    'maximum_turn_rate', 'maximum_longitudinal_acceleration',
                    'maximum_turn_acceleration',
                    'maximum_horizontal_speed',
                    'covariance_floor', 'max_update_gap',
                    'vertical_velocity_decay_time', 'maximum_vertical_displacement',
                    'maximum_vertical_speed')
        for name in nonnegative + positive:
            value = float(getattr(self, name))
            if not math.isfinite(value) or value < 0. or (name in positive and value == 0.):
                raise ValueError(f'{name} has an invalid value')
            object.__setattr__(self, name, value)
        matrix = matrix.copy()
        matrix.setflags(write=False)
        object.__setattr__(self, 'transition_matrix', matrix)
        object.__setattr__(self, 'initial_probabilities', tuple(
            probabilities / probabilities.sum()))


@dataclass(frozen=True)
class UpdateStatus:
    accepted: bool
    reason: str = ''


@dataclass(frozen=True)
class ImmForecast:
    source_stamp: float
    observation_stamp: float
    horizons: np.ndarray
    position: np.ndarray
    velocity: np.ndarray
    acceleration: np.ndarray
    position_covariance: np.ndarray
    model_probabilities: np.ndarray
    state: np.ndarray
    state_covariance: np.ndarray
    conditional_position: np.ndarray
    conditional_velocity: np.ndarray
    conditional_acceleration: np.ndarray
    conditional_position_covariance: np.ndarray
    model_disagreement: np.ndarray


def _symmetric_psd(covariance, floor):
    """Symmetrize roundoff; repair only eigenvalues below the stated floor."""
    covariance = (covariance + np.swapaxes(covariance, -1, -2)) * .5
    if not np.isfinite(covariance).all():
        raise FloatingPointError('nonfinite covariance')
    eigenvalues, eigenvectors = np.linalg.eigh(covariance)
    if np.any(eigenvalues < floor):
        covariance = ((eigenvectors * np.maximum(eigenvalues, floor)[..., None, :])
                      @ np.swapaxes(eigenvectors, -1, -2))
        covariance = (covariance + np.swapaxes(covariance, -1, -2)) * .5
    return covariance


def mix_models(states, covariances, probabilities, transition_matrix):
    """
    Markov interaction including covariance of the component means.

    c[j] = sum_i mu[i] M[i,j]; mixing[i,j] = mu[i] M[i,j] / c[j].
    A destination with exactly zero prior probability keeps its conditional
    state for diagnostics, with zero posterior support; it is never revived
    by an arbitrary probability floor or a uniform reset.
    """
    states = np.asarray(states, dtype=float)
    covariances = np.asarray(covariances, dtype=float)
    joint = np.asarray(probabilities)[:, None] * np.asarray(transition_matrix)
    prior = joint.sum(axis=0)
    weights = np.divide(joint, prior[None, :], out=np.zeros_like(joint),
                        where=prior[None, :] > 0.)
    mixed_states = weights.T @ states
    differences = states[:, None, :] - mixed_states[None, :, :]
    mixed_covariances = np.einsum(
        'ij,ijkl->jkl', weights, covariances[:, None, :, :]
        + differences[..., :, None] * differences[..., None, :])
    unsupported = prior == 0.
    mixed_states[unsupported] = states[unsupported]
    mixed_covariances[unsupported] = covariances[unsupported]
    return mixed_states, mixed_covariances, prior


def _transition_many(state, horizons, model):
    """Analytic Jacobian, 12-node Gauss quadrature of Cartesian kinematics."""
    state = np.asarray(state, dtype=float)
    times = np.asarray(horizons, dtype=float)
    count = len(times)
    output = np.repeat(state[None, :], count, axis=0)
    jacobian = np.repeat(_IDENTITY[None, :, :], count, axis=0)
    if model == 'CV':
        output[:, :2] += times[:, None] * state[2:4]
        output[:, 4:] = 0.
        jacobian[:, 0, 2] = times
        jacobian[:, 1, 3] = times
        jacobian[:, 4:, :] = 0.
        return output, jacobian
    if model == 'CTRV':
        # Closed-form constant-turn motion avoids nested quadrature in both
        # the forecast and its process-noise integral. Series derivatives
        # preserve omega observability as omega approaches zero.
        omega = state[4]
        angle = omega * times
        sine, cosine = np.sin(angle), np.cos(angle)
        s = times * np.sinc(angle / np.pi)
        c = .5 * omega * times ** 2 * np.sinc(angle / (2. * np.pi)) ** 2
        small = np.abs(angle) < 1e-3
        ds = np.empty_like(times)
        dc = np.empty_like(times)
        ds[small] = (-omega * times[small] ** 3 / 3.
                     + omega ** 3 * times[small] ** 5 / 30.
                     - omega ** 5 * times[small] ** 7 / 840.)
        dc[small] = (times[small] ** 2 / 2. - omega ** 2 * times[small] ** 4 / 8.
                     + omega ** 4 * times[small] ** 6 / 144.)
        if np.any(~small):
            ds[~small] = (angle[~small] * cosine[~small] - sine[~small]) / omega ** 2
            dc[~small] = (angle[~small] * sine[~small] - (1. - cosine[~small])) / omega ** 2
        vx, vy = state[2:4]
        output[:, 0] += s * vx - c * vy
        output[:, 1] += c * vx + s * vy
        output[:, 2] = cosine * vx - sine * vy
        output[:, 3] = sine * vx + cosine * vy
        output[:, 5:] = 0.
        jacobian[:, 0, 2] = jacobian[:, 1, 3] = s
        jacobian[:, 0, 3] = -c
        jacobian[:, 1, 2] = c
        jacobian[:, 0, 4] = ds * vx - dc * vy
        jacobian[:, 1, 4] = dc * vx + ds * vy
        jacobian[:, 2, 2] = jacobian[:, 3, 3] = cosine
        jacobian[:, 2, 3] = -sine
        jacobian[:, 3, 2] = sine
        jacobian[:, 2, 4] = -times * output[:, 3]
        jacobian[:, 3, 4] = times * output[:, 2]
        jacobian[:, 5:, :] = 0.
        return output, jacobian
    if model not in MODEL_NAMES:
        raise ValueError('unknown IMM motion model')

    velocity = state[2:4]
    speed = math.hypot(*velocity)
    # At rest longitudinal direction is unobservable. No arbitrary yaw or
    # acceleration direction is manufactured. Velocity uncertainty remains.
    # A tiny Cartesian direction cannot identify a positive acceleration
    # heading: its 1/s Jacobian would manufacture an arbitrarily strong turn
    # direction from roundoff. Braking still applies at any nonzero speed so
    # propagation through a sub-threshold stopping state remains a flow.
    moving = speed > 1e-3 or (speed > 0. and state[5] < 0.)
    unit = velocity / speed if moving else np.zeros(2)
    unit_jacobian = ((np.eye(2) - np.outer(unit, unit)) / speed
                     if moving else np.zeros((2, 2)))
    omega = state[4]
    acceleration = state[5] if model == 'CTRA' and moving else 0.
    alpha = state[6] if model == 'CTRA' else 0.
    # Longitudinal braking stops at zero speed. Negative scalar speed would
    # reverse Cartesian heading and change the meaning of a_long on the next
    # propagation, breaking both the flow and its reported acceleration.
    stop_time = -speed / acceleration if acceleration < 0. else math.inf
    integration_times = np.minimum(times, stop_time)
    stopped = times >= stop_time

    def kinematics(t):
        angle = omega * t + .5 * alpha * t * t
        cosine, sine = np.cos(angle), np.sin(angle)
        rotation = np.empty(t.shape + (2, 2))
        rotation[..., 0, 0] = cosine
        rotation[..., 0, 1] = -sine
        rotation[..., 1, 0] = sine
        rotation[..., 1, 1] = cosine
        longitudinal = velocity + acceleration * t[..., None] * unit
        rotated = np.einsum('...ij,...j->...i', rotation, longitudinal)
        derivative_v = rotation @ (
            np.eye(2) + acceleration * t[..., None, None] * unit_jacobian)
        normal = np.stack((-rotated[..., 1], rotated[..., 0]), axis=-1)
        derivative_omega = t[..., None] * normal
        derivative_a = t[..., None] * np.einsum('...ij,j->...i', rotation, unit)
        derivative_alpha = .5 * t[..., None] ** 2 * normal
        return rotated, derivative_v, derivative_omega, derivative_a, derivative_alpha

    t_quad = integration_times[:, None] * _QUADRATURE_NODES
    values = kinematics(t_quad)
    integral = [np.einsum('nq...,q,n->n...', value, _QUADRATURE_WEIGHTS, integration_times)
                for value in values]
    endpoint = kinematics(times)
    output[:, :2] += integral[0]
    output[:, 2:4] = endpoint[0]
    output[stopped, 2:4] = 0.
    output[:, 4] = omega + alpha * times
    jacobian[:, :2, 2:4] = integral[1]
    jacobian[:, 2:4, 2:4] = endpoint[1]
    jacobian[:, :2, 4] = integral[2]
    jacobian[:, 2:4, 4] = endpoint[2]
    if model == 'CTRV':
        output[:, 5:] = 0.
        jacobian[:, 5:, :] = 0.
        jacobian[:, :4, 5:] = 0.
    else:
        jacobian[:, :2, 5] = integral[3]
        jacobian[:, 2:4, 5] = endpoint[3]
        jacobian[:, :2, 6] = integral[4]
        jacobian[:, 2:4, 6] = endpoint[4]
        jacobian[:, 4, 6] = times
    # The position boundary term is velocity(stop)*d(stop)=0. The velocity
    # map is non-differentiable exactly at stop; use its stopped/right branch
    # Jacobian, retain process uncertainty, and explicitly document this EKF
    # approximation rather than claiming a smooth transition there.
    jacobian[stopped, 2:4, :] = 0.
    return output, jacobian


def _process_covariance(state, times, model, config):
    """
    PSD continuous-noise quadrature with frozen-posterior linearization.

    Q(t)=integral F(lag,x_post) L Qc L' F(lag,x_post)' dlag. This is exact
    for CV and the straight nominal CTRA integrator. Curved CTRA uses the
    same local EKF approximation as its Gaussian covariance; it is not an
    exact nonlinear stochastic transition law. In particular noise in a
    and alpha must affect future position within the interval, not only
    the final latent variance. Calibration must assess this approximation.
    """
    index = MODEL_NAMES.index(model)
    spectral = np.zeros(7)
    spectral[2:4] = config.acceleration_noise[index] ** 2
    spectral[4] = config.turn_rate_noise[index] ** 2
    if model == 'CTRA':
        spectral[5] = config.longitudinal_jerk_noise ** 2
        spectral[6] = config.turn_jerk_noise ** 2
    if not np.any(spectral):
        return np.zeros((len(times), 7, 7))
    _, jacobians = _transition_many(state, (times[:, None] * _NOISE_NODES).ravel(), model)
    jacobians = jacobians.reshape(len(times), len(_NOISE_NODES), 7, 7)
    effective = np.broadcast_to(spectral, (len(times), 1, 7)).copy()
    stopped = np.zeros(len(times), dtype=bool)
    if model == 'CTRA' and state[5] < 0.:
        stopped = times >= math.hypot(*state[2:4]) / -state[5]
        effective[stopped, 0, 2:4] = 0.
    covariance = ((jacobians * effective[:, None, :, :]) @ np.swapaxes(jacobians, -1, -2))
    integrated = np.einsum('nqij,q,n->nij', covariance, _NOISE_WEIGHTS, times)
    # A stopped deterministic mean cannot project away later Cartesian
    # acceleration noise. At this non-differentiable boundary retain a CV
    # additive driving covariance, including P/V cross terms that let new
    # position observations recover velocity. Latent noise is unchanged.
    stopped_times = times[stopped]
    variance = config.acceleration_noise[index] ** 2
    for position, velocity in ((0, 2), (1, 3)):
        integrated[stopped, position, position] += variance * stopped_times ** 3 / 3.
        integrated[stopped, velocity, velocity] += variance * stopped_times
        integrated[stopped, position, velocity] += variance * stopped_times ** 2 / 2.
        integrated[stopped, velocity, position] += variance * stopped_times ** 2 / 2.
    return integrated


def _propagate_many(state, covariance, times, model, config):
    future, jacobian = _transition_many(state, times, model)
    propagated = (jacobian @ covariance @ np.swapaxes(jacobian, -1, -2)
                  + _process_covariance(state, times, model, config))
    if not np.isfinite(future).all():
        raise FloatingPointError('nonfinite predicted state')
    return future, _symmetric_psd(propagated, config.covariance_floor)


def propagate_model(state, covariance, dt, model, config=None):
    """Pure one-model prediction for matching-initial-state comparisons."""
    config = config or ImmConfig()
    dt = float(dt)
    state = np.asarray(state, dtype=float)
    covariance = np.asarray(covariance, dtype=float)
    if (state.shape != (7,) or covariance.shape != (7, 7)
            or not np.isfinite(state).all() or not np.isfinite(covariance).all()
            or not math.isfinite(dt) or dt < 0.):
        raise ValueError('invalid model propagation input')
    future, propagated = _propagate_many(
        state, covariance, np.array((dt,)), model, config)
    return future[0], propagated[0]


def _fuse(states, covariances, probabilities):
    mean = np.einsum('m,m...s->...s', probabilities, states)
    differences = states - mean
    disagreement = np.einsum('m,m...i,m...j->...ij', probabilities,
                             differences, differences)
    covariance = np.einsum('m,m...ij->...ij', probabilities, covariances) + disagreement
    return mean, covariance, disagreement


class ImmPredictor:
    """
    One unique raw-position update per acquisition stamp.

    No ROS, truth, KF states, route, receipt-time derivative or online truth
    calibration is used. Forecast calls do not modify posterior filter state.
    """

    def __init__(self, config=None):
        self.config = config or ImmConfig()
        self.reset()

    def reset(self):
        """Clear a session on an explicit transport/mission clock reset."""
        self.stamp = None
        self.states = None
        self.covariances = None
        self.probabilities = np.array(self.config.initial_probabilities, dtype=float)
        self.vertical_state = None
        self.vertical_covariance = None
        self.last_log_likelihoods = None
        self.update_count = 0
        self.projection_count = 0

    def _initialize(self, stamp, position, covariance):
        self.states = np.zeros((3, 7))
        self.states[:, :2] = position[:2]
        initial = np.diag((1., 1., self.config.initial_velocity_std ** 2,
                           self.config.initial_velocity_std ** 2,
                           self.config.initial_turn_rate_std ** 2,
                           self.config.initial_acceleration_std ** 2,
                           self.config.initial_turn_acceleration_std ** 2))
        initial[:2, :2] = covariance[:2, :2]
        self.covariances = np.repeat(initial[None, :, :], 3, axis=0)
        self.vertical_state = np.array((position[2], 0.))
        self.vertical_covariance = np.diag((covariance[2, 2],
                                            self.config.maximum_vertical_speed ** 2))
        self.stamp = stamp
        self.update_count += 1

    def update(self, stamp, position, covariance):
        """
        Update only at a strictly newer acquisition epoch.

        Invalid/duplicate/late data never changes the posterior. A genuine
        observation after an excessive gap starts a new uncertain track.
        Covariance xy/z correlations are not used by the separate vertical
        model; conservative block inflation retains their variance effect.
        """
        try:
            stamp = float(stamp)
            position = np.asarray(position, dtype=float)
            covariance = np.asarray(covariance, dtype=float)
            if covariance.size == 9:
                covariance = covariance.reshape(3, 3)
        except (ValueError, TypeError):
            return UpdateStatus(False, 'INVALID_OBSERVATION')
        if (not math.isfinite(stamp) or stamp <= 0. or position.shape != (3,)
                or not np.isfinite(position).all() or covariance.shape != (3, 3)
                or not np.isfinite(covariance).all()
                or not np.allclose(covariance, covariance.T, atol=1e-10, rtol=1e-10)):
            return UpdateStatus(False, 'INVALID_OBSERVATION')
        if np.linalg.eigvalsh(covariance).min() < -1e-10:
            return UpdateStatus(False, 'INVALID_OBSERVATION_COVARIANCE')
        if self.stamp is not None and stamp <= self.stamp:
            return UpdateStatus(False, 'DUPLICATE_OBSERVATION' if stamp == self.stamp
                                else 'OUT_OF_ORDER_OBSERVATION')
        covariance = _symmetric_psd(covariance, self.config.covariance_floor)
        # Removing xy/z cross terms is otherwise an unjustified confidence
        # change. Diagonal inflation by absolute discarded cross terms makes
        # the block-independent measurement covariance dominate the full R.
        cross = np.abs(covariance[:2, 2])
        covariance = covariance.copy()
        covariance[0, 0] += cross[0]
        covariance[1, 1] += cross[1]
        covariance[2, 2] += cross.sum()
        covariance[:2, 2] = covariance[2, :2] = 0.
        reason = ''
        if self.stamp is not None and stamp - self.stamp > self.config.max_update_gap:
            self.reset()
            reason = 'REINITIALIZED_AFTER_GAP'
        if self.stamp is None:
            self._initialize(stamp, position, covariance)
            return UpdateStatus(True, reason)
        dt = stamp - self.stamp
        states, covariances, prior = mix_models(
            self.states, self.covariances, self.probabilities,
            self.config.transition_matrix)
        posterior_states = np.empty_like(states)
        posterior_covariances = np.empty_like(covariances)
        projection_count = 0
        log_likelihoods = np.empty(3)
        try:
            for index, model in enumerate(MODEL_NAMES):
                predicted, predicted_covariance = propagate_model(
                    states[index], covariances[index], dt, model, self.config)
                innovation = position[:2] - predicted[:2]
                innovation_covariance = _symmetric_psd(
                    predicted_covariance[:2, :2] + covariance[:2, :2],
                    self.config.covariance_floor)
                gain = np.linalg.solve(innovation_covariance,
                                       predicted_covariance[:2, :]).T
                posterior_states[index] = predicted + gain @ innovation
                residual_projection = _IDENTITY.copy()
                residual_projection[:, :2] -= gain
                # Joseph form preserves PSD even with ill-conditioned R.
                posterior_covariances[index] = _symmetric_psd(
                    residual_projection @ predicted_covariance @ residual_projection.T
                    + gain @ covariance[:2, :2] @ gain.T,
                    self.config.covariance_floor)
                bounds = np.array((self.config.maximum_turn_rate,
                                   self.config.maximum_longitudinal_acceleration,
                                   self.config.maximum_turn_acceleration))
                projected = posterior_states[index].copy()
                speed = math.hypot(*projected[2:4])
                if speed > self.config.maximum_horizontal_speed:
                    projected[2:4] *= self.config.maximum_horizontal_speed / speed
                projected[4:] = np.clip(projected[4:], -bounds, bounds)
                shift = posterior_states[index] - projected
                if np.any(shift):
                    # Engineering prior on latent estimates, not a truth
                    # correction or covariance truncation. Retain the second
                    # moment around the shifted mean via P + delta delta'.
                    posterior_states[index] = projected
                    posterior_covariances[index] = _symmetric_psd(
                        posterior_covariances[index] + np.outer(shift, shift),
                        self.config.covariance_floor)
                    projection_count += 1
                _, log_determinant = np.linalg.slogdet(innovation_covariance)
                log_likelihoods[index] = -.5 * (
                    2. * math.log(2. * math.pi) + log_determinant
                    + innovation @ np.linalg.solve(innovation_covariance, innovation))
            support = prior > 0.
            # Subtract before adding log prior: enormous but tied residuals
            # must preserve the Markov prior despite floating-point spacing.
            relative_likelihood = log_likelihoods[support] - np.max(log_likelihoods[support])
            log_weights = relative_likelihood + np.log(prior[support])
            weights = np.exp(log_weights - np.max(log_weights))
            probabilities = np.zeros(3)
            probabilities[support] = weights / weights.sum()
            vertical_transition = np.array(((1., dt), (0., 1.)))
            vertical_q = self.config.vertical_acceleration_noise ** 2 * np.array(
                ((dt ** 3 / 3., dt ** 2 / 2.), (dt ** 2 / 2., dt)))
            vertical_state = vertical_transition @ self.vertical_state
            vertical_covariance = (vertical_transition @ self.vertical_covariance
                                   @ vertical_transition.T + vertical_q)
            vertical_gain = vertical_covariance[:, 0] / (
                vertical_covariance[0, 0] + covariance[2, 2])
            vertical_state += vertical_gain * (position[2] - vertical_state[0])
            vertical_state[1] = np.clip(vertical_state[1],
                                        -self.config.maximum_vertical_speed,
                                        self.config.maximum_vertical_speed)
            vertical_projection = np.eye(2)
            vertical_projection[:, 0] -= vertical_gain
            vertical_covariance = _symmetric_psd(
                vertical_projection @ vertical_covariance @ vertical_projection.T
                + np.outer(vertical_gain, vertical_gain) * covariance[2, 2],
                self.config.covariance_floor)
            if (not np.isfinite(posterior_states).all()
                    or not np.isfinite(probabilities).all()
                    or not np.isfinite(log_likelihoods).all()):
                raise FloatingPointError('nonfinite posterior')
        except (np.linalg.LinAlgError, FloatingPointError):
            return UpdateStatus(False, 'IMM_NUMERICAL_FAILURE')
        self.states = posterior_states
        self.covariances = posterior_covariances
        self.probabilities = probabilities
        self.vertical_state = vertical_state
        self.vertical_covariance = vertical_covariance
        self.last_log_likelihoods = log_likelihoods
        self.stamp = stamp
        self.update_count += 1
        self.projection_count += projection_count
        return UpdateStatus(True)

    def predict(self, horizons):
        """Forecast at the last acquisition epoch without mutating the filter."""
        if self.stamp is None:
            raise ValueError('NO_OBSERVATION')
        return self.forecast_at(self.stamp, horizons)

    def forecast_at(self, source_stamp, horizons):
        """
        Propagate from raw posterior to source_stamp and then each horizon.

        Model probabilities are held fixed for conditional futures: each
        component describes continued membership of its current model, not
        an unreported model-switching tree. Fusion includes between-model
        spread, and all three conditional trajectories remain available.
        """
        if self.stamp is None:
            raise ValueError('NO_OBSERVATION')
        source_stamp = float(source_stamp)
        horizons = np.asarray(horizons, dtype=float)
        if (horizons.ndim != 1 or len(horizons) == 0 or not np.isfinite(horizons).all()
                or (horizons < 0.).any() or not math.isfinite(source_stamp)
                or source_stamp < self.stamp):
            raise ValueError('invalid prediction epochs')
        elapsed = source_stamp - self.stamp
        # Include an explicit source state in one batch, even if no user
        # horizon is zero. Every requested horizon is relative to source.
        times = np.concatenate(((elapsed,), elapsed + horizons))
        states, covariances = [], []
        for index, model in enumerate(MODEL_NAMES):
            predicted, covariance = _propagate_many(
                self.states[index], self.covariances[index], times, model, self.config)
            states.append(predicted)
            covariances.append(covariance)
        states, covariances = np.asarray(states), np.asarray(covariances)
        fused_state, fused_covariance, _ = _fuse(
            states[:, 0], covariances[:, 0], self.probabilities)
        horizontal, horizontal_covariance, disagreement = _fuse(
            states[:, 1:], covariances[:, 1:], self.probabilities)
        vertical_time = times[1:]
        decay_time = self.config.vertical_velocity_decay_time
        vertical_coefficient = decay_time * (-np.expm1(-vertical_time / decay_time))
        displacement = self.vertical_state[1] * vertical_coefficient
        bounded_displacement = np.clip(displacement,
                                       -self.config.maximum_vertical_displacement,
                                       self.config.maximum_vertical_displacement)
        vertical_velocity = self.vertical_state[1] * np.exp(-vertical_time / decay_time)
        vertical_velocity[np.abs(displacement) > self.config.maximum_vertical_displacement] = 0.
        vertical_position = self.vertical_state[0] + bounded_displacement
        vertical_acceleration = -vertical_velocity / decay_time
        vertical_f = np.stack((np.ones_like(vertical_time), vertical_coefficient), axis=-1)
        vertical_variance = (
            np.einsum('ni,ij,nj->n', vertical_f, self.vertical_covariance, vertical_f)
            + self.config.vertical_acceleration_noise ** 2 * vertical_time ** 3 / 3.)
        count = len(horizons)
        conditional_position = np.zeros((3, count, 3))
        conditional_velocity = np.zeros_like(conditional_position)
        conditional_acceleration = np.zeros_like(conditional_position)
        conditional_covariance = np.zeros((3, count, 3, 3))
        conditional_position[:, :, :2] = states[:, 1:, :2]
        conditional_position[:, :, 2] = vertical_position
        conditional_velocity[:, :, :2] = states[:, 1:, 2:4]
        conditional_velocity[:, :, 2] = vertical_velocity
        conditional_covariance[:, :, :2, :2] = covariances[:, 1:, :2, :2]
        conditional_covariance[:, :, 2, 2] = vertical_variance
        for index in (1, 2):
            velocity = states[index, 1:, 2:4]
            speed = np.linalg.norm(velocity, axis=1)
            identified = (speed > 1e-3) | ((speed > 0.) & (states[index, 1:, 5] < 0.))
            unit = np.divide(velocity, speed[:, None], out=np.zeros_like(velocity),
                             where=identified[:, None])
            normal = np.stack((-velocity[:, 1], velocity[:, 0]), axis=-1)
            conditional_acceleration[index, :, :2] = states[index, 1:, 4, None] * normal
            if index == 2:
                conditional_acceleration[index, :, :2] += states[index, 1:, 5, None] * unit
        conditional_acceleration[:, :, 2] = vertical_acceleration
        position = np.column_stack((horizontal[:, :2], vertical_position))
        velocity = np.column_stack((horizontal[:, 2:4], vertical_velocity))
        acceleration = np.einsum('m,mni->ni', self.probabilities, conditional_acceleration)
        position_covariance = np.zeros((count, 3, 3))
        position_covariance[:, :2, :2] = horizontal_covariance[:, :2, :2]
        position_covariance[:, 2, 2] = vertical_variance
        model_disagreement = np.zeros_like(position_covariance)
        model_disagreement[:, :2, :2] = disagreement[:, :2, :2]
        if (not np.isfinite(acceleration).all()
                or not np.isfinite(conditional_acceleration).all()
                or not np.isfinite(position_covariance).all()):
            raise FloatingPointError('nonfinite forecast')
        return ImmForecast(source_stamp, self.stamp, horizons.copy(), position, velocity,
                           acceleration, position_covariance, self.probabilities.copy(),
                           fused_state, _symmetric_psd(
                               fused_covariance, self.config.covariance_floor),
                           conditional_position, conditional_velocity,
                           conditional_acceleration, conditional_covariance,
                           model_disagreement)
