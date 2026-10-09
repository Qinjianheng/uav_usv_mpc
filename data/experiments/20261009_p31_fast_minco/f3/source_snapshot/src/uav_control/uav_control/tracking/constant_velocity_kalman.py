"""Constant-velocity Kalman filter for three-dimensional target tracking."""

import numpy as np


class ConstantVelocityKalmanFilter:
    """Estimate 3-D position and velocity from position measurements."""

    STATE_SIZE = 6
    POSITION_SIZE = 3

    def __init__(
        self,
        process_acceleration_std,
        measurement_position_std,
        initial_velocity_std,
    ):
        self.process_acceleration_std = max(
            float(process_acceleration_std),
            1e-6,
        )
        self.measurement_position_std = max(
            float(measurement_position_std),
            1e-6,
        )
        self.initial_velocity_std = max(
            float(initial_velocity_std),
            1e-6,
        )
        self.state = np.zeros(self.STATE_SIZE, dtype=float)
        self.covariance = np.eye(self.STATE_SIZE, dtype=float)
        self.initialized = False

        self.measurement_matrix = np.zeros(
            (self.POSITION_SIZE, self.STATE_SIZE),
            dtype=float,
        )
        self.measurement_matrix[:, :self.POSITION_SIZE] = np.eye(3)
        self.measurement_covariance = (
            self.measurement_position_std ** 2 * np.eye(3)
        )

    def initialize(
        self,
        position,
        measurement_covariance=None,
    ):
        measurement = self._validated_position(position)
        covariance = self._validated_measurement_covariance(
            measurement_covariance
        )

        self.state.fill(0.0)
        self.state[:self.POSITION_SIZE] = measurement

        velocity_variance = self.initial_velocity_std ** 2
        self.covariance = np.zeros(
            (self.STATE_SIZE, self.STATE_SIZE),
            dtype=float,
        )
        self.covariance[:3, :3] = covariance
        self.covariance[3:, 3:] = (
            velocity_variance * np.eye(3)
        )
        self.initialized = True

    def predict(self, dt):
        self._require_initialized()
        dt = float(dt)
        if not np.isfinite(dt) or dt < 0.0:
            raise ValueError('dt must be finite and non-negative')
        if dt == 0.0:
            return

        transition = self.transition_matrix(dt)
        self.state = transition @ self.state
        self.covariance = (
            transition @ self.covariance @ transition.T
            + self.process_covariance(dt)
        )
        self.covariance = 0.5 * (
            self.covariance + self.covariance.T
        )

    def update(
        self,
        position,
        measurement_covariance=None,
    ):
        self._require_initialized()
        measurement = self._validated_position(position)
        covariance = self._validated_measurement_covariance(
            measurement_covariance
        )

        innovation = (
            measurement
            - self.measurement_matrix @ self.state
        )
        innovation_covariance = (
            self.measurement_matrix
            @ self.covariance
            @ self.measurement_matrix.T
            + covariance
        )
        kalman_gain = np.linalg.solve(
            innovation_covariance,
            self.measurement_matrix @ self.covariance,
        ).T

        self.state = self.state + kalman_gain @ innovation

        identity = np.eye(self.STATE_SIZE)
        correction = (
            identity
            - kalman_gain @ self.measurement_matrix
        )
        self.covariance = (
            correction
            @ self.covariance
            @ correction.T
            + kalman_gain
            @ covariance
            @ kalman_gain.T
        )
        self.covariance = 0.5 * (
            self.covariance + self.covariance.T
        )

    def predicted_position(self, horizon):
        predicted_state, _ = self.project(horizon)
        return predicted_state[:self.POSITION_SIZE].copy()

    def project(self, horizon):
        self._require_initialized()
        horizon = float(horizon)
        if not np.isfinite(horizon) or horizon < 0.0:
            raise ValueError('horizon must be finite and non-negative')
        transition = self.transition_matrix(horizon)
        projected_state = transition @ self.state
        projected_covariance = (
            transition @ self.covariance @ transition.T
            + self.process_covariance(horizon)
        )
        return projected_state, projected_covariance

    @staticmethod
    def transition_matrix(dt):
        transition = np.eye(6)
        transition[0, 3] = dt
        transition[1, 4] = dt
        transition[2, 5] = dt
        return transition

    def process_covariance(self, dt):
        acceleration_variance = self.process_acceleration_std ** 2
        position_variance = 0.25 * dt ** 4 * acceleration_variance
        cross_covariance = 0.5 * dt ** 3 * acceleration_variance
        velocity_variance = dt ** 2 * acceleration_variance
        covariance = np.zeros((6, 6), dtype=float)

        for axis in range(3):
            velocity_axis = axis + 3
            covariance[axis, axis] = position_variance
            covariance[axis, velocity_axis] = cross_covariance
            covariance[velocity_axis, axis] = cross_covariance
            covariance[velocity_axis, velocity_axis] = velocity_variance

        return covariance

    def _validated_measurement_covariance(self, covariance):
        if covariance is None:
            return self.measurement_covariance.copy()

        matrix = np.asarray(covariance, dtype=float)
        if matrix.shape == (9,):
            matrix = matrix.reshape((3, 3))

        if matrix.shape != (3, 3):
            raise ValueError(
                'measurement covariance must be 3x3'
            )
        if not np.all(np.isfinite(matrix)):
            raise ValueError(
                'measurement covariance must be finite'
            )

        matrix = 0.5 * (matrix + matrix.T)
        if np.linalg.eigvalsh(matrix).min() < -1e-12:
            raise ValueError(
                'measurement covariance must be '
                'positive semidefinite'
            )
        return matrix

    @staticmethod
    def _validated_position(position):
        measurement = np.asarray(position, dtype=float)
        if measurement.shape != (3,):
            raise ValueError('position must contain exactly three values')
        if not np.all(np.isfinite(measurement)):
            raise ValueError('position must contain only finite values')
        return measurement

    def _require_initialized(self):
        if not self.initialized:
            raise RuntimeError('filter has not been initialized')
