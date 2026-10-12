"""
Lightweight minimum-control polynomial trajectories.

This module implements the fixed-waypoint, fixed-duration MINCO mapping for
third-order integrator dynamics.  Each trajectory piece is a quintic
polynomial and minimizes integrated squared jerk while satisfying complete
position/velocity/acceleration boundary states and intermediate positions.
"""

import math
from collections import OrderedDict
from dataclasses import dataclass

import numpy as np


@dataclass(frozen=True)
class MincoTrajectorySample:
    """Position and derivatives sampled from a MINCO trajectory."""

    position: tuple
    velocity: tuple
    acceleration: tuple
    jerk: tuple


class MincoS3Trajectory:
    """Piecewise-quintic minimum-jerk trajectory in three dimensions."""

    polynomial_degree = 5
    _mapping_cache = OrderedDict()
    _mapping_cache_limit = 256

    def __init__(
        self,
        start_position,
        start_velocity,
        start_acceleration,
        end_position,
        end_velocity,
        end_acceleration,
        intermediate_positions=(),
        durations=(1.0,),
    ):
        """Map boundary states, waypoints, and durations to coefficients."""
        self.durations = self._durations(durations)
        self.piece_count = len(self.durations)
        intermediate_positions = tuple(
            self._vector(position, 'intermediate position')
            for position in intermediate_positions
        )
        if len(intermediate_positions) != self.piece_count - 1:
            raise ValueError(
                'intermediate position count must equal piece count minus one'
            )

        start_position = self._vector(start_position, 'start position')
        start_velocity = self._vector(start_velocity, 'start velocity')
        start_acceleration = self._vector(
            start_acceleration,
            'start acceleration',
        )
        end_position = self._vector(end_position, 'end position')
        end_velocity = self._vector(end_velocity, 'end velocity')
        end_acceleration = self._vector(end_acceleration, 'end acceleration')

        matrix, values = self._minimum_control_system(
            start_position,
            start_velocity,
            start_acceleration,
            end_position,
            end_velocity,
            end_acceleration,
            intermediate_positions,
        )
        cache_key = self.durations
        mapping = self._mapping_cache.get(cache_key)
        if mapping is None:
            try:
                # For the small fixed-piece online problem, cache the linear
                # map after the same nonsingular MINCO system is prewarmed.
                # Residual checking below guards numerical conditioning.
                mapping = np.linalg.inv(matrix)
            except np.linalg.LinAlgError as error:
                raise ValueError(
                    'MINCO coefficient mapping is singular'
                ) from error
            self._mapping_cache[cache_key] = mapping
            if len(self._mapping_cache) > self._mapping_cache_limit:
                self._mapping_cache.popitem(last=False)
        else:
            self._mapping_cache.move_to_end(cache_key)
        coefficients = mapping @ values
        residual = np.max(np.abs(matrix @ coefficients - values))
        if not np.all(np.isfinite(coefficients)) or residual > 1e-6:
            raise ValueError(
                'MINCO coefficient mapping is numerically unstable'
            )
        self.coefficients = coefficients.reshape(self.piece_count, 6, 3)
        self.duration = float(sum(self.durations))
        self._cumulative_times = np.cumsum(self.durations)
        # Python-float mirrors of the immutable geometry.  ``sample`` is the
        # hottest function in the planning loop, where rebuilding numpy
        # scalars and calling np.searchsorted for every sample dominated the
        # cost of reachability and dense constraint validation.
        self._piece_start_times = (0.0,) + tuple(
            float(value) for value in self._cumulative_times[:-1]
        )
        self._sample_coefficients = tuple(
            tuple(
                tuple(float(value) for value in power)
                for power in piece
            )
            for piece in self.coefficients
        )

    @staticmethod
    def durations_from_logits(total_duration, logits):
        """
        Map Euclidean variables to positive durations with fixed sum.

        This is the fixed-total-time diffeomorphism used by GCOPTER.  The last
        logit is anchored at zero to remove the softmax translation ambiguity.
        """
        total_duration = float(total_duration)
        logits = np.asarray(tuple(float(value) for value in logits), dtype=float)
        if (
            not math.isfinite(total_duration)
            or total_duration <= 0.0
            or not np.all(np.isfinite(logits))
        ):
            raise ValueError('duration logits and total duration must be finite')
        augmented = np.concatenate((logits, np.zeros(1, dtype=float)))
        augmented -= np.max(augmented)
        weights = np.exp(augmented)
        durations = total_duration * weights / np.sum(weights)
        if not np.all(durations > 0.0):
            raise ValueError('duration transformation must remain positive')
        return tuple(float(value) for value in durations)

    @staticmethod
    def map_to_ball(center, radius, latent):
        """Smoothly map an unconstrained vector into a closed spatial ball."""
        center = np.asarray(tuple(float(value) for value in center), dtype=float)
        latent = np.asarray(tuple(float(value) for value in latent), dtype=float)
        radius = float(radius)
        if (
            center.shape != (3,)
            or latent.shape != (3,)
            or not np.all(np.isfinite(center))
            or not np.all(np.isfinite(latent))
            or not math.isfinite(radius)
            or radius < 0.0
        ):
            raise ValueError('ball mapping requires finite 3-D vectors and radius')
        squared_norm = float(np.dot(latent, latent))
        mapped = center + 2.0 * radius * latent / (squared_norm + 1.0)
        return tuple(float(value) for value in mapped)

    @staticmethod
    def _vector(values, name):
        vector = tuple(float(value) for value in values)
        if (
            len(vector) != 3
            or not all(math.isfinite(value) for value in vector)
        ):
            raise ValueError(f'{name} must contain three finite values')
        return vector

    @staticmethod
    def _durations(values):
        durations = tuple(float(value) for value in values)
        if not durations or not all(
            math.isfinite(value) and value > 0.0 for value in durations
        ):
            raise ValueError('durations must contain positive finite values')
        return durations

    @staticmethod
    def _basis(time, derivative):
        row = np.zeros(6, dtype=float)
        for power in range(derivative, 6):
            multiplier = math.factorial(power) / math.factorial(
                power - derivative
            )
            row[power] = multiplier * time ** (power - derivative)
        return row

    def _minimum_control_system(
        self,
        start_position,
        start_velocity,
        start_acceleration,
        end_position,
        end_velocity,
        end_acceleration,
        intermediate_positions,
    ):
        """Build the square MINCO map using optimal junction conditions."""
        size = 6 * self.piece_count
        matrix = np.zeros((size, size), dtype=float)
        values = np.zeros((size, 3), dtype=float)
        row = 0

        for derivative, boundary in enumerate((
            start_position,
            start_velocity,
            start_acceleration,
        )):
            matrix[row, 0:6] = self._basis(0.0, derivative)
            values[row] = boundary
            row += 1

        final_offset = 6 * (self.piece_count - 1)
        final_duration = self.durations[-1]
        for derivative, boundary in enumerate((
            end_position,
            end_velocity,
            end_acceleration,
        )):
            matrix[row, final_offset:final_offset + 6] = self._basis(
                final_duration,
                derivative,
            )
            values[row] = boundary
            row += 1

        for junction, waypoint in enumerate(intermediate_positions):
            left_offset = 6 * junction
            right_offset = left_offset + 6
            left_duration = self.durations[junction]

            # The waypoint is imposed on both adjacent pieces.  Continuity of
            # derivatives one through four is the optimality condition of the
            # minimum integrated squared-jerk problem.
            matrix[row, left_offset:left_offset + 6] = self._basis(
                left_duration,
                0,
            )
            values[row] = waypoint
            row += 1
            matrix[row, right_offset:right_offset + 6] = self._basis(0.0, 0)
            values[row] = waypoint
            row += 1
            for derivative in range(1, 5):
                matrix[row, left_offset:left_offset + 6] = self._basis(
                    left_duration,
                    derivative,
                )
                matrix[row, right_offset:right_offset + 6] = -self._basis(
                    0.0,
                    derivative,
                )
                row += 1

        if row != size:
            raise RuntimeError('incorrect MINCO system dimension')
        return matrix, values

    def _piece_and_local_time(self, time):
        time = float(time)
        if not math.isfinite(time):
            raise ValueError('sample time must be finite')
        time = min(max(time, 0.0), self.duration)
        # Plain scan over at most six piece boundaries.  It reproduces
        # np.searchsorted(..., side='right') exactly, including the tie at a
        # shared boundary, without the per-sample numpy call overhead.
        piece = 0
        start_time = 0.0
        for index in range(1, self.piece_count):
            boundary = self._piece_start_times[index]
            if time < boundary:
                break
            piece = index
            start_time = boundary
        return piece, time - start_time

    def sample(self, time):
        """Sample position, velocity, acceleration, and jerk."""
        piece, local_time = self._piece_and_local_time(time)
        c0, c1, c2, c3, c4, c5 = self._sample_coefficients[piece]
        time2 = local_time * local_time
        time3 = time2 * local_time
        time4 = time3 * local_time
        time5 = time4 * local_time
        # Same left-associative evaluation order as the vectorized form, so
        # the returned values are bit-identical.
        position = tuple(
            a + b * local_time + c * time2 + d * time3
            + e * time4 + f * time5
            for a, b, c, d, e, f in zip(c0, c1, c2, c3, c4, c5)
        )
        velocity = tuple(
            b + 2.0 * c * local_time + 3.0 * d * time2
            + 4.0 * e * time3 + 5.0 * f * time4
            for b, c, d, e, f in zip(c1, c2, c3, c4, c5)
        )
        acceleration = tuple(
            2.0 * c + 6.0 * d * local_time + 12.0 * e * time2
            + 20.0 * f * time3
            for c, d, e, f in zip(c2, c3, c4, c5)
        )
        jerk = tuple(
            6.0 * d + 24.0 * e * local_time + 60.0 * f * time2
            for d, e, f in zip(c3, c4, c5)
        )
        return MincoTrajectorySample(
            position,
            velocity,
            acceleration,
            jerk,
        )

    def quadrature_samples(self, intervals_per_piece):
        """Yield trapezoidal samples with their integration weights."""
        intervals_per_piece = int(intervals_per_piece)
        if intervals_per_piece < 1:
            raise ValueError('quadrature intervals per piece must be positive')
        piece_start = 0.0
        for duration in self.durations:
            step = duration / intervals_per_piece
            for index in range(intervals_per_piece + 1):
                weight = 0.5 if index in (0, intervals_per_piece) else 1.0
                time = piece_start + index * step
                yield self.sample(time), weight * step
            piece_start += duration

    def control_effort(self):
        """Return the exact integral of squared jerk over all pieces."""
        effort = 0.0
        for duration, coefficients in zip(self.durations, self.coefficients):
            c3, c4, c5 = coefficients[3], coefficients[4], coefficients[5]
            effort += float(
                36.0 * np.dot(c3, c3) * duration
                + 72.0 * np.dot(c3, c4) * duration**2
                + (
                    120.0 * np.dot(c3, c5)
                    + 192.0 * np.dot(c4, c4)
                ) * duration**3
                + 360.0 * np.dot(c4, c5) * duration**4
                + 720.0 * np.dot(c5, c5) * duration**5
            )
        return max(effort, 0.0)
