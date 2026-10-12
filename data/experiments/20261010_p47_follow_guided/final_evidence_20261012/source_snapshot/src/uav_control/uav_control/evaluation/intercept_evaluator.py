"""Pure truth-only interception evaluation and event-based statistics."""

import csv
from collections import deque
from dataclasses import dataclass
from datetime import datetime
import json
import math
from pathlib import Path

from .uav_heading_diagnostics import UAV_HEADING_CSV_FIELDS


# Distance from the PX4 local-position reference to the lowest point of the
# vehicle body.  The X500 landing-gear skids reach 0.227 m below base_link, so
# a reference altitude of ``-body_lower_extent`` already puts the skids at the
# sea surface.  Testing "reference point above sea_surface_z" instead lets the
# vehicle fly with its gear submerged and never raises SEA_CONTACT, which is
# why a physical water strike was not terminating the run.
DEFAULT_BODY_LOWER_EXTENT = 0.23


def image_pose_diagnostics(observation):
    """Log bearing and PX4 heading from the existing image-time geometry."""
    values = {'image_bearing': math.nan, 'px4_heading': math.nan}
    if not observation.geometry_diagnostics_enabled:
        return values
    u, cx, fx = (
        float(getattr(observation, name))
        for name in ('mask_centroid_u', 'camera_cx', 'camera_fx')
    )
    if all(math.isfinite(value) for value in (u, cx, fx)) and fx > 0.0:
        values['image_bearing'] = math.atan((u - cx) / fx)
    quaternion = tuple(float(getattr(observation, name)) for name in (
        'interpolated_attitude_w', 'interpolated_attitude_x',
        'interpolated_attitude_y', 'interpolated_attitude_z',
    ))
    norm = math.hypot(*quaternion)
    if all(math.isfinite(value) for value in quaternion) and 0.0 < norm < math.inf:
        w, x, y, z = (value / norm for value in quaternion)
        values['px4_heading'] = math.atan2(
            2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z),
        )
    return values


@dataclass(frozen=True)
class KinematicState:
    """Three-dimensional position and velocity in local NED coordinates."""

    position: tuple
    velocity: tuple


class TimestampedStateHistory:
    """Bounded kinematic history with linear interpolation in ROS time."""

    def __init__(self, maximum_age=0.25):
        self.maximum_age = max(float(maximum_age), 1e-3)
        self._samples = deque()

    @property
    def latest_stamp(self):
        return self._samples[-1][0] if self._samples else None

    def clear(self):
        self._samples.clear()

    def add(self, stamp, state):
        stamp = float(stamp)
        if not math.isfinite(stamp):
            raise ValueError('state history stamp must be finite')
        if self._samples and stamp < self._samples[-1][0] - 1e-9:
            return False
        if self._samples and abs(stamp - self._samples[-1][0]) <= 1e-9:
            self._samples[-1] = stamp, state
        else:
            self._samples.append((stamp, state))
        cutoff = stamp - self.maximum_age
        while len(self._samples) > 2 and self._samples[1][0] < cutoff:
            self._samples.popleft()
        return True

    @staticmethod
    def _blend(first, second, fraction):
        return tuple(
            start + fraction * (finish - start)
            for start, finish in zip(first, second)
        )

    def state_at(self, stamp):
        stamp = float(stamp)
        if not self._samples:
            return None
        if stamp < self._samples[0][0] - 1e-9:
            return None
        if stamp > self._samples[-1][0] + 1e-9:
            return None
        for index, (sample_stamp, state) in enumerate(self._samples):
            if abs(stamp - sample_stamp) <= 1e-9:
                return state
            if sample_stamp > stamp and index > 0:
                previous_stamp, previous = self._samples[index - 1]
                span = max(sample_stamp - previous_stamp, 1e-9)
                fraction = (stamp - previous_stamp) / span
                return KinematicState(
                    position=self._blend(
                        previous.position,
                        state.position,
                        fraction,
                    ),
                    velocity=self._blend(
                        previous.velocity,
                        state.velocity,
                        fraction,
                    ),
                )
        return self._samples[-1][1]


def synchronize_histories(uav_history, target_history):
    """Return both histories interpolated at their newest common time."""
    if (
        uav_history.latest_stamp is None
        or target_history.latest_stamp is None
    ):
        return None
    stamp = min(uav_history.latest_stamp, target_history.latest_stamp)
    uav = uav_history.state_at(stamp)
    target = target_history.state_at(stamp)
    if uav is None or target is None:
        return None
    return stamp, uav, target


@dataclass(frozen=True)
class EvaluationResult:
    """Terminal result derived only from evaluator truth inputs."""

    mission_id: int
    success: bool
    outcome: str
    reason: str
    elapsed_time: float
    minimum_distance: float
    horizontal_distance: float
    vertical_error: float
    relative_speed: float
    closing_speed: float
    maximum_horizontal_speed: float
    maximum_vertical_speed: float
    maximum_horizontal_acceleration: float
    maximum_vertical_acceleration: float
    detail: str = ''


def _norm(values):
    return math.sqrt(sum(float(value) ** 2 for value in values))


def _percentile(values, fraction):
    if not values:
        return 0.0
    ordered = sorted(float(value) for value in values)
    index = min(
        max(int(math.ceil(float(fraction) * len(ordered))) - 1, 0),
        len(ordered) - 1,
    )
    return ordered[index]


def _error_summary(values):
    values = [float(value) for value in values]
    return {
        'count': len(values),
        'available': bool(values),
        'p50': _percentile(values, 0.50) if values else None,
        'p95': _percentile(values, 0.95) if values else None,
        'rmse': (
            math.sqrt(sum(value * value for value in values) / len(values))
            if values else None
        ),
    }


def _signed_error_summary(values):
    values = [float(value) for value in values]
    ordered = sorted(values)
    return {
        'count': len(values),
        'available': bool(values),
        'mean': sum(values) / len(values) if values else None,
        'median': (
            ordered[len(ordered) // 2]
            if len(ordered) % 2 == 1
            else 0.5 * (
                ordered[len(ordered) // 2 - 1]
                + ordered[len(ordered) // 2]
            )
            if ordered else None
        ),
        'rmse': (
            math.sqrt(sum(value * value for value in values) / len(values))
            if values else None
        ),
        'p95_abs': _percentile([abs(value) for value in values], 0.95)
        if values else None,
    }


class VisionMetricAccumulator:
    """Keep event-based raw RGB-D and current KF errors separate."""

    def __init__(self):
        self._camera = {}
        self._strata = {}
        self._kf_position = []
        self._kf_velocity = []
        self._kf_ages = []

    @staticmethod
    def _camera_name(source):
        source = str(source).lower()
        if source.startswith('down') or '/down' in source:
            return 'down'
        return 'front'

    def observe_raw(
        self,
        source,
        measurement_stamp,
        receipt_stamp,
        estimate,
        truth,
        valid,
        truth_available=True,
        distance_bin='UNKNOWN',
        motion_regime='UNKNOWN',
        approach_phase='UNKNOWN',
        rejection_reason='',
    ):
        camera = self._camera_name(source)
        values = self._camera.setdefault(camera, {
            'total': 0,
            'valid': 0,
            'axis_x': [],
            'axis_y': [],
            'axis_z': [],
            'signed_axis_x': [],
            'signed_axis_y': [],
            'signed_axis_z': [],
            'horizontal': [],
            'position_3d': [],
            'ages': [],
            'loss_started_at': None,
            'longest_loss': 0.0,
            'rejections': {},
        })
        values['total'] += 1
        stratum_key = (
            camera,
            str(distance_bin),
            str(motion_regime),
            str(approach_phase),
        )
        stratum = self._strata.setdefault(stratum_key, {
            'total': 0,
            'valid': 0,
            'position_3d': [],
            'ages': [],
        })
        stratum['total'] += 1
        if not valid:
            reason = str(rejection_reason).strip()
            if reason:
                values['rejections'][reason] = (
                    values['rejections'].get(reason, 0) + 1
                )
            stamp = float(measurement_stamp)
            if not math.isfinite(stamp) or stamp <= 0.0:
                stamp = float(receipt_stamp)
            if math.isfinite(stamp) and stamp > 0.0:
                if values['loss_started_at'] is None:
                    values['loss_started_at'] = stamp
            return
        age = float(receipt_stamp) - float(measurement_stamp)
        if math.isfinite(age) and age >= 0.0:
            values['ages'].append(age)
            stratum['ages'].append(age)
        values['valid'] += 1
        stratum['valid'] += 1
        if values['loss_started_at'] is not None:
            values['longest_loss'] = max(
                values['longest_loss'],
                float(receipt_stamp) - values['loss_started_at'],
            )
            values['loss_started_at'] = None
        if not truth_available:
            return
        error = tuple(
            float(left) - float(right)
            for left, right in zip(estimate, truth)
        )
        if len(error) != 3 or not all(map(math.isfinite, error)):
            return
        values['axis_x'].append(abs(error[0]))
        values['axis_y'].append(abs(error[1]))
        values['axis_z'].append(abs(error[2]))
        values['signed_axis_x'].append(error[0])
        values['signed_axis_y'].append(error[1])
        values['signed_axis_z'].append(error[2])
        values['horizontal'].append(math.hypot(error[0], error[1]))
        values['position_3d'].append(_norm(error))
        stratum['position_3d'].append(_norm(error))

    def observe_kf(
        self,
        stamp,
        position,
        velocity,
        truth_position,
        truth_velocity,
        source_stamp=None,
    ):
        position_error = tuple(
            float(left) - float(right)
            for left, right in zip(position, truth_position)
        )
        velocity_error = tuple(
            float(left) - float(right)
            for left, right in zip(velocity, truth_velocity)
        )
        if (
            len(position_error) == 3
            and all(map(math.isfinite, position_error))
        ):
            self._kf_position.append(_norm(position_error))
        if (
            len(velocity_error) == 3
            and all(map(math.isfinite, velocity_error))
        ):
            self._kf_velocity.append(_norm(velocity_error))
        if source_stamp is not None:
            age = float(stamp) - float(source_stamp)
            if math.isfinite(age) and age >= 0.0:
                self._kf_ages.append(age)

    def summary(self):
        summary = {}
        for camera, values in sorted(self._camera.items()):
            summary[camera] = {
                'observation_count': values['total'],
                'valid_observation_count': values['valid'],
                'valid_observation_rate': (
                    values['valid'] / values['total']
                    if values['total'] else 0.0
                ),
                'raw_position_x': _error_summary(values['axis_x']),
                'raw_position_y': _error_summary(values['axis_y']),
                'raw_position_z': _error_summary(values['axis_z']),
                'raw_signed_position_x': _signed_error_summary(
                    values['signed_axis_x']
                ),
                'raw_signed_position_y': _signed_error_summary(
                    values['signed_axis_y']
                ),
                'raw_signed_position_z': _signed_error_summary(
                    values['signed_axis_z']
                ),
                'raw_position_horizontal': _error_summary(
                    values['horizontal']
                ),
                'raw_position_3d': _error_summary(values['position_3d']),
                'observation_age': _error_summary(values['ages']),
                'longest_continuous_loss': values['longest_loss'],
                'rejection_histogram': dict(sorted(
                    values['rejections'].items()
                )),
            }
        summary['kf_position_3d'] = _error_summary(self._kf_position)
        summary['kf_velocity_3d'] = _error_summary(self._kf_velocity)
        summary['kf_state_age'] = _error_summary(self._kf_ages)
        summary['strata'] = [
            {
                'camera': key[0],
                'distance_bin': key[1],
                'motion_regime': key[2],
                'approach_phase': key[3],
                'observation_count': values['total'],
                'valid_observation_rate': (
                    values['valid'] / values['total']
                    if values['total'] else 0.0
                ),
                'raw_position_3d': _error_summary(values['position_3d']),
                'observation_age': _error_summary(values['ages']),
            }
            for key, values in sorted(self._strata.items())
        ]
        return summary


class PlannerEventAccumulator:
    """Count each planner event once rather than once per control sample."""

    def __init__(self):
        self._planner_events = {}
        self._completion_stamps = []
        self._compute_times = []
        self._reachability_times = []
        self._generation_times = []
        self._validation_times = []
        self._publish_ages = []
        self._publication_delays = []
        self._executed_plan_ids = set()
        self._controller_event_count = 0
        self._hold_event_count = 0
        self._tracker_rejection_histogram = {}

    def observe_planner(
        self,
        mission_id,
        plan_id,
        success,
        failure_reason,
        compute_time,
        generation_time,
        completion_stamp,
        reachability_time=0.0,
        validation_time=0.0,
        input_age_at_publish=0.0,
        completion_to_publish_delay=0.0,
        admission_wait=False,
    ):
        """Store a completed event if its mission/plan identity is new."""
        key = int(mission_id), int(plan_id)
        if key in self._planner_events:
            return False
        event = {
            'success': bool(success),
            'failure_reason': str(failure_reason),
            'compute_time': max(float(compute_time), 0.0),
            'reachability_time': max(float(reachability_time), 0.0),
            'generation_time': max(float(generation_time), 0.0),
            'validation_time': max(float(validation_time), 0.0),
            'completion_stamp': float(completion_stamp),
            'input_age_at_publish': max(float(input_age_at_publish), 0.0),
            'completion_to_publish_delay': max(
                float(completion_to_publish_delay),
                0.0,
            ),
            'admission_wait': bool(admission_wait),
        }
        self._planner_events[key] = event
        self._completion_stamps.append(event['completion_stamp'])
        self._compute_times.append(event['compute_time'])
        self._reachability_times.append(event['reachability_time'])
        self._generation_times.append(event['generation_time'])
        self._validation_times.append(event['validation_time'])
        self._publish_ages.append(event['input_age_at_publish'])
        self._publication_delays.append(
            event['completion_to_publish_delay']
        )
        return True

    def observe_controller(
        self,
        mission_id,
        plan_id,
        status,
        rejection_reason='',
    ):
        """Record execution and hold events without duplicating plan IDs."""
        self._controller_event_count += 1
        status = str(status)
        if status in ('PLAN_ACCEPTED', 'TRACKING') and int(plan_id) > 0:
            self._executed_plan_ids.add((int(mission_id), int(plan_id)))
        if status in ('NO_VALID_PLAN', 'SAFE_WAIT', 'HOLD'):
            self._hold_event_count += 1
        if status == 'PLAN_REJECTED' and rejection_reason:
            reason = str(rejection_reason)
            self._tracker_rejection_histogram[reason] = (
                self._tracker_rejection_histogram.get(reason, 0) + 1
            )

    def summary(self, elapsed_time):
        """Return numeric event rates even when no event has occurred."""
        events = list(self._planner_events.values())
        succeeded = sum(event['success'] for event in events)
        admission_waits = sum(event['admission_wait'] for event in events)
        failed = len(events) - succeeded - admission_waits
        deadlines = sum(
            event['failure_reason'] == 'DEADLINE_EXCEEDED'
            for event in events
        )
        failure_histogram = {}
        for event in events:
            if event['success'] or event['admission_wait']:
                continue
            reason = event['failure_reason']
            failure_histogram[reason] = failure_histogram.get(reason, 0) + 1
        primary_failure = (
            max(
                failure_histogram,
                key=lambda reason: (failure_histogram[reason], reason),
            )
            if failure_histogram else ''
        )
        completed_solver_events = succeeded + failed
        executed_successes = sum(
            key in self._executed_plan_ids and event['success']
            for key, event in self._planner_events.items()
        )
        elapsed_time = max(float(elapsed_time), 0.0)
        stamps = sorted(self._completion_stamps)
        if len(stamps) >= 2 and stamps[-1] > stamps[0]:
            completion_hz = (len(stamps) - 1) / (stamps[-1] - stamps[0])
        else:
            completion_hz = 0.0
        return {
            'planner_started': len(events),
            'planner_completed': len(events),
            'planner_succeeded': succeeded,
            'planner_failed': failed,
            'planner_admission_wait': admission_waits,
            'planner_deadline': deadlines,
            'planner_failure_histogram': dict(sorted(
                failure_histogram.items()
            )),
            'planner_primary_failure_reason': primary_failure,
            'planner_success_rate': (
                succeeded / completed_solver_events
                if completed_solver_events else 0.0
            ),
            'attempt_rate': (
                len(events) / elapsed_time if elapsed_time > 0.0 else 0.0
            ),
            'execution_rate': (
                executed_successes / succeeded if succeeded else 0.0
            ),
            'hold_rate': (
                self._hold_event_count / self._controller_event_count
                if self._controller_event_count else 0.0
            ),
            'tracker_rejection_histogram': dict(sorted(
                self._tracker_rejection_histogram.items()
            )),
            'actual_completion_hz': completion_hz,
            'planner_compute_p50': _percentile(self._compute_times, 0.50),
            'planner_compute_p95': _percentile(self._compute_times, 0.95),
            'planner_compute_max': max(self._compute_times, default=0.0),
            'reachability_compute_p50': _percentile(
                self._reachability_times,
                0.50,
            ),
            'reachability_compute_p95': _percentile(
                self._reachability_times,
                0.95,
            ),
            'reachability_compute_max': max(
                self._reachability_times,
                default=0.0,
            ),
            'generation_compute_p50': _percentile(
                self._generation_times,
                0.50,
            ),
            'generation_compute_p95': _percentile(
                self._generation_times,
                0.95,
            ),
            'generation_compute_max': max(
                self._generation_times,
                default=0.0,
            ),
            'validation_compute_p50': _percentile(
                self._validation_times,
                0.50,
            ),
            'validation_compute_p95': _percentile(
                self._validation_times,
                0.95,
            ),
            'validation_compute_max': max(
                self._validation_times,
                default=0.0,
            ),
            'planner_source_age_at_publish_p50': _percentile(
                self._publish_ages,
                0.50,
            ),
            'planner_source_age_at_publish_p95': _percentile(
                self._publish_ages,
                0.95,
            ),
            'planner_source_age_at_publish_max': max(
                self._publish_ages,
                default=0.0,
            ),
            'planner_publish_delay_p50': _percentile(
                self._publication_delays,
                0.50,
            ),
            'planner_publish_delay_p95': _percentile(
                self._publication_delays,
                0.95,
            ),
            'planner_publish_delay_max': max(
                self._publication_delays,
                default=0.0,
            ),
        }


class RuntimePerformanceAccumulator:
    """Aggregate runtime rates and callback costs by target-distance band."""

    BUCKETS = (
        ('less_than_2m', lambda distance: distance < 2.0),
        ('between_2m_and_5m', lambda distance: distance <= 5.0),
        ('between_5m_and_10m', lambda distance: distance <= 10.0),
        ('greater_than_10m', lambda _distance: True),
    )

    def __init__(self):
        self._values = {
            name: {}
            for name, _predicate in self.BUCKETS
        }

    def observe(self, distance, metrics):
        distance = float(distance)
        if not math.isfinite(distance):
            return
        bucket = next(
            name
            for name, predicate in self.BUCKETS
            if predicate(distance)
        )
        for name, value in dict(metrics).items():
            value = float(value)
            if math.isfinite(value) and value >= 0.0:
                self._values[bucket].setdefault(str(name), []).append(value)

    def summary(self):
        document = {}
        for bucket, metrics in self._values.items():
            document[bucket] = {}
            for name, values in metrics.items():
                document[bucket][name] = {
                    'count': len(values),
                    'p50': _percentile(values, 0.50),
                    'p95': _percentile(values, 0.95),
                    'max': max(values, default=0.0),
                }
        return document


class InterceptEvaluatorCore:
    """Detect capture, sea contact, timeout, and motion extrema from truth."""

    def __init__(
        self,
        capture_radius=0.25,
        sea_surface_z=0.0,
        enable_sea_contact_failure=True,
        maximum_duration=30.0,
        body_lower_extent=DEFAULT_BODY_LOWER_EXTENT,
    ):
        self.capture_radius = float(capture_radius)
        self.sea_surface_z = float(sea_surface_z)
        self.enable_sea_contact_failure = bool(enable_sea_contact_failure)
        self.maximum_duration = float(maximum_duration)
        self.body_lower_extent = max(float(body_lower_extent), 0.0)
        self.reset()

    @property
    def body_contact_z(self):
        """Return the reference altitude at which the body reaches water."""
        return self.sea_surface_z - self.body_lower_extent

    def reset(self):
        """Clear all mission-scoped evaluation state."""
        self.mission_id = 0
        self.started_at = None
        self.previous_time = None
        self.previous_uav = None
        self.previous_target = None
        self.detail = ''
        self.minimum_distance = math.inf
        self.closest_horizontal_distance = math.inf
        self.closest_vertical_error = math.inf
        self.maximum_horizontal_speed = 0.0
        self.maximum_vertical_speed = 0.0
        self.maximum_horizontal_acceleration = 0.0
        self.maximum_vertical_acceleration = 0.0
        self.result = None

    def begin(self, mission_id, now):
        """Start a clean truth-evaluation interval for one mission."""
        self.reset()
        self.mission_id = int(mission_id)
        self.started_at = float(now)

    @staticmethod
    def _relative(uav, target):
        return tuple(
            target_value - uav_value
            for target_value, uav_value in zip(
                target.position,
                uav.position,
            )
        )

    @staticmethod
    def _closest_relative(relative_start, relative_end):
        delta = tuple(
            end - start
            for start, end in zip(relative_start, relative_end)
        )
        denominator = sum(value * value for value in delta)
        if denominator <= 1e-12:
            fraction = 0.0
        else:
            fraction = -sum(
                start * change
                for start, change in zip(relative_start, delta)
            ) / denominator
            fraction = min(max(fraction, 0.0), 1.0)
        return tuple(
            start + fraction * change
            for start, change in zip(relative_start, delta)
        )

    def _update_closest(self, relative):
        horizontal = math.hypot(relative[0], relative[1])
        distance = math.hypot(horizontal, relative[2])
        if distance < self.minimum_distance:
            self.minimum_distance = distance
            self.closest_horizontal_distance = horizontal
            self.closest_vertical_error = relative[2]

    def _capture_fraction(self, start, end):
        delta = tuple(finish - begin for begin, finish in zip(start, end))
        c = sum(value * value for value in start) - self.capture_radius**2
        if c <= 0.0:
            return 0.0
        a = sum(value * value for value in delta)
        if a <= 1e-12:
            return None
        b = 2.0 * sum(value * change for value, change in zip(start, delta))
        discriminant = b * b - 4.0 * a * c
        if discriminant < 0.0:
            return None
        root = math.sqrt(discriminant)
        candidates = (
            (-b - root) / (2.0 * a),
            (-b + root) / (2.0 * a),
        )
        valid = [value for value in candidates if 0.0 <= value <= 1.0]
        return min(valid) if valid else None

    def _sea_fraction(self, start_z, end_z):
        """
        Return where the body first reaches the water inside the interval.

        The PX4 local-position reference sits ``body_lower_extent`` above the
        lowest point of the airframe, so the body makes contact at a reference
        altitude of ``sea_surface_z - body_lower_extent`` instead of at the sea
        surface itself.
        """
        if not self.enable_sea_contact_failure:
            return None
        contact_z = self.body_contact_z
        if start_z >= contact_z:
            return 0.0
        delta = end_z - start_z
        if delta <= 0.0:
            return None
        fraction = (contact_z - start_z) / delta
        return fraction if 0.0 <= fraction <= 1.0 else None

    def _sea_contact_detail(self, reference_z):
        """Name how deep the reference point already was at body contact."""
        if float(reference_z) >= self.sea_surface_z:
            return 'REFERENCE_POINT_BELOW_SEA_SURFACE'
        return 'BODY_LOWEST_POINT_AT_SEA_SURFACE'

    @staticmethod
    def instantaneous_metrics(uav, target):
        """Return range decomposition and relative motion from truth."""
        relative_position = InterceptEvaluatorCore._relative(uav, target)
        relative_velocity = tuple(
            target_value - uav_value
            for target_value, uav_value in zip(
                target.velocity,
                uav.velocity,
            )
        )
        distance = _norm(relative_position)
        horizontal = math.hypot(
            relative_position[0],
            relative_position[1],
        )
        relative_speed = _norm(relative_velocity)
        closing_speed = (
            -sum(
                position * velocity
                for position, velocity in zip(
                    relative_position,
                    relative_velocity,
                )
            ) / distance
            if distance > 1e-9 else 0.0
        )
        return (
            distance,
            horizontal,
            relative_position[2],
            relative_speed,
            closing_speed,
        )

    def _finish(self, success, reason, now, uav, target):
        metrics = self.instantaneous_metrics(uav, target)
        self._update_closest(self._relative(uav, target))
        self.result = EvaluationResult(
            mission_id=self.mission_id,
            success=bool(success),
            outcome='SUCCESS' if success else 'FAILURE',
            reason=str(reason),
            elapsed_time=max(float(now) - self.started_at, 0.0),
            minimum_distance=self.minimum_distance,
            horizontal_distance=metrics[1],
            vertical_error=metrics[2],
            relative_speed=metrics[3],
            closing_speed=metrics[4],
            maximum_horizontal_speed=self.maximum_horizontal_speed,
            maximum_vertical_speed=self.maximum_vertical_speed,
            maximum_horizontal_acceleration=(
                self.maximum_horizontal_acceleration
            ),
            maximum_vertical_acceleration=self.maximum_vertical_acceleration,
            detail=self.detail,
        )
        return self.result

    def update(self, now, uav, target):
        """Consume synchronized truth and return a terminal event."""
        if self.started_at is None or self.result is not None:
            return self.result
        now = float(now)
        relative = self._relative(uav, target)
        self._update_closest(relative)
        self.maximum_horizontal_speed = max(
            self.maximum_horizontal_speed,
            math.hypot(uav.velocity[0], uav.velocity[1]),
        )
        self.maximum_vertical_speed = max(
            self.maximum_vertical_speed,
            abs(uav.velocity[2]),
        )

        event = None
        if self.previous_uav is not None and self.previous_target is not None:
            previous_relative = self._relative(
                self.previous_uav,
                self.previous_target,
            )
            self._update_closest(
                self._closest_relative(previous_relative, relative)
            )
            capture_fraction = self._capture_fraction(
                previous_relative,
                relative,
            )
            sea_fraction = self._sea_fraction(
                self.previous_uav.position[2],
                uav.position[2],
            )
            if capture_fraction is not None and (
                sea_fraction is None or capture_fraction <= sea_fraction
            ):
                event = True, 'CAPTURE_RADIUS_REACHED'
            elif sea_fraction is not None:
                self.detail = self._sea_contact_detail(uav.position[2])
                event = False, 'SEA_CONTACT'
            dt = now - self.previous_time
            if dt > 1e-6:
                acceleration = tuple(
                    (current - previous) / dt
                    for current, previous in zip(
                        uav.velocity,
                        self.previous_uav.velocity,
                    )
                )
                self.maximum_horizontal_acceleration = max(
                    self.maximum_horizontal_acceleration,
                    math.hypot(acceleration[0], acceleration[1]),
                )
                self.maximum_vertical_acceleration = max(
                    self.maximum_vertical_acceleration,
                    abs(acceleration[2]),
                )
        else:
            if _norm(relative) <= self.capture_radius:
                event = True, 'CAPTURE_RADIUS_REACHED'
            elif (
                self.enable_sea_contact_failure
                and uav.position[2] >= self.body_contact_z
            ):
                self.detail = self._sea_contact_detail(uav.position[2])
                event = False, 'SEA_CONTACT'

        self.previous_time = now
        self.previous_uav = uav
        self.previous_target = target
        if event is not None:
            return self._finish(event[0], event[1], now, uav, target)
        if now - self.started_at >= self.maximum_duration:
            return self._finish(False, 'TIMEOUT', now, uav, target)
        return None


@dataclass(frozen=True)
class ArtifactPaths:
    """Paths produced for one evaluation run."""

    csv_path: Path
    summary_path: Path
    config_path: Path
    diagnostics_path: Path = None
    visual_path: Path = None


class ExperimentArtifactWriter:
    """Write isolated sample, summary, and configuration artifacts."""

    CSV_FIELDS = (
        'time', 'run_elapsed_time', 'intercept_elapsed_time',
        'intercept_started', 'sample_kind', 'truth_available', 'uav_available',
        'mission_id', 'phase',
        'uav_x', 'uav_y', 'uav_z', 'uav_vx', 'uav_vy', 'uav_vz',
        'target_x', 'target_y', 'target_z',
        'target_vx', 'target_vy', 'target_vz',
        'distance', 'horizontal_distance', 'vertical_error',
        'relative_speed', 'closing_speed',
        'approach_phase', 'first_terminal_approach',
        'controller_status', 'plan_id',
        'bearing_approach_active', 'bearing_age_at_control',
        'prediction_age', 'trajectory_age',
        'target_visible', 'target_locked', 'search_state', 'search_direction',
        'last_valid_observation_age', 'last_valid_image_bearing',
        'search_yaw_rate_command',
        'consecutive_valid_frames', 'consecutive_lost_frames',
        'kf_state_age', 'prediction_sample_age', 'planner_source_age',
        'yaw_owner',
        'tracker_rejection_reason',
        'planner_event_id', 'planner_result', 'planner_failure_reason',
        'planner_source_age_at_publish',
        'planner_completion_to_publish_delay',
        'terminal_admission', 'terminal_admission_reason',
        'selected_t_go', 'contact_stamp', 'remaining_t_go',
        'terminal_mode', 'planned_capture_margin', 'target_yaw',
        'body_clearance', 'sea_safety_state', 'sea_safety_margin',
        'gazebo_real_time_factor',
        'front_rgb_hz', 'front_depth_hz',
        'down_rgb_hz', 'down_depth_hz',
        'rgbd_localizer_compute_time',
        'front_monitor_compute_time', 'down_monitor_compute_time',
        'tracker_hz', 'tracker_callback_time', 'planner_compute_time',
        'prediction_0p5_error', 'prediction_1p0_error',
        'prediction_2p0_error',
        'kf_prediction_0p5_error',
        'kf_prediction_1p0_error',
        'kf_prediction_2p0_error',
        'shadow_bctra_prediction_0p5_error',
        'shadow_bctra_prediction_1p0_error',
        'shadow_bctra_prediction_2p0_error',
    )
    VISUAL_FIELDS = (
        'measurement_stamp', 'receipt_stamp', 'processed_stamp',
        'published_stamp', 'source', 'valid', 'observation_valid',
        'truth_available', 'rejection_reason',
        'rgb_raw_stamp', 'depth_raw_stamp',
        'rgb_receipt_stamp', 'depth_receipt_stamp',
        'rgb_mapped_stamp', 'depth_mapped_stamp',
        'image_measurement_stamp',
        'rgb_depth_acquisition_skew',
        'pose_history_start_stamp', 'pose_history_end_stamp',
        'position_source_stamp', 'position_mapped_stamp',
        'attitude_source_stamp', 'attitude_mapped_stamp',
        'position_history_start_stamp', 'position_history_end_stamp',
        'attitude_history_start_stamp', 'attitude_history_end_stamp',
        'image_clock_offset', 'image_clock_mapping_mode',
        'image_clock_status', 'image_clock_reset_count',
        'image_clock_anchor_sim_stamp',
        'image_clock_anchor_system_stamp',
        'image_clock_reference_age', 'image_clock_sync_quality',
        'image_measurement_time_source', 'px4_clock_offset',
        'px4_clock_reset_count', 'px4_clock_calibration_count',
        'px4_clock_recalibration_count', 'px4_clock_status',
        'approach_phase', 'distance_bin', 'motion_regime',
        'confidence', 'red_pixel_count', 'valid_depth_ratio',
        'target_range', 'view_angle', 'image_bearing', 'px4_heading',
        'geometry_diagnostics_enabled',
        'mask_centroid_u', 'mask_centroid_v',
        'projection_centroid_u', 'projection_centroid_v',
        'mask_bbox_left', 'mask_bbox_top',
        'mask_bbox_right', 'mask_bbox_bottom',
        'valid_depth_count', 'depth_min', 'depth_median', 'depth_mad',
        'camera_fx', 'camera_fy', 'camera_cx', 'camera_cy',
        'camera_translation_x', 'camera_translation_y',
        'camera_translation_z', 'camera_pitch_down',
        'surface_camera_x', 'surface_camera_y', 'surface_camera_z',
        'center_camera_x', 'center_camera_y', 'center_camera_z',
        'target_body_flu_x', 'target_body_flu_y', 'target_body_flu_z',
        'interpolated_uav_x', 'interpolated_uav_y',
        'interpolated_uav_z',
        'interpolated_attitude_w', 'interpolated_attitude_x',
        'interpolated_attitude_y', 'interpolated_attitude_z',
        'target_reference_z_offset',
        'gazebo_entity_available', 'gazebo_entity_clock_status',
        'gazebo_entity_clock_reset_count',
        'gazebo_entity_raw_stamp', 'gazebo_entity_mapped_stamp',
        'gazebo_entity_center_x', 'gazebo_entity_center_y',
        'gazebo_entity_center_z',
        'gazebo_entity_reference_x', 'gazebo_entity_reference_y',
        'gazebo_entity_reference_z',
        'entity_expected_projection_u', 'entity_expected_projection_v',
        'projection_error_u', 'projection_error_v',
        'entity_expected_camera_x', 'entity_expected_camera_y',
        'entity_expected_camera_z',
        'camera_center_error_x', 'camera_center_error_y',
        'camera_center_error_z', 'camera_center_error_3d',
        'entity_expected_body_flu_x', 'entity_expected_body_flu_y',
        'entity_expected_body_flu_z',
        'body_flu_error_x', 'body_flu_error_y',
        'body_flu_error_z', 'body_flu_error_3d',
        'vision_to_entity_x', 'vision_to_entity_y',
        'vision_to_entity_z', 'vision_to_entity_3d',
        'entity_to_truth_x', 'entity_to_truth_y',
        'entity_to_truth_z', 'entity_to_truth_3d',
        'position_x', 'position_y', 'position_z',
        'covariance_xx', 'covariance_yy', 'covariance_zz',
        'truth_x', 'truth_y', 'truth_z',
        'error_x', 'error_y', 'error_z',
        'horizontal_error', 'position_3d_error', 'observation_age',
    ) + tuple(field for field in UAV_HEADING_CSV_FIELDS if field != 'px4_heading')

    def __init__(
        self,
        log_directory,
        mission_id,
        config,
        prefix=None,
        detailed_diagnostics_enabled=False,
        visual_evaluation_enabled=False,
    ):
        directory = Path(log_directory).expanduser()
        directory.mkdir(parents=True, exist_ok=True)
        if prefix is None:
            prefix = 'modular_intercept_' + datetime.now().strftime(
                '%Y%m%d_%H%M%S_%f'
            )
        stem = f'{prefix}_mission_{int(mission_id)}'
        self.paths = ArtifactPaths(
            csv_path=directory / f'{stem}.csv',
            summary_path=directory / f'{stem}_summary.json',
            config_path=directory / f'{stem}_config.yaml',
            diagnostics_path=(
                directory / f'{stem}_diagnostics.jsonl'
                if detailed_diagnostics_enabled else None
            ),
            visual_path=(
                directory / f'{stem}_vision.csv'
                if visual_evaluation_enabled else None
            ),
        )
        self.mission_id = int(mission_id)
        self._stream = self.paths.csv_path.open(
            'x',
            newline='',
            encoding='utf-8',
            buffering=1,
        )
        self._writer = csv.DictWriter(
            self._stream,
            fieldnames=self.CSV_FIELDS,
            extrasaction='ignore',
        )
        self._writer.writeheader()
        self._diagnostics_stream = (
            self.paths.diagnostics_path.open(
                'x', encoding='utf-8', buffering=1
            )
            if self.paths.diagnostics_path is not None else None
        )
        self._visual_stream = (
            self.paths.visual_path.open(
                'x', newline='', encoding='utf-8', buffering=1
            )
            if self.paths.visual_path is not None else None
        )
        self._visual_writer = (
            csv.DictWriter(
                self._visual_stream,
                fieldnames=self.VISUAL_FIELDS,
                extrasaction='ignore',
            )
            if self._visual_stream is not None else None
        )
        if self._visual_writer is not None:
            self._visual_writer.writeheader()
        with self.paths.config_path.open('x', encoding='utf-8') as stream:
            json.dump(config, stream, ensure_ascii=False, indent=2)
            stream.write('\n')
        self._finalized = False

    def append_sample(self, sample):
        """Append one evaluation sample outside the control process."""
        if self._finalized:
            raise RuntimeError('experiment artifacts are already finalized')
        self._writer.writerow(dict(sample))

    def append_detail_event(self, event_type, event_id, values):
        """Write one optional diagnostic event instead of repeating samples."""
        if self._finalized:
            raise RuntimeError('experiment artifacts are already finalized')
        if self._diagnostics_stream is None:
            return False
        document = {
            'event_type': str(event_type),
            'event_id': str(event_id),
            **dict(values),
        }
        self._diagnostics_stream.write(
            json.dumps(document, ensure_ascii=False) + '\n'
        )
        return True

    def append_visual_event(self, values):
        """Write one RGB-D observation event, never one copy per sample."""
        if self._finalized:
            raise RuntimeError('experiment artifacts are already finalized')
        if self._visual_writer is None:
            return False
        self._visual_writer.writerow(dict(values))
        return True

    def finalize(self, summary):
        """Write exactly one summary and close the line-buffered CSV."""
        if self._finalized:
            return self.paths
        document = dict(summary)
        document['mission_id'] = self.mission_id
        with self.paths.summary_path.open('x', encoding='utf-8') as stream:
            json.dump(document, stream, ensure_ascii=False, indent=2)
            stream.write('\n')
        self._stream.close()
        if self._diagnostics_stream is not None:
            self._diagnostics_stream.close()
        if self._visual_stream is not None:
            self._visual_stream.close()
        self._finalized = True
        return self.paths
