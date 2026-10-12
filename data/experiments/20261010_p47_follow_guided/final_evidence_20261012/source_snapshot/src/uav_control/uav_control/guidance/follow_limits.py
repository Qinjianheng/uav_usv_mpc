"""Explicit research-only envelope and immutable receiver-owned calibration snapshot."""
from dataclasses import asdict, dataclass, replace
import hashlib
import json
import math

from uav_control.controllers.follow_mpc_seed import MpcConfig


@dataclass(frozen=True)
class ResearchConfig(MpcConfig):
    """Opt-in sensitivity envelope, never evidence of PX4/hardware qualification."""

    def validate(self):
        """Widen only XY acceleration/yaw within the requested research range."""
        values = self.maximum_horizontal_acceleration, self.maximum_yaw_rate
        if (not all(math.isfinite(v) and v > 0 for v in values)
                or values[0] > 4.5 or values[1] > 1.5
                or self.maximum_horizontal_jerk > 6 or self.maximum_vertical_jerk > 4
                or self.maximum_input_age > .125):
            raise ValueError('INVALID_RESEARCH_LIMITS')
        MpcConfig.validate(replace(self, maximum_horizontal_acceleration=min(values[0], 3.),
                                   maximum_yaw_rate=min(values[1], 1.)))


@dataclass(frozen=True)
class ConstraintSnapshot:
    """Canonical full physical/camera policy; JSON is immutable, decoded values are copies."""

    payload: str

    @property
    def fingerprint(self):
        """Bind every local threshold and calibration field, not a publisher version label."""
        return hashlib.sha256(self.payload.encode()).hexdigest()

    @property
    def values(self):
        """Decode a fresh copy so callers cannot mutate the trusted snapshot."""
        return json.loads(self.payload)


def constraint_snapshot(model):
    """Exclude solver scheduling knobs; include every physical, temporal and optical limit."""
    model.config.validate()
    physical = {k: v for k, v in asdict(model.config).items()
                if k.startswith('maximum_') and k != 'maximum_iterations'
                or k in ('follow_distance', 'flight_altitude', 'sea_surface_z',
                         'reserve_clearance', 'response_delay', 'braking_acceleration')}
    data = dict(schema='follow-constraints-p43-v1', mpc=physical,
                intrinsics=asdict(model.intrinsics), extrinsics=asdict(model.extrinsics),
                target=asdict(model.target), visibility=asdict(model.visibility),
                attitude=asdict(model.attitude_config))
    return ConstraintSnapshot(json.dumps(data, sort_keys=True, separators=(',', ':'),
                                         allow_nan=False))
