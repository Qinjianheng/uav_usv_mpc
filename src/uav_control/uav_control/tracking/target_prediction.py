"""Timestamp-preserving target prediction independent of ROS transport."""

from dataclasses import dataclass



@dataclass(frozen=True)
class TargetKinematicState:
    """One timestamped target state in the local planning frame."""

    stamp: float
    position: tuple
    velocity: tuple
    observation_stamp: float = None

    def __post_init__(self):
        if self.observation_stamp is None:
            object.__setattr__(self, 'observation_stamp', self.stamp)


@dataclass(frozen=True)
class PredictedSample:
    """One point in a target prediction series."""

    relative_time: float
    position: tuple
    velocity: tuple
    acceleration: tuple


@dataclass(frozen=True)
class PredictionResult:
    """Transport-neutral target prediction result."""

    mission_id: int
    sequence_id: int
    source_stamp: float
    generated_stamp: float
    valid_until: float
    source: str
    model: str
    horizon: float
    samples: tuple
    turn_rate: float
    turn_acceleration: float
    longitudinal_acceleration: float
    valid: bool
    invalid_reason: str
    observation_stamp: float = None
