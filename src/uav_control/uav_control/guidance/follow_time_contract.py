"""Explicit experimental target ages; native navigation and attitude retain125ms bounds."""
from dataclasses import dataclass


@dataclass(frozen=True)
class InputTimeContract:
    """Acquisition expiry differs from the age of an already propagated forecast state."""

    profile: str
    observation_age: float
    prediction_age: float
    navigation_age: float = .125
    attitude_age: float = .125


def input_time_contract(profile='baseline_125'):
    """Only named, fingerprinted experiment profiles can extend target ages."""
    ages = {'baseline_125': .125, 'experiment_150': .150, 'experiment_200': .200}
    if profile not in ages:
        raise ValueError('UNKNOWN_TARGET_TIME_PROFILE')
    return InputTimeContract(profile, ages[profile], ages[profile])


def context_deadline(context):
    """Intersect original input epochs; a new prediction never renews a native pose."""
    c = input_time_contract(getattr(context, 'target_time_profile', 'baseline_125'))
    source = getattr(context, 'prediction_source_stamp', getattr(context, 'source_stamp', 0.))
    return min(context.navigation_stamp+c.navigation_age,
               context.attitude_stamp+c.attitude_age,
               context.observation_stamp+c.observation_age, source+c.prediction_age,
               getattr(context, 'prediction_valid_until', float('inf')))
