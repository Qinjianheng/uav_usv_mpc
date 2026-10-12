"""Numerical earliest-reachable interception-point solver."""

import math


def earliest_reachable_intercept(
    predict_target_state,
    uav_position,
    horizontal_speed,
    vertical_speed,
    max_prediction_time,
    search_step,
):
    """
    Return the earliest reachable predicted target state and time.

    ``predict_target_state`` must accept a non-negative horizon in seconds and
    return a sequence whose first three values are target position.  Horizontal
    and vertical travel occur simultaneously, matching the controller's
    independent horizontal and vertical speed limits.
    """
    uav_x, uav_y, uav_z = (
        float(value) for value in uav_position
    )
    horizontal_speed = _positive_value(
        horizontal_speed,
        'horizontal speed',
    )
    vertical_speed = _positive_value(vertical_speed, 'vertical speed')
    max_prediction_time = _positive_value(
        max_prediction_time,
        'maximum prediction time',
    )
    search_step = min(
        _positive_value(search_step, 'search step'),
        max_prediction_time,
    )

    def predicted_state_and_gap(horizon):
        state = tuple(float(value) for value in predict_target_state(horizon))
        if len(state) < 3 or not all(
            math.isfinite(value) for value in state
        ):
            raise ValueError(
                'predicted target state must contain finite values'
            )
        horizontal_time = math.hypot(
            state[0] - uav_x,
            state[1] - uav_y,
        ) / horizontal_speed
        vertical_time = abs(state[2] - uav_z) / vertical_speed
        return state, max(horizontal_time, vertical_time) - horizon

    initial_state, initial_gap = predicted_state_and_gap(0.0)
    if initial_gap <= 1e-9:
        return initial_state, 0.0

    previous_time = 0.0
    sample_count = max(
        math.ceil(max_prediction_time / search_step),
        1,
    )

    for sample_index in range(1, sample_count + 1):
        sample_time = min(
            sample_index * search_step,
            max_prediction_time,
        )
        _, sample_gap = predicted_state_and_gap(sample_time)
        if sample_gap <= 0.0:
            lower_time = previous_time
            upper_time = sample_time
            for _ in range(14):
                middle_time = 0.5 * (lower_time + upper_time)
                _, middle_gap = predicted_state_and_gap(middle_time)
                if middle_gap <= 0.0:
                    upper_time = middle_time
                else:
                    lower_time = middle_time
            final_state, _ = predicted_state_and_gap(upper_time)
            return final_state, upper_time

        previous_time = sample_time
        if sample_time >= max_prediction_time:
            break

    final_state, _ = predicted_state_and_gap(max_prediction_time)
    return final_state, max_prediction_time


def _positive_value(value, name):
    value = float(value)
    if not math.isfinite(value) or value <= 0.0:
        raise ValueError(f'{name} must be finite and positive')
    return value
