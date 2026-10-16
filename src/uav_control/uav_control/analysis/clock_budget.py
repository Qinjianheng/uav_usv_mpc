"""Read-only clock-domain accounting, independent of retired experiment drivers."""
import numpy as np


def clock_budget(row, times, estimated_offsets):
    """
    Split apparent pose age using only a causal PX4 timesync sample.

    This is diagnostic accounting, never an online timestamp correction.
    Offsets use the recorded PX4 microsecond units and original sign.
    """
    query = float(row['rgb_raw_stamp'])
    index = int(np.searchsorted(times, query, side='right')) - 1
    if index < 0:
        return None
    clock_lag = (float(row['image_clock_offset'])
                 + float(estimated_offsets[index]) * 1e-6)
    age = float(row['processed_stamp']) - float(row['position_history_end_stamp'])
    return {
        'clock_lag_s': clock_lag,
        'apparent_pose_age_s': age,
        'residual_age_s': age - clock_lag,
        'timesync_sample_age_s': query - times[index],
        'image_lead_position_s': float(row['image_measurement_stamp'])
        - float(row['position_history_end_stamp']),
        'receipt_to_process_s': float(row['processed_stamp']) - float(row['receipt_stamp']),
    }
