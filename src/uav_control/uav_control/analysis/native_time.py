"""Read-only native-time geometry and bounded interpolation diagnostics."""
import numpy as np


def project_gnss(latitude, longitude, ref_lat, ref_lon):
    """PX4 spherical azimuthal equidistant projection (earth radius 6371000)."""
    lat, lon = np.radians(latitude), np.radians(longitude)
    lat0, lon0 = np.radians(ref_lat), np.radians(ref_lon)
    dlon = lon - lon0
    cos_c = np.sin(lat0) * np.sin(lat) + np.cos(lat0) * np.cos(lat) * np.cos(dlon)
    c = np.arccos(np.clip(cos_c, -1, 1))
    scale = np.ones_like(c)
    np.divide(c, np.sin(c), out=scale, where=np.abs(c) > 1e-12)
    return 6371000 * scale[:, None] * np.column_stack([
        np.cos(lat0) * np.sin(lat) - np.sin(lat0) * np.cos(lat) * np.cos(dlon),
        np.cos(lat) * np.sin(dlon)])


def interpolate_inside(times, source_times, values, maximum_gap_us=100000):
    """Require a bounded bracket; return NaN for missing, distant or reset data."""
    times = np.asarray(times, dtype=float)
    source_times = np.asarray(source_times, dtype=float)
    values = np.asarray(values, dtype=float)
    if len(source_times) < 2 or np.any(np.diff(source_times) <= 0):
        raise ValueError('native source times must be strictly increasing')
    right = np.searchsorted(source_times, times, side='right')
    right = np.clip(right, 1, len(source_times) - 1)
    left = right - 1
    gap = source_times[right] - source_times[left]
    valid = ((times >= source_times[0]) & (times <= source_times[-1])
             & (gap <= maximum_gap_us))
    exact_left = times == source_times[left]
    exact_right = times == source_times[right]
    valid |= exact_left | exact_right
    fraction = (times - source_times[left]) / gap
    result = values[left] + fraction[:, None] * (values[right] - values[left])
    result[exact_left] = values[left[exact_left]]
    result[exact_right] = values[right[exact_right]]
    result[~valid] = np.nan
    return result
