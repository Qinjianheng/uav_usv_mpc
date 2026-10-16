"""
ROS Time/Duration conversion formerly shared with the old predictor.

The original conversion semantics are intentionally retained.
"""
from builtin_interfaces.msg import Duration, Time


def _split_nanoseconds(seconds):
    nanoseconds = round(float(seconds) * 1e9)
    return nanoseconds // 1_000_000_000, nanoseconds % 1_000_000_000


def seconds_to_time(seconds):
    """Convert floating-point seconds to a normalized ROS time message."""
    seconds_part, nanoseconds_part = _split_nanoseconds(seconds)
    message = Time()
    message.sec = int(seconds_part)
    message.nanosec = int(nanoseconds_part)
    return message


def seconds_to_duration(seconds):
    """Convert floating-point seconds to a normalized ROS duration."""
    seconds_part, nanoseconds_part = _split_nanoseconds(seconds)
    message = Duration()
    message.sec = int(seconds_part)
    message.nanosec = int(nanoseconds_part)
    return message
