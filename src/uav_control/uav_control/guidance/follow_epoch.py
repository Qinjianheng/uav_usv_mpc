"""Receiver-owned boot/clock identity; proposals have no mutation interface to this authority."""
import math
import secrets


class ReceiverClock:
    """One random boot identity per actual receiver, and monotone reset generation within it."""

    def __init__(self, boot_id=None):
        """Create one stable boot identity, allowing deterministic test injection."""
        self.boot_id = boot_id or secrets.token_hex(16)
        self.generation, self.last_ros, self.last_native = 0, None, None

    def observe(self, now, native_sample=None):
        """Detect ROS reversal or native navigation sample reset, not ordinary repeats."""
        if not math.isfinite(now) or now <= 0:
            raise ValueError('INVALID_CLOCK')
        if native_sample is not None and (not isinstance(native_sample, int)
                                          or native_sample <= 0):
            raise ValueError('INVALID_NATIVE_SAMPLE')
        reversed_ros = self.last_ros is not None and now < self.last_ros
        reversed_native = (native_sample is not None and self.last_native is not None
                           and native_sample < self.last_native)
        if reversed_ros or reversed_native:
            self.generation += 1
        self.last_ros = now
        if native_sample is not None:
            self.last_native = native_sample
        return self.generation
