"""Monotone Y authority handover; a lost image never reactivates FOLLOW estimation."""
import math


class ModeCoordinator:
    """The sole Tracker owns this state and both mutually exclusive PX4 interfaces."""

    def __init__(self, handover_timeout=.5):
        self.phase, self.epoch, self.y_stamp = 'FOLLOW_ACTIVE', 0, 0.
        self.bodyrate_requested = None
        self.reason, self.handover_timeout = '', float(handover_timeout)

    def accept_y(self, now, qualified):
        """Only an already-accepted mission Y may revoke FOLLOW authority."""
        if self.phase != 'FOLLOW_ACTIVE':
            return
        self.epoch += 1
        self.y_stamp = float(now)
        self.phase = 'Y_HANDOVER'
        if not qualified:
            self.fail('P76_BODYRATE_QUALIFICATION_REQUIRED')

    def ready(self, now, spatial_inactive, image_source, native_fresh):
        """Require a new legal image after Y and a stopped spatial estimator."""
        self.timeout(now)
        return bool(self.phase == 'Y_HANDOVER' and spatial_inactive and native_fresh
                    and self.y_stamp < image_source <= now and now-image_source <= .125)

    def request_bodyrate(self, now):
        """Mark the actual first bounded Body-rate output before waiting for PX4 ACK."""
        if self.phase == 'Y_HANDOVER' and self.bodyrate_requested is None:
            self.bodyrate_requested = float(now)

    def confirm(self, now, ack_stamp, exclusive):
        """Accept only a fresh PX4 ACK after the actual mode request."""
        self.timeout(now)
        if (self.phase == 'Y_HANDOVER' and self.bodyrate_requested is not None
                and self.bodyrate_requested <= ack_stamp <= now and now-ack_stamp <= .125
                and exclusive):
            self.phase = 'PAPER_INTERCEPT_ACTIVE'
            return True
        return False

    def timeout(self, now):
        """Bound handover latency; never renew it with repeated predicted states."""
        if (self.phase == 'Y_HANDOVER'
                and (not math.isfinite(now) or now-self.y_stamp > self.handover_timeout)):
            self.fail('P76_HANDOVER_TIMEOUT')

    def fail(self, reason):
        """Latch recovery without silently restarting either live target estimator."""
        if self.phase != 'CAPTURED':
            self.phase, self.reason = 'SAFE_RECOVERY', str(reason)

    def captured(self):
        """Only the native evaluator/mission terminal may close the interception."""
        self.phase = 'CAPTURED'
