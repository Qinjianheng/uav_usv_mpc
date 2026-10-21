"""Schema2 guidance authority is independent of estimator activity and the PX4 interface."""
import math


class ModeCoordinator:
    """Keep Body-rate through Y; a safety latch can never reauthorize MINCO or PNG."""

    def __init__(self, handover_timeout=.5):
        self.phase, self.epoch, self.y_stamp = 'FOLLOW_ACTIVE', 0, 0.
        self.bodyrate_requested = None
        self.reason, self.handover_timeout = '', float(handover_timeout)

    @property
    def guidance_owner(self):
        """Permission to consume an estimate is separate from estimate validity."""
        owners = {'FOLLOW_ACTIVE': 'MINCO', 'PAPER_INTERCEPT_ACTIVE': 'PAPER'}
        return owners.get(self.phase, 'NONE')

    def accept_y(self, now, qualified):
        """Only the accepted mission event may atomically withdraw FOLLOW authority."""
        if self.phase != 'FOLLOW_ACTIVE':
            return
        self.epoch += 1
        self.y_stamp = float(now)
        self.phase = 'Y_HANDOVER'
        if not qualified:
            self.fail('P76_BODYRATE_QUALIFICATION_REQUIRED')

    def ready(self, now, follow_revoked, image_source, native_fresh, image_ready=True):
        """Warm image history stays valid; no spatial shutdown or post-Y reset is required."""
        self.timeout(now)
        return bool(self.phase == 'Y_HANDOVER' and follow_revoked and image_ready and native_fresh
                    and 0 < image_source <= now and now-image_source <= .125)

    def request_bodyrate(self, now):
        """Mark authority handover only; the physical interface was already Body-rate."""
        if self.phase == 'Y_HANDOVER' and self.bodyrate_requested is None:
            self.bodyrate_requested = float(now)

    def confirm(self, now, ack_stamp, exclusive, maximum_mode_age=.65):
        """Accept an unchanged Body-rate mode with a fresh native ACK preceding Y."""
        self.timeout(now)
        if (self.phase == 'Y_HANDOVER' and self.bodyrate_requested is not None
                and 0 < ack_stamp <= now and now-ack_stamp <= maximum_mode_age and exclusive):
            self.phase = 'PAPER_INTERCEPT_ACTIVE'
            return True
        return False

    def timeout(self, now):
        """Bound guidance transfer without extending navigation or image validity."""
        if (self.phase == 'Y_HANDOVER'
                and (not math.isfinite(now) or now-self.y_stamp > self.handover_timeout)):
            self.fail('P76_HANDOVER_TIMEOUT')

    def fail(self, reason):
        """Revoke both target guidance authorities on a safety transition."""
        if self.phase != 'CAPTURED':
            self.phase, self.reason = 'SAFE_RECOVERY', str(reason)

    def captured(self):
        """Only independent native evaluator/mission terminal may close capture."""
        self.phase = 'CAPTURED'
