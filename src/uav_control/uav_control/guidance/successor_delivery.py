"""Bounded delivery opportunities; no trajectory admission or lease authority."""
from dataclasses import dataclass
import math


@dataclass
class DeliveryWindow:
    """One normal attempt and one fresh-input retry for one actual receiver parent."""

    identity: object
    attempts: int = 0
    observation: float = -math.inf
    failed: bool = False
    plan: int = 0
    retry_at: float = math.inf

    def eligible(self, observation, now):
        """Allow a missing-ACK retry without inferring execution authority."""
        return (observation > self.observation and self.attempts < 2
                and (self.attempts == 0 or self.failed or now >= self.retry_at))

    def dispatched(self, observation, plan, start):
        """Count submitted workers only; a budget wait consumes no opportunity."""
        self.attempts += 1
        self.observation, self.plan = observation, plan
        self.failed, self.retry_at = False, start-.05
