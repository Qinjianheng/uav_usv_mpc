"""Pure future handover dry-run. No transport, control acknowledgement or TTL renewal."""
from dataclasses import dataclass
import math

import numpy as np


@dataclass(frozen=True)
class CandidateContract:
    """Explicit original-source expiry, coverage and fully checked planning curve."""

    trajectory_id: str
    mission_id: int
    clock_generation: int
    source_stamp: float
    execution_start: float
    execution_end: float
    input_valid_until: float
    prediction_coverage_end: float
    handover_state: tuple  # P/V/A/yaw/yaw-rate, length 11
    validated: bool
    state: str = 'VALIDATED'
    holding_valid_until: float = 0.
    safety_model_id: str = ''
    prediction_sequence_id: int = 0
    revalidated_sequence_id: int = 0


@dataclass(frozen=True)
class OldContract:
    """Only a recorded authorized acceptance and actual trajectory sample are usable."""

    trajectory_id: str
    mission_id: int
    clock_generation: int
    execution_start: float
    valid_until: float
    acceptance_state: str
    handover_state: tuple
    sample_stamp: float


@dataclass(frozen=True)
class HandoverResult:
    """ELIGIBLE_DRY_RUN never means Tracker accepted or an execution holding contract."""

    eligible: bool = False
    reason: str = ''
    state: str = 'REJECTED'
    accepted_by_tracker: bool = False


def transition(state, target, receiver_acknowledged=False):
    """
    Check lifecycle transitions; shadow callers cannot acknowledge actual acceptance.

    Expired/rejected identities are terminal. A new proposal needs a new identity
    and original input validation; changing a label cannot revive an old curve.
    """
    allowed = {'PROPOSED': ('VALIDATED', 'REJECTED', 'EXPIRED'),
               'VALIDATED': ('ELIGIBLE', 'REJECTED', 'EXPIRED'),
               'ELIGIBLE': ('ACCEPTED', 'REJECTED', 'EXPIRED'),
               'ACCEPTED': ('ACTIVE', 'EXPIRED'), 'ACTIVE': ('EXPIRED',)}
    if target not in allowed.get(state, ()):
        raise ValueError('ILLEGAL_TRANSITION')
    if target == 'ACCEPTED' and not receiver_acknowledged:
        raise ValueError('AUTHORIZED_RECEIVER_ACK_REQUIRED')
    return target


def dry_run(candidate, old, now, mission_id, generation, follow,
            tolerances=(1e-3, 1e-3, 1e-3, 1e-3, 1e-3)):
    """Fail closed on identity, expiry, coverage and measured old-trajectory continuity."""
    c = candidate
    reason = ''
    stamps = (now, c.source_stamp, c.execution_start, c.execution_end,
              c.input_valid_until, c.prediction_coverage_end)
    if (not all(math.isfinite(x) and x > 0 for x in stamps) or not c.trajectory_id
            or len(c.handover_state) != 11 or not np.all(np.isfinite(c.handover_state))
            or len(tolerances) != 5 or not np.all(np.isfinite(tolerances))
            or min(tolerances) < 0 or c.mission_id < 1 or c.clock_generation < 0
            or c.source_stamp > now or c.execution_end <= c.execution_start
            or c.input_valid_until > c.source_stamp+.125+1e-9):
        reason = 'INVALID_CONTRACT'
    elif c.state == 'EXPIRED':
        reason = 'CANDIDATE_EXPIRED'
    elif not follow:
        reason = 'MISSION_NOT_FOLLOW'
    elif mission_id != c.mission_id:
        reason = 'MISSION_CHANGED'
    elif generation != c.clock_generation:
        reason = 'CLOCK_GENERATION_CHANGED'
    elif now >= c.input_valid_until:
        reason = 'INPUT_EXPIRED'
    elif now >= c.execution_start:
        reason = 'EXECUTION_START_MISSED'
    elif (c.execution_start < c.source_stamp
          or c.execution_end > c.prediction_coverage_end):
        reason = 'PREDICTION_COVERAGE'
    elif not c.validated or c.state not in ('VALIDATED', 'ELIGIBLE'):
        reason = 'CANDIDATE_NOT_VALIDATED'
    elif old is None:
        reason = 'OLD_TRAJECTORY_UNAVAILABLE'
    elif (not old.trajectory_id or old.acceptance_state not in ('ACCEPTED', 'ACTIVE')
          or old.mission_id != mission_id or old.clock_generation != generation
          or not all(math.isfinite(x) and x > 0 for x in (
              old.execution_start, old.valid_until, old.sample_stamp))
          or len(old.handover_state) != 11 or not np.all(np.isfinite(old.handover_state))):
        reason = 'OLD_TRAJECTORY_UNAUTHORIZED'
    elif not old.execution_start <= c.execution_start < old.valid_until:
        reason = 'OLD_TRAJECTORY_EXPIRED'
    elif abs(old.sample_stamp-c.execution_start) > 1e-9:
        reason = 'HANDOVER_EPOCH_MISMATCH'
    else:
        delta = np.asarray(c.handover_state)-np.asarray(old.handover_state)
        delta[9] = math.atan2(math.sin(delta[9]), math.cos(delta[9]))
        norms = (np.linalg.norm(delta[:3]), np.linalg.norm(delta[3:6]),
                 np.linalg.norm(delta[6:9]), abs(delta[9]), abs(delta[10]))
        if any(x > limit for x, limit in zip(norms, tolerances)):
            reason = 'HANDOVER_DISCONTINUITY'
        elif c.prediction_sequence_id != c.revalidated_sequence_id:
            reason = 'PREDICTION_UPDATED_WITHOUT_REVALIDATION'
        elif (not c.safety_model_id or not math.isfinite(c.holding_valid_until)
              or not c.execution_end <= c.holding_valid_until <= c.prediction_coverage_end):
            reason = 'HOLDING_CONTRACT_UNESTABLISHED'
    return HandoverResult(not reason, reason or 'DRY_RUN_ONLY',
                          'ELIGIBLE_DRY_RUN' if not reason else 'REJECTED')
