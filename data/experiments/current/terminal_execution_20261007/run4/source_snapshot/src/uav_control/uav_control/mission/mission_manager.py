"""Pure recoverable mission state machine for modular interception."""

from enum import IntEnum
import math


class MissionPhase(IntEnum):
    """Stable numeric phases shared with MissionState.msg."""

    INIT = 0
    GROUND_HOLD = 1
    TAKEOFF = 2
    FOLLOW = 3
    FAR_GUIDANCE = 4
    MINCO_READY = 5
    MINCO_TRACKING = 6
    TERMINAL_MINCO = 7
    PLAN_RECOVERY = 8
    SAFE_WAIT = 9
    CAPTURE = 10
    FAILURE = 11
    ABORTED = 12
    TARGET_ACQUIRE = 13
    TARGET_LOCK = 14
    REACQUIRE = 15
    SAFE_RECOVERY = 16


class MissionManagerCore:
    """Own commands and state transitions, but no control algorithms."""

    def __init__(
        self,
        maximum_tracker_age=0.125,
        minimum_plan_remaining_time=0.20,
        plan_recovery_timeout=0.50,
        terminal_time_threshold=1.0,
        terminal_distance_threshold=2.0,
        lock_confirmation_duration=0.05,
    ):
        self.maximum_tracker_age = float(maximum_tracker_age)
        self.minimum_plan_remaining_time = float(
            minimum_plan_remaining_time
        )
        self.plan_recovery_timeout = float(plan_recovery_timeout)
        self.terminal_time_threshold = float(terminal_time_threshold)
        self.terminal_distance_threshold = float(terminal_distance_threshold)
        self.lock_confirmation_duration = float(lock_confirmation_duration)
        if (not math.isfinite(self.lock_confirmation_duration)
                or self.lock_confirmation_duration <= 0.0):
            raise ValueError(
                'lock_confirmation_duration must be finite and positive')
        self.mission_id = 0
        self.phase = MissionPhase.INIT
        self.flight_ready = False
        self.intercept_requested = False
        self.completed = False
        self.active_plan_id = 0
        self.last_tracker_accept_time = None
        self.recovery_started_at = None
        self.last_transition_time = 0.0
        self.target_locked = False
        self.terminal_execution_deadline = None
        self.terminal_execution_plan_id = 0

    def _transition(self, phase, now):
        changed = self.phase != MissionPhase(phase)
        self.phase = MissionPhase(phase)
        if self.phase != MissionPhase.TERMINAL_MINCO:
            self.terminal_execution_deadline = None
            self.terminal_execution_plan_id = 0
        if changed:
            self.last_transition_time = float(now)
        return changed

    def set_flight_ready(self, ready, now=0.0):
        """Reflect external PX4 preparation without owning PX4 control."""
        self.flight_ready = bool(ready)
        if self.flight_ready and self.phase == MissionPhase.INIT:
            self._transition(MissionPhase.GROUND_HOLD, now)
            return True
        if not self.flight_ready and self.phase == MissionPhase.GROUND_HOLD:
            self._transition(MissionPhase.INIT, now)
            return True
        return False

    def handle_command(self, command, now):
        """Apply one X/Y/R/Q command and report whether it was accepted."""
        command = str(command).strip().upper()
        now = float(now)
        if command == 'R':
            self.mission_id += 1
            self.intercept_requested = False
            self.completed = False
            self.active_plan_id = 0
            self.last_tracker_accept_time = None
            self.recovery_started_at = None
            self.target_locked = False
            destination = (
                MissionPhase.GROUND_HOLD
                if self.flight_ready
                else MissionPhase.INIT
            )
            self._transition(destination, now)
            return True
        if command == 'Q':
            self.completed = True
            self._transition(MissionPhase.ABORTED, now)
            return True
        if command == 'X':
            if not self.flight_ready or self.phase != MissionPhase.GROUND_HOLD:
                return False
            if self.mission_id == 0:
                self.mission_id = 1
            self._transition(MissionPhase.TAKEOFF, now)
            return True
        if command == 'Y':
            if self.phase not in (
                MissionPhase.FOLLOW, MissionPhase.TARGET_LOCK,
            ) or (self.phase == MissionPhase.TARGET_LOCK
                  and not self.target_locked):
                return False
            self.intercept_requested = True
            if self.target_locked:
                self._transition(MissionPhase.FAR_GUIDANCE, now)
            return True
        return False

    def mark_takeoff_complete(self, now):
        """Enter continuous FOLLOW; RGB/KF quality selects its internal command."""
        if self.phase != MissionPhase.TAKEOFF:
            return False
        self._transition(MissionPhase.FOLLOW, now)
        return True

    def observe_visibility(self, mission_id, state, locked, now,
                           execution_deadline=None, execution_plan_id=0):
        """Consume only the command owner's visual/safety decision."""
        if int(mission_id) != self.mission_id or self.completed:
            return False
        if self.phase in (MissionPhase.INIT, MissionPhase.GROUND_HOLD,
                          MissionPhase.TAKEOFF):
            return False
        self.target_locked = bool(locked)
        if state == 'TERMINAL_COMMITTED':
            if (self.phase != MissionPhase.TERMINAL_MINCO
                    or not self.intercept_requested
                    or execution_plan_id <= 0
                    or execution_plan_id != self.active_plan_id
                    or execution_deadline is None
                    or not math.isfinite(execution_deadline)
                    or not now < execution_deadline <= now + .7 + 1e-9):
                self.active_plan_id = 0
                return self._transition(MissionPhase.SAFE_RECOVERY, now)
            if self.terminal_execution_deadline is None:
                self.terminal_execution_deadline = execution_deadline
                self.terminal_execution_plan_id = execution_plan_id
            else:
                self.terminal_execution_deadline = min(
                    self.terminal_execution_deadline, execution_deadline)
            return False
        if self.phase == MissionPhase.FOLLOW:
            if self.intercept_requested and locked:
                return self._transition(MissionPhase.FAR_GUIDANCE, now)
            # RGB acquisition, centering, and temporary loss stay within
            # FOLLOW. Only a precise lock grants a queued Y request authority.
            return False
        if state == 'SAFE_RECOVERY':
            self.active_plan_id = 0
            return self._transition(MissionPhase.SAFE_RECOVERY, now)
        if not locked and state in ('REACQUIRE', 'SAFE_WAIT'):
            self.active_plan_id = 0
            destination = (MissionPhase.REACQUIRE if state == 'REACQUIRE'
                           else MissionPhase.SAFE_WAIT)
            return self._transition(destination, now)
        if locked and self.phase in (
            MissionPhase.TARGET_ACQUIRE, MissionPhase.REACQUIRE,
            MissionPhase.SAFE_WAIT, MissionPhase.SAFE_RECOVERY,
        ):
            return self._transition(MissionPhase.TARGET_LOCK, now)
        return False

    def observe_planner(self, success, plan_id, now):
        """Record no state: solver success is not tracker acceptance."""
        del success, plan_id, now
        return False

    def observe_tracker(
        self,
        mission_id,
        plan_id,
        status,
        prediction_age,
        remaining_time,
        now,
        target_distance=math.inf,
    ):
        """Accept only a fresh, currently executable tracker decision."""
        now = float(now)
        current_mission = int(mission_id) == self.mission_id
        fresh = (
            0.0 <= float(prediction_age) <= self.maximum_tracker_age
            and float(remaining_time) >= self.minimum_plan_remaining_time
        )
        tracking = (
            str(status) in ('PLAN_ACCEPTED', 'TRACKING')
            and int(plan_id) > 0
        )
        if (
            current_mission
            and fresh
            and tracking
            and self.intercept_requested
            and self.target_locked
            and self.phase not in (
                MissionPhase.REACQUIRE, MissionPhase.SAFE_RECOVERY,
                MissionPhase.TARGET_ACQUIRE, MissionPhase.TARGET_LOCK,
            )
        ):
            new_plan = int(plan_id) != self.active_plan_id
            self.active_plan_id = int(plan_id)
            self.last_tracker_accept_time = now
            self.recovery_started_at = None
            terminal = (
                float(remaining_time) <= self.terminal_time_threshold
                or float(target_distance) <= self.terminal_distance_threshold
            )
            if terminal or self.phase == MissionPhase.TERMINAL_MINCO:
                self._transition(MissionPhase.TERMINAL_MINCO, now)
            elif new_plan or self.phase in (
                MissionPhase.FAR_GUIDANCE,
                MissionPhase.PLAN_RECOVERY,
                MissionPhase.SAFE_WAIT,
            ):
                self._transition(MissionPhase.MINCO_READY, now)
            return True

        if (
            current_mission
            and str(status) == 'NO_VALID_PLAN'
            and self.phase in (
                MissionPhase.MINCO_READY,
                MissionPhase.MINCO_TRACKING,
                MissionPhase.TERMINAL_MINCO,
            )
        ):
            self.recovery_started_at = now
            self._transition(MissionPhase.PLAN_RECOVERY, now)
        return False

    def tick(self, now, far_guidance_available=False):
        """Advance transient and timeout-driven recoverable phases."""
        now = float(now)
        if (self.terminal_execution_deadline is not None
                and now >= self.terminal_execution_deadline):
            self.active_plan_id = 0
            self._transition(MissionPhase.SAFE_RECOVERY, now)
            return self.phase
        if (
            self.phase == MissionPhase.TARGET_LOCK
            and self.target_locked
            and now - self.last_transition_time + 1e-9
            >= self.lock_confirmation_duration
        ):
            destination = (
                MissionPhase.FAR_GUIDANCE if self.intercept_requested
                else MissionPhase.FOLLOW)
            self._transition(destination, now)
        elif self.phase == MissionPhase.MINCO_READY:
            self._transition(MissionPhase.MINCO_TRACKING, now)
        elif (
            self.phase == MissionPhase.PLAN_RECOVERY
            and self.recovery_started_at is not None
            and now - self.recovery_started_at >= self.plan_recovery_timeout
        ):
            self._transition(MissionPhase.SAFE_WAIT, now)
        elif (
            self.phase == MissionPhase.SAFE_WAIT
            and bool(far_guidance_available)
            and self.target_locked
            and self.intercept_requested
        ):
            self._transition(MissionPhase.FAR_GUIDANCE, now)
        return self.phase

    def mark_capture(self, now):
        """Reserved external/manual completion; evaluator never invokes it."""
        self.completed = True
        self._transition(MissionPhase.CAPTURE, now)

    def mark_failure(self, now):
        """Reserved external/manual failure; evaluator never invokes it."""
        self.completed = True
        self._transition(MissionPhase.FAILURE, now)
