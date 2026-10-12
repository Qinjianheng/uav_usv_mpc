"""Reliable terminal Gazebo pause action independent of target rendering."""

from dataclasses import dataclass
import time


@dataclass(frozen=True)
class GazeboPauseResult:
    """Auditable result of one terminal pause action."""

    terminal_event: str
    gazebo_pause_requested: bool
    gazebo_pause_succeeded: bool
    attempts: int


class GazeboTerminalPauser:
    """Retry a bounded external Gazebo pause request after a terminal event."""

    def __init__(
        self,
        pause_request,
        maximum_attempts=2,
        retry_delay=0.05,
        sleep=time.sleep,
    ):
        self.pause_request = pause_request
        self.maximum_attempts = max(int(maximum_attempts), 1)
        self.retry_delay = max(float(retry_delay), 0.0)
        self.sleep = sleep

    def pause(self, terminal_event):
        succeeded = False
        attempts = 0
        for attempts in range(1, self.maximum_attempts + 1):
            try:
                succeeded = bool(self.pause_request())
            except (RuntimeError, TypeError, ValueError):
                succeeded = False
            if succeeded:
                break
            if attempts < self.maximum_attempts and self.retry_delay > 0.0:
                self.sleep(self.retry_delay)
        return GazeboPauseResult(
            terminal_event=str(terminal_event),
            gazebo_pause_requested=True,
            gazebo_pause_succeeded=succeeded,
            attempts=attempts,
        )


class GazeboWorldPauseClient:
    """Small Gazebo Transport client dedicated only to world control."""

    def __init__(self, world_name='default', request_timeout_ms=250):
        from gz.msgs10.boolean_pb2 import Boolean
        from gz.msgs10.world_control_pb2 import WorldControl
        from gz.transport13 import Node as GazeboTransportNode

        world_name = str(world_name).strip().strip('/')
        if not world_name:
            raise ValueError('Gazebo world name must not be empty')
        self._boolean_type = Boolean
        self._world_control_type = WorldControl
        self._node = GazeboTransportNode()
        self._service = f'/world/{world_name}/control'
        self._timeout_ms = max(int(request_timeout_ms), 1)

    def pause_world(self):
        request = self._world_control_type()
        request.pause = True
        executed, response = self._node.request(
            self._service,
            request,
            self._world_control_type,
            self._boolean_type,
            self._timeout_ms,
        )
        return bool(executed and response and response.data)
