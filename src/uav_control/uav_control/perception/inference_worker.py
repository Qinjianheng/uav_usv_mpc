"""A single coalescing worker lets ROS receive newer frames during CPU inference."""
import hashlib
import inspect
import marshal
import os
from pathlib import Path
import sys
import threading


def loaded_code_identity(functions):
    """Identify this process's loaded code, separately from potentially changed disk bytes."""
    loaded = {}
    for function in functions:
        path = Path(inspect.getfile(function)).resolve()
        loaded[function.__qualname__] = dict(
            code_sha256=hashlib.sha256(marshal.dumps(function.__code__)).hexdigest(),
            source_path=str(path), source_sha256=hashlib.sha256(path.read_bytes()).hexdigest())
    return dict(pid=os.getpid(), executable=sys.executable, python=sys.version, loaded_code=loaded)


class TriggeredWorker:
    """Run one callback at a time; the caller owns its bounded latest-frame slot."""

    def __init__(self, callback):
        self.callback, self.error = callback, None
        self.wake, self.stop = threading.Event(), threading.Event()
        self.thread = threading.Thread(target=self._run, name='yolo_single_inference', daemon=True)
        self.thread.start()

    def trigger(self):
        """Coalesce notifications without queuing images or running a second model copy."""
        self.wake.set()

    def _run(self):
        try:
            while not self.stop.is_set():
                # Periodic wake expires a pending frame even if the trusted clock stops.
                self.wake.wait(.025)
                self.wake.clear()
                if not self.stop.is_set():
                    self.callback()
        except Exception as error:
            self.error = error

    def close(self):
        """Stop before ROS publishers are destroyed; never silently orphan an active worker."""
        self.stop.set()
        self.wake.set()
        self.thread.join(timeout=5.)
        if self.thread.is_alive():
            raise RuntimeError('YOLO_WORKER_SHUTDOWN_TIMEOUT')
