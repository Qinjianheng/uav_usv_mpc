#!/usr/bin/env python3
"""Bounded single-writer diagnostic storage outside ROS/control callbacks."""
import gzip
import json
import queue
import threading
import time


class BoundedRecorder:
    """Retain immutable input packet references; overflow is an explicit session failure."""

    def __init__(self, output, convert, capacity=4096):
        self.output, self.convert = output, convert
        self.queue = queue.Queue(maxsize=capacity)
        self.error, self.overflows, self.high_water = None, 0, 0
        self.serialization_seconds, self.serialization_spans = [], []
        self.closed = False
        self.thread = threading.Thread(target=self._run, name='diagnostic_writer', daemon=True)
        self.thread.start()

    def submit(self, stream, packet):
        """Never wait for disk/serialization; preserve failure when the bounded queue fills."""
        if self.closed or self.error is not None:
            return False
        try:
            self.queue.put_nowait((stream, packet))
            self.high_water = max(self.high_water, self.queue.qsize())
            return True
        except queue.Full:
            self.overflows += 1
            return False

    def _run(self):
        streams = {}
        try:
            for key, filename in (('packets', 'synchronized_packets.jsonl.gz'),
                                  ('callbacks', 'actual_callback_inputs.jsonl.gz')):
                streams[key] = gzip.open(self.output/filename, 'xt')
            while True:
                item = self.queue.get()
                if item is None:
                    break
                began = time.perf_counter()
                key, packet = item
                streams[key].write(json.dumps(self.convert(packet))+'\n')
                ended = time.perf_counter()
                self.serialization_seconds.append(ended-began)
                self.serialization_spans.append((began, ended))
        except Exception as error:
            self.error = error
        finally:
            for stream in streams.values():
                stream.close()

    def close(self):
        """Drain after executor spin stops; this is never called in the 200 Hz timer."""
        self.closed = True
        if self.thread.is_alive():
            try:
                self.queue.put(None, timeout=5.)
            except queue.Full as error:
                raise RuntimeError('DIAGNOSTIC_RECORDER_DRAIN_TIMEOUT') from error
        self.thread.join(timeout=10.)
        if self.thread.is_alive() or self.error is not None:
            raise RuntimeError('DIAGNOSTIC_RECORDER_FAILED') from self.error
