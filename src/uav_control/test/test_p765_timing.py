"""Bounded inference/recording workers must not block ROS reception or hide losses."""
import threading

from uav_control.perception.inference_worker import TriggeredWorker


def test_worker_allows_reception_during_inference_and_coalesces_pending_frames():
    entered, release, latest_done = threading.Event(), threading.Event(), threading.Event()
    lock, pending, processed = threading.Lock(), [1], []

    def process():
        with lock:
            frame, pending[0] = pending[0], None
        if frame is None:
            return
        processed.append(frame)
        if frame == 1:
            entered.set()
            assert release.wait(2)
        if frame == 3:
            latest_done.set()

    worker = TriggeredWorker(process)
    try:
        worker.trigger()
        assert entered.wait(2)
        with lock:
            pending[0] = 2
        worker.trigger()
        with lock:
            pending[0] = 3
        worker.trigger()
        release.set()
        assert latest_done.wait(2)
    finally:
        release.set()
        worker.close()
    assert processed == [1, 3]


def test_worker_failure_is_explicit_and_close_is_bounded():
    failed = threading.Event()

    def process():
        failed.set()
        raise ValueError('synthetic inference crash')

    worker = TriggeredWorker(process)
    worker.trigger()
    assert failed.wait(2)
    worker.close()
    assert isinstance(worker.error, ValueError)


def test_recorder_serialization_cannot_block_receiver_and_overflow_is_explicit(tmp_path):
    import json
    import gzip
    from test_p761_calibration import script
    m = script('p765_recording')
    entered, release = threading.Event(), threading.Event()

    def convert(packet):
        entered.set()
        assert release.wait(2)
        return packet

    recorder = m.BoundedRecorder(tmp_path, convert, capacity=1)
    try:
        assert recorder.submit('packets', {'stamp': 1})
        assert entered.wait(2)
        assert recorder.submit('packets', {'stamp': 2})
        assert not recorder.submit('packets', {'stamp': 3})
        assert recorder.overflows == 1
    finally:
        release.set()
        recorder.close()
    rows = [json.loads(line) for line in gzip.open(tmp_path/'synchronized_packets.jsonl.gz', 'rt')]
    assert rows == [{'stamp': 1}, {'stamp': 2}]


def test_stale_before_inference_is_not_reported_as_measured_inference_delay():
    from types import SimpleNamespace as Row
    from sensor_msgs.msg import Image
    from builtin_interfaces.msg import Time
    from uav_control.perception.yolo_target_detector import YoloTargetDetector
    image = Image(height=480, width=640, step=1920, encoding='rgb8')
    image.header.stamp = Time(sec=2, nanosec=123)
    publications = []
    state = Row(lock=threading.RLock(), pending=(image, 7, 0, 0., 10.12),
                clock=Row(map_time=lambda *args: (10., '')), clock_resets=0,
                waiting_clock=False, now=lambda: 10.13, replaced_frames=3,
                runtime_identity={'test_fixture': True},
                contract={'detector': {'model_sha256': 'a'*64},
                          'camera': {'width': 640, 'height': 480, 'horizontal_fov': 1.047}},
                publisher=Row(publish=publications.append),
                info_pub=Row(publish=lambda m: None), status_pub=Row(publish=publications.append))
    YoloTargetDetector.process(state)
    assert publications[0].rejection_reason == 'YOLO_SOURCE_STALE_BEFORE_INFERENCE'
    assert publications[0].raw_stamp == image.header.stamp
    assert publications[0].stamp == Time(sec=10)
    assert not publications[0].valid
    import json
    assert json.loads(publications[-1].data)['inference_executed'] is False


def test_nonflight_capture_does_not_launch_without_separate_authorization(tmp_path):
    import pytest
    from test_p761_calibration import script
    module = script('p765_nonflight_session')
    with pytest.raises(ValueError, match='SEPARATE_HUMAN_AUTHORIZATION'):
        module.execute(tmp_path/'nonflight', '')
    assert not (tmp_path/'nonflight').exists()


def test_resource_monitor_keeps_pid_start_identity_and_requires_exact_run_env(tmp_path):
    from test_p761_calibration import script
    m = script('p765_resource_monitor')
    proc = tmp_path/'proc'
    path = proc/'123'
    path.mkdir(parents=True)
    fields = ['0']*22
    fields[11], fields[12], fields[17], fields[19], fields[21] = '10', '20', '2', '987', '32'
    (path/'stat').write_text('123 (has spaces) '+' '.join(fields))
    (proc/'stat').write_text('cpu 10 0 2 30 0 0 0 0 0 0\ncpu0 10 0 2 30 0 0 0 0 0 0\n')
    (path/'comm').write_text('yolo')
    (path/'cmdline').write_bytes(b'python\0detector.py\0')
    (path/'environ').write_bytes(b'UAV_USV_RESEARCH_DIRECTORY=/different\0')
    assert m.sample(tmp_path, proc=proc)['processes'] == []
    (path/'environ').write_bytes(('UAV_USV_RESEARCH_DIRECTORY='+str(tmp_path)+'\0').encode())
    snapshot = m.sample(tmp_path, proc=proc)
    assert snapshot['cpu_ticks_per_core']['cpu0'][:4] == [10, 0, 2, 30]
    row = snapshot['processes'][0]
    assert row['pid'] == 123 and row['start_ticks'] == 987 and row['threads'] == 2
    assert row['cpu_seconds'] > 0 and row['rss_bytes'] > 0


def test_capture_manifest_existence_does_not_prove_raw_capture_completed(tmp_path):
    import json
    import pytest
    from test_p761_calibration import script
    module = script('p765_nonflight_session')
    (tmp_path/'manifest.json').write_text(json.dumps(dict(saved_samples=0, files={})))
    with pytest.raises(ValueError, match='P765_ROI'):
        module.validate_capture_artifacts(tmp_path)


def test_capture_requires_completed_terminal_and_hash_valid_float32_sample(tmp_path):
    import hashlib
    import json
    import numpy as np
    import pytest
    from test_p761_calibration import script
    module = script('p765_nonflight_session')
    np.savez_compressed(tmp_path/'frame.npz',
                        depth=np.array([[float('nan'), 3]], dtype=np.float32),
                        rgb=np.zeros((1, 2, 3), np.uint8), mask=np.ones((1, 2), bool))
    stamp = dict(sec=10, nanosec=1)
    (tmp_path/'frame.json').write_text(json.dumps(dict(
        rgb_raw_stamp=stamp, depth_raw_stamp=stamp, detection_raw_stamp=stamp)))
    event = dict(reason='SESSION_END', metadata=dict(duration_s=30., failure=''))
    (tmp_path/'events.jsonl').write_text(json.dumps(event)+'\n')
    files = {name: hashlib.sha256((tmp_path/name).read_bytes()).hexdigest()
             for name in ('events.jsonl', 'frame.npz', 'frame.json')}
    manifest = dict(saved_samples=1, files=files)
    (tmp_path/'manifest.json').write_text(json.dumps(manifest))
    assert module.validate_capture_artifacts(tmp_path)['completed']
    (tmp_path/'events.jsonl').write_text(json.dumps(dict(event, metadata=dict(
        duration_s=1., failure='INTERRUPTED')))+'\n')
    manifest['files']['events.jsonl'] = hashlib.sha256(
        (tmp_path/'events.jsonl').read_bytes()).hexdigest()
    (tmp_path/'manifest.json').write_text(json.dumps(manifest))
    with pytest.raises(ValueError, match='P765_ROI'):
        module.validate_capture_artifacts(tmp_path)


def test_clock_reset_during_packet_build_cannot_publish_old_valid_detection(monkeypatch):
    from types import SimpleNamespace as Row
    from sensor_msgs.msg import Image
    from builtin_interfaces.msg import Time
    from uav_control.perception import yolo_target_detector as module
    from uav_control.perception.yolo_inference import Detection
    image = Image(height=480, width=640, step=1920, encoding='rgb8', data=bytes(640*480*3))
    image.header.stamp = Time(sec=2)
    published = []
    state = Row(lock=threading.RLock(), pending=(image, 7, 0, 0., 10.01),
                clock=Row(map_time=lambda *args: (10., '')), clock_resets=0,
                waiting_clock=False, now=lambda: 10.1, replaced_frames=0,
                runtime_identity={'test_fixture': True},
                inference=Row(infer=lambda rgb: Detection(True, 'VALID')),
                contract={'detector': {'model_sha256': 'a'*64},
                          'camera': {'width': 640, 'height': 480, 'horizontal_fov': 1.047}},
                publisher=Row(publish=published.append), info_pub=Row(publish=lambda m: None),
                status_pub=Row(publish=lambda m: None))
    original = module.time_message

    def reset_during_build(value):
        state.clock_resets = 1
        return original(value)

    monkeypatch.setattr(module, 'time_message', reset_during_build)
    module.YoloTargetDetector.process(state)
    assert len(published) == 1
    assert not published[0].valid
    assert published[0].rejection_reason == 'CLOCK_RESET'


def test_capture_cli_returns_failure_when_capture_or_e1_fails(tmp_path, monkeypatch):
    import sys
    from test_p761_calibration import script
    module = script('p765_nonflight_session')
    monkeypatch.setattr(module, 'execute', lambda *a: dict(success=False))
    arguments = ['p765_nonflight_session', '--output', str(tmp_path),
                 '--execute-once', '--authorization', 'TEST_ONLY']
    monkeypatch.setattr(sys, 'argv', arguments)
    assert module.main() == 2
