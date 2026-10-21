"""Y requires fresh post-command imagery and a real exclusive PX4 mode acknowledgement."""
from uav_control.control.p76_mode_coordinator import ModeCoordinator


def test_handover_requires_inactive_spatial_fresh_image_and_px4_ack():
    mode = ModeCoordinator(.5)
    mode.accept_y(10., qualified=True)
    assert mode.phase == 'Y_HANDOVER'
    assert not mode.ready(10.05, spatial_inactive=True, image_source=9.99, native_fresh=True)
    assert mode.ready(10.05, spatial_inactive=True, image_source=10.01, native_fresh=True)
    mode.request_bodyrate(10.05)
    assert not mode.confirm(10.06, ack_stamp=10.04, exclusive=True)
    assert mode.confirm(10.07, ack_stamp=10.06, exclusive=True)
    assert mode.phase == 'PAPER_INTERCEPT_ACTIVE'
    mode.fail('IMAGE_STALE')
    assert mode.phase == 'SAFE_RECOVERY'
    assert not mode.confirm(10.08, ack_stamp=10.08, exclusive=True)


def test_unqualified_or_timed_out_handover_latches_recovery():
    mode = ModeCoordinator(.5)
    mode.accept_y(1., qualified=False)
    assert mode.phase == 'SAFE_RECOVERY'
    mode = ModeCoordinator(.5)
    mode.accept_y(2., qualified=True)
    mode.timeout(2.51)
    assert mode.phase == 'SAFE_RECOVERY'


def test_actual_px4_serializer_selects_only_body_rate_and_normalized_thrust():
    from types import SimpleNamespace
    from uav_control.control.p76_tracker_runtime import P76TrackerRuntime
    modes, rates = [], []
    runtime = P76TrackerRuntime.__new__(P76TrackerRuntime)
    runtime.node = SimpleNamespace(_timestamp_us=lambda: 10_000_000,
                                   offboard_pub=SimpleNamespace(publish=modes.append))
    runtime.rates_pub = SimpleNamespace(publish=rates.append)
    runtime._emit_rates([.1, .2, .3], -.5, 10.)
    assert modes[0].body_rate
    assert not any((modes[0].position, modes[0].velocity, modes[0].acceleration,
                    modes[0].attitude))
    assert list(rates[0].thrust_body) == [0., 0., -.5]


def test_image_loss_after_bodyrate_prestream_latches_recovery_before_velocity_output():
    from types import SimpleNamespace
    from uav_control.control.p76_tracker_runtime import P76TrackerRuntime
    runtime = P76TrackerRuntime.__new__(P76TrackerRuntime)
    runtime.node = SimpleNamespace(_ros_seconds=lambda: 10.1, navigation_frame_fault='')
    runtime.mode = ModeCoordinator(.5)
    runtime.mode.accept_y(10., qualified=True)
    runtime.mode.request_bodyrate(10.05)
    runtime.initialized = True
    runtime.contract = {'paper': {'paper_controller_target_rate_hz': 200.}}
    runtime.last_tick, runtime.last_report = 10.095, 10.1
    runtime.fatal_dispatched = False
    runtime._image_valid = lambda now: False
    recoveries = []
    runtime._recovery = lambda now, dt: recoveries.append(runtime.mode.phase)
    runtime.tick()
    assert runtime.mode.reason == 'P76_HANDOVER_IMAGE_SOURCE_STALE'
    assert recoveries == ['SAFE_RECOVERY']
    assert not runtime.mode.confirm(10.11, 10.1, exclusive=True)
