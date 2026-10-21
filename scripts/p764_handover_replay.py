#!/usr/bin/env python3
"""Finite P764 software-only E2 tests; no executor, simulator, or PX4 publication."""
import argparse
from collections import Counter, deque
import json
import os
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import patch

import numpy as np
from rclpy.time import Time
from scipy.spatial.transform import Rotation
from uav_usv_interfaces.msg import TargetImageState

from uav_control.common.p76_contract import load_contract
from uav_control.common.p76_native import NativePoseHistory, NativeReferenceHistory
from uav_control.common.p76_readiness import sha256
from uav_control.control.p76_bodyrate_safety import BodyrateSeaBarrier
from uav_control.control.p76_minco_bodyrate_tracker import BodyrateTracker, sample_reference
from uav_control.control.p76_mode_coordinator import ModeCoordinator
from uav_control.control.p76_paper_controller import PaperController
from uav_control.control.p76_paper_thrust import CalibratedThrustMap
from uav_control.control.p76_recovery import BodyrateRecovery
from uav_control.control.p76_tracker_runtime import P76TrackerRuntime
from uav_control.guidance.follow_contract import FollowCurve

ROOT = Path(__file__).resolve().parents[1]


def engineering_tracker(contract):
    """Explicit ideal test map, not measured thrust evidence or a saved qualification."""
    mapping = CalibratedThrustMap([0., 19.6, 60.], [0., .5, 1.], qualified=True)
    tracker = BodyrateTracker(contract, 2., 60., mapping)
    tracker.initialize(np.zeros(3), -.5)
    return tracker


def fixed_curve():
    """A complete synthetic polynomial reference with native NED/seconds/radians units."""
    xyz = ((0., 0., -20.), (4., 0., 0.), (0., 0., 0.),
           (0., 0., 0.), (0., 0., 0.), (0., 0., 0.))
    return FollowCurve(7, 1, 1, 1, 0, 10., 10., 10., 10., 11., 11., 10., 11., 11.,
                       (1.,), xyz, ((0., .2, 0., 0.),))


def control_checks(contract):
    """Predetermined axis signs, limit probes and four 20 s ideal-actuator response cases."""
    cases = []
    # FRD rates: NED +X needs negative pitch; NED +Y needs positive roll.
    for feedback in ('position', 'velocity'):
        for axis in range(3):
            for sign in (-1., 1.):
                tracker = engineering_tracker(contract)
                ref = np.zeros(11)
                ref[axis+(0 if feedback == 'position' else 3)] = sign
                out = tracker.step(ref, np.zeros(3), np.zeros(3), np.eye(3), .005, np.zeros(3))
                correct = out['acceleration'][axis]*sign > 0
                if axis == 0:
                    correct &= out['rates'][1]*sign < 0
                elif axis == 1:
                    correct &= out['rates'][0]*sign > 0
                else:
                    correct &= (out['thrust']+.5)*sign > 0
                cases.append(dict(feedback=feedback, axis='NED_'+'xyz'[axis], sign=sign,
                                  acceleration=out['acceleration'].tolist(),
                                  rates=out['rates'].tolist(), thrust=out['thrust'],
                                  passed=bool(correct)))
    transforms = []
    yaw90 = np.array([[0., -1., 0.], [1., 0., 0.], [0., 0., 1.]])
    for axis, rate_axis in ((0, 0), (1, 1)):
        for sign in (-1., 1.):
            tracker = engineering_tracker(contract)
            ref = np.zeros(11)
            ref[axis], ref[9] = sign, np.pi/2
            out = tracker.step(ref, np.zeros(3), np.zeros(3), yaw90, .005, np.zeros(3))
            transforms.append(dict(ned_axis='xy'[axis], sign=sign, heading=np.pi/2,
                                   rates=out['rates'].tolist(),
                                   passed=bool(out['rates'][rate_axis]*sign < 0)))
    curve = fixed_curve()
    reference = sample_reference(curve, 10.2, 10.5)
    expected = [0.8, 0., -20., 4., 0., 0., 0., 0., 0., .04, .2]
    same_epoch = bool(np.allclose(reference, expected))
    lease_rejected, invalid_rejected = False, False
    try:
        sample_reference(curve, 10.5, 10.5)
    except ValueError:
        lease_rejected = True
    try:
        sample_reference(SimpleNamespace(sample=lambda epoch: np.zeros(10)), 10.2, 10.5)
    except ValueError:
        invalid_rejected = True
    tracker = engineering_tracker(contract)
    trace, previous = [], (np.zeros(3), -.5, np.zeros(3))
    limits = np.array([contract['candidate']['maximum_body_roll_pitch_rate']]*2
                      + [contract['candidate']['maximum_body_yaw_rate']])
    valid = True
    for index in range(800):
        ref = np.zeros(11)
        ref[:3] = [100., -100., -100.] if index < 400 else [-100., 100., 100.]
        ref[9:11] = [1., 10.] if index < 400 else [-1., -10.]
        out = tracker.step(ref, np.zeros(3), np.zeros(3), np.eye(3), .005, np.zeros(3))
        rates, thrust, acceleration = out['rates'], out['thrust'], out['acceleration']
        valid &= bool(np.all(np.abs(rates) <= limits+1e-9) and -1 <= thrust < 0
                      and np.linalg.norm(acceleration) <= 5.+1e-9
                      and np.linalg.norm(rates-previous[0]) <= .015+1e-9
                      and abs(thrust-previous[1]) <= .01+1e-9
                      and np.linalg.norm(acceleration-previous[2]) <= .015+1e-9)
        previous = rates.copy(), thrust, acceleration.copy()
        trace.append(dict(rates=rates.tolist(), thrust=thrust,
                          acceleration=acceleration.tolist()))
    responses = []
    for lag in (.02, .12, .24, .5):
        tracker = engineering_tracker(contract)
        p, v, omega = np.array([-1., .5, .5]), np.zeros(3), np.zeros(3)
        rotation, force, maximum_error, maximum_speed = np.eye(3), 19.6, 0., 0.
        for _ in range(4000):
            out = tracker.step(np.zeros(11), p, v, rotation, .005, omega)
            omega += .005*(out['rates']-omega)/lag
            requested = np.interp(-out['thrust'], [0., .5, 1.], [0., 19.6, 60.])
            force += .005*(requested-force)/lag
            rotation = rotation@Rotation.from_rotvec(omega*.005).as_matrix()
            acceleration = np.array([0., 0., 9.8])-rotation[:, 2]*force/2.
            v += .005*acceleration
            p += .005*v
            maximum_error = max(maximum_error, float(np.linalg.norm(p)))
            maximum_speed = max(maximum_speed, float(np.linalg.norm(v)))
        responses.append(dict(lag_seconds=lag, duration_seconds=20., dt_seconds=.005,
                              initial_error=float(np.linalg.norm([-1., .5, .5])),
                              final_error=float(np.linalg.norm(p)),
                              final_speed=float(np.linalg.norm(v)), maximum_error=maximum_error,
                              maximum_speed=maximum_speed, passed=bool(np.linalg.norm(p) < .15)))
    return dict(schema_version=2, direction_cases=cases, response_cases=responses,
                coordinate_transform_cases=transforms,
                same_epoch_pva_yaw_verified=same_epoch, constraints_passed=valid,
                constraint_probe_outputs=len(trace), lease_expiry_rejected=lease_rejected,
                invalid_reference_rejected=invalid_rejected,
                simplified_plant=('2 kg point mass; ideal calibrated piecewise thrust map; '
                                  'first-order force/rate actuator lag; no PX4/Gazebo'),
                gains_fitted_to_history=False, flight_qualified=False,
                passed=bool(all(r['passed'] for r in cases+transforms+responses)
                            and same_epoch and valid
                            and lease_rejected and invalid_rejected))


class ObservedPaper(PaperController):
    """Instrument entry to the real controller, preserving its computation and side effects."""

    def initialize(self, *args):
        self.initializations += 1
        return super().initialize(*args)

    def step(self, *args, **kwargs):
        self.calls[self.phase()] += 1
        return super().step(*args, **kwargs)


class NativeFixtureNode:
    """The only available state is own native navigation; 3D target reads are tripwires."""

    def __getattr__(self, name):
        if 'target' in name or name in ('tracking_state', 'target_state'):
            self.target_reads += 1
            raise AssertionError('P764_TARGET3D_READ:'+name)
        raise AttributeError(name)

    def _revoke_nominal_follow(self, reason):
        self.revocations += 1
        self.follow_receiver.active = self.follow_receiver.pending = None


def handover_replay(contract):
    """Real runtime dispatch with synthetic native inputs and an explicitly injected Y permit."""
    runtime = P76TrackerRuntime.__new__(P76TrackerRuntime)
    node = NativeFixtureNode()
    node.now, node.target_reads, node.revocations = 10., 0, 0
    node._ros_seconds = lambda: node.now
    node._timestamp_us = lambda: round(node.now*1e6)
    messages, heartbeats, phases = [], [], []
    node.offboard_pub = SimpleNamespace(publish=heartbeats.append)
    node.vehicle_armed, node.navigation_frame_fault = True, ''
    node.mission_state_name, node.mission_id, node.current_heading = 'FOLLOW', 1, 0.
    node.flight_guidance = SimpleNamespace(flight_altitude=-20.)
    node.follow_navigation_history = deque(maxlen=256)
    node._follow_context = lambda: SimpleNamespace(
        following=True, visible=True, navigation_fresh=True, target_fresh=True,
        prediction_fresh=True, observation_fresh=True, recovering=False)
    node.tracker = SimpleNamespace(active_trajectory=None)
    node.pending_trajectory = node.terminal_execution = None
    samples = Counter()
    real_curve = fixed_curve()

    class CurveProbe:
        def __getattr__(self, name):
            return getattr(real_curve, name)

        def sample(self, epoch):
            samples[runtime.mode.phase] += 1
            return real_curve.sample(epoch)
    old_curve = CurveProbe()
    node.follow_receiver = SimpleNamespace(active=old_curve, pending=old_curve, deadline=11.)
    runtime.node, runtime.contract, runtime.workspace = node, contract, str(ROOT)
    runtime.outer, runtime.mode = engineering_tracker(contract), ModeCoordinator()
    runtime.history, runtime.references = NativePoseHistory(), NativeReferenceHistory()
    runtime.recovery, runtime.recovery_phase = BodyrateRecovery(3., .24), None
    barrier = BodyrateSeaBarrier([[0., 0., 0.]], .02, .24, 3.)
    runtime.controller = ObservedPaper(contract, 2., 60., runtime.outer.map, barrier)
    controller = runtime.controller
    controller.calls, controller.initializations = Counter(), 0
    controller.phase = lambda: runtime.mode.phase
    png_initializations = []
    png_initialize = controller.png.initialize

    def initialize_png(*args):
        png_initializations.append(runtime.mode.phase)
        return png_initialize(*args)
    controller.png.initialize = initialize_png
    runtime.rates_pub = SimpleNamespace(publish=messages.append)
    runtime.mode_pub = SimpleNamespace(publish=phases.append)
    runtime.diagnostic_pub = SimpleNamespace(publish=lambda message: None)
    runtime.follow_revoked, runtime.initialized, runtime.fatal_dispatched = False, False, False
    runtime.last_tick, runtime.last_report = None, 1000.
    runtime.mode_wait_started, runtime.waypoint, runtime.waypoint_state = None, None, None
    runtime.image_ack, runtime.spatial_ack, runtime.image = None, None, None
    runtime.land_observation = {}
    # This fixture starts in established airborne control. Startup is independently tested.
    runtime.control_state, runtime.previous_armed = 'FLIGHT_ACTIVE', True
    runtime.armed_at, runtime.initialization_count = 9., 1
    runtime.initialization_reason, runtime.initialization_margin = '', None
    runtime.initialization_stamp = 9.
    trace = []

    def tick(epoch, image_ready=True):
        epoch = round(epoch, 9)
        node.now = epoch
        node.follow_navigation_history.append(dict(stamp=epoch, p=[4*(epoch-10), 0., -20.],
                                                   v=[4., 0., 0.]))
        runtime.history.add(epoch, [1., 0., 0., 0.], [0., 0., 0.], [1e-6]*3)
        runtime.references.add(epoch, 0.)
        runtime.px4_mode = epoch, dict(offboard=True, rates=True, position=False, velocity=False,
                                       attitude=False, altitude=False, climb_rate=False)
        runtime.executed_thrust = epoch, np.array([0., 0., -.5])
        stamp = Time(nanoseconds=round(epoch*1e9)).to_msg()
        runtime.image = TargetImageState(valid=True, mode_epoch=runtime.mode.epoch,
                                         model_sha256=contract['detector']['model_sha256'],
                                         state=[0., 0., 0., 0.])
        runtime.image.stamp = runtime.image.pose_stamp = runtime.image.source_stamp = stamp
        runtime.image_ack = dict(schema_version=2, epoch=runtime.mode.epoch, active=True,
                                 ready=image_ready, control_authorized=True, stamp=epoch)
        runtime.tick()
        trace.append(dict(epoch=epoch, phase=runtime.mode.phase,
                          output=dict(runtime.last_emitted),
                          guidance_owner=runtime.mode.guidance_owner))
    for index in range(5):
        tick(10.+index*.005)
    before_y = dict(runtime.last_emitted)
    with patch.dict(os.environ, {'UAV_USV_P761_STAGE': 'C'}), patch(
            'uav_control.control.p76_tracker_runtime.validate_follow_qualification',
            return_value={'synthetic_unit_authorization_only': True}):
        runtime.accept_y(10.025)
        runtime.accept_y(10.025)  # Repeated operator messages cannot withdraw authority again.
    tick(10.025, image_ready=False)
    tick(10.030, image_ready=False)
    tick(10.035)
    for index in range(4):
        tick(10.040+index*.005)
    # A late old proposal cannot gain authority, even if inserted into the receiver memory.
    node.follow_receiver.active = old_curve
    rejected = False
    try:
        epoch, p, v, pose, _ = runtime._snapshot(10.055)
        runtime._follow_output(10.055, .005, epoch, p, v, pose)
    except ValueError as error:
        rejected = 'AUTHORITY_REVOKED' in str(error)
    tick(10.058)
    runtime.mode.fail('P764_SYNTHETIC_IMAGE_LOSS')
    before_recovery = sum(controller.calls.values())+sum(samples.values())
    for index in range(4):
        tick(10.060+index*.005)
    after_recovery = sum(controller.calls.values())+sum(samples.values())
    # Check each adjacent actual emission, including the first handover and first paper output.
    continuity = True
    for left, right in zip([before_y]+[t['output'] for t in trace[5:]],
                           [t['output'] for t in trace[5:]]):
        elapsed = right['stamp']-left['stamp']
        continuity &= bool(np.linalg.norm(np.subtract(right['rates'], left['rates']))
                           <= 3.*elapsed+1e-9
                           and abs(right['thrust']-left['thrust']) <= 2.*elapsed+1e-9)
    body_only = bool(heartbeats and len(heartbeats) == len(messages)
                     and all(h.body_rate and not any((h.position, h.velocity, h.acceleration,
                                                     h.attitude)) for h in heartbeats))
    passed = bool(controller.calls['FOLLOW_ACTIVE'] == controller.calls['Y_HANDOVER'] == 0
                  and controller.calls['PAPER_INTERCEPT_ACTIVE'] >= 2
                  and controller.initializations == len(png_initializations) == 1
                  and node.revocations == 1 and rejected and after_recovery == before_recovery
                  and continuity and node.target_reads == 0 and body_only)
    return dict(schema_version=2, passed=passed, flight_qualified=False,
                scope=('Actual runtime/controller dispatch using synthetic native feedback, '
                       'memory publishers and injected positive Y authorization; no ROS executor'),
                paper_step_calls_by_phase={phase: controller.calls[phase] for phase in (
                    'FOLLOW_ACTIVE', 'Y_HANDOVER', 'PAPER_INTERCEPT_ACTIVE', 'SAFE_RECOVERY')},
                paper_initialize_calls=controller.initializations,
                png_initialize_calls=len(png_initializations), minco_revocations=node.revocations,
                minco_polynomial_samples_by_phase={phase: samples[phase] for phase in (
                    'FOLLOW_ACTIVE', 'Y_HANDOVER', 'PAPER_INTERCEPT_ACTIVE', 'SAFE_RECOVERY')},
                old_minco_explicit_reentry_rejected=rejected,
                recovery_target_guidance_calls=after_recovery-before_recovery,
                first_y_output_continuous=bool(continuity), target3d_reads=node.target_reads,
                bodyrate_only=body_only, single_memory_outlet_messages=len(messages), trace=trace)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--recorded-summary', type=Path, required=True)
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=True)
    contract_path = ROOT/'src/uav_usv_bringup/config/p76_contract.yaml'
    contract = load_contract(contract_path)
    inputs = {str(path.resolve()): sha256(path) for path in (
        Path(__file__), contract_path, args.recorded_summary,
        ROOT/'scripts/p763_control_replay.py',
        ROOT/'data/experiments/20261021_p761_01/frozen/models/p76_model_manifest.json',
        ROOT/'data/experiments/20261021_p761_01/session_A1_03/identity.json',
        ROOT/'data/experiments/20261021_p761_01/session_A1_03/calibration.jsonl')}
    for name in ('p76_tracker_runtime', 'p76_mode_coordinator', 'p76_minco_bodyrate_tracker',
                 'p76_recovery', 'p76_paper_controller', 'p76_paper_png', 'p76_paper_thrust'):
        path = ROOT/'src/uav_control/uav_control/control'/(name+'.py')
        inputs[str(path)] = sha256(path)
    control = control_checks(contract)
    recorded = json.loads(args.recorded_summary.read_text())
    control.update(recorded_counterfactual=recorded, source_inputs=inputs,
                   logic_passed=control['passed'] and recorded['logic_passed'])
    handover = dict(handover_replay(contract), source_inputs=inputs)
    reports = (('e2_control_replay.json', control), ('e2_handover_replay.json', handover))
    for name, result in reports:
        with (args.output/name).open('x') as stream:
            json.dump(result, stream, indent=2)
            stream.write('\n')
    print(json.dumps(dict(control=control['logic_passed'], handover=handover['passed'],
                          flight_qualified=False)))
    return 0 if control['logic_passed'] and handover['passed'] else 2


if __name__ == '__main__':
    raise SystemExit(main())
