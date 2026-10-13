#!/usr/bin/env python3
"""Bounded P5.3 diagnostic rounds and conditional actual-F2-qualified F3."""
import argparse
from dataclasses import replace
import json
from pathlib import Path

import p5_minco_sitl as harness
import p52_minco_sitl as previous
from uav_control.guidance.follow_admission import AdmissionConfig

WORKSPACE = Path(__file__).resolve().parents[1]
ROOT = WORKSPACE/'data/experiments/20261013_p53_closed_loop_01'
KIND = 'P53_TESTED_PROTOCOL_CANDIDATE'
allow_f3 = previous.allow_f3


def round_root(number):
    """Every diagnostic round keeps its original exclusive attempt claims."""
    if type(number) is not int or not 1 <= number <= 9:
        raise ValueError('BOUNDED_P53_ROUND_REQUIRED')
    return ROOT.with_name(f'20261013_p53_closed_loop_{number:02d}')


def flight_admission():
    """Allow the legitimate 5m-offset turn velocity in the practical research round."""
    return replace(AdmissionConfig(), relative_enter=1.8, relative_exit=2.5)


def f3_retry_root(number, attempt=2):
    """One additional observer-diagnosis F3; retain the actual passed F2 gate."""
    base = round_root(number)
    if type(attempt) is not int or attempt not in (2, 3, 4):
        raise ValueError('BOUNDED_F3_ATTEMPT_REQUIRED')
    if not (base/'attempt_f3.json').exists():
        raise ValueError('PRIOR_F3_ATTEMPT_REQUIRED')
    summary = (base/'f2_summary.json').read_text()
    if not allow_f3(json.loads(summary)):
        raise ValueError('F2_SUSTAINED_HEALTHY_EXECUTION_REQUIRED')
    if attempt > 2:
        prior = base/f'f3_retry_{attempt-1:02d}/f3_h16_01'
        stop, ended = prior/'bounded_stop.json', prior/'watchdog_end.json'
        if (not (prior.parent/'attempt_f3.json').exists() or not stop.exists()
                or not ended.exists() or (prior/'X_command.txt').exists()
                or json.loads(stop.read_text()).get('reason') != 'HEALTH_MONITOR_STALLED'
                or json.loads(ended.read_text()).get('x_time', 'MISSING') is not None):
            raise ValueError('PRE_X_RECORDER_STALL_REQUIRED')
    retry = base/f'f3_retry_{attempt:02d}'
    if (retry/'attempt_f3.json').exists():
        raise ValueError('F3_RETRY_ALREADY_CONSUMED')
    retry.mkdir(exist_ok=True)
    (retry/'f2_summary.json').write_text(summary)
    return retry


class GroundStartupHealth:
    """Bound DDS recorder initialization before the session has ever issued X."""

    def __init__(self, output):
        self.output = output
        self.first_record_observed = None

    def __call__(self, returncode, health, now, started):
        reason = harness.health_monitor_reason(returncode, health.get('written'), now, started)
        written = health.get('written')
        if written is not None and self.first_record_observed is None:
            self.first_record_observed = now
        ground = (health.get('control', {}).get('status') in (None, 'INIT', 'GROUND_HOLD')
                  and health.get('vehicle', {}).get('armed', 1) == 1
                  and not (self.output/'X_command.txt').exists())
        if (reason == 'HEALTH_MONITOR_STALLED' and written is not None and 0 <= now-written
                and ground and 0 <= now-self.first_record_observed <= 5.):
            return None
        return reason


class FollowHealthGuard:
    """Observe brief original-FOLLOW reacquisition; never grant MINCO authority."""

    maximum_observer_input_age = .3

    def __init__(self, initial_acquisition=False):
        self.unlocked_since = None
        self.initial_acquisition = initial_acquisition
        self.acquisition_since = None
        self.follow_lock_seen = False

    def __call__(self, health, now):
        decision = previous.previous.health_decision(
            health, now, maximum_observer_input_age=self.maximum_observer_input_age)
        control = health.get('control', {})
        if (self.initial_acquisition and not self.follow_lock_seen
                and control.get('status') == 'FOLLOW'):
            if self.acquisition_since is None:
                self.acquisition_since = now
            elapsed = now-self.acquisition_since
            observed = dict(health, control=dict(control, locked=True))
            decision = previous.previous.health_decision(
                observed, now, maximum_observer_input_age=self.maximum_observer_input_age)
            if decision['fatal'] is None:
                if elapsed < 0:
                    decision['fatal'] = 'INITIAL_FOLLOW_ACQUISITION_CLOCK_REVERSED'
                elif elapsed > 60.:
                    decision['fatal'] = 'INITIAL_FOLLOW_ACQUISITION_TIMEOUT'
                elif control.get('locked'):
                    self.follow_lock_seen = True
                else:
                    decision['inhibit'] = 'INITIAL_FOLLOW_ACQUISITION'
                    health['follow_settling'] = dict(
                        health.get('follow_settling', {}), settled=False,
                        reason='INITIAL_FOLLOW_NOT_LOCKED')
            decision.update(initial_acquisition_seconds=elapsed, initial_acquisition_limit=60.)
            health['p53_decision'] = decision
            return decision['fatal']
        if control.get('status') == 'MINCO_FOLLOW':
            self.follow_lock_seen = True
        recovering = control.get('status') == 'FOLLOW' and not control.get('locked')
        if recovering:
            # Check every other original physical/input fault independently of
            # the lock bit. The sole Tracker still brakes/reacquires and its
            # strict admission cannot accept a trajectory while unlocked.
            observed = dict(health, control=dict(control, locked=True))
            decision = previous.previous.health_decision(
                observed, now, maximum_observer_input_age=self.maximum_observer_input_age)
            if self.unlocked_since is None:
                self.unlocked_since = now
            elapsed = now-self.unlocked_since
            if decision['fatal'] is None:
                if elapsed < 0:
                    decision['fatal'] = 'FOLLOW_LOCK_RECOVERY_CLOCK_REVERSED'
                elif elapsed > .2:
                    decision['fatal'] = 'FOLLOW_LOCK_RECOVERY_TIMEOUT'
                else:
                    decision['inhibit'] = 'FOLLOW_LOCK_RECOVERY'
            decision.update(lock_recovery_seconds=elapsed, lock_recovery_limit=.2)
        else:
            if self.unlocked_since is not None and decision['fatal'] is None:
                elapsed = now-self.unlocked_since
                if elapsed < 0:
                    decision['fatal'] = 'FOLLOW_LOCK_RECOVERY_CLOCK_REVERSED'
                elif elapsed > .2:
                    decision['fatal'] = 'FOLLOW_LOCK_RECOVERY_TIMEOUT'
            self.unlocked_since = None
        health['p53_decision'] = decision
        return decision['fatal']


def validate_release(release):
    if release.get('kind') != KIND:
        return False
    # Same named historical H1.6 evidence; changed protocol/scheduling uses fresh P5.3 tests.
    return previous.validate_release(dict(release, kind='P52_TESTED_PROTOCOL_CANDIDATE'))


def require_readiness(path, authorized, qualification=None):
    """No attempt is consumed until permission, independent review and hashes all pass."""
    if not authorized:
        raise ValueError('EXPLICIT_P53_SITL_AUTHORIZATION_REQUIRED')
    readiness = json.loads(path.read_text())
    verification = path.parent/'verification.json'
    qualification = qualification or path.parent/'qualification.json'
    if (readiness.get('kind') != KIND or readiness.get('a_e_pass') is not True
            or not readiness.get('source_checks')
            or harness.sha(verification) != readiness.get('verification_sha256')
            or harness.sha(qualification) != readiness.get('qualification_sha256')):
        raise ValueError('P53_A_E_NOT_PASSED')
    verified = json.loads(verification.read_text())
    if verified.get('passed') is not True or verified.get('independent_review_pass') is not True:
        raise ValueError('P53_A_E_NOT_PASSED')
    for row in readiness['source_checks']:
        source = Path(row['path'])
        source = source if source.is_absolute() else WORKSPACE/source
        if harness.sha(source) != row['sha256']:
            raise ValueError('READINESS_SOURCE_CHANGED')
    return readiness


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('output', type=Path)
    parser.add_argument('--scene', choices=('f2', 'f3'), default='f2')
    parser.add_argument('--round', type=int, choices=(1, 2, 3, 4, 5, 6, 7, 8, 9), default=1)
    parser.add_argument('--f3-attempt', type=int, choices=(1, 2, 3, 4), default=1)
    parser.add_argument('--qualification', type=Path, required=True)
    parser.add_argument('--readiness', type=Path, required=True)
    parser.add_argument('--authorize-f2', action='store_true')
    parser.add_argument('--authorize-f3', action='store_true')
    args = parser.parse_args()
    authorized = args.authorize_f2 if args.scene == 'f2' else args.authorize_f3
    require_readiness(args.readiness.resolve(), authorized, args.qualification.resolve())
    if args.scene != 'f3' and args.f3_attempt != 1:
        parser.error('--f3-attempt is only valid for scene f3')
    task_root = (f3_retry_root(args.round, args.f3_attempt) if args.f3_attempt > 1
                 else round_root(args.round))
    admission = flight_admission() if args.round >= 4 else AdmissionConfig()
    from p51_execution_audit import audit
    raise SystemExit(harness.run(
        args.output.resolve(), args.qualification.resolve(), args.scene,
        task_root=task_root,
        authorization='Human authorized autonomous P5.3 diagnosis; one attempt per scene/round',
        health_check=FollowHealthGuard(initial_acquisition=True),
        health_monitor_check=GroundStartupHealth(args.output.resolve()),
        health_start_marker='Starting UAV-USV experiment...',
        research_config=WORKSPACE/'src/uav_usv_bringup/config/p51_follow_research.yaml',
        extra_audit=audit, f3_gate=allow_f3, release_gate=validate_release,
        flight_parameters=dict(
            follow_admission_enabled=True, follow_minco_acceleration_feedforward=args.round >= 5,
            **{'follow_admission_'+key: value for key, value in vars(admission).items()})))
