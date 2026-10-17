#!/usr/bin/env python3
"""Prepare one P6 case; execute only a qualified explicit isolated 60s flight."""

import argparse
from contextlib import contextmanager
from dataclasses import asdict
import json
import math
import os
from pathlib import Path
import shutil

import yaml

import p5_minco_sitl as harness
import p53_minco_sitl as previous
from uav_control.usv_motion_profiles import MOTION_MODES, MotionParameters, normalize_motion_mode

WORKSPACE = Path(__file__).resolve().parents[1]
EXPERIMENT_ROOT = WORKSPACE / 'data/experiments'
R11 = EXPERIMENT_ROOT / '20261013_p53_closed_loop_11/f3_h16_01/config'
DEFAULT_FLIGHT = R11 / 'flight.yaml'
DEFAULT_RESEARCH = R11 / 'research.yaml'
DEFAULT_PILOT_FLIGHT = R11.parent.parent / 'f2_h16_01/config/flight.yaml'
KIND = 'P6_TESTED_PROTOCOL_CANDIDATE'
# Forward-only protocol: historical A/C cases remain frozen, not rerunnable.
GROUPS = {
    'B': ('imm', 'nominal', False),
    'D': ('imm', 'robust', True),
}


def time_contract_for_configs(planner, tracker, imm):
    """Require explicit agreement; never infer an experiment from a widened YAML number."""
    from uav_control.guidance.follow_time_contract import input_time_contract
    profile = planner.get('target_time_profile', 'baseline_125')
    contract = input_time_contract(profile)
    if (tracker.get('follow_target_time_profile', 'baseline_125') != profile
            or imm.get('target_time_profile', 'baseline_125') != profile
            or imm.get('input_timeout', .125) != contract.observation_age
            or tracker.get('follow_admission_target_observation_age', .125)
            != contract.observation_age
            or planner.get('maximum_input_age', .125) > contract.navigation_age):
        raise ValueError('P6_TARGET_TIME_CONTRACT_MISMATCH')
    return contract


def matrix_conditions():
    """A comparison plan only; no automatic launch/retry loop is provided."""
    return [dict(motion_mode=MOTION_MODES[index], mode_id=index, group=group,
                 prediction_model=GROUPS[group][0], visibility_mode=GROUPS[group][1],
                 adaptive_viewpoint=GROUPS[group][2], observation_seconds=60.)
            for index in (2, 1, 3, 4, 5) for group in GROUPS]


def _write_json(path, value):
    with path.open('x') as stream:
        json.dump(value, stream, indent=2, allow_nan=False)
        stream.write('\n')


def _manifest(directory, paths):
    with (directory / 'SHA256SUMS').open('x') as stream:
        for path in sorted(paths):
            stream.write(f'{harness.sha(path)}  {path.relative_to(directory)}\n')


def prepare_case(output, motion_mode, group, flight_config=None,
                 research_config=DEFAULT_RESEARCH, calibration_file=None,
                 imm_shadow=False, role='experiment', seed=0):
    """Save full explicit input configs into an exclusive case, without flight."""
    output = Path(output).resolve()
    if not output.is_relative_to(EXPERIMENT_ROOT.resolve()) or output == EXPERIMENT_ROOT.resolve():
        raise ValueError('NEW_P6_EXPERIMENT_DIRECTORY_REQUIRED')
    if output.exists():
        raise FileExistsError(str(output))
    mode = normalize_motion_mode(motion_mode)
    group = str(group).upper()
    if group not in GROUPS or role not in ('pilot', 'baseline', 'experiment'):
        raise ValueError('P6_ROLE_MISMATCH')
    if imm_shadow:
        raise ValueError('P6_IMM_SHADOW_REMOVED_USE_UNIQUE_PRIMARY')
    predictor, visibility, adaptive = GROUPS[group]
    if ((role == 'pilot' and (mode != 'linear' or group != 'B'))
            or (role == 'baseline' and (mode != 'figure_eight' or group != 'B'))):
        raise ValueError('P6_ROLE_MISMATCH')
    if type(seed) is not int or not 0 <= seed < 2 ** 32:
        raise ValueError('P6_SEED_REQUIRED')
    calibration = Path(calibration_file).resolve() if calibration_file else None
    if visibility == 'robust' and (calibration is None or not calibration.is_file()):
        raise ValueError('P6_CALIBRATION_REQUIRED')
    # Changing only trajectory_type on an F3 figure-eight config accidentally
    # made the pilot move at 4 m/s sideways during takeoff. A default pilot
    # inherits the complete original F2 flight snapshot (2 m/s along +X).
    flight_source = Path(flight_config or (
        DEFAULT_PILOT_FLIGHT if role == 'pilot' else DEFAULT_FLIGHT)).resolve()
    research_source = Path(research_config).resolve()
    flight = yaml.safe_load(flight_source.read_text())
    research = yaml.safe_load(research_source.read_text())
    tracker = flight['trajectory_tracker_node']['ros__parameters']
    planner = research['p4_follow_planner_node']['ros__parameters']
    motion = flight['moving_target']['ros__parameters']
    # Every motion/IMM declaration is explicit in the saved full flight file,
    # including inactive IMM defaults for comparison provenance. User values
    # survive; only the prediction topic follows the actual launch override.
    for name, value in asdict(MotionParameters()).items():
        if name == 'integration_step':
            name, value = 'update_rate_hz', 1. / value
        else:
            name = {'linear_vx': 'velocity_x', 'linear_vy': 'velocity_y'}.get(name, name)
        motion.setdefault(name, value)
    from uav_control.tracking.imm_target_predictor_node import node_parameter_defaults
    defaults = json.loads(json.dumps(node_parameter_defaults(), allow_nan=False))
    imm = flight.setdefault('imm_target_predictor_node', {}).setdefault('ros__parameters', {})
    for name, value in defaults.items():
        imm.setdefault(name, value)
    imm['prediction_topic'] = '/planning/target_prediction'
    time_contract = time_contract_for_configs(planner, tracker, imm)
    # Validate/copy only; calibrated evidence and tuning are supplied by the
    # independently qualified root task, never manufactured by this driver.
    from uav_control.guidance.p6_follow import P6Policy
    derived = {'calibration_id', 'calibration_sha256', 'calibration_family', 'version',
               'covariance_scale', 'covariance_floor_m2', 'empirical_scale', 'distribution'}
    risk = {}
    for name, default in asdict(P6Policy()).items():
        if name in derived:
            continue
        key = 'p6_' + name
        if key in planner and key in tracker and planner[key] != tracker[key]:
            raise ValueError('P6_PLANNER_RECEIVER_POLICY_MISMATCH: ' + key)
        risk[key] = planner.get(key, tracker.get(key, default))
    risk.update(p6_visibility_mode=visibility, p6_adaptive_viewpoint=adaptive,
                p6_calibration_file=str(output / 'config/calibration.json') if calibration else '')
    tracker.update(risk)
    planner.update(risk)
    reference_mode = planner.get('follow_reference_mode', 'target')
    if (reference_mode not in ('target', 'rear_point')
            or tracker.get('follow_reference_mode', 'target') != reference_mode):
        raise ValueError('P6_REFERENCE_MODE_MISMATCH')
    yaw = float(planner['maximum_yaw_rate'])
    if (not math.isfinite(yaw) or yaw <= 0
            or float(tracker.get('follow_research_yaw_rate', 1.)) != yaw):
        raise ValueError('P6_YAW_LIMIT_MISMATCH')
    tracker['follow_reference_mode'] = planner['follow_reference_mode'] = reference_mode
    tracker.update(follow_minco_enabled=False, follow_minco_shadow_check=False,
                   follow_research_horizontal_acceleration=3., follow_research_yaw_rate=yaw)
    planner['follow_execution_enabled'] = True
    motion['trajectory_type'] = mode
    mode_id = next(index for index, value in MOTION_MODES.items() if value == mode)
    scene = 'f3' if role == 'baseline' else 'f2'
    effective_shadow = False
    case_id = f'{role}_{scene}_m{mode_id}_{group}_s{seed}'
    environment = dict(UAV_USV_P6_PREDICTOR=predictor,
                       UAV_USV_P6_IMM_SHADOW='true' if imm_shadow else 'false',
                       UAV_USV_P6_RECORD='1')
    output.mkdir(parents=True, exist_ok=False)
    (output / '.gitignore').write_text('*\n')
    (output / 'COLCON_IGNORE').touch()
    (output / 'config').mkdir()
    (output / 'config/flight.yaml').write_text(yaml.safe_dump(flight, sort_keys=False))
    (output / 'config/research.yaml').write_text(yaml.safe_dump(research, sort_keys=False))
    artifacts = [output / 'config/flight.yaml', output / 'config/research.yaml']
    if calibration is not None:
        shutil.copyfile(calibration, output / 'config/calibration.json')
        artifacts.append(output / 'config/calibration.json')
    _write_json(output / 'case.json', dict(
        kind=KIND, case_id=case_id, role=role, scene=scene, motion_mode=mode, mode_id=mode_id,
        group=group, prediction_model=predictor, visibility_mode=visibility,
        adaptive_viewpoint=adaptive, imm_shadow_requested=bool(imm_shadow),
        imm_shadow_effective=effective_shadow, observation_seconds=60., seed=seed,
        target_time_profile=time_contract.profile, input_time_contract=asdict(time_contract),
        seed_scope='metadata only; deterministic profiles; no noise injection by driver',
        launch_environment=environment, output=str(output / 'flight'),
        config_sources={str(path): harness.sha(path) for path in (flight_source, research_source)},
        calibration_source=str(calibration) if calibration else None,
        calibration_source_sha256=harness.sha(calibration) if calibration else None,
        status='PREPARED_NOT_FLOWN', execution_authorized=False))
    artifacts.append(output / 'case.json')
    _manifest(output, artifacts)
    return output


def verify_case(directory):
    directory = Path(directory).resolve()
    try:
        harness.verify_manifest(directory)
    except (OSError, ValueError) as exc:
        raise ValueError('P6_CASE_ARTIFACT_CHANGED') from exc
    metadata = json.loads((directory / 'case.json').read_text())
    if (metadata.get('kind') != KIND or metadata.get('observation_seconds') != 60.
            or Path(metadata.get('output', '')).resolve() != directory / 'flight'):
        raise ValueError('P6_CASE_SCOPE_MISMATCH')
    flight = yaml.safe_load((directory/'config/flight.yaml').read_text())
    research = yaml.safe_load((directory/'config/research.yaml').read_text())
    contract = time_contract_for_configs(
        research['p4_follow_planner_node']['ros__parameters'],
        flight['trajectory_tracker_node']['ros__parameters'],
        flight.get('imm_target_predictor_node', {}).get('ros__parameters', {}))
    if (metadata.get('target_time_profile', 'baseline_125') != contract.profile
            or metadata.get('input_time_contract', asdict(contract)) != asdict(contract)):
        raise ValueError('P6_TARGET_TIME_CONTRACT_MISMATCH')
    metadata['target_time_profile'] = contract.profile
    metadata['case_manifest_sha256'] = harness.sha(directory / 'SHA256SUMS')
    return metadata


def critical_sources():
    """Hash the integrated runtime inventory, including both independent receivers."""
    paths = set((WORKSPACE / 'scripts').glob('*.py'))
    paths.update((WORKSPACE / 'src/uav_control/uav_control').rglob('*.py'))
    paths.update((WORKSPACE / 'src/uav_control/test').glob('test_p6*.py'))
    paths.update((WORKSPACE / 'src/uav_usv_bringup/launch').glob('*.py'))
    paths.update((WORKSPACE / 'src/uav_usv_interfaces/msg').glob('*.msg'))
    paths.update((WORKSPACE / 'src/uav_usv_bringup/config').glob('p6*.yaml'))
    paths.update(WORKSPACE / path for path in (
        'scripts/start_px4_ros2.sh', 'scripts/uav_lab.sh',
        'src/uav_control/package.xml', 'src/uav_control/setup.py'))
    return sorted(paths)


def validate_release(release):
    return bool(release.get('kind') == KIND and release.get('offline_gate_pass') is True
                and release.get('candidate') == harness.CANDIDATE)


def _qualification(qualification, metadata):
    release = json.loads(Path(qualification).read_text())
    if (not validate_release(release)
            or metadata['case_id'] not in release.get('qualified_cases', [])):
        raise ValueError('P6_CASE_NOT_QUALIFIED')
    if release.get('qualified_case_manifests', {}).get(metadata['case_id']) != (
            metadata['case_manifest_sha256']):
        raise ValueError('P6_CASE_CONFIG_NOT_QUALIFIED')
    if metadata['prediction_model'] == 'imm':
        from p61r_imm_qualification import validate_primary_evidence
        validate_primary_evidence(release, metadata)
    if metadata['visibility_mode'] == 'robust' and release.get('robust_gate_pass') is not True:
        raise ValueError('P6_ROBUST_QUALIFICATION_REQUIRED')
    approved = {(WORKSPACE / row['path']).resolve() for row in release.get('source_checks', [])}
    if not {path.resolve() for path in critical_sources()} <= approved:
        raise ValueError('P6_SOURCES_NOT_QUALIFIED')
    harness.validate_release_sources(release)
    return release


def write_readiness(output, case, verification, qualification):
    """Bind existing review/test evidence; never create passing qualification claims."""
    metadata = verify_case(case)
    verified = json.loads(Path(verification).read_text())
    if verified.get('passed') is not True or verified.get('independent_review_pass') is not True:
        raise ValueError('P6_VERIFICATION_AND_REVIEW_REQUIRED')
    _qualification(qualification, metadata)
    _write_json(Path(output), dict(
        kind=KIND, case_id=metadata['case_id'],
        case_sha256=harness.sha(Path(case) / 'case.json'),
        case_manifest_sha256=harness.sha(Path(case) / 'SHA256SUMS'),
        verification=str(Path(verification).resolve()),
        verification_sha256=harness.sha(verification),
        qualification_sha256=harness.sha(qualification),
        source_checks=[dict(path=str(path.relative_to(WORKSPACE)), sha256=harness.sha(path))
                       for path in critical_sources()],
        status='TESTED_PROTOCOL_CANDIDATE_NOT_FLIGHT_SUCCESS'))


def require_readiness(readiness, case, qualification):
    metadata = verify_case(case)
    record = json.loads(Path(readiness).read_text())
    if (record.get('kind') != KIND or record.get('case_id') != metadata['case_id']
            or record.get('case_sha256') != harness.sha(Path(case) / 'case.json')
            or record.get('case_manifest_sha256') != harness.sha(Path(case) / 'SHA256SUMS')
            or record.get('qualification_sha256') != harness.sha(qualification)):
        raise ValueError('P6_READINESS_SCOPE_MISMATCH')
    verification = Path(record['verification'])
    if harness.sha(verification) != record.get('verification_sha256'):
        raise ValueError('P6_VERIFICATION_AND_REVIEW_REQUIRED')
    verified = json.loads(verification.read_text())
    if verified.get('passed') is not True or verified.get('independent_review_pass') is not True:
        raise ValueError('P6_VERIFICATION_AND_REVIEW_REQUIRED')
    rows = record.get('source_checks', [])
    expected = {str(path.relative_to(WORKSPACE)) for path in critical_sources()}
    if {row['path'] for row in rows} != expected or any(
            harness.sha(WORKSPACE / row['path']) != row['sha256'] for row in rows):
        raise ValueError('P6_READINESS_SOURCE_CHANGED')
    _qualification(qualification, metadata)
    return record


def require_prior_run(case, prior_summary):
    """Actual healthy linear pilot precedes baseline; baseline precedes experiments."""
    metadata = verify_case(case)
    if metadata['role'] == 'pilot':
        return None
    expected_role = 'pilot' if metadata['role'] == 'baseline' else 'baseline'
    try:
        prior_path = Path(prior_summary)
        prior = json.loads(prior_path.read_text())
        prior_case = verify_case(prior_path.parent)
        actual_path = prior_path.parent / 'flight/p5_run_summary.json'
        actual = json.loads(actual_path.read_text())
        valid = (prior.get('role') == prior_case['role'] == expected_role
                 and prior.get('actual_summary_sha256') == harness.sha(actual_path)
                 and (prior_path.parent / 'flight/offline_evidence.jsonl').is_file()
                 and previous.allow_f3(actual) and actual.get('observation_seconds', 0.) >= 59.5
                 and prior.get('healthy') is True and prior.get('completed') is True)
    except (OSError, ValueError, TypeError, KeyError):
        valid = False
    if not valid:
        raise ValueError('P6_PRIOR_ACTUAL_RUN_REQUIRED')
    return actual


@contextmanager
def _launch_environment(values):
    previous_values = {name: os.environ.get(name) for name in values}
    os.environ.update(values)
    try:
        yield
    finally:
        for name, value in previous_values.items():
            if value is None:
                os.environ.pop(name, None)
            else:
                os.environ[name] = value


def _actual_execution_audit(output, rows, first_minco, scene):
    from p51_execution_audit import audit
    # The original audit's f3 branch is exactly the requested 60s window;
    # scene only selects that duration, never changes raw lease/input limits.
    return audit(output, rows, first_minco, 'f3')


def run_case(case, qualification, readiness, execute=False, prior_summary=None):
    if not execute:
        raise ValueError('P6_EXPLICIT_EXECUTION_REQUIRED')
    case = Path(case).resolve()
    metadata = verify_case(case)
    require_readiness(readiness, case, qualification)
    prior = require_prior_run(case, prior_summary)
    if metadata['scene'] == 'f3':
        _write_json(case / 'f2_summary.json', prior)
        _write_json(case / 'prior_pilot_reference.json', dict(
            path=str(Path(prior_summary).resolve()), sha256=harness.sha(prior_summary)))
    output = case / 'flight'
    with _launch_environment(metadata['launch_environment']):
        result = harness.run(
            output, Path(qualification).resolve(), metadata['scene'], task_root=case,
            authorization='P6 autonomous isolated SITL authorized; one explicit qualified case',
            health_check=previous.FollowHealthGuard(initial_acquisition=True),
            health_monitor_check=previous.GroundStartupHealth(output),
            health_start_marker='Starting UAV-USV experiment...',
            research_config=case / 'config/research.yaml',
            overlay_config=case / 'config/flight.yaml',
            observation_seconds=60., extra_audit=_actual_execution_audit,
            f3_gate=previous.allow_f3, release_gate=validate_release)
    summary_path = output / 'p5_run_summary.json'
    summary = json.loads(summary_path.read_text()) if summary_path.exists() else dict(
        completed=False, healthy=False, reason='P6_ACTUAL_SUMMARY_MISSING')
    window_complete = (summary.get('observation_seconds', 0.) >= 59.5
                       and summary.get('diagnostic_coverage', 0.) >= .95)
    _write_json(case / 'p6_run_summary.json', dict(
        **summary, role=metadata['role'], motion_mode=metadata['motion_mode'],
        group=metadata['group'], case_id=metadata['case_id'], harness_returncode=result,
        actual_summary_sha256=harness.sha(summary_path) if summary_path.exists() else None,
        qualification_sha256=harness.sha(qualification), readiness_sha256=harness.sha(readiness),
        window_evidence_complete=window_complete,
        note='Actual P5 owner/native/ACK audit; prepared trajectories do not prove execution'))
    return result if summary.get('completed') and summary.get('healthy') and window_complete else 1


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    commands = parser.add_subparsers(dest='command', required=True)
    commands.add_parser('matrix', help='print planned IMM-only B/D conditions; does not launch')
    prepare_parser = commands.add_parser('prepare', help='save an exclusive full-config case')
    prepare_parser.add_argument('output', type=Path)
    prepare_parser.add_argument('--motion', required=True)
    prepare_parser.add_argument('--group', choices=tuple(GROUPS), default='B')
    prepare_parser.add_argument('--role', choices=('pilot', 'baseline', 'experiment'),
                                default='experiment')
    prepare_parser.add_argument('--flight-config', type=Path)
    prepare_parser.add_argument('--research-config', type=Path, default=DEFAULT_RESEARCH)
    prepare_parser.add_argument('--calibration', type=Path)
    prepare_parser.add_argument('--imm-shadow', action='store_true')
    prepare_parser.add_argument('--seed', type=int, default=0)
    ready = commands.add_parser('readiness', help='bind existing passing review/test evidence')
    ready.add_argument('output', type=Path)
    ready.add_argument('--case', type=Path, required=True)
    ready.add_argument('--verification', type=Path, required=True)
    ready.add_argument('--qualification', type=Path, required=True)
    run = commands.add_parser('run', help='one explicit qualified actual flight, no auto retries')
    run.add_argument('case', type=Path)
    run.add_argument('--qualification', type=Path, required=True)
    run.add_argument('--readiness', type=Path, required=True)
    run.add_argument('--prior-summary', type=Path)
    run.add_argument('--execute', action='store_true')
    args = parser.parse_args(argv)
    if args.command == 'matrix':
        print(json.dumps(matrix_conditions(), indent=2))
    elif args.command == 'prepare':
        print(prepare_case(args.output, args.motion, args.group, args.flight_config,
                           args.research_config, args.calibration, args.imm_shadow,
                           args.role, args.seed))
    elif args.command == 'readiness':
        write_readiness(args.output, args.case, args.verification, args.qualification)
    else:
        return run_case(args.case, args.qualification, args.readiness,
                        args.execute, args.prior_summary)
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
