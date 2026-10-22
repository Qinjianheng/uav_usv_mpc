#!/usr/bin/env python3
"""Offline admission checks on raw evidence; never called by a control callback."""
import gzip
import json
import math
from pathlib import Path
import tempfile

import numpy as np

from p763_control_replay import gain_analysis, recorded_replay
from p764_diagnostic_collect import COMMAND_TOPICS, analyze_coverage, plain
from p764_handover_replay import control_checks, handover_replay
from p764_real_replay import replay
from uav_control.common.p76_contract import load_contract
from uav_control.common.p76_readiness import sha256, source_fingerprint
from uav_control.tracking.image_dkf_node import ImageDkfNode
from uav_control.tracking.spatial_dkf_node import SpatialDkfNode

ROOT = Path(__file__).resolve().parents[1]
OLD_TASK = ROOT/'data/experiments/20261021_p761_01'


def verify_hashes(logs, reason):
    """Reject missing or changed evidence before parsing any saved claim."""
    if not logs or any(not Path(p).is_file() or sha256(p) != h for p, h in logs.items()):
        raise ValueError(reason)


def raw_path(logs, name):
    paths = [Path(p) for p in logs if Path(p).name == name]
    if len(paths) != 1:
        raise ValueError('P765_E1_RAW_INPUT_REQUIRED:'+name)
    return paths[0]


def json_lines(path):
    opener = gzip.open if str(path).endswith('.gz') else open
    with opener(path, 'rt') as stream:
        return [json.loads(line) for line in stream]


def validate_experiment_budget(path, authorized_id, output, stage):
    """A separately supplied new approval cannot consume the historical two free slots."""
    if path is None or not authorized_id:
        raise ValueError('P765_NEW_APPROVED_EXPERIMENT_BUDGET_REQUIRED')
    data = json.loads(Path(path).read_text())
    task = Path(data.get('task_root', '')).resolve()
    if (data.get('schema_version') != 1
            or data.get('kind') != 'P765_SEPARATELY_APPROVED_EXPERIMENT_BUDGET'
            or data.get('budget_id') != authorized_id
            or data.get('approval_source') != 'EXPLICIT_USER_APPROVAL'
            or not str(data.get('approval_reference', '')).strip()
            or type(data.get('session_limit')) is not int or data['session_limit'] <= 0
            or not isinstance(data.get('stages'), list) or stage not in data['stages']
            or not task.is_relative_to(ROOT/'data/experiments')
            or task == (ROOT/'data/experiments') or task == OLD_TASK
            or task.is_relative_to(OLD_TASK) or OLD_TASK.is_relative_to(task)
            or not output.resolve().is_relative_to(task)
            or output.resolve().parent != task
            or not output.name.startswith('session_')):
        raise ValueError('P765_NEW_BUDGET_SCOPE_OR_APPROVAL_INVALID')
    used = len(list(task.glob('session_*/environment.json')))
    if used >= data['session_limit']:
        raise ValueError('P765_NEW_EXPERIMENT_BUDGET_EXHAUSTED')
    return dict(manifest=str(Path(path).resolve()), sha256=sha256(path),
                budget_id=authorized_id, task_root=str(task), session_limit=data['session_limit'],
                sessions_used=used, independently_approved=True)


def e1_callback_recomputation(events):
    """Run real callbacks and compare full posterior state at each recorded phase transition."""
    transitions = []

    def snapshot(node):
        core = node.filter.filter if hasattr(node.filter, 'filter') else node.filter
        return json.dumps(plain(dict(anchor=core.anchor, checkpoints=core.checkpoints,
                                     source_stamp=core.source_stamp,
                                     camera_anchor=getattr(node.filter, 'anchor_rotation', None))),
                          sort_keys=True)

    def observed(cls):
        original = cls.mode_changed

        def changed(node, *args):
            before = snapshot(node)
            original(node, *args)
            transitions.append(dict(node=cls.__name__, phase=node.p76_phase,
                                    preserved=before == snapshot(node)))
        return type(cls.__name__, (cls,), {'mode_changed': changed})
    if not events or any(not math.isfinite(float(e['receipt'])) for e in events):
        raise ValueError('P765_E1_RAW_CALLBACKS_REQUIRED')
    if any(right['receipt'] < left['receipt'] for left, right in zip(events, events[1:])):
        raise ValueError('P765_E1_RAW_RECEIPT_ORDER_INVALID')
    result = replay(events, tuple(observed(c) for c in (SpatialDkfNode, ImageDkfNode)))
    result['posterior_and_covariance_preserved'] = bool(transitions) and all(
        row['preserved'] for row in transitions)
    result['recomputed_transitions'] = transitions
    return result


def validate_e1_streams(paths, raw):
    """Tie all E1 streams to one capture run and its original acquisition file hashes."""
    names = ('synchronized_packets.jsonl.gz', 'actual_callback_inputs.jsonl.gz',
             'phase_labels.json')
    run_id, pins = raw.get('run_id'), raw.get('raw_inputs', {})
    if (len({p.resolve().parent for p in paths.values()}) != 1
            or not isinstance(run_id, str) or not run_id.strip()
            or not isinstance(pins, dict) or set(pins) != set(names)
            or any(pins[name] != sha256(paths[name]) for name in names)):
        raise ValueError('P765_E1_STREAM_BINDING_IDENTITY_OR_HASH_MISMATCH')
    records = json_lines(paths[names[0]])
    events = json_lines(paths[names[1]])
    phases = json.loads(paths[names[2]].read_text())
    if any(not isinstance(rows, list) or not rows
           or any(not isinstance(row, dict) or row.get('run_id') != run_id for row in rows)
           for rows in (records, events, phases)):
        raise ValueError('P765_E1_STREAM_BINDING_RUN_ID_MISMATCH')
    return records, events, phases


def validate_e1_raw(e1):
    """Recompute real-input coverage, callback continuity and actual executor sample statistics."""
    logs = e1.get('raw_evidence', {})
    verify_hashes(logs, 'P765_E1_RAW_EVIDENCE_INVALID')
    paths = {name: raw_path(logs, name) for name in (
        'synchronized_packets.jsonl.gz', 'actual_callback_inputs.jsonl.gz',
        'phase_labels.json', 'e1_executor_raw.json')}
    raw = json.loads(paths['e1_executor_raw.json'].read_text())
    if (raw.get('schema_version') != 1
            or raw.get('evidence_class') != 'ACTUAL_ROS_EXECUTOR_DISCONNECTED_SHADOW'
            or raw.get('sources') != source_fingerprint(ROOT)
            or raw.get('contract_sha256') != sha256(
                ROOT/'src/uav_usv_bringup/config/p76_contract.yaml')):
        raise ValueError('P765_E1_RAW_CURRENT_IDENTITY_REQUIRED')
    if raw.get('recorder_overflows') != 0:
        raise ValueError('P765_E1_RECORDER_INCOMPLETE')
    graphs = raw.get('graphs', [])
    if not graphs or any(not set(COMMAND_TOPICS+('/perception/target_detection',)).issubset(g)
                         for g in graphs):
        raise ValueError('P765_E1_RAW_GRAPH_REQUIRED')
    records, events, phases = validate_e1_streams(paths, raw)
    coverage = analyze_coverage(records, phases, graphs)
    timing = recompute_executor_timing(raw)
    if not coverage['qualified'] or raw.get('failure') or not timing['qualified']:
        raise ValueError('P765_E1_RECOMPUTED_NOT_QUALIFIED')
    replayed = e1_callback_recomputation(events)
    if (not replayed['posterior_and_covariance_preserved']
            or replayed['spatial_distinct_updates_after_y'] <= 0
            or replayed['illegal_authorizations'] != 0
            or not replayed['transitions']
            or not all(t['valid_before'] for t in replayed['transitions'])):
        raise ValueError('P765_E1_RECOMPUTED_NOT_QUALIFIED')
    return dict(coverage=coverage, timing=timing, replay=replayed, flight_qualified=False)


def recompute_executor_timing(raw):
    """Derive original 200 Hz deadlines from measured times, ignoring saved pass assertions."""
    times, costs, epochs = (np.asarray(raw.get(k, []), dtype=float) for k in (
        'timer_times', 'shadow_costs', 'shadow_epochs'))
    if (any(x.ndim != 1 or not np.isfinite(x).all() for x in (times, costs, epochs))
            or len(times) != len(costs) or len(times) < 2
            or np.any(np.diff(times) <= 0) or np.any(costs < 0) or np.any(epochs <= 0)
            or len(epochs) > len(times)):
        raise ValueError('P765_E1_RAW_TIMING_INVALID')
    missed = sum(max(0, int(round(x/.005))-1) for x in np.diff(times))
    return dict(missed_cycles=missed, compute_deadline_misses=int(sum(costs > .005)),
                successful_native_computations=len(epochs),
                qualified=bool(len(epochs) >= 200 and not missed and max(costs) <= .005))


def validate_e2_evidence(e2):
    """Rerun the unchanged fixed stress/dispatch and historical input replay before admission."""
    inputs = e2.get('source_inputs', {})
    verify_hashes(inputs, 'P765_E2_RAW_OR_SOURCE_EVIDENCE_INVALID')
    required = [ROOT/'scripts'/n for n in (
        'p764_handover_replay.py', 'p763_control_replay.py')]
    required += [ROOT/'src/uav_usv_bringup/config/p76_contract.yaml']
    required += [ROOT/'src/uav_control/uav_control/control'/(n+'.py') for n in (
        'p76_tracker_runtime', 'p76_mode_coordinator', 'p76_minco_bodyrate_tracker',
        'p76_recovery', 'p76_paper_controller', 'p76_paper_png', 'p76_paper_thrust')]
    if any(str(p) not in inputs for p in required):
        raise ValueError('P765_E2_CURRENT_SOURCE_IDENTITY_REQUIRED')
    contract = load_contract(ROOT/'src/uav_usv_bringup/config/p76_contract.yaml')
    fixed = control_checks(contract)
    if not fixed['passed']:
        raise ValueError('P765_E2_FIXED_STRESS_NOT_QUALIFIED')
    handover = handover_replay(contract)
    recorded = e2.get('recorded_counterfactual', {}).get('recorded_replay', {})
    path = Path(recorded.get('source', ''))
    if not path.is_file() or sha256(path) != recorded.get('source_sha256'):
        raise ValueError('P765_E2_RECORDED_RAW_INPUT_REQUIRED')
    with tempfile.TemporaryDirectory(prefix='p765_e2_admission_') as directory:
        recomputed = recorded_replay(path, contract, Path(directory))
    if not (handover['passed'] and gain_analysis()['passed'] and recomputed['passed']
            and recomputed['output_bounds_and_continuity']):
        raise ValueError('P765_E2_RECOMPUTED_NOT_QUALIFIED')
    return dict(fixed=fixed, handover=handover, recorded=recomputed, flight_qualified=False)


def capture_identity(contract, model_root, workspace=ROOT):
    """Resolve the current measured A/model/PX4/DDS identity before selecting a previous C."""
    from uav_control.common.p76_readiness import follow_physical_identity
    return dict(sources=source_fingerprint(workspace), contract_sha256=sha256(contract['_path']),
                physical_identity=follow_physical_identity(contract, workspace, model_root))


def native_capture(path, summary):
    """Re-evaluate chronological native positions and body clearance at the original 0.25 m."""
    from uav_control.evaluation.gazebo_capture_geometry import _clock_width
    from uav_control.evaluation.intercept_evaluator import InterceptEvaluatorCore, KinematicState
    if (summary.get('capture_geometry_source') != 'gazebo_native'
            or summary.get('evaluation_capture_radius') != .25
            or summary.get('body_contact_geometry') != 'SDF_MODEL_ORIGIN_FULL_ATTITUDE'
            or summary.get('outcome') != 'SUCCESS'
            or summary.get('gazebo_pause_succeeded') is not True):
        raise ValueError('P765_NATIVE_CAPTURE_EVALUATOR_CONTRACT_REQUIRED')
    samples = [r for r in json_lines(path) if r.get('kind') == 'EVALUATED']
    if not samples:
        raise ValueError('P765_NATIVE_CAPTURE_RAW_SAMPLES_REQUIRED')
    evaluator = InterceptEvaluatorCore(
        capture_radius=.25, sea_surface_z=0., enable_sea_contact_failure=True)
    evaluator.begin(1, samples[0]['stamp'])
    last, result = None, None
    for row in samples:
        positions = np.asarray([row['uav_position'], row['target_position']], dtype=float)
        values = [row[k] for k in (
            'stamp', 'native_stamp', 'body_lowest_z', 'maximum_bracket', 'decision_stamp')]
        widths = [_clock_width(row[k], row['native_stamp'], row['stamp']) for k in (
            'uav_clock_bracket', 'target_clock_bracket')]
        if (positions.shape != (2, 3) or not np.isfinite(positions).all()
                or not np.isfinite(values).all() or row['native_stamp'] <= 0
                or last is not None and (row['stamp'] <= last['stamp']
                                         or row['native_stamp'] <= last['native_stamp'])
                or not 0 <= row['decision_stamp']-row['stamp'] <= .125
                or any(width is None for width in widths)
                or not math.isclose(row['maximum_bracket'], max(widths), abs_tol=1e-9)
                or not math.isclose(float(np.linalg.norm(positions[1]-positions[0])),
                                    row['distance'], abs_tol=1e-9)):
            raise ValueError('P765_NATIVE_CAPTURE_INVALID_NATIVE_GEOMETRY')
        result = evaluator.update(row['stamp'], KinematicState(tuple(positions[0]), (0.,)*3),
                                  KinematicState(tuple(positions[1]), (0.,)*3),
                                  capture_segment=False, sea_segment=False,
                                  body_lowest_z=row['body_lowest_z'])
        last = row
        if result is not None:
            break
    if result is None or not result.success or result.reason != 'CAPTURE_RADIUS_REACHED':
        raise ValueError('P765_NATIVE_CAPTURE_NOT_REPRODUCED')
    return dict(qualified=True, radius=.25, source=str(path), source_sha256=sha256(path),
                terminal_native_stamp=last['native_stamp'], terminal_stamp=last['stamp'],
                minimum_distance=result.minimum_distance,
                no_sea_contact_before_capture=True, geometry='gazebo_native',
                scope='Native evaluation replay; online guidance authority is a separate gate')


def capture_evidence(session, current):
    """Bind successful C evaluator/observer bytes to the current complete physical identity."""
    session = Path(session).resolve()
    run = json.loads((session/'run_result.json').read_text())
    saved = json.loads((session/'validation_identity.json').read_text())
    if saved != current:
        raise ValueError('P765_C_CURRENT_IDENTITY_MISMATCH')
    if (run.get('stage') != 'C' or run.get('success') is not True
            or run.get('reason') != 'CAPTURE_RADIUS_REACHED' or run.get('y_sent') is None):
        raise ValueError('P765_CURRENT_SUCCESSFUL_C_REQUIRED')
    summaries = list(session.glob('*_summary.json'))
    if len(summaries) != 1:
        raise ValueError('P765_NATIVE_CAPTURE_EVALUATOR_LOG_REQUIRED')
    summary_path = summaries[0]
    raw_path = summary_path.with_name(summary_path.name.replace('_summary.json', '_capture.jsonl'))
    if not raw_path.is_file():
        raise ValueError('P765_NATIVE_CAPTURE_RAW_LOG_REQUIRED')
    measured = native_capture(raw_path, json.loads(summary_path.read_text()))
    observer = session/'validation.jsonl'
    terminals = [r['message'] for r in json_lines(observer) if r.get('kind') == 'result']
    if (len(terminals) != 1 or terminals[0].get('success') is not True
            or terminals[0].get('capture_radius') != .25
            or terminals[0].get('reason') != 'CAPTURE_RADIUS_REACHED'):
        raise ValueError('P765_NATIVE_CAPTURE_OBSERVER_RESULT_MISMATCH')
    logs = {str(p): sha256(p) for p in (
        session/'run_result.json', session/'validation_identity.json', observer,
        summary_path, raw_path)}
    return dict(schema_version=1, kind='P765_CURRENT_C_NATIVE_CAPTURE_BINDING',
                identity=current, raw_evidence=logs, native_capture=measured)


def validate_previous_c(session, current):
    """D requires the same independently replayed successful C, not a saved success flag."""
    if session is None:
        raise ValueError('P765_PREVIOUS_C_EVIDENCE_REQUIRED_FOR_D')
    session = Path(session).resolve()
    path = session/'capture_qualification.json'
    if not path.is_file():
        raise ValueError('P765_PREVIOUS_C_BINDING_REQUIRED_FOR_D')
    data = json.loads(path.read_text())
    if data.get('identity') != current:
        raise ValueError('P765_C_CURRENT_IDENTITY_MISMATCH')
    verify_hashes(data.get('raw_evidence', {}), 'P765_C_RAW_EVIDENCE_CHANGED_OR_MISSING')
    recomputed = capture_evidence(session, current)
    if data != recomputed:
        raise ValueError('P765_C_NATIVE_BINDING_RECOMPUTATION_MISMATCH')
    return dict(recomputed, binding_file=str(path), binding_sha256=sha256(path))
