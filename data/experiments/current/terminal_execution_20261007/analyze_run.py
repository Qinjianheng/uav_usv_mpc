"""Recompute command, capture, sensing and freeze audits without editing raw data."""

import argparse
import csv
import gzip
import json
import math
from pathlib import Path


def stamp(value):
    return value['sec'] + value['nanosec'] * 1e-9


def analyze(run, output):
    output.mkdir(parents=True, exist_ok=True)
    probe = run / 'probe.jsonl'
    stream = probe.open() if probe.exists() else gzip.open(run / 'probe.jsonl.gz', 'rt')
    with stream:
        records = [json.loads(line) for line in stream]
    result_record = next(row for row in records if row['kind'] == 'result')
    result = result_record['data']
    end = stamp(result['stamp'])
    start = end - result['elapsed_time']
    trace = []
    for row in records:
        if row['kind'] != 'controller':
            continue
        d = row['data']
        epoch = stamp(d['stamp'])
        if not start <= epoch <= end or d['status'].startswith('PLAN_'):
            continue
        v = d['command_velocity']
        trace.append(dict(epoch=epoch, elapsed=epoch - start, status=d['status'],
                          plan_id=d['plan_id'], speed_xy=math.hypot(v['x'], v['y']),
                          vz=v['z'], locked=d['target_locked'],
                          remaining=d['remaining_time'], kf_age=d['kf_state_age'],
                          safety=d['safety_state']))
    with (output / 'command_trace.csv').open('w') as stream:
        writer = csv.DictWriter(stream, fieldnames=list(trace[0]))
        writer.writeheader()
        writer.writerows(trace)
    first = next(i for i, row in enumerate(trace) if row['status'] == 'TRACKING')
    finish = next((i for i in range(first + 1, len(trace))
                   if trace[i]['status'] not in ('TRACKING', 'TERMINAL_COMMITTED')),
                  len(trace))
    approach = trace[first:finish]
    pairs = list(zip(approach, approach[1:]))
    drops = [{'elapsed':b['elapsed'], 'before':a['speed_xy'], 'after':b['speed_xy']}
             for a, b in pairs if b['speed_xy'] < a['speed_xy'] - 1e-5]
    entry = trace[first - 1] if first else None
    report = dict(result=result,
                  first_minco=dict(start=approach[0]['elapsed'],
                                   finish=approach[-1]['elapsed'],
                                   previous_status=entry['status'] if entry else None,
                                   previous_speed=entry['speed_xy'] if entry else None,
                                   entry_speed=approach[0]['speed_xy'],
                                   last_speed=approach[-1]['speed_xy'],
                                   decreases=drops,
                                   recovery_before_result=finish < len(trace)),
                  committed_samples_before_result=sum(
                      row['status'] == 'TERMINAL_COMMITTED' for row in trace))
    samples = []
    for filename in (run / 'raw').glob('*.csv'):
        if filename.name.endswith('_vision.csv'):
            continue
        for row in csv.DictReader(filename.open()):
            if (row['truth_available'] == 'True' and row['uav_available'] == 'True'
                    and row['intercept_elapsed_time']):
                elapsed = float(row['intercept_elapsed_time'])
                if 0. <= elapsed <= result['elapsed_time']:
                    samples.append(row)
    if samples:
        closest = min(samples, key=lambda row: float(row['distance']))
        last_second = [row for row in samples
                       if float(row['intercept_elapsed_time']) >= result['elapsed_time'] - 1.]
        speeds = [math.hypot(float(row['uav_vx']), float(row['uav_vy']))
                  for row in last_second]
        if speeds:
            report['last_second_physical_speed'] = dict(
                count=len(speeds), minimum=min(speeds), maximum=max(speeds))
        report['closest_sample'] = {key: closest[key] for key in (
            'intercept_elapsed_time', 'distance', 'horizontal_distance', 'vertical_error',
            'phase', 'controller_status', 'sea_safety_state')}
    terminal_errors = []
    for filename in (run / 'raw').glob('*_vision.csv'):
        for row in csv.DictReader(filename.open()):
            if (float(row['processed_stamp']) <= end and row['truth_available'] == 'True'
                    and row['rejection_reason'] == ''
                    and row['approach_phase'] == 'TERMINAL_APPROACH'):
                terminal_errors.append(float(row['position_3d_error']))
    if terminal_errors:
        values = sorted(terminal_errors)
        report['terminal_vision'] = dict(count=len(values),
            rmse=math.sqrt(sum(x*x for x in values) / len(values)),
            p95=values[min(math.ceil(.95 * len(values)) - 1, len(values) - 1)])
    after = [row for row in records if row['receipt_monotonic']
             >= result_record['receipt_monotonic'] + 1.]
    worlds = [row['data'] for row in after if row['kind'] == 'world_stats']
    truths = [row['data'] for row in after if row['kind'] == 'truth_evaluation_only']
    report['freeze'] = dict(applicable=result['success'],
                            pause_seen=any(row['paused'] for row in worlds),
                            world_samples=len(worlds), truth_samples=len(truths))
    if worlds:
        report['freeze']['world_clock_span'] = (max(row['sim_time'] for row in worlds)
                                                 - min(row['sim_time'] for row in worlds))
    if truths:
        report['freeze']['truth_position_span'] = max(
            max(row['position'][axis] for row in truths)
            - min(row['position'][axis] for row in truths) for axis in ('x', 'y', 'z'))
        report['freeze']['maximum_target_speed'] = max(math.sqrt(sum(
            row['velocity'][axis] ** 2 for axis in ('x', 'y', 'z'))) for row in truths)
    native = []
    for filename in sorted(run.glob('world_state_*.txt')):
        from google.protobuf.text_format import Parse
        from gz.msgs10.serialized_map_pb2 import SerializedStepMap
        message = Parse(filename.read_text(), SerializedStepMap())
        state = {'paused': message.stats.paused,
                 'sim_time': message.stats.sim_time.sec + message.stats.sim_time.nsec * 1e-9}
        for entity in message.state.entities.values():
            components = {value.type: value.component for value in entity.components.values()}
            name = components.get(17448053894352336366)
            if name in (b'x500_mono_cam_0', b'usv_target'):
                state[name.decode()] = [float(x) for x in
                                        components[10918813941671183356].decode().split()]
        native.append(state)
    report['freeze']['native_world_states'] = native
    report['freeze']['native_poses_identical'] = (len(native) >= 2 and
                                                all(x == native[0] for x in native[1:]))
    (output / 'audit.json').write_text(json.dumps(report, indent=2) + '\n')
    print(json.dumps(report, indent=2))


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('run', type=Path)
    parser.add_argument('output', type=Path)
    args = parser.parse_args()
    analyze(args.run, args.output)
