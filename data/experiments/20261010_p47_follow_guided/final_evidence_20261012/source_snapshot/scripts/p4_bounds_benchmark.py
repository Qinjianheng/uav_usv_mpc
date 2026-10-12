#!/usr/bin/env python3
"""Matched input benchmark of conservative precheck, without profiler or flight authority."""
import argparse
from dataclasses import replace
import json
from pathlib import Path
import time

import numpy as np

import follow_minco_experiment as base
from p4_follow_replay import load, quantiles
from uav_control.controllers.progress_follow_solver import ProgressFollowSolver


def run(path, output):
    """Compare every coefficient and admission on the exact same frozen 1.2s requests."""
    rows, cases = [], load([path])
    with (output/'bounds_comparison.jsonl').open('x') as stream:
        for _, line, fingerprint, req, model in cases:
            pair = []
            for enabled in (False, True):
                solver = ProgressFollowSolver(model, rolling=False,
                                              wall_clock=lambda: req.now_stamp)
                solver.fast = replace(solver.fast, bernstein_precheck=enabled)
                started = time.perf_counter()
                result = solver.solve(req)
                elapsed = time.perf_counter()-started
                pair.append(result)
                row = dict(line=line, fingerprint=fingerprint, bound_enabled=enabled,
                           seconds=elapsed, valid=result.valid, status=result.solver_status,
                           initialization=result.timing.get('initialization'),
                           validation=result.timing.get('validation'),
                           xyz=result.metrics.get('xyz_coefficients'),
                           yaw=result.metrics.get('yaw_coefficients'))
                rows.append(row)
                stream.write(json.dumps(base.base.json_safe(row), allow_nan=False)+'\n')
            if pair[0].valid != pair[1].valid or pair[0].solver_status != pair[1].solver_status:
                raise ValueError('ADMISSION_CHANGED_WITH_BOUND')
            if pair[0].valid:
                assert np.array_equal(pair[0].metrics['xyz_coefficients'],
                                      pair[1].metrics['xyz_coefficients'])
                assert np.array_equal(pair[0].metrics['yaw_coefficients'],
                                      pair[1].metrics['yaw_coefficients'])
    summary = {str(flag): dict(
        count=sum(r['bound_enabled'] == flag for r in rows),
        feasible=sum(r['valid'] for r in rows if r['bound_enabled'] == flag),
        seconds=quantiles([r['seconds'] for r in rows
                          if r['bound_enabled'] == flag]))
               for flag in (False, True)}
    summary['all_admissions_and_feasible_coefficients_equal'] = True
    (output/'bounds_summary.json').write_text(json.dumps(summary, indent=2))
    print(json.dumps(summary))


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('input', type=Path)
    parser.add_argument('output', type=Path)
    args = parser.parse_args()
    base.base._core()
    run(args.input, args.output)
