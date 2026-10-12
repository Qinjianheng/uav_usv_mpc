"""P47 replay must isolate algorithm/budget axes and retain causal populations."""
from pathlib import Path
import hashlib
import json
import sys

import pytest

from test_follow_research_algorithms import request
from test_progress_follow import model
from test_p46_follow_study import comparison_row, shifted

sys.path.insert(0, str(Path(__file__).resolve().parents[3]/'scripts'))


def test_main_groups_keep_original_engines_and_isolate_refinement(monkeypatch):
    import p47_follow_study as study
    calls = []

    class Engine:
        def __init__(self, original, **kwargs):
            calls.append((original, kwargs))

    monkeypatch.setattr(study, 'P44FollowSolver', Engine)
    monkeypatch.setattr(study, 'FollowGuidedMinco', Engine)
    monkeypatch.setattr(study, 'p47_engine', lambda: Engine)
    original, now = model(), lambda: 100.
    for arm in study.experiment_arms('main').values():
        study.make(original, arm, 1.2, now)
    assert calls[0][1] == dict(duration=1.2, rolling=False, wall_clock=now,
                               ablation='D', refinement='none')
    assert calls[1][1] == dict(duration=1.2, rolling=False, wall_clock=now,
                               refinement='none', fallback_enabled=True)
    c, d = calls[2][1], calls[3][1]
    assert c['terminal_mode'] == d['terminal_mode'] == 'qva'
    assert c['refinement'] is False and d['refinement'] is True
    assert {k: v for k, v in c.items() if k != 'refinement'} == {
        k: v for k, v in d.items() if k != 'refinement'}


def test_terminal_and_budget_suites_change_one_axis_only():
    from p47_follow_study import experiment_arms
    terminal = list(experiment_arms('terminal').values())
    assert {arm['terminal_mode'] for arm in terminal} == {'a', 'va', 'qva'}
    assert all(arm['refinement'] for arm in terminal)
    assert len({arm['scheduling'] for arm in terminal}) == 1
    budget = list(experiment_arms('budget').values())
    assert {arm['scheduling'] for arm in budget} == {'legacy', 'adaptive'}
    assert len({(arm['terminal_mode'], arm['refinement']) for arm in budget}) == 1


def test_native_cadence_and_matched_window_are_shared_with_p46():
    import p46_follow_study as baseline
    import p47_follow_study as study
    assert study.read_scenes is baseline.read_scenes
    assert study.run_case is baseline.run_case
    assert study.matched is baseline.matched
    assert study.evaluation is baseline.evaluation
    raw = request()
    cases = [('fixture', i, str(i), shifted(raw, epoch), model())
             for i, epoch in enumerate((100., 101.02, 102.04), 1)]
    cadence = study.cadence(cases)
    assert cadence['count'] == 3
    assert cadence['interval_seconds']['p50'] == pytest.approx(1.02)
    assert cadence['fabricated_samples'] == 0
    groups = {'A': [comparison_row(1, 20., 2.), comparison_row(2, 30., 4.)],
              'D': [comparison_row(1, 20., None), comparison_row(2, 30., 1.)]}
    report = study.matched(groups)
    assert report['steady_count'] == 1
    assert report['groups']['A']['after20']['fixed'] == 4.


def test_boundary_offset_preserves_measurement_and_projection_distinction():
    from p47_follow_study import boundary_offset
    req = shifted(request(2.), 100.)
    payload = dict(req.__dict__)
    payload['context'] = dict(req.context.__dict__)
    payload['state'] = list(req.state)
    payload['state'][0] += 2.
    row = dict(request=payload)
    offset = boundary_offset(row)
    assert offset['projected_measurement']['position'] == pytest.approx(2.)
    assert offset['measurement']['position'] != pytest.approx(2.)
    assert offset['navigation_epoch'] == 100.
    assert offset['execution_epoch'] == pytest.approx(100.15)


def test_rejection_categories_do_not_mislabel_fallback_as_independent_success():
    from p47_follow_study import summarize
    row = comparison_row(1, 100., 0., 0.)
    row.update(seconds=.03, stop_reason=None, request=None, result=dict(
        valid=True, solver_status='SAMPLED_FEASIBLE', timing={},
        constraint_violations={}, metrics={'follow_guided': dict(
            independent_valid=False, independent_status='HORIZONTAL_JERK_LIMIT',
            initial_status='HORIZONTAL_JERK_LIMIT', fallback_used=True,
            fallback_valid=True)}))
    report = summarize([row], 'D', 100., 100.)
    assert report['feasible'] == report['fallback_valid'] == 1
    assert report['independent_valid'] == 0
    assert report['rejections']['independent']['jerk'] == 1
    assert report['rejections']['final']['jerk'] == 0
    assert report['final_feasible_fraction'] == 1.
    assert report['independent_feasible_fraction'] == 0.


def test_budget_stress_delays_clock_without_rewriting_request(monkeypatch):
    import p47_follow_study as study
    from uav_control.controllers.follow_mpc_seed import MpcSeedResult
    from uav_control.guidance.fast_follow_minco import freshness
    req = shifted(request(), 100.)
    seen = []

    class Engine:
        def __init__(self, original, **kwargs):
            self.now = kwargs['wall_clock']

        def solve(self, actual):
            seen.append((actual, self.now()))
            return MpcSeedResult(actual.context, solver_status='CONTROLLED_REJECTION')

    monkeypatch.setattr(study, 'p47_engine', lambda: Engine)
    arm = dict(engine='p47', terminal_mode='va', refinement=True,
               scheduling='adaptive', remaining_seconds=.012)
    result = study.make(model(), arm, 1.2, lambda: req.now_stamp).solve(req)
    until, _ = freshness(req, req.now_stamp)
    assert seen[0][0] is req
    assert seen[0][1] == pytest.approx(until-.02-.012)
    assert result.metrics['budget_stress']['remaining_compute_seconds'] == pytest.approx(.012)
    assert result.context == req.context


def test_output_guard_rejects_historical_or_existing_evidence(tmp_path, monkeypatch):
    import p47_follow_study as study
    root = tmp_path/'new_comparison'
    monkeypatch.setattr(study, 'OUTPUT_ROOT', root)
    with pytest.raises(ValueError, match='P47_COMPARISON_OUTPUT_REQUIRED'):
        study.validate_output(tmp_path/'historical')
    root.mkdir()
    with pytest.raises(FileExistsError):
        study.validate_output(root)
    study.validate_output(root/'new_run')


def test_exact_jerk_integral_includes_all_cross_terms():
    from p47_follow_study import coefficient_quality
    # x=t^3+t^4+t^5, jerk=6+24t+60t^2; integral over [0,1] is 2052.
    coefficients = [[[0., 0., 0.], [0., 0., 0.], [0., 0., 0.],
                     [1., 0., 0.], [1., 0., 0.], [1., 0., 0.]]]
    quality = coefficient_quality(dict(xyz_coefficients=coefficients, durations=[1.],
                                       jerk_integral=1656.))
    assert quality['exact_jerk_integral'] == pytest.approx(2052.)
    assert quality['legacy_reported_jerk_integral'] == 1656.
    assert quality['derivative_peaks']['horizontal_jerk']['value'] == pytest.approx(90.)


def test_runner_persists_suites_and_hashes_without_touching_sources(tmp_path, monkeypatch):
    import p47_follow_study as study
    root, historical = tmp_path/'comparison', tmp_path/'old_summary.json'
    historical.write_text('{"frozen": true}\n')
    old_bytes = historical.read_bytes()
    monkeypatch.setattr(study, 'OUTPUT_ROOT', root)
    monkeypatch.setattr(study, 'HISTORICAL', historical)
    req = shifted(request(), 100.)
    cases = [('fixture', 1, 'source-line', req, model())]
    monkeypatch.setattr(study, 'read_scenes', lambda limit: (
        {'f2': dict(single=cases, rolling=cases)}, {'f2': dict(original_request_count=1)}))
    monkeypatch.setattr(study, 'source_manifest', lambda: dict(sha256={}, git_head='fixture'))

    def fake_run(cases, path, group, horizon, rolling=False, factory=None):
        row = comparison_row(1, 100., 0., 0.)
        row.update(seconds=.001, group=group, raw_request={'identity': 1}, request=None,
                   constraint_fingerprint='same', stop_reason=None, result=dict(
                       valid=True, solver_status='SAMPLED_FEASIBLE', timing={},
                       metrics={'follow_guided': {'independent_valid': True}}))
        if rolling:
            row['continuous_coverage_until'] = 101.2
        path.write_text(json.dumps(row)+'\n')
        return [row]

    monkeypatch.setattr(study, 'run_case', fake_run)
    report = study.run(root/'first', horizons=(1.2,),
                       suites=('main', 'budget_stress'), plots=False)
    assert len(report['configurations']) == 16
    assert report['same_input_checks']['f2_h1.2_main']['all_identical']
    assert 'f2_h1.2_main' in report['matched_rolling']
    assert not any('stress' in key for key in report['matched_rolling'])
    assert historical.read_bytes() == old_bytes
    assert len(list((root/'first').glob('*_analysis.jsonl'))) == 16
    for line in (root/'first'/'SHA256SUMS').read_text().splitlines():
        expected, relative = line.split('  ', 1)
        assert hashlib.sha256((root/'first'/relative).read_bytes()).hexdigest() == expected
