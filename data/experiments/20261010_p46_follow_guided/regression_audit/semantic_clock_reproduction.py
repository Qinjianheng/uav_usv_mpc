"""Exercise real geometry with a deterministic 40 ms scheduling stall per assessment."""
import sys
import time
from pathlib import Path
from unittest.mock import patch

root = Path(__file__).resolve().parents[4]
sys.path.insert(0, str(root/'src/uav_control/test'))
import test_follow_revalidation as tests
from uav_control.guidance.follow_problem import FollowProblem

assess = FollowProblem.assess
revalidate = tests.revalidate_prediction


def stalled_assessment(*args, **kwargs):
    started = time.perf_counter()
    result = assess(*args, **kwargs)
    while time.perf_counter()-started < .04:
        pass
    return result


def logged_revalidation(*args, **kwargs):
    updated, report = revalidate(*args, **kwargs)
    print('report:', report['valid'], report['reason'], 'elapsed:', report['elapsed'])
    return updated, report


failures = 0
with patch.object(FollowProblem, 'assess', stalled_assessment):
    with patch.object(tests, 'revalidate_prediction', logged_revalidation):
        for name in (
                'test_same_geometry_new_version_checks_full_curve_preserves_boundary_and_navigation',
                'test_changed_prediction_behind_camera_rejects_even_small_solver_cost'):
            try:
                getattr(tests, name)()
                print(name, 'PASS')
            except AssertionError:
                failures += 1
                print(name, 'ASSERTION_FAILED')
sys.exit(bool(failures))
