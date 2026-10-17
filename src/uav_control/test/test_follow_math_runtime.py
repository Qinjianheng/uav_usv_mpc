"""A local numerical thread cap changes resource use without changing the equations."""
import numpy as np

from uav_control.guidance.follow_math_runtime import configure_numerical_threads


def test_local_blas_cap_reports_actual_backend_and_preserves_linear_solution():
    matrix = np.arange(36., dtype=float).reshape(6, 6)+np.eye(6)*100.
    rhs = np.arange(6., dtype=float)
    original = np.linalg.solve(matrix, rhs)
    report = configure_numerical_threads(1)
    assert report['requested_threads'] == 1
    if report['supported']:
        assert report['actual_threads'] == 1
    np.testing.assert_allclose(np.linalg.solve(matrix, rhs), original, rtol=1e-13, atol=1e-15)
