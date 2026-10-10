"""Exclusive monotonic attribution must not add nested cumulative times."""
from uav_control.guidance import follow_profile as profile


def test_nested_intervals_are_exclusive_and_counts_are_local():
    ticks = iter((0., 1., 3., 6., 8., 10.))
    with profile.CycleProfile(7, clock=lambda: next(ticks)) as cycle:
        with profile.stage('candidate'):
            with profile.stage('fov'):
                profile.count('projection_samples', 9)
    report = cycle.report()
    assert report['cycle_id'] == 7
    assert report['stages']['candidate']['seconds'] == 4.
    assert report['stages']['fov']['seconds'] == 3.
    assert report['unattributed_seconds'] == 3.
    assert report['total_seconds'] == 10.
    assert report['counts'] == {'projection_samples': 9}


def test_no_context_is_a_transparent_call():
    @profile.profiled('stage')
    def operation(x):
        return x + 1
    assert operation(9) == 10
    profile.count('not_recorded')
    assert profile.current() is None
