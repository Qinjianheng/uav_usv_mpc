"""Audit snapshots retain ownership while avoiding generic copies of atomic values."""
from collections import namedtuple
from dataclasses import dataclass

import numpy as np

from uav_control.common.record_snapshot import record_snapshot


@dataclass(frozen=True)
class Context:
    """Immutable identity and epoch in a representative audit record."""

    mission: int
    acquisition: float


@dataclass(frozen=True)
class Record:
    """Immutable container with independently mutable diagnostic leaves."""

    context: Context
    samples: tuple
    metrics: dict


def test_snapshot_preserves_complete_dataclass_payload_and_tuple_shape():
    result = record_snapshot(Record(Context(3, 1792123842.125),
                                    ((1., -2., 0.), (2., -2., .1)),
                                    {'valid': True, 'reason': '', 'expiry': None}))
    assert result == {'context': {'mission': 3, 'acquisition': 1792123842.125},
                      'samples': ((1., -2., 0.), (2., -2., .1)),
                      'metrics': {'valid': True, 'reason': '', 'expiry': None}}
    assert type(result['samples']) is tuple
    assert type(result['samples'][0]) is tuple


def test_snapshot_isolates_mutable_metrics_and_numpy_buffers_in_both_directions():
    original = Record(Context(3, 42.), (), {'history': [1., {'v': np.array([2., 3.])}]})
    snapshot = record_snapshot(original)
    original.metrics['history'][0] = 9.
    original.metrics['history'][1]['v'][0] = 8.
    assert snapshot['metrics']['history'][0] == 1.
    np.testing.assert_array_equal(snapshot['metrics']['history'][1]['v'], [2., 3.])
    snapshot['metrics']['history'][1]['v'][1] = 7.
    assert original.metrics['history'][1]['v'][1] == 3.


def test_atomic_subclasses_keep_custom_deepcopy_semantics():
    class TrackedFloat(float):
        def __deepcopy__(self, memo):
            return {'copied_value': float(self)}

    snapshot = record_snapshot(Record(Context(3, 42.), (), {'custom': TrackedFloat(2.)}))
    assert snapshot['metrics']['custom'] == {'copied_value': 2.}


def test_namedtuple_retains_type_and_recursively_converts_dataclass_fields():
    Pair = namedtuple('Pair', 'context values')
    original = Record(Context(3, 42.), (), {'pair': Pair(Context(4, 43.), [1.])})
    result = record_snapshot(original)['metrics']['pair']
    assert type(result) is Pair
    assert result.context == {'mission': 4, 'acquisition': 43.}
    original.metrics['pair'].values[0] = 2.
    assert result.values == [1.]
