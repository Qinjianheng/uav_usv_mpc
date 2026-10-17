"""Make isolated dataclass audit snapshots without copying immutable scalar leaves."""

from copy import deepcopy
from dataclasses import fields, is_dataclass


_ATOMIC_TYPES = frozenset((type(None), bool, int, float, complex, str, bytes))


def record_snapshot(value):
    """Preserve dataclass/container shapes and deepcopy semantics for mutable leaves."""
    if type(value) in _ATOMIC_TYPES:
        return value
    if is_dataclass(value) and not isinstance(value, type):
        return {field.name: record_snapshot(getattr(value, field.name)) for field in fields(value)}
    if isinstance(value, tuple) and hasattr(value, '_fields'):
        return type(value)(*(record_snapshot(item) for item in value))
    if isinstance(value, (tuple, list)):
        return type(value)(record_snapshot(item) for item in value)
    if isinstance(value, dict):
        return type(value)((record_snapshot(key), record_snapshot(item))
                           for key, item in value.items())
    return deepcopy(value)
