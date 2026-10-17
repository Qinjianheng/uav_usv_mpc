"""Bound native numerical concurrency in the owning planner/receiver process only."""
import ctypes

from numpy.core import _multiarray_umath


def configure_numerical_threads(count=1):
    """Use the loaded OpenBLAS public API; report unsupported backends explicitly."""
    if isinstance(count, bool) or not isinstance(count, int) or count < 1:
        raise ValueError('INVALID_NUMERICAL_THREAD_COUNT')
    library = ctypes.CDLL(_multiarray_umath.__file__)
    for suffix in ('64_', ''):
        setter = getattr(library, 'openblas_set_num_threads'+suffix, None)
        getter = getattr(library, 'openblas_get_num_threads'+suffix, None)
        if setter is not None and getter is not None:
            setter.argtypes, setter.restype = [ctypes.c_int], None
            getter.argtypes, getter.restype = [], ctypes.c_int
            before = getter()
            setter(count)
            return dict(supported=True, backend='OpenBLAS'+suffix,
                        previous_threads=before, requested_threads=count, actual_threads=getter())
    return dict(supported=False, backend='unidentified', requested_threads=count,
                actual_threads=None, previous_threads=None)
