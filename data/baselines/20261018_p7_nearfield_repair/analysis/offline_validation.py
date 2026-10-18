"""Re-run the independent regression fixture and retain its actual trajectory."""
import hashlib
import importlib.util
import json
from pathlib import Path
import sys

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np

root = Path(__file__).resolve().parent
workspace = root.parents[2]
sys.path.insert(0, str(workspace/'src/uav_control'))
path = workspace/'src/uav_control/test/test_p7_nearfield_braking.py'
spec = importlib.util.spec_from_file_location('p7_nearfield_fixture', path)
fixture = importlib.util.module_from_spec(spec)
spec.loader.exec_module(fixture)
original = fixture.P7VisualGuidance
reference = fixture.capture_reference
records, states, finished = [], {}, {}


def recorded_reference(p, q, w):
    states['p'] = np.array(p).copy()
    return reference(p, q, w)


class Recorder(original):
    def step(self, *args, **kwargs):
        out = super().step(*args, **kwargs)
        records.append([args[0]-10., out.distance, out.error_v, out.closing_speed,
                        *states['p'], *out.velocity, self.image_v_rate])
        return out


def trace(frame, event, arg):
    if (event == 'return' and frame.f_code.co_name ==
            'test_speed_heave_lag_counterexample_captures_without_leaving_camera'):
        for name in ('t', 'distance', 'maximum_error'):
            finished[name] = float(frame.f_locals[name])
    return trace


fixture.P7VisualGuidance = Recorder
fixture.capture_reference = recorded_reference
results = []
fig, axes = plt.subplots(1, 2, figsize=(11, 4))
for phase in (0., np.pi/2., np.pi, 3.*np.pi/2.):
    records.clear()
    sys.settrace(trace)
    try:
        fixture.test_speed_heave_lag_counterexample_captures_without_leaving_camera(phase)
    finally:
        sys.settrace(None)
    rows = np.asarray(records)
    results.append(dict(phase_rad=phase, capture_elapsed_s=finished['t'],
                        capture_distance_m=finished['distance'],
                        maximum_center_error_v=finished['maximum_error'],
                        minimum_sampled_height_m=float(min(-rows[:, 6]))))
    np.savetxt(root/f'offline_phase_{len(results)}.csv', rows, delimiter=',',
               header='time,range_xy,error_v,closure,px,py,pz,command_vx,command_vy,image_v_rate',
               comments='')
    axes[0].plot(rows[:, 0], rows[:, 1], label=f'phase {phase:.2f}')
    axes[1].plot(rows[:, 0], rows[:, 2])
axes[0].set(xlabel='time (s)', ylabel='horizontal range (m)')
axes[0].legend()
axes[1].set(xlabel='time (s)', ylabel='vertical normalized center error')
axes[1].axhline(np.tan(1.74/2.)*.75, linestyle='--', color='red', label='sensor bottom')
axes[1].axhline(.7, linestyle=':', color='gray', label='braking reference')
axes[1].legend()
fig.tight_layout()
fig.savefig(root/'offline_validation.png', dpi=160)
(root/'offline_validation.json').write_text(json.dumps(dict(
    fixture_sha256=hashlib.sha256(path.read_bytes()).hexdigest(), results=results,
    scope='Offline fixture, level body, causal KF source, independent camera and lag; '
          'production sea guard retained. This does not qualify real flight.'), indent=2)+'\n')
print(json.dumps(results, indent=2))
