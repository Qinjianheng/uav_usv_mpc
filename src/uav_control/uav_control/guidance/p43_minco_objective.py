"""Normalized P/V/A MINCO objective; full-pose smooth sphere proxy is ranking only."""
import numpy as np

from uav_control.guidance.fast_minco_objective import FastMincoObjective
from uav_control.guidance.follow_profile import profiled
from uav_control.guidance.follow_reference import forecast_viewpoint


class TrackingMincoObjective(FastMincoObjective):
    """Existing analytic MINCO adjoint with explicit interpolated moving reference derivatives."""

    def __init__(self, problem, seed, config, beta=0., previous_jerk=(0., 0., 0.),
                 reference=None, tracking_weights=(4./25, 2./9, 1./9),
                 continuity_weight=.01/36, yaw_optimize=True):
        """Freeze reference interpolation once for this objective."""
        super().__init__(problem, seed, config, yaw_optimize, 'full')
        self.reference_times = (np.linspace(0., self.total, 9) if reference is None
                                else np.asarray(reference.times))
        samples = ([forecast_viewpoint(problem, t, beta)[:3] for t in self.reference_times]
                   if reference is None else None)
        self.references = (np.asarray(samples).transpose(1, 0, 2) if reference is None else
                           np.asarray((reference.positions, reference.velocities,
                                       reference.accelerations)))
        self.tracking_weights = tracking_weights
        self.continuity_weight = continuity_weight
        self.previous_jerk = np.asarray(previous_jerk)
        self.beta = beta

    def local_partials(self, args):
        """Batch independent complex-step directions into one geometry kernel call."""
        directions = [(k, axis) for k, field in enumerate(args)
                      for axis in range(field.shape[1] if field.ndim == 2 else 1)]
        n, blocks = len(args[0]), len(directions)
        stacked = [np.concatenate([field.astype(complex)]*blocks) for field in args]
        for block, (k, axis) in enumerate(directions):
            rows = slice(block*n, (block+1)*n)
            if args[k].ndim == 2:
                stacked[k][rows, axis] += 1e-24j
            else:
                stacked[k][rows] += 1e-24j
        imaginary = self.local_cost(*stacked).imag.reshape(blocks, n)/1e-24
        partials = [np.empty_like(field) for field in args]
        for derivative, (k, axis) in zip(imaginary, directions):
            if args[k].ndim == 2:
                partials[k][:, axis] = derivative
            else:
                partials[k][:] = derivative
        self.counts['local_complex_steps'] += blocks
        return partials

    def tracking(self, fields, times):
        """Normalize P/V/A errors and jerk switching with exact piecewise time partials."""
        index = np.clip(np.searchsorted(self.reference_times, times, side='right')-1,
                        0, len(self.reference_times)-2)
        dt = (self.reference_times[index+1]-self.reference_times[index])[:, None]
        fraction = (times-self.reference_times[index])[:, None]/dt
        weights = self.tracking_weights
        cost, time_partial = np.zeros(len(times)), np.zeros(len(times))
        partials = []
        for k, weight in enumerate(weights):
            slope = (self.references[k, index+1]-self.references[k, index])/dt
            ref = self.references[k, index]+fraction*dt*slope
            delta = fields[k]-ref
            gradient = 2*weight*delta
            cost += weight*np.sum(delta**2, axis=1)
            time_partial -= np.sum(gradient*slope, axis=1)
            partials.append(gradient)
        difference = fields[3]-self.previous_jerk
        cost += self.continuity_weight*np.sum(difference**2, axis=1) + .01*self.beta**2
        partials.append(2*self.continuity_weight*difference)
        return cost, partials, time_partial

    @profiled('qt_yaw_optimization')
    def __call__(self, x):
        """Q/T gradients use the existing coefficient adjoint; no variablewise objective FD."""
        return super().__call__(x)
