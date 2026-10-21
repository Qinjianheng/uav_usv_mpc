"""P763 engineering PVA outer loop; independent of the unchanged paper Y guidance."""
import math

import numpy as np

from .p76_paper_thrust import physical_thrust


def sample_reference(curve, epoch, deadline):
    """Sample all eleven polynomial quantities at the actual control epoch, inside its lease."""
    if not math.isfinite(epoch) or not epoch < deadline:
        raise ValueError('P763_REFERENCE_LEASE_EXPIRED')
    values = np.asarray(curve.sample(epoch), dtype=float)
    if values.shape != (11,) or not np.isfinite(values).all():
        raise ValueError('P763_REFERENCE_PVA_INVALID')
    return values


class SupervisorSink:
    """The old 20Hz supervisor has no PX4 outlet and grants no interpolated velocity reference."""

    def __init__(self):
        self.discarded = 0

    def publish(self, _message):
        """Retain only a counter; never promote a discrete velocity command to a trajectory."""
        self.discarded += 1


class OutputLimiter:
    """One physical rate/collective continuity boundary shared across FOLLOW/Y/recovery."""

    def __init__(self, contract):
        c = contract['candidate']
        self.bound = np.array((c['maximum_body_roll_pitch_rate'],)*2
                              + (c['maximum_body_yaw_rate'],))
        self.rate_slew = c['maximum_body_angular_acceleration']
        self.thrust_slew = contract['bodyrate_follow']['maximum_thrust_slew']
        self.rates, self.thrust = np.zeros(3), None

    def initialize(self, rates, thrust):
        """Seed executed values; reject a native state outside the calibrated envelope."""
        rates = np.asarray(rates, dtype=float)
        if (rates.shape != (3,) or not np.isfinite(rates).all()
                or np.any(np.abs(rates) > self.bound) or not -1 <= thrust <= 0):
            raise ValueError('P763_OUTPUT_SEED_OUTSIDE_CALIBRATION')
        self.rates, self.thrust = rates.copy(), float(thrust)

    def apply(self, rates, thrust, dt):
        """Bound slew using actual elapsed time, including the first paper output after Y."""
        rates = np.asarray(rates, dtype=float)
        if (not 0 < dt <= .125 or rates.shape != (3,) or not np.isfinite(rates).all()
                or not math.isfinite(thrust) or not -1 <= thrust <= 0):
            raise ValueError('P763_OUTPUT_INVALID')
        if self.thrust is None:
            raise ValueError('P763_EXECUTED_OUTPUT_SEED_REQUIRED')
        delta = np.clip(rates, -self.bound, self.bound)-self.rates
        self.rates += delta*min(1., self.rate_slew*dt/max(np.linalg.norm(delta), 1e-12))
        self.thrust += float(np.clip(thrust-self.thrust,
                                     -self.thrust_slew*dt, self.thrust_slew*dt))
        return self.rates.copy(), self.thrust


class WaypointReference:
    """Explicit quintic setup/hold trajectory, never a held supervisor velocity message."""

    def __init__(self, epoch, position, velocity, goal, duration, yaw):
        if not math.isfinite(duration) or duration <= 0:
            raise ValueError('P763_WAYPOINT_DURATION')
        self.start, self.end, self.duration, self.yaw = epoch, epoch+duration, duration, yaw
        p, v, goal = map(lambda x: np.asarray(x, dtype=float), (position, velocity, goal))
        if any(x.shape != (3,) or not np.isfinite(x).all() for x in (p, v, goal)):
            raise ValueError('P763_WAYPOINT_VALUES')
        self.goal = goal
        t = duration
        matrix = np.array(((t**3, t**4, t**5), (3*t*t, 4*t**3, 5*t**4),
                           (6*t, 12*t*t, 20*t**3)))
        self.coefficients = np.vstack((p, v, np.zeros(3), np.linalg.solve(
            matrix, np.vstack((goal-p-v*t, -v, np.zeros(3))))))

    def sample(self, epoch):
        """Hold the terminal point after completion; never extrapolate a polynomial."""
        if epoch < self.start:
            raise ValueError('P763_WAYPOINT_BEFORE_START')
        if epoch >= self.end:
            return np.r_[self.goal, np.zeros(6), self.yaw, 0.]
        t = epoch-self.start
        values = [np.polynomial.polynomial.polyval(
            t, np.polynomial.polynomial.polyder(self.coefficients, m=d, axis=0))
            for d in range(3)]
        return np.concatenate(values+[np.array((self.yaw, 0.))])


class BodyrateTracker:
    """Offline-candidate critically damped P/V feedback and constrained FRD attitude tracking."""

    def __init__(self, contract, mass, maximum_thrust, thrust_map):
        self.c, self.mass = contract, mass
        self.maximum_thrust, self.map = maximum_thrust, thrust_map
        self.settings = contract['bodyrate_follow']
        self.previous_acceleration = np.zeros(3)
        self.limiter = OutputLimiter(contract)

    def initialize(self, rates, thrust):
        """Keep a single executed output state through all authorized guidance owners."""
        self.limiter.initialize(rates, thrust)

    def limit_output(self, output, dt):
        """Apply physical interface limits without changing the paper's requested guidance law."""
        output = dict(output)
        output['rates'], output['thrust'] = self.limiter.apply(
            output['rates'], output['thrust'], dt)
        self.previous_acceleration = np.asarray(output['acceleration'], dtype=float).copy()
        return output

    def step(self, reference, position, velocity, rotation, dt, angular_rates,
             feedback_reference=None):
        """Execution PVA feedforward and native-epoch P/V errors, without extrapolation."""
        ref, p, v, r, omega = map(lambda x: np.asarray(x, dtype=float),
                                  (reference, position, velocity, rotation, angular_rates))
        feedback = ref if feedback_reference is None else np.asarray(
            feedback_reference, dtype=float)
        if (ref.shape != (11,) or p.shape != (3,) or v.shape != (3,) or r.shape != (3, 3)
                or feedback.shape != (11,) or not np.isfinite(feedback).all()
                or omega.shape != (3,)
                or not all(np.isfinite(x).all() for x in (ref, p, v, r, omega))
                or not np.allclose(r.T@r, np.eye(3), atol=1e-6) or np.linalg.det(r) < 0
                or not 0 < dt <= .125):
            raise ValueError('P763_BODYRATE_INPUT')
        s, gravity = self.settings, self.c['dynamics']['gravity']
        acceleration = ref[6:9]+s['kp']*(feedback[:3]-p)+s['kv']*(feedback[3:6]-v)
        acceleration *= min(1., s['maximum_acceleration']/max(np.linalg.norm(acceleration), 1e-12))
        horizontal = np.linalg.norm(acceleration[:2])
        tilt_bound = max(0., gravity-acceleration[2])*math.tan(s['maximum_tilt'])
        acceleration[:2] *= min(1., tilt_bound/max(horizontal, 1e-12))
        change = acceleration-self.previous_acceleration
        acceleration = self.previous_acceleration+change*min(
            1., s['maximum_jerk']*dt/max(np.linalg.norm(change), 1e-12))
        self.previous_acceleration = acceleration.copy()
        down = np.array((0., 0., gravity))-acceleration
        down /= np.linalg.norm(down)
        heading = np.array((math.cos(ref[9]), math.sin(ref[9]), 0.))
        right = np.cross(down, heading)
        right /= np.linalg.norm(right)
        desired = np.column_stack((np.cross(right, down), right, down))
        error = .5*(desired.T@r-r.T@desired)
        rates = -s['attitude_gain']*np.array((error[2, 1], error[0, 2], error[1, 0]))
        rates += r.T@np.array((0., 0., ref[10]))
        force = physical_thrust(self.mass, r, acceleration, gravity, self.maximum_thrust)
        thrust = self.map.to_body_z(force)
        return self.limit_output(dict(rates=rates, thrust=thrust, force_newtons=force,
                                      acceleration=acceleration, reference=ref), dt)
