"""Independent A1/A2 body control and fail-closed conservative pulse admission.

These controls are offline candidates, not qualified flight dynamics. No paper guidance
is imported. The historical physical map is explicitly unqualified and only usable
inside this isolated calibration path after exact model/PX4 identity verification.
"""
import json
import math
from pathlib import Path
from types import SimpleNamespace

import numpy as np
from scipy.spatial.transform import Rotation

from uav_control.control.p76_paper_thrust import CalibratedThrustMap


ARCHITECTURE = 'parallel_dkf_unified_bodyrate'
WORST_RESPONSE_DELAY = .24
SPEED_LIMIT = 7.


class BootstrapThrustMap(CalibratedThrustMap):
    """Historical motion-derived curve, deliberately incapable of formal qualification."""

    def __init__(self, force, normalized):
        super().__init__(force, normalized, qualified=False)

    def to_body_z(self, force_newtons, require_qualified=False):
        if require_qualified:
            raise ValueError('P763_BOOTSTRAP_NOT_QUALIFICATION')
        return super().to_body_z(force_newtons, require_qualified=False)


def load_bootstrap(session, current_identity):
    """Compare physical identities only; changed runtime sources are not inherited as A."""
    session = Path(session)
    identity = json.loads((session/'identity.json').read_text())
    if any(identity.get(k) != current_identity.get(k) for k in ('model', 'px4')):
        raise ValueError('P763_BOOTSTRAP_IDENTITY_MISMATCH')
    analysis = json.loads((session/'analysis.json').read_text())
    curve = analysis['measured_curve']
    result = BootstrapThrustMap([r['force'] for r in curve], [r['normalized'] for r in curve])
    result.source = str(session)
    result.hover = float(analysis['hover_normalized'])
    return result


def pulse_admission(*, velocity, acceleration, sample_age, duration, mass,
                    maximum_thrust, gravity, upward_room, clearance,
                    upward_braking, downward_braking):
    """Bound a high pulse plus known worst execution delay and native sample age.

    No instantaneous motor response is assumed. Static maximum bounds continuing
    acceleration even if the command is already being replaced. A measured response
    later improving this bound requires a new physical qualification, not a new limit.
    """
    v, a = np.asarray(velocity, float), np.asarray(acceleration, float)
    scalars = (sample_age, duration, mass, maximum_thrust, gravity, upward_room,
               clearance, upward_braking, downward_braking)
    if (v.shape != (3,) or a.shape != (3,) or not np.isfinite(v).all()
            or not np.isfinite(a).all() or not all(math.isfinite(x) for x in scalars)
            or duration < 0 or min(mass, maximum_thrust, gravity,
                                   upward_braking, downward_braking) <= 0):
        raise ValueError('P763_PULSE_GUARD_INPUT')
    reasons = []
    if not 0 <= sample_age <= .125:
        reasons.append('NATIVE_SAMPLE_AGE')
    horizon = duration+WORST_RESPONSE_DELAY+max(0., sample_age)
    upward_acceleration = max(0., maximum_thrust/mass-gravity, -a[2])
    up, down = max(0., -v[2]), max(0., v[2])
    peak_up = up+upward_acceleration*horizon
    peak_speed = math.hypot(float(np.linalg.norm(v[:2])), peak_up)
    rise = up*horizon+.5*upward_acceleration*horizon*horizon
    rise += peak_up*peak_up/(2*upward_braking)
    # During execution delay a full-power command can only reduce descent. Keep
    # the existing descent velocity until braking is confirmed; do not credit it.
    drop = down*horizon+down*down/(2*downward_braking)
    if peak_speed >= SPEED_LIMIT:
        reasons.append('PREDICTED_SPEED_LIMIT')
    if rise >= upward_room:
        reasons.append('PREDICTED_UPPER_ALTITUDE')
    if clearance-drop <= 2.:
        reasons.append('PREDICTED_LOWER_CLEARANCE')
    return dict(admitted=not reasons, reasons=reasons, speed_limit=SPEED_LIMIT,
                worst_response_delay=WORST_RESPONSE_DELAY, sample_age=sample_age,
                duration=duration, horizon=horizon, predicted_peak_speed=peak_speed,
                maximum_upward_acceleration=upward_acceleration,
                upward_stopping_distance=rise, downward_stopping_distance=drop,
                upward_room=upward_room, clearance=clearance,
                scope='Conservative logic bound; no physical response qualification')


class CalibrationBodyHold:
    """Small bounded local P/V outer loop used only for calibration brake and hold."""

    def __init__(self, mass, gravity, mapping):
        self.mass, self.gravity, self.mapping = mass, gravity, mapping
        self.rates, self.last_acceleration = np.zeros(3), np.zeros(3)
        self.thrust = None

    def initialize(self, rates, thrust):
        self.rates, self.thrust = np.asarray(rates, float).copy(), float(thrust)

    def step(self, target, position, velocity, rotation, dt, yaw, *, brake=False,
             desired_velocity=None):
        p, v, goal = map(lambda x: np.asarray(x, float), (position, velocity, target))
        r = np.asarray(rotation, float)
        if (any(x.shape != (3,) or not np.isfinite(x).all() for x in (p, v, goal))
                or r.shape != (3, 3) or not np.isfinite(r).all() or not 0 < dt <= .02):
            raise ValueError('P763_BODY_HOLD_INPUT')
        vr = np.zeros(3) if desired_velocity is None else np.asarray(desired_velocity, float)
        acceleration = np.clip(1.2*(goal-p)+2.2*(vr-v), -4., 4.)
        override = bool(brake and abs(v[2]) > .25)
        if override:
            # Upward velocity requires less than hover force; downward velocity
            # requires more. This command explicitly replaces the last high pulse.
            acceleration[2] = 4.*math.copysign(1., -v[2])
        else:
            change = acceleration-self.last_acceleration
            acceleration = self.last_acceleration+change*min(
                1., 12.*dt/max(np.linalg.norm(change), 1e-9))
        self.last_acceleration = acceleration.copy()
        specific = np.array([0., 0., self.gravity])-acceleration
        specific[:2] *= min(1., max(.01, specific[2])*math.tan(.5)
                            / max(np.linalg.norm(specific[:2]), 1e-9))
        z = specific/np.linalg.norm(specific)
        heading = np.array([math.cos(yaw), math.sin(yaw), 0.])
        y = np.cross(z, heading)
        y /= np.linalg.norm(y)
        desired = np.column_stack((np.cross(y, z), y, z))
        rates = np.clip(4.*Rotation.from_matrix(r.T@desired).as_rotvec(),
                        [-1.5, -1.5, -1.2], [1.5, 1.5, 1.2])
        change = rates-self.rates
        self.rates += change*min(1., 3.*dt/max(np.linalg.norm(change), 1e-9))
        force = float(np.clip(self.mass*np.linalg.norm(specific),
                              self.mapping.force[0], self.mapping.force[-1]))
        self.thrust = -self.mapping.to_body_z(force)
        return SimpleNamespace(rates=self.rates.copy(), thrust=self.thrust,
                               force_newtons=force, acceleration=acceleration,
                               explicit_high_pulse_override=override)
