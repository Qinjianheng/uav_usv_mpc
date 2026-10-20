"""Integrated Yan paper candidate, with explicit equation14 and PX4 engineering adapters."""
import math

import numpy as np

from .p76_paper_attitude import attitude_rates
from .p76_paper_ibvs import (fov_holding_velocity, fov_vertical_excursion_bound, image_yaw_rate)
from .p76_paper_png import DualPlanePng, camera_los, paper_angles, wrapped_difference
from .p76_paper_thrust import physical_thrust


class PaperController:
    """No P7 velocity controller, endpoint preview or target-state input is used here."""

    def __init__(self, contract, mass, maximum_thrust, thrust_map, barrier):
        self.c = contract
        self.mass, self.maximum_thrust = mass, maximum_thrust
        self.thrust_map, self.barrier = thrust_map, barrier
        self.png = DualPlanePng(contract['paper']['png_ky'], contract['paper']['png_kz'])
        camera = contract['camera']
        focal = camera['width']/(2*math.tan(camera['horizontal_fov']/2))
        self.calibration = (camera['width']/2, camera['height']/2, focal, focal)
        self.pitch = math.radians(camera['pitch_down_degrees'])
        self.previous_q, self.previous_rates = None, np.zeros(3)

    def initialize(self, image_state, rotation, velocity, epoch, angular_rates):
        """Seed measured velocity/LOS and actual rates, never an uninitialized PNG dive."""
        center = np.asarray(image_state[:2])+self.calibration[:2]
        los = camera_los(center, self.calibration, rotation, self.pitch)
        self.png.initialize(los, velocity, epoch)
        self.previous_q = paper_angles(los)[0]
        self.previous_rates = np.asarray(angular_rates, dtype=float).copy()

    def step(self, image_state, rotation, position, velocity, estimate_stamp, dt,
             navigation_age=0.):
        """Use pixel PD, dual-plane PNG, equation16 holding, SO(3), force and calibrated map."""
        if not 0 < dt <= .125:
            raise ValueError('P76_CONTROL_DT')
        candidate, paper = self.c['candidate'], self.c['paper']
        gravity = self.c['dynamics']['gravity']
        image_state, velocity = np.asarray(image_state), np.asarray(velocity)
        if (image_state.shape != (4,) or not np.isfinite(image_state).all()
                or velocity.shape != (3,) or not np.isfinite(velocity).all()):
            raise ValueError('P76_CONTROL_INPUT')
        center = image_state[:2]+self.calibration[:2]
        los = camera_los(center, self.calibration, rotation, self.pitch)
        direction = self.png.update(los, velocity, estimate_stamp)
        vertical = paper_angles(los)[0]
        q_delta = 0. if self.previous_q is None else wrapped_difference(vertical, self.previous_q)
        self.previous_q = vertical
        # Equation16: actual remaining pixel margin limits dynamics-induced tilt.
        remaining = self.c['camera']['height']/2-abs(float(image_state[1]))
        angle = math.atan2(max(0., remaining), self.calibration[3])-abs(q_delta)
        if angle <= 0:
            raise ValueError('P76_FOV_HOLDING_INFEASIBLE')
        ka = min(paper['paper_ka'], gravity*math.tan(angle))
        desired = np.asarray(fov_holding_velocity(
            velocity, direction, dt, max(1e-9, ka), candidate['maximum_horizontal_speed'],
            candidate['capture_vertical_speed_limit']))
        acceleration = (desired-velocity)/dt
        if np.linalg.norm(acceleration) > paper['paper_maneuver_acceleration_target']+1e-6:
            raise ValueError('P76_DYNAMIC_LIMIT')
        safety = self.barrier.check(position, velocity, rotation, acceleration, navigation_age, dt)
        if safety == 'UNRECOVERABLE':
            raise ValueError('P76_SEA_UNRECOVERABLE')
        if safety == 'BRAKE':
            acceleration = np.array((0., 0., -self.barrier.braking))
        yaw = image_yaw_rate(image_state[0], image_state[2], paper['ibvs_kp'], paper['ibvs_kd'])
        rates = attitude_rates(rotation, acceleration, yaw, gravity)
        limits = np.array((candidate['maximum_body_roll_pitch_rate'],)*2
                          + (candidate['maximum_body_yaw_rate'],))
        rates = np.clip(rates, -limits, limits)
        slew = candidate['maximum_body_angular_acceleration']*dt
        change = rates-self.previous_rates
        rates = self.previous_rates+change*min(1., slew/max(np.linalg.norm(change), 1e-12))
        self.previous_rates = rates.copy()
        force = physical_thrust(self.mass, rotation, acceleration, gravity, self.maximum_thrust)
        thrust = self.thrust_map.to_body_z(force)
        return dict(rates=rates, thrust=thrust, force_newtons=force,
                    acceleration=acceleration, los=los, safety=safety, ka=ka,
                    fov_bound=fov_vertical_excursion_bound(q_delta, ka, gravity,
                                                           self.calibration[3]))
