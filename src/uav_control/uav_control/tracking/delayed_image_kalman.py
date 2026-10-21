"""Four-state delayed image filter with explicit calibrated rotation compensation."""
import numpy as np

from .delayed_spatial_kalman import DelayedSpatialKalman, DelayedEstimate


def _jacobian(function, vector):
    epsilon = 1e-4
    return np.column_stack([(function(vector+epsilon*axis)-function(vector-epsilon*axis))
                            / (2*epsilon) for axis in np.eye(len(vector))])


class DelayedImageKalman:
    """Filter in an initial camera tangent chart; publish actual current-pose pixel errors."""

    def __init__(self, calibration, acceleration_std=30., initial_velocity_std=100.,
                 history_seconds=1., maximum_source_age=.125):
        cx, cy, fx, fy = calibration
        self.center = np.array((cx, cy))
        self.intrinsic = np.array(((fx, 0., cx), (0., fy, cy), (0., 0., 1.)))
        self.inverse = np.linalg.inv(self.intrinsic)
        self.filter = DelayedSpatialKalman(acceleration_std, initial_velocity_std,
                                           history_seconds, maximum_source_age, dimensions=2)
        self.anchor_rotation = None

    def reset(self):
        """Discard the chart and posterior after a clock/frame discontinuity."""
        self.filter.reset()
        self.anchor_rotation = None

    @staticmethod
    def _rotation(rotation):
        rotation = np.asarray(rotation, dtype=float)
        if (rotation.shape != (3, 3) or not np.isfinite(rotation).all()
                or not np.allclose(rotation.T@rotation, np.eye(3), atol=1e-6)
                or np.linalg.det(rotation) < 0):
            raise ValueError('IMAGE_DKF_ROTATION')
        return rotation

    def _project(self, point, homography):
        ray = homography@np.r_[point, 1.]
        if ray[2] <= 1e-3 or not np.isfinite(ray).all():
            raise ValueError('IMAGE_DKF_TANGENT_CHART')
        return ray[:2]/ray[2]

    def update(self, center_uv, covariance, stamp, now, camera_to_world):
        """Use the full acquisition-pose rotation and propagate pixel measurement covariance."""
        rotation = self._rotation(camera_to_world)
        was_uninitialized = self.anchor_rotation is None
        if self.anchor_rotation is None:
            self.anchor_rotation = rotation.copy()
        homography = self.intrinsic@self.anchor_rotation.T@rotation@self.inverse
        center = np.asarray(center_uv, dtype=float)
        def function(point): return self._project(point, homography)
        projected = function(center)-self.center
        jacobian = _jacobian(function, center)
        noise = jacobian@np.asarray(covariance)@jacobian.T
        accepted = self.filter.update(projected, noise, stamp, now)
        if was_uninitialized and not accepted:
            self.anchor_rotation = None
        return accepted

    def predict(self, now, camera_to_world, angular_velocity_camera=(0., 0., 0.),
                orientation_covariance=None):
        """Project at a real delivered pose epoch; never extrapolate vehicle navigation."""
        estimate = self.filter.predict(now)
        if self.anchor_rotation is None:
            return estimate
        rotation = self._rotation(camera_to_world)
        homography = self.intrinsic@rotation.T@self.anchor_rotation@self.inverse
        omega = np.asarray(angular_velocity_camera)

        def project_state(state):
            point = state[:2]+self.center
            def transform(value): return self._project(value, homography)
            image = transform(point)
            jacobian = _jacobian(transform, point)
            x, y = (image-self.center)/np.diag(self.intrinsic)[:2]
            rotational = np.array(((x*y, -(1+x*x), y),
                                   (1+y*y, -x*y, -x)))@omega
            derivative = jacobian@state[2:]+np.diag(self.intrinsic)[:2]*rotational
            return np.r_[image-self.center, derivative]
        state = project_state(estimate.state)
        jacobian = _jacobian(project_state, estimate.state)
        covariance = jacobian@estimate.covariance@jacobian.T
        if orientation_covariance is not None:
            noise = np.asarray(orientation_covariance, dtype=float)
            if (noise.shape != (3, 3) or not np.isfinite(noise).all()
                    or np.linalg.eigvalsh(noise).min() < 0):
                raise ValueError('IMAGE_DKF_POSE_COVARIANCE')
            x, y = state[:2]/np.diag(self.intrinsic)[:2]
            angular_jacobian = np.diag(np.diag(self.intrinsic)[:2])@np.array(
                ((x*y, -(1+x*x), y), (1+y*y, -x*y, -x)))
            covariance[:2, :2] += angular_jacobian@noise@angular_jacobian.T
        return DelayedEstimate(estimate.stamp, estimate.source_stamp, state, covariance,
                               estimate.valid, estimate.reason)
