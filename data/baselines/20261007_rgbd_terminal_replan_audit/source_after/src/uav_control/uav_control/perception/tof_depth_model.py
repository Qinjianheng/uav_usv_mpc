"""
Uncalibrated functional ToF range model over ideal aligned depth images.

This models radial range limits, random error, quantization and missing returns.
It does not simulate light transport, multipath, sunlight or water reflectance.
"""

import math

import numpy as np


class TofDepthModel:
    """Return optical-axis depth for the existing RGB-D localization interface."""

    def __init__(self, *, horizontal_fov=1.74, minimum_range=.25,
                 maximum_range=25., noise_std=.01, range_noise_scale=.001,
                 dropout_probability=.01, quantization=.001, seed=0):
        values = (horizontal_fov, minimum_range, maximum_range, noise_std,
                  range_noise_scale, dropout_probability, quantization)
        if not all(math.isfinite(value) for value in values):
            raise ValueError('ToF model parameters must be finite')
        if not 0 < horizontal_fov < math.pi:
            raise ValueError('horizontal_fov must be in (0, pi)')
        if not 0 < minimum_range < maximum_range:
            raise ValueError('range limits must be positive and ordered')
        if noise_std < 0 or range_noise_scale < 0 or quantization < 0:
            raise ValueError('noise and quantization must be nonnegative')
        if not 0 <= dropout_probability <= 1:
            raise ValueError('dropout_probability must be in [0, 1]')
        self.horizontal_fov = horizontal_fov
        self.minimum_range, self.maximum_range = minimum_range, maximum_range
        self.noise_std, self.range_noise_scale = noise_std, range_noise_scale
        self.dropout_probability, self.quantization = dropout_probability, quantization
        self.rng = np.random.default_rng(seed)
        self._shape, self._ray_scale = None, None

    def measure(self, depth):
        """Gate and perturb slant range, then convert back to aligned depth."""
        depth = np.asarray(depth, dtype=np.float32)
        if depth.ndim != 2 or min(depth.shape) < 1:
            raise ValueError('depth must be a nonempty two-dimensional image')
        if depth.shape != self._shape:
            height, width = depth.shape
            focal = width / (2. * math.tan(self.horizontal_fov / 2.))
            rows, columns = np.indices(depth.shape, dtype=np.float32)
            self._ray_scale = np.sqrt(
                1. + ((columns - width / 2.) / focal) ** 2
                + ((rows - height / 2.) / focal) ** 2,
            )
            self._shape = depth.shape
        ranges = depth * self._ray_scale
        valid = np.isfinite(ranges) & (ranges >= self.minimum_range)
        valid &= ranges <= self.maximum_range
        measured = ranges.copy()
        sigma = self.noise_std + self.range_noise_scale * ranges[valid]
        measured[valid] += self.rng.normal(0., sigma)
        if self.quantization:
            measured[valid] = (
                np.rint(measured[valid] / self.quantization) * self.quantization
            )
        valid &= (measured >= self.minimum_range) & (measured <= self.maximum_range)
        if self.dropout_probability:
            valid &= self.rng.random(depth.shape) >= self.dropout_probability
        output = np.full(depth.shape, np.nan, dtype=np.float32)
        output[valid] = measured[valid] / self._ray_scale[valid]
        return output


def decode_depth_image(message):
    """Read metre-valued 32FC1 data with row stride and byte order preserved."""
    if message.encoding != '32FC1':
        raise ValueError('ToF input must be 32FC1 in metres')
    if message.width < 1 or message.height < 1:
        raise ValueError('depth image must be nonempty')
    if message.step < message.width * 4:
        raise ValueError('depth row stride is shorter than the image width')
    if len(message.data) < message.height * message.step:
        raise ValueError('depth buffer is truncated')
    dtype = '>f4' if message.is_bigendian else '<f4'
    return np.ndarray((message.height, message.width), dtype=dtype,
                      buffer=bytes(message.data), strides=(message.step, 4))
