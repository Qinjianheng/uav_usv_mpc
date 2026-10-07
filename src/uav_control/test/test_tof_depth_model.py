"""Functional ToF models preserve acquisition epochs and measure radial range."""

import importlib
import importlib.util
from types import SimpleNamespace

import numpy as np
import pytest


def model_module():
    name = 'uav_control.perception.tof_depth_model'
    assert importlib.util.find_spec(name) is not None, 'ToF model is missing'
    return importlib.import_module(name)


def test_range_gate_uses_radial_distance_not_axial_depth():
    cls = model_module().TofDepthModel
    model = cls(horizontal_fov=np.pi / 2, minimum_range=.25,
                maximum_range=5., noise_std=0., range_noise_scale=0.,
                dropout_probability=0., quantization=0.)
    depth = np.full((2, 4), 4.5, dtype=np.float32)
    result = model.measure(depth)
    assert np.isnan(result[1, 0]) and np.isnan(result[1, 3])
    assert result[1, 2] == pytest.approx(4.5)
    assert np.array_equal(depth, np.full((2, 4), 4.5, dtype=np.float32))


def test_invalid_ranges_stay_invalid_and_no_noise_is_identity():
    cls = model_module().TofDepthModel
    model = cls(noise_std=0., range_noise_scale=0., quantization=0.,
                dropout_probability=0., maximum_range=25.)
    depth = np.array([[np.nan, np.inf, 0., -.5, .05, 1., 30.]], np.float32)
    output = model.measure(depth)
    assert np.isnan(output[0, :5]).all()
    assert output[0, 5] == pytest.approx(1., abs=1e-6)
    assert np.isnan(output[0, 6])


def test_noise_reproducible_and_dropout_can_reject_entire_frame():
    cls = model_module().TofDepthModel
    options = dict(seed=47, noise_std=.02, range_noise_scale=.001,
                   dropout_probability=0., quantization=0.)
    depth = np.full((20, 24), 2., dtype=np.float32)
    a, b = cls(**options).measure(depth), cls(**options).measure(depth)
    assert np.array_equal(a, b)
    assert np.std(a) > .005
    assert cls(dropout_probability=1.).measure(depth).dtype == np.float32
    assert np.isnan(cls(dropout_probability=1.).measure(depth)).all()


@pytest.mark.parametrize('options', [
    {'minimum_range': -1.}, {'maximum_range': .1},
    {'horizontal_fov': np.pi}, {'noise_std': -1.},
    {'dropout_probability': 1.1}, {'quantization': -1.},
    {'range_noise_scale': np.nan},
])
def test_invalid_model_parameters_fail_explicitly(options):
    with pytest.raises(ValueError):
        model_module().TofDepthModel(**options)


def test_depth_decode_preserves_row_padding_and_byte_order():
    decode = model_module().decode_depth_image
    values = np.array([[1., 2., 99.], [3., 4., 99.]], dtype='>f4')
    message = SimpleNamespace(encoding='32FC1', height=2, width=2,
                              step=12, is_bigendian=1,
                              data=values.tobytes())
    assert np.array_equal(decode(message), [[1., 2.], [3., 4.]])
    message.encoding = '16UC1'
    with pytest.raises(ValueError, match='32FC1'):
        decode(message)
    message.encoding = '32FC1'
    message.data = b'bad'
    with pytest.raises(ValueError):
        decode(message)


def test_depth_node_preserves_original_image_stamp_and_frame():
    name = 'uav_control.perception.tof_depth_node'
    assert importlib.util.find_spec(name) is not None, 'ToF ROS adapter is missing'
    module = importlib.import_module(name)
    from sensor_msgs.msg import Image

    node = object.__new__(module.TofDepthNode)
    node.model = model_module().TofDepthModel(
        noise_std=0., range_noise_scale=0., dropout_probability=0.,
        quantization=0.,
    )
    published = []
    node.depth_pub = SimpleNamespace(publish=published.append)
    image = Image(height=1, width=1, encoding='32FC1', step=4)
    image.header.stamp.sec, image.header.stamp.nanosec = 10, 123456789
    image.header.frame_id = 'front_camera_optical_frame'
    image.data = np.array([2.], dtype='<f4').tobytes()
    node.on_depth(image)
    assert len(published) == 1
    assert published[0].header == image.header
    assert published[0].step == 4
    assert published[0].encoding == '32FC1'
    assert model_module().decode_depth_image(published[0])[0, 0] == 2.


def test_monitor_accepts_modeled_ros_depth_without_ideal_fallback():
    import threading
    from types import SimpleNamespace
    from sensor_msgs.msg import Image
    from uav_control.common.runtime_performance import RateMeter
    from uav_control.perception.front_tof_monitor import FrontTofMonitor

    monitor = SimpleNamespace(
        shutting_down=False, depth_rate_meter=RateMeter(window_seconds=1.),
        lock=threading.Lock(), last_depth_analysis_time=-float('inf'),
        analysis_period=0., last_depth_time=-float('inf'), depth=None,
        last_monitor_compute_time=0.,
    )
    data = np.array([[1., np.nan]], dtype='<f4')
    image = Image(width=2, height=1, encoding='32FC1', step=8,
                  data=data.tobytes())
    FrontTofMonitor.ros_depth_callback(monitor, image)
    np.testing.assert_equal(monitor.depth, data)
    assert monitor.last_depth_time > 0
    before = monitor.depth
    image.encoding = '16UC1'
    FrontTofMonitor.ros_depth_callback(monitor, image)
    assert monitor.depth is before
