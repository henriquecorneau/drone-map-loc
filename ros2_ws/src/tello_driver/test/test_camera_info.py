import os

import pytest

from tello_driver.camera_info import load_orbslam_camera, make_camera_info

# ros2_ws/src/tello_driver/test -> repository root (or /home/dev in the container)
CONFIG = os.path.join(os.path.dirname(__file__), '..', '..', '..', '..', 'config', 'tello_02.yaml')


@pytest.fixture
def camera():
    return load_orbslam_camera(CONFIG)


def test_load_tello_calibration(camera):
    assert camera['width'] == 960 and camera['height'] == 720
    assert camera['fx'] == pytest.approx(921.170702)
    assert camera['cy'] == pytest.approx(351.238301)
    assert camera['k2'] == pytest.approx(0.105152)


def test_camera_info_full_resolution(camera):
    msg = make_camera_info(camera, 960, 720, 'tello_camera_optical_frame')
    assert msg.header.frame_id == 'tello_camera_optical_frame'
    assert (msg.width, msg.height) == (960, 720)
    assert msg.distortion_model == 'plumb_bob'
    assert list(msg.k) == pytest.approx([921.170702, 0, 459.904354, 0, 919.018377, 351.238301, 0, 0, 1])
    assert msg.p[3] == 0.0 and msg.p[7] == 0.0  # monocular: no baseline
    assert list(msg.d) == pytest.approx([-0.033458, 0.105152, 0.001256, -0.006647, 0.0])


def test_camera_info_scales_intrinsics(camera):
    msg = make_camera_info(camera, 480, 360, 'cam')
    assert msg.k[0] == pytest.approx(921.170702 / 2)
    assert msg.k[2] == pytest.approx(459.904354 / 2)
    assert msg.k[5] == pytest.approx(351.238301 / 2)
    assert list(msg.d) == pytest.approx([-0.033458, 0.105152, 0.001256, -0.006647, 0.0])


def test_missing_file_raises():
    with pytest.raises(FileNotFoundError):
        load_orbslam_camera('/nao/existe.yaml')
