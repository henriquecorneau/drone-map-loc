import numpy as np
import pytest
from scipy.spatial.transform import Rotation

from tello_driver.conversions import (
    tello_accel_to_mps2, tello_attitude_to_quaternion, tello_velocity_to_mps, twist_to_rc)


def rotation(q_xyzw: np.ndarray) -> Rotation:
    return Rotation.from_quat(q_xyzw)


@pytest.mark.parametrize('roll,pitch,yaw', [(0, 0, 0), (10, -20, 30), (-45, 15, 170), (5, 5, -90)])
def test_attitude_is_valid_rotation(roll, pitch, yaw):
    q = tello_attitude_to_quaternion(roll, pitch, yaw)
    assert q.shape == (4,)
    assert np.isclose(np.linalg.norm(q), 1.0)
    R = rotation(q).as_matrix()
    assert np.allclose(R @ R.T, np.eye(3))
    assert np.isclose(np.linalg.det(R), 1.0)


@pytest.mark.parametrize('roll,pitch,yaw', [(10, -20, 30), (-45, 15, 170)])
def test_attitude_frd_to_flu_flips_pitch_and_yaw(roll, pitch, yaw):
    expected = Rotation.from_euler('ZYX', [-yaw, -pitch, roll], degrees=True)
    q = tello_attitude_to_quaternion(roll, pitch, yaw)
    assert (rotation(q) * expected.inv()).magnitude() < 1e-9


def test_yaw_clockwise_points_nose_right():
    # Tello yaw grows clockwise (cw); in FLU, right is -y
    nose = rotation(tello_attitude_to_quaternion(0, 0, 90)).apply([1, 0, 0])
    assert np.allclose(nose, [0, -1, 0], atol=1e-9)


def test_pitch_up_points_nose_up():
    nose = rotation(tello_attitude_to_quaternion(0, 30, 0)).apply([1, 0, 0])
    assert nose[2] > 0.49


def test_accel_at_rest_points_up():
    assert np.allclose(tello_accel_to_mps2(0, 0, -1000), [0, 0, 9.80665])


def test_velocity_frd_to_flu():
    assert np.allclose(tello_velocity_to_mps(10, 5, -2, scale=0.1), [1.0, -0.5, 0.2])


def test_twist_to_rc_axes_and_signs():
    # forward 0.2 m/s, left 0.1 m/s, up 0.3 m/s, counter-clockwise 0.875 rad/s
    lr, fb, ud, yaw = twist_to_rc(0.2, 0.1, 0.3, 0.875, 1.0, 1.0, 1.75, 100)
    assert (lr, fb, ud, yaw) == (-10, 20, 30, -50)


def test_twist_to_rc_saturates():
    assert twist_to_rc(5.0, -5.0, 5.0, -10.0, 1.0, 1.0, 1.75, 40) == (40, 40, 40, 40)
    assert twist_to_rc(1.0, 0.0, 0.0, 0.0, 1.0, 1.0, 1.75, 250) == (0, 100, 0, 0)
