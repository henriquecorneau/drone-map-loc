"""Unit and frame conversions between the Tello SDK and ROS conventions.

The Tello reports attitude in degrees, distances in cm and acceleration in
thousandths of g. The SDK does not document its body axes; the values here
assume FRD (x forward, y right, z down), which matches the sign of the
accelerometer at rest (agz ~ -1000) and of the yaw (grows with `cw`). ROS uses
FLU for the body (REP-103), so x is kept and y, z are flipped:
R_flu = S @ R_frd @ S with S = diag(1, -1, -1).

Confirm each sign with the drone in hand before flying (tilt it, rotate it).
"""

import numpy as np
from scipy.spatial.transform import Rotation

G = 9.80665  # m/s^2

# FRD <-> FLU axis flip; its own inverse.
S_FLU_FRD = np.diag([1.0, -1.0, -1.0])


def tello_attitude_to_quaternion(roll_deg: float, pitch_deg: float, yaw_deg: float) -> np.ndarray:
    """Tello roll/pitch/yaw (degrees, FRD, ZYX order) to a FLU quaternion.

    The yaw is relative to the heading at power-on, not to magnetic north.

    Returns:
        np.ndarray: quaternion in **xyzw** order, shape (4,), float64.
    """
    R_frd = Rotation.from_euler('ZYX', [yaw_deg, pitch_deg, roll_deg], degrees=True).as_matrix()
    R_flu = S_FLU_FRD @ R_frd @ S_FLU_FRD
    return Rotation.from_matrix(R_flu).as_quat()  # xyzw


def tello_accel_to_mps2(agx: float, agy: float, agz: float) -> np.ndarray:
    """Tello acceleration (0.001 g, FRD) to specific force in m/s^2, FLU.

    At rest the result is about (0, 0, +9.81): the accelerometer measures the
    reaction to gravity, which points up.

    Returns:
        np.ndarray: shape (3,), float64.
    """
    return S_FLU_FRD @ np.array([agx, agy, agz], dtype=np.float64) * (G / 1000.0)


def tello_velocity_to_mps(vgx: float, vgy: float, vgz: float, scale: float) -> np.ndarray:
    """Tello velocity to m/s in FLU.

    The unit is not documented by the SDK (usually dm/s, so `scale=0.1`) and
    neither is the frame; this assumes the same FRD body axes as the attitude.

    Returns:
        np.ndarray: shape (3,), float64.
    """
    return S_FLU_FRD @ np.array([vgx, vgy, vgz], dtype=np.float64) * scale


def twist_to_rc(
    vx: float, vy: float, vz: float, wz: float,
    max_linear_speed: float, max_vertical_speed: float, max_angular_speed: float,
    rc_limit: int,
) -> tuple[int, int, int, int]:
    """FLU velocity command to Tello `rc` values.

    Args:
        vx, vy, vz: linear velocity in m/s (x forward, y left, z up).
        wz: yaw rate in rad/s (positive counter-clockwise seen from above).
        max_linear_speed, max_vertical_speed: speed in m/s that maps to rc 100.
        max_angular_speed: yaw rate in rad/s that maps to rc 100.
        rc_limit: saturation for every channel, 0 to 100.

    Returns:
        tuple: (left_right, forward_backward, up_down, yaw) as ints. The Tello
            uses positive = right and positive yaw = clockwise, so y and the
            yaw are flipped.
    """
    limit = max(0, min(100, int(rc_limit)))

    def scale(value: float, full_scale: float) -> int:
        rc = int(round(100.0 * value / full_scale))
        return max(-limit, min(limit, rc))

    return (
        scale(-vy, max_linear_speed),
        scale(vx, max_linear_speed),
        scale(vz, max_vertical_speed),
        scale(-wz, max_angular_speed),
    )
