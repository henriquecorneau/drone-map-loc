"""CameraInfo from the ORB-SLAM3 settings file of the Tello (config/tello_02.yaml)."""

import cv2
from sensor_msgs.msg import CameraInfo


def load_orbslam_camera(path: str) -> dict:
    """Read the pinhole intrinsics from an ORB-SLAM3 settings file.

    The file uses OpenCV's `%YAML:1.0` dialect, which PyYAML cannot parse.

    Returns:
        dict: fx, fy, cx, cy (px), k1, k2, p1, p2, k3, width, height (px).
    """
    fs = cv2.FileStorage(path, cv2.FILE_STORAGE_READ)
    if not fs.isOpened():
        raise FileNotFoundError(f'Não foi possível abrir {path}')
    try:
        values = {key: fs.getNode(f'Camera1.{key}').real()
                  for key in ('fx', 'fy', 'cx', 'cy', 'k1', 'k2', 'p1', 'p2', 'k3')}
        values['width'] = int(fs.getNode('Camera.width').real())
        values['height'] = int(fs.getNode('Camera.height').real())
    finally:
        fs.release()
    if values['fx'] <= 0 or values['width'] <= 0:
        raise ValueError(f'{path} não tem Camera1.fx/Camera.width válidos')
    return values


def make_camera_info(camera: dict, width: int, height: int, frame_id: str) -> CameraInfo:
    """Build a CameraInfo for images of size width x height (px).

    The intrinsics are scaled if the image size differs from the calibration
    (distortion coefficients do not depend on the image size).
    """
    sx = width / camera['width']
    sy = height / camera['height']
    fx, fy = camera['fx'] * sx, camera['fy'] * sy
    cx, cy = camera['cx'] * sx, camera['cy'] * sy

    msg = CameraInfo()
    msg.header.frame_id = frame_id
    msg.width = width
    msg.height = height
    msg.distortion_model = 'plumb_bob'
    msg.d = [camera['k1'], camera['k2'], camera['p1'], camera['p2'], camera['k3']]
    msg.k = [fx, 0.0, cx, 0.0, fy, cy, 0.0, 0.0, 1.0]
    msg.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
    msg.p = [fx, 0.0, cx, 0.0, 0.0, fy, cy, 0.0, 0.0, 0.0, 1.0, 0.0]
    return msg
