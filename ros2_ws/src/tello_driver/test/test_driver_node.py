"""Driver node tests with a fake TelloZune (no drone, no network)."""

import os
import time
from unittest.mock import MagicMock, patch

import numpy as np
import pytest
import rclpy
from geometry_msgs.msg import Twist
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, Imu, Range
from std_srvs.srv import Trigger

CONFIG = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..', '..', '..', 'config', 'tello_02.yaml'))


@pytest.fixture
def node():
    rclpy.init(args=['--ros-args', '-p', f'camera_info_file:={CONFIG}'])
    fake = MagicMock()
    fake.start_tello.return_value = True
    fake.get_frame_stamped.side_effect = lambda timeout: time.sleep(0.01)  # no video: None
    fake.state_stamp = None
    fake.get_state.return_value = {'bat': 80.0, 'h': 0.0}
    with patch('tello_driver.driver_node.TelloZune', return_value=fake):
        from tello_driver.driver_node import TelloDriver
        n = TelloDriver()
    yield n
    n.running = False
    n.video_thread.join(timeout=1.0)
    n.destroy_node()
    rclpy.shutdown()


def twist(x=0.0, y=0.0, z=0.0, wz=0.0) -> Twist:
    msg = Twist()
    msg.linear.x, msg.linear.y, msg.linear.z, msg.angular.z = x, y, z, wz
    return msg


def test_connects_and_arms_library_watchdog(node):
    node.tello.start_tello.assert_called_once()
    node.tello.set_rc_watchdog.assert_called_once_with(0.5)


def test_cmd_vel_ignored_before_takeoff(node):
    node.on_cmd_vel(twist(x=0.2))
    node.tello.send_rc_control.assert_not_called()


def test_cmd_vel_after_takeoff_sends_rc(node):
    node.on_takeoff(Trigger.Request(), Trigger.Response())
    node.tello.takeoff.assert_called_once()
    node.on_cmd_vel(twist(x=0.2, y=0.1, wz=0.875))
    node.tello.send_rc_control.assert_called_with(-10, 20, 0, -40)  # yaw saturated at rc_limit 40


def test_takeoff_refused_with_low_battery(node):
    node.tello.get_state.return_value = {'bat': 15.0}
    response = node.on_takeoff(Trigger.Request(), Trigger.Response())
    assert not response.success
    node.tello.takeoff.assert_not_called()


def test_cmd_vel_timeout_stops_drone(node):
    node.on_takeoff(Trigger.Request(), Trigger.Response())
    node.on_cmd_vel(twist(x=0.2))
    node.check_cmd_vel_timeout()                       # recent: nothing
    assert node.tello.send_rc_control.call_args.args == (0, 20, 0, 0)
    node.last_cmd_time -= 1.0
    node.check_cmd_vel_timeout()
    assert node.tello.send_rc_control.call_args.args == (0, 0, 0, 0)
    node.tello.send_rc_control.reset_mock()
    node.last_cmd_time -= 1.0
    node.check_cmd_vel_timeout()                       # already stopped: no resend
    node.tello.send_rc_control.assert_not_called()


def test_land_disarms_and_stops(node):
    node.on_takeoff(Trigger.Request(), Trigger.Response())
    node.on_land(Trigger.Request(), Trigger.Response())
    node.tello.land.assert_called_once()
    node.tello.send_rc_control.assert_called_with(0, 0, 0, 0)
    node.tello.send_rc_control.reset_mock()
    node.on_cmd_vel(twist(x=0.2))
    node.tello.send_rc_control.assert_not_called()


def test_emergency_cuts_motors_and_disarms(node):
    node.on_takeoff(Trigger.Request(), Trigger.Response())
    node.on_emergency(Trigger.Request(), Trigger.Response())
    node.tello.emergency_stop.assert_called_once()
    assert not node.flying


def test_publishes_stamped_frames(node):
    received = []
    node.create_subscription(Image, 'tello/camera', received.append, qos_profile_sensor_data)
    stamp = 1_700_000_000.25
    node.tello.get_frame_stamped.side_effect = lambda timeout: (np.zeros((720, 960, 3), np.uint8), stamp)
    deadline = time.time() + 3.0
    while not received and time.time() < deadline:
        rclpy.spin_once(node, timeout_sec=0.1)
    assert received, 'nenhuma imagem publicada'
    msg = received[0]
    assert (msg.width, msg.height, msg.encoding) == (960, 720, 'bgr8')
    assert msg.header.frame_id == 'tello_camera_optical_frame'
    assert msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9 == pytest.approx(stamp)


def test_publishes_state_once_per_packet(node):
    imus, tofs = [], []
    node.create_subscription(Imu, 'tello/imu', imus.append, qos_profile_sensor_data)
    node.create_subscription(Range, 'tello/tof', tofs.append, qos_profile_sensor_data)
    node.tello.state_stamp = 1_700_000_000.0
    node.tello.get_state.return_value = {'roll': 0.0, 'pitch': 0.0, 'yaw': 0.0,
                                         'agx': 0.0, 'agy': 0.0, 'agz': -1000.0, 'tof': 85.0, 'bat': 80.0}
    node.publish_state()
    node.publish_state()                               # same packet: not republished
    deadline = time.time() + 2.0
    while (not imus or not tofs) and time.time() < deadline:
        rclpy.spin_once(node, timeout_sec=0.1)
    assert len(imus) == 1 and len(tofs) == 1
    assert imus[0].linear_acceleration.z == pytest.approx(9.80665)
    assert imus[0].orientation.w == pytest.approx(1.0)
    assert tofs[0].range == pytest.approx(0.85)
