"""Keyboard teleop tests: key logic without a terminal, node without a driver."""

import os
import time

import pytest
import rclpy
from geometry_msgs.msg import Twist
from std_srvs.srv import Trigger

from tello_driver.teleop_keyboard import KeyTeleop, TeleopKeyboard, read_keys


@pytest.fixture
def keys():
    return KeyTeleop(linear_speed=0.3, vertical_speed=0.2, angular_speed=0.6, key_timeout=0.7)


@pytest.mark.parametrize('key, expected', [
    ('w', (0.3, 0.0, 0.0, 0.0)),
    ('s', (-0.3, 0.0, 0.0, 0.0)),
    ('a', (0.0, 0.3, 0.0, 0.0)),     # FLU: left is +y
    ('d', (0.0, -0.3, 0.0, 0.0)),
    ('r', (0.0, 0.0, 0.2, 0.0)),
    ('f', (0.0, 0.0, -0.2, 0.0)),
    ('q', (0.0, 0.0, 0.0, 0.6)),     # counter-clockwise is +wz
    ('e', (0.0, 0.0, 0.0, -0.6)),
])
def test_move_keys_in_flu(keys, key, expected):
    assert keys.press(key, now=10.0) is None
    assert keys.command(now=10.1) == pytest.approx(expected)


def test_motion_expires_after_key_timeout(keys):
    keys.press('w', now=10.0)
    assert keys.command(now=10.69)[0] == pytest.approx(0.3)
    assert keys.command(now=10.71) == (0.0, 0.0, 0.0, 0.0)
    keys.press('w', now=10.6)                          # autorepeat keeps it alive
    assert keys.command(now=11.2)[0] == pytest.approx(0.3)


def test_last_key_wins_and_space_stops(keys):
    keys.press('w', now=10.0)
    keys.press('q', now=10.1)
    assert keys.command(now=10.2) == pytest.approx((0.0, 0.0, 0.0, 0.6))
    keys.press(' ', now=10.3)
    assert keys.command(now=10.3) == (0.0, 0.0, 0.0, 0.0)


def test_speed_scale_is_clamped(keys):
    assert keys.press('+', now=0.0) == 'speed'
    keys.press('w', now=0.0)
    assert keys.command(now=0.0)[0] == pytest.approx(0.33)
    for _ in range(30):
        keys.press('-', now=0.0)
    assert keys.scale == pytest.approx(KeyTeleop.SCALE_MIN)
    for _ in range(30):
        keys.press('+', now=0.0)
    assert keys.scale == pytest.approx(KeyTeleop.SCALE_MAX)


def test_takeoff_and_land_stop_motion(keys):
    keys.press('w', now=10.0)
    assert keys.press('t', now=10.1) == 'takeoff'
    assert keys.command(now=10.1) == (0.0, 0.0, 0.0, 0.0)
    keys.press('w', now=10.2)
    assert keys.press('l', now=10.3) == 'land'
    assert keys.command(now=10.3) == (0.0, 0.0, 0.0, 0.0)


def test_emergency_needs_two_presses_within_window(keys):
    assert keys.press('!', now=10.0) == 'emergency_armed'
    assert keys.press('!', now=11.5) == 'emergency_armed'   # too late: re-armed
    assert keys.press('!', now=12.0) == 'emergency'
    assert keys.press('!', now=12.1) == 'emergency_armed'   # disarmed after firing


def test_other_key_disarms_emergency(keys):
    keys.press('!', now=10.0)
    keys.press('w', now=10.1)
    assert keys.press('!', now=10.2) == 'emergency_armed'


def test_unknown_and_uppercase_keys_ignored(keys):
    for key in 'WxZ1\n':
        assert keys.press(key, now=10.0) is None
    assert keys.command(now=10.0) == (0.0, 0.0, 0.0, 0.0)


def test_read_keys_drops_escape_sequences():
    r, w = os.pipe()
    try:
        os.write(w, b'\x1b[A')                             # up arrow
        assert read_keys(r, 0.1) == ''
        os.write(w, b'wq')
        assert read_keys(r, 0.1) == 'wq'
        assert read_keys(r, 0.01) == ''
    finally:
        os.close(r)
        os.close(w)


@pytest.fixture
def node():
    rclpy.init()
    n = TeleopKeyboard()
    yield n
    n.destroy_node()
    rclpy.shutdown()


def spin_collect(node, received, count, timeout=2.0):
    deadline = time.time() + timeout
    while len(received) < count and time.time() < deadline:
        rclpy.spin_once(node, timeout_sec=0.05)


def test_publishes_while_moving_then_a_few_zeros(node):
    received: list[Twist] = []
    node.create_subscription(Twist, 'cmd_vel', received.append, 10)
    node.publish_cmd()                                 # idle: silent
    node.handle_key('w')
    node.publish_cmd()
    node.keys.stop()
    for _ in range(TeleopKeyboard.STOP_REPEATS + 2):
        node.publish_cmd()
    spin_collect(node, received, 1 + TeleopKeyboard.STOP_REPEATS)
    spin_collect(node, received, 99, timeout=0.3)      # nothing else arrives
    assert len(received) == 1 + TeleopKeyboard.STOP_REPEATS
    assert received[0].linear.x == pytest.approx(0.3)
    assert all(m.linear.x == 0.0 for m in received[1:])


def test_service_without_driver_is_reported_not_raised(node):
    assert node.call('takeoff') is None
    assert not node.took_off


def test_takeoff_then_exit_lands(node):
    calls = []

    def fake_service(name):
        def handler(request, response):
            calls.append(name)
            response.success, response.message = True, 'ok'
            return response
        return handler

    node.create_service(Trigger, 'tello/takeoff', fake_service('takeoff'))
    node.create_service(Trigger, 'tello/land', fake_service('land'))
    deadline = time.time() + 2.0
    while not node.clients_by_action['takeoff'].service_is_ready() and time.time() < deadline:
        rclpy.spin_once(node, timeout_sec=0.05)
    node.handle_key('t')
    while 'takeoff' not in calls and time.time() < deadline:
        rclpy.spin_once(node, timeout_sec=0.05)
    assert node.took_off
    future = node.call('land')                         # what exit() sends when land_on_exit
    while not future.done() and time.time() < deadline + 2.0:
        rclpy.spin_once(node, timeout_sec=0.05)
    assert calls == ['takeoff', 'land']
    assert not node.took_off
