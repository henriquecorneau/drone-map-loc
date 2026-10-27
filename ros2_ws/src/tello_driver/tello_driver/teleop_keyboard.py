"""Keyboard teleop for the Tello driver.

Publishes /cmd_vel (FLU) while a movement key is held and calls the driver's
takeoff, land and emergency services. A terminal has no key-release events, so
holding a key relies on the keyboard autorepeat: each press keeps the motion
alive for `key_timeout` seconds. See the package README.
"""

import math
import os
import select
import sys
import termios
import threading
import time
import tty

import rclpy
from geometry_msgs.msg import Twist
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions
from std_srvs.srv import Trigger

# key: direction (vx, vy, vz, wz) in FLU; multiplied by the speeds below
MOVE_KEYS = {
    'w': (1.0, 0.0, 0.0, 0.0),    # forward
    's': (-1.0, 0.0, 0.0, 0.0),   # back
    'a': (0.0, 1.0, 0.0, 0.0),    # left
    'd': (0.0, -1.0, 0.0, 0.0),   # right
    'r': (0.0, 0.0, 1.0, 0.0),    # up
    'f': (0.0, 0.0, -1.0, 0.0),   # down
    'q': (0.0, 0.0, 0.0, 1.0),    # yaw left (counter-clockwise)
    'e': (0.0, 0.0, 0.0, -1.0),   # yaw right (clockwise)
}

HELP = """
Teleop do Tello (/cmd_vel). Segure a tecla para mover; soltou, o drone paira.

   q  w  e        w/s  frente / trás        r  subir
   a  s  d        a/d  esquerda / direita   f  descer
                  q/e  girar esq. / dir.    espaço  parar (pairar)

   t  decolar     l  pousar     + / -  velocidade +-10%
   !  duas vezes em 1 s: EMERGÊNCIA (corta os motores, o drone cai)
   Ctrl+C  sair (pousa se decolou por aqui e land_on_exit=true)
"""


class KeyTeleop:
    """Key presses to a velocity command, without ROS or a terminal (testable)."""

    SPEED_STEP = 0.1
    SCALE_MIN, SCALE_MAX = 0.1, 2.0
    EMERGENCY_WINDOW = 1.0  # s between the two '!' presses

    def __init__(self, linear_speed: float, vertical_speed: float, angular_speed: float,
                 key_timeout: float) -> None:
        self.speeds = (linear_speed, linear_speed, vertical_speed, angular_speed)  # m/s, m/s, m/s, rad/s
        self.key_timeout = key_timeout
        self.scale = 1.0
        self.direction = (0.0, 0.0, 0.0, 0.0)
        self.last_move_time = -math.inf
        self.emergency_armed_time = -math.inf

    def press(self, key: str, now: float) -> str | None:
        """Handle one key at time `now` (s, monotonic).

        Returns:
            The action for the caller ('takeoff', 'land', 'emergency',
            'emergency_armed', 'speed') or None for movement and unknown keys.
        """
        if key != '!':
            self.emergency_armed_time = -math.inf
        if key in MOVE_KEYS:
            self.direction = MOVE_KEYS[key]
            self.last_move_time = now
            return None
        if key == ' ':
            self.stop()
            return None
        if key in '+=':
            self.scale = min(self.SCALE_MAX, round(self.scale + self.SPEED_STEP, 2))
            return 'speed'
        if key in '-_':
            self.scale = max(self.SCALE_MIN, round(self.scale - self.SPEED_STEP, 2))
            return 'speed'
        if key == 't':
            self.stop()
            return 'takeoff'
        if key == 'l':
            self.stop()
            return 'land'
        if key == '!':
            self.stop()
            if now - self.emergency_armed_time <= self.EMERGENCY_WINDOW:
                self.emergency_armed_time = -math.inf
                return 'emergency'
            self.emergency_armed_time = now
            return 'emergency_armed'
        return None

    def stop(self) -> None:
        self.direction = (0.0, 0.0, 0.0, 0.0)
        self.last_move_time = -math.inf

    def command(self, now: float) -> tuple[float, float, float, float]:
        """(vx, vy, vz, wz) in m/s and rad/s, FLU; zero once the key timed out."""
        if now - self.last_move_time > self.key_timeout:
            return (0.0, 0.0, 0.0, 0.0)
        return tuple(d * s * self.scale for d, s in zip(self.direction, self.speeds))


class TeleopKeyboard(Node):
    STOP_REPEATS = 3  # zero Twists sent after a motion ends, then silence

    def __init__(self) -> None:
        super().__init__('tello_teleop_keyboard')

        self.declare_parameter('linear_speed', 0.3)     # m/s at scale 1.0
        self.declare_parameter('vertical_speed', 0.3)   # m/s at scale 1.0
        self.declare_parameter('angular_speed', 0.6)    # rad/s at scale 1.0
        self.declare_parameter('key_timeout', 0.7)      # s; above the keyboard autorepeat delay
        self.declare_parameter('rate', 10.0)            # Hz
        self.declare_parameter('land_on_exit', True)

        p = lambda name: self.get_parameter(name).value  # noqa: E731
        self.keys = KeyTeleop(p('linear_speed'), p('vertical_speed'), p('angular_speed'), p('key_timeout'))
        self.land_on_exit: bool = p('land_on_exit')
        self.took_off = False
        self.stop_repeats_left = 0
        self.lock = threading.Lock()

        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.clients_by_action = {
            'takeoff': self.create_client(Trigger, 'tello/takeoff'),
            'land': self.create_client(Trigger, 'tello/land'),
            'emergency': self.create_client(Trigger, 'tello/emergency'),
        }
        self.create_timer(1.0 / p('rate'), self.publish_cmd)

    def handle_key(self, key: str) -> None:
        with self.lock:
            action = self.keys.press(key, time.monotonic())
            scale = self.keys.scale
        if action == 'speed':
            self.get_logger().info(f'Velocidade {scale:.0%}')
        elif action == 'emergency_armed':
            self.get_logger().warn('Aperte ! de novo em 1 s para CORTAR OS MOTORES')
        elif action is not None:
            self.call(action)

    def publish_cmd(self) -> None:
        """Send the current command while moving, plus a few zeros when it stops."""
        with self.lock:
            vx, vy, vz, wz = self.keys.command(time.monotonic())
        if any((vx, vy, vz, wz)):
            self.stop_repeats_left = self.STOP_REPEATS
        elif self.stop_repeats_left > 0:
            self.stop_repeats_left -= 1
        else:
            return
        msg = Twist()
        msg.linear.x, msg.linear.y, msg.linear.z, msg.angular.z = vx, vy, vz, wz
        self.cmd_pub.publish(msg)

    def call(self, action: str):
        """Call a driver service without blocking (land blocks in the driver for seconds)."""
        client = self.clients_by_action[action]
        if not client.service_is_ready():
            self.get_logger().error(f'Serviço {client.srv_name} indisponível: o tello_driver está rodando?')
            return None
        if action == 'takeoff':
            self.took_off = True
        elif action in ('land', 'emergency'):
            self.took_off = False
        self.get_logger().info(f'{action}...')
        future = client.call_async(Trigger.Request())
        future.add_done_callback(lambda f: self.on_response(action, f))
        return future

    def on_response(self, action: str, future) -> None:
        response = future.result()
        if response is None:
            self.get_logger().error(f'{action}: sem resposta')
        elif response.success:
            self.get_logger().info(f'{action}: {response.message}')
        else:
            self.get_logger().error(f'{action} recusado: {response.message}')
            if action == 'takeoff':
                self.took_off = False

    def exit(self) -> None:
        """Hover, and land if this teleop took off and land_on_exit is set."""
        with self.lock:
            self.keys.stop()
        self.cmd_pub.publish(Twist())
        if self.land_on_exit and self.took_off:
            future = self.call('land')
            deadline = time.monotonic() + 1.0                 # only until the request is out
            while future is not None and not future.done() and time.monotonic() < deadline:
                time.sleep(0.05)
        elif self.took_off:
            self.get_logger().warn('Saindo com o drone no ar: ele fica pairando')


def read_keys(fd: int, timeout: float) -> str:
    """Characters typed within `timeout` s; escape sequences (arrows, F-keys) are dropped."""
    ready, _, _ = select.select([fd], [], [], timeout)
    if not ready:
        return ''
    data = os.read(fd, 32).decode(errors='ignore')
    return '' if data.startswith('\x1b') else data


def main(args=None) -> None:
    if not sys.stdin.isatty():
        print('teleop_keyboard precisa de um terminal interativo (use ros2 run, não ros2 launch)',
              file=sys.stderr)
        sys.exit(1)
    # Ctrl+C raises KeyboardInterrupt here instead of shutting rclpy down, so exit() can still publish
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    node = TeleopKeyboard()
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()

    fd = sys.stdin.fileno()
    settings = termios.tcgetattr(fd)
    print(HELP)
    try:
        tty.setcbreak(fd)                                     # one key at a time, no echo
        while rclpy.ok():
            for key in read_keys(fd, 0.1):
                node.handle_key(key)
    except KeyboardInterrupt:
        pass
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, settings)
        node.exit()
        executor.shutdown()
        spin_thread.join(timeout=1.0)                         # rclpy aborts if it is still spinning
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
