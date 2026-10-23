"""ROS 2 driver for the DJI Tello built on tello-zune.

Publishes the camera with capture stamps, the camera info and the telemetry,
and turns /cmd_vel into `rc` commands with a watchdog. See the package README.
"""

import math
import sys
import threading
import time

import rclpy
from cv_bridge import CvBridge
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from geometry_msgs.msg import Twist, TwistStamped
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import BatteryState, CameraInfo, Image, Imu, Range
from std_srvs.srv import Trigger
from tello_zune import TelloZune

from tello_driver.camera_info import load_orbslam_camera, make_camera_info
from tello_driver.conversions import (
    tello_accel_to_mps2, tello_attitude_to_quaternion, tello_velocity_to_mps, twist_to_rc)


def stamp_from_wall_time(seconds: float):
    """time.time() (s) to builtin_interfaces/Time. ROS uses the same clock without use_sim_time."""
    return Time(nanoseconds=int(seconds * 1e9)).to_msg()


class TelloDriver(Node):
    def __init__(self) -> None:
        super().__init__('tello_driver')

        self.declare_parameter('tello_ip', '192.168.10.1')
        self.declare_parameter('camera_info_file', '/home/dev/config/tello_02.yaml')
        self.declare_parameter('camera_frame', 'tello_camera_optical_frame')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('image_width', 960)
        self.declare_parameter('image_height', 720)
        self.declare_parameter('state_rate', 20.0)              # Hz
        self.declare_parameter('cmd_vel_timeout', 0.5)          # s
        self.declare_parameter('max_linear_speed', 1.0)         # m/s at rc 100 (to calibrate)
        self.declare_parameter('max_vertical_speed', 1.0)       # m/s at rc 100 (to calibrate)
        self.declare_parameter('max_angular_speed', 1.75)       # rad/s at rc 100 (to calibrate)
        self.declare_parameter('rc_limit', 40)                  # 0-100, saturation of every channel
        self.declare_parameter('velocity_scale', 0.1)           # Tello velocity unit -> m/s (dm/s?)
        self.declare_parameter('land_on_shutdown', True)

        p = lambda name: self.get_parameter(name).value  # noqa: E731
        self.camera_frame: str = p('camera_frame')
        self.base_frame: str = p('base_frame')
        self.cmd_vel_timeout: float = p('cmd_vel_timeout')
        self.rc_scale = dict(
            max_linear_speed=p('max_linear_speed'),
            max_vertical_speed=p('max_vertical_speed'),
            max_angular_speed=p('max_angular_speed'),
            rc_limit=p('rc_limit'),
        )
        self.velocity_scale: float = p('velocity_scale')
        self.land_on_shutdown: bool = p('land_on_shutdown')

        camera = load_orbslam_camera(p('camera_info_file'))
        self.camera_info = make_camera_info(camera, p('image_width'), p('image_height'), self.camera_frame)

        self.bridge = CvBridge()
        self.image_pub = self.create_publisher(Image, 'tello/camera', qos_profile_sensor_data)
        self.info_pub = self.create_publisher(CameraInfo, 'tello/camera_info', qos_profile_sensor_data)
        self.imu_pub = self.create_publisher(Imu, 'tello/imu', qos_profile_sensor_data)
        self.tof_pub = self.create_publisher(Range, 'tello/tof', qos_profile_sensor_data)
        self.velocity_pub = self.create_publisher(TwistStamped, 'tello/velocity', qos_profile_sensor_data)
        self.battery_pub = self.create_publisher(BatteryState, 'tello/battery', 10)
        self.state_pub = self.create_publisher(DiagnosticArray, 'tello/state', 10)

        # cmd_vel only moves the drone between takeoff and land/emergency
        self.flying = False
        self.last_cmd_time = 0.0
        self.moving = False
        self.cmd_lock = threading.Lock()

        self.tello = TelloZune(TELLOIP=p('tello_ip'))
        self.tello.set_image_size((p('image_width'), p('image_height')))
        if not self.tello.start_tello():
            raise RuntimeError('Não conectou ao Tello. O PC está no Wi-Fi TELLO-XXXXXX?')
        # Fallback in the library if this node stops sending (e.g. the process hangs)
        self.tello.set_rc_watchdog(self.cmd_vel_timeout)
        self.get_logger().info(f'Conectado ao Tello, bateria {self.tello.get_state().get("bat", "?")}%')

        self.control_group = MutuallyExclusiveCallbackGroup()
        self.service_group = MutuallyExclusiveCallbackGroup()  # land() blocks for seconds
        self.create_subscription(Twist, 'cmd_vel', self.on_cmd_vel, 10, callback_group=self.control_group)
        self.create_timer(0.1, self.check_cmd_vel_timeout, callback_group=self.control_group)
        self.create_timer(1.0 / p('state_rate'), self.publish_state, callback_group=self.control_group)
        self.create_service(Trigger, 'tello/takeoff', self.on_takeoff, callback_group=self.service_group)
        self.create_service(Trigger, 'tello/land', self.on_land, callback_group=self.service_group)
        self.create_service(Trigger, 'tello/emergency', self.on_emergency, callback_group=self.control_group)

        self.last_state_stamp = None
        self.running = True
        self.video_thread = threading.Thread(target=self.video_loop, daemon=True)
        self.video_thread.start()

    # Video -------------------------------------------------------------------

    def video_loop(self) -> None:
        """Publish every new frame with the time it left the decoder."""
        while self.running and rclpy.ok():
            result = self.tello.get_frame_stamped(timeout=0.5)
            if result is None:
                self.get_logger().warn('Sem vídeo do Tello', throttle_duration_sec=5.0)
                continue
            frame, stamp = result                               # (h, w, 3) uint8 BGR, float s
            msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
            msg.header.stamp = stamp_from_wall_time(stamp)
            msg.header.frame_id = self.camera_frame
            self.camera_info.header.stamp = msg.header.stamp
            self.image_pub.publish(msg)
            self.info_pub.publish(self.camera_info)

    # Telemetry ---------------------------------------------------------------

    def publish_state(self) -> None:
        stamp_s = self.tello.state_stamp
        if stamp_s is None or stamp_s == self.last_state_stamp:
            return
        self.last_state_stamp = stamp_s
        state = self.tello.get_state()
        stamp = stamp_from_wall_time(stamp_s)

        diag = DiagnosticArray()
        diag.header.stamp = stamp
        status = DiagnosticStatus(name='tello', hardware_id='tello', level=DiagnosticStatus.OK,
                                  message='raw SDK state (degrees, cm, 0.001 g)')
        status.values = [KeyValue(key=k, value=str(v)) for k, v in state.items()]
        diag.status = [status]
        self.state_pub.publish(diag)

        if {'roll', 'pitch', 'yaw'} <= state.keys():
            imu = Imu()
            imu.header.stamp = stamp
            imu.header.frame_id = self.base_frame
            qx, qy, qz, qw = tello_attitude_to_quaternion(state['roll'], state['pitch'], state['yaw'])
            imu.orientation.x, imu.orientation.y, imu.orientation.z, imu.orientation.w = qx, qy, qz, qw
            imu.orientation_covariance = [math.radians(1.0) ** 2, 0.0, 0.0,
                                          0.0, math.radians(1.0) ** 2, 0.0,
                                          0.0, 0.0, math.radians(5.0) ** 2]
            imu.angular_velocity_covariance[0] = -1.0  # not provided by the SDK
            if {'agx', 'agy', 'agz'} <= state.keys():
                ax, ay, az = tello_accel_to_mps2(state['agx'], state['agy'], state['agz'])
                imu.linear_acceleration.x, imu.linear_acceleration.y, imu.linear_acceleration.z = ax, ay, az
            else:
                imu.linear_acceleration_covariance[0] = -1.0
            self.imu_pub.publish(imu)

        if 'tof' in state:
            tof = Range()
            tof.header.stamp = stamp
            tof.header.frame_id = self.base_frame
            tof.radiation_type = Range.INFRARED
            tof.field_of_view = math.radians(25.0)
            tof.min_range, tof.max_range = 0.1, 3.0
            tof.range = state['tof'] / 100.0                    # cm -> m
            self.tof_pub.publish(tof)

        if {'vgx', 'vgy', 'vgz'} <= state.keys():
            vel = TwistStamped()
            vel.header.stamp = stamp
            vel.header.frame_id = self.base_frame
            vx, vy, vz = tello_velocity_to_mps(state['vgx'], state['vgy'], state['vgz'], self.velocity_scale)
            vel.twist.linear.x, vel.twist.linear.y, vel.twist.linear.z = vx, vy, vz
            self.velocity_pub.publish(vel)

        if 'bat' in state:
            bat = BatteryState()
            bat.header.stamp = stamp
            bat.percentage = state['bat'] / 100.0
            bat.present = True
            bat.voltage = bat.current = bat.charge = bat.capacity = bat.design_capacity = math.nan
            self.battery_pub.publish(bat)

    # Commands ----------------------------------------------------------------

    def on_cmd_vel(self, msg: Twist) -> None:
        if not self.flying:
            self.get_logger().warn('cmd_vel ignorado: drone não decolou por /tello/takeoff',
                                   throttle_duration_sec=5.0)
            return
        rc = twist_to_rc(msg.linear.x, msg.linear.y, msg.linear.z, msg.angular.z, **self.rc_scale)
        with self.cmd_lock:
            self.tello.send_rc_control(*rc)
            self.last_cmd_time = time.monotonic()
            self.moving = any(rc)

    def check_cmd_vel_timeout(self) -> None:
        with self.cmd_lock:
            if self.moving and time.monotonic() - self.last_cmd_time > self.cmd_vel_timeout:
                self.get_logger().warn('cmd_vel parado: enviando rc 0 0 0 0 (pairar)')
                self.stop_motion()

    def stop_motion(self) -> None:
        """rc 0 0 0 0 so the drone hovers in place. Call with cmd_lock held."""
        self.tello.send_rc_control(0, 0, 0, 0)
        self.moving = False

    def on_takeoff(self, request, response):
        if self.tello.get_state().get('bat', 100.0) < 20.0:
            response.success, response.message = False, 'Bateria abaixo de 20%: não decola'
            return response
        self.tello.takeoff()
        self.flying = True
        response.success, response.message = True, 'takeoff enfileirado'
        return response

    def on_land(self, request, response):
        with self.cmd_lock:
            self.flying = False
            self.tello.clear_command_queue()
            self.stop_motion()
        self.tello.land()                                       # blocks until 'ok' or retries end
        response.success, response.message = True, 'land enviado'
        return response

    def on_emergency(self, request, response):
        with self.cmd_lock:
            self.flying = False
            self.moving = False
            self.tello.emergency_stop()                         # cuts the motors: the drone falls
        response.success, response.message = True, 'emergency enviado (motores cortados)'
        return response

    # Shutdown ----------------------------------------------------------------

    def shutdown(self) -> None:
        self.running = False
        with self.cmd_lock:
            self.flying = False
            self.stop_motion()
        if self.land_on_shutdown and self.tello.get_state().get('h', 0.0) > 0:
            self.get_logger().info('Encerrando com o drone no ar: pousando')
            self.tello.clear_command_queue()
            self.tello.land()
        self.tello.end_tello()
        self.video_thread.join(timeout=1.0)


def main(args=None) -> None:
    rclpy.init(args=args)
    try:
        node = TelloDriver()
    except (RuntimeError, FileNotFoundError, ValueError) as e:
        rclpy.logging.get_logger('tello_driver').fatal(str(e))
        rclpy.shutdown()
        sys.exit(1)
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
