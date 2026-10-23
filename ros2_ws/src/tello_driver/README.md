# tello_driver

ROS 2 Humble driver for the DJI Tello, built on
[tello-zune](https://github.com/zunedrones/tello_zune). It publishes the
camera with capture stamps, the camera info and the telemetry, and turns
`/cmd_vel` into Tello `rc` commands with a watchdog.

Requires tello-zune with `get_frame_stamped()`, `get_state()` and
`set_rc_watchdog()` ([zunedrones/tello_zune#34](https://github.com/zunedrones/tello_zune/pull/34));
the `Dockerfile` installs it pinned to that commit (`TELLO_ZUNE_REF`).

## Run

Connect the host to the Tello Wi-Fi (`TELLO-XXXXXX`). The container uses
`network_mode: host`, so it receives the Tello UDP ports (8889 commands,
8890 telemetry, 11111 video).

```bash
docker compose exec orbslam bash
source /opt/ros/humble/setup.bash
cd ~/ros2_ws && colcon build --symlink-install --packages-select tello_driver
source install/setup.bash
ros2 launch tello_driver tello_driver.launch.py
```

Take off, move and land:

```bash
ros2 service call /tello/takeoff std_srvs/srv/Trigger
ros2 topic pub -r 10 /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.1}}"   # Ctrl+C: hovers after 0.5 s
ros2 service call /tello/land std_srvs/srv/Trigger
```

## Interface

### Published

| Topic | Type | QoS | Content |
| --- | --- | --- | --- |
| `/tello/camera` | `sensor_msgs/Image` | sensor data | `bgr8`, 960x720, `header.stamp` = time the frame left the decoder, frame `tello_camera_optical_frame` |
| `/tello/camera_info` | `sensor_msgs/CameraInfo` | sensor data | From `camera_info_file` (`plumb_bob`), same stamp as the image |
| `/tello/imu` | `sensor_msgs/Imu` | sensor data | Orientation (FLU, yaw relative to power-on heading) and specific force in m/s²; no angular velocity |
| `/tello/tof` | `sensor_msgs/Range` | sensor data | Downward ToF distance in m (0.1 to ~3 m) |
| `/tello/velocity` | `geometry_msgs/TwistStamped` | sensor data | SDK velocity in FLU, m/s. **Unit and frame unverified**: do not use for control yet |
| `/tello/battery` | `sensor_msgs/BatteryState` | 10 | `percentage` (0-1) |
| `/tello/state` | `diagnostic_msgs/DiagnosticArray` | 10 | Every raw SDK field (degrees, cm, 0.001 g), for bags and debugging |

The telemetry topics are published once per SDK state packet, stamped with
its arrival time.

### Subscribed

| Topic | Type | Content |
| --- | --- | --- |
| `/cmd_vel` | `geometry_msgs/Twist` | FLU: `linear.x` forward, `linear.y` left, `linear.z` up (m/s), `angular.z` counter-clockwise (rad/s). Ignored until `/tello/takeoff` |

### Services (`std_srvs/Trigger`)

| Service | Effect |
| --- | --- |
| `/tello/takeoff` | Queues `takeoff` and enables `/cmd_vel`. Refused below 20% battery |
| `/tello/land` | Disables `/cmd_vel`, sends `rc 0 0 0 0` and lands (blocks until the drone answers) |
| `/tello/emergency` | **Cuts the motors: the drone falls.** Only to avoid hitting someone or something |

### Parameters

See [`config/tello_driver.yaml`](config/tello_driver.yaml). The main ones:

| Parameter | Default | Meaning |
| --- | --- | --- |
| `camera_info_file` | `/home/dev/config/tello_02.yaml` | ORB-SLAM3 settings with the Tello calibration |
| `cmd_vel_timeout` | 0.5 s | Without `/cmd_vel` for this long, sends `rc 0 0 0 0` (hover). Also armed in tello-zune as a fallback |
| `max_linear_speed`, `max_vertical_speed` | 1.0 m/s | Speed mapped to rc 100. **Not documented by the SDK; calibrate** |
| `max_angular_speed` | 1.75 rad/s | Yaw rate mapped to rc 100. **Calibrate** |
| `rc_limit` | 40 | Saturation of every rc channel (0-100). Keep it low in the first flights |
| `land_on_shutdown` | true | Lands if the node stops with the drone in the air (`h > 0`) |

## Conventions

- `/cmd_vel` and `/tello/imu` follow REP-103 (FLU body, m, rad).
- The Tello SDK does not document its body axes. The driver assumes FRD
  (x forward, y right, z down), which matches the accelerometer at rest
  (`agz` ≈ -1000) and the yaw growing with `cw`; FRD → FLU keeps x and flips
  y and z, so roll stays and pitch and yaw change sign.
- The `rc` channels are positive to the right and clockwise, so `linear.y` and
  `angular.z` change sign.

## Bench test (before the first flight)

With the drone powered on, **propellers off or the drone held in hand**, and
the driver running:

1. **Stamps and rate:** `ros2 topic hz /tello/camera` shows ~30 Hz;
   `ros2 topic echo --once /tello/camera --field header` shows a non-zero stamp.
2. **Latency:** film a stopwatch on the computer screen and compare the time in
   the image with `header.stamp`; the difference is the Wi-Fi + decoder latency
   that the stamp does not include.
3. **Attitude signs** (`ros2 topic echo /tello/imu`, quaternion xyzw):
   rotate the drone counter-clockwise seen from above → yaw grows (FLU);
   nose down → pitch positive (FLU); right side down → roll negative (FLU).
4. **Acceleration at rest:** `linear_acceleration.z` ≈ +9.8.
5. **ToF:** lift the drone ~1 m → `/tello/tof` ≈ 1.0.
6. **Velocity unit:** walk with the drone at a known speed and compare with
   `/tello/velocity`; adjust `velocity_scale`.
7. **cmd_vel watchdog** (drone on the ground, not armed): `ros2 topic pub`
   without `/tello/takeoff` must log that `cmd_vel` is ignored.

If a sign in step 3 is wrong, fix `tello_driver/conversions.py` and its tests
before flying.

## Tests

```bash
colcon test --packages-select tello_driver --event-handlers console_direct+
```

They use a fake `TelloZune`, so they run without the drone. A numeric check of
the frame conversions (orthonormality, det = +1, quaternion round trip,
FRD → FLU signs) is in `test/test_conversions.py`.
