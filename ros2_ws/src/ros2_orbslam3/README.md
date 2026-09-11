# ROS2 ORB-SLAM 3

ROS2 node for [ORB-SLAM 3](https://github.com/UZ-SLAMLab/ORB_SLAM3).

Currently, supports Monocular and RGB-D modes.
The node subscribes to `/camera/rgb/image_color` (monocular)
or to `/camera/rgb/image_color` AND `/camera/depth/image` (RGB-D)
and publishes:

- Camera pose (`/pose_orb1`)
- The map points being tracked (`/tracked_mappoints`)
- All map points (`/all_mappoints`)
- The anotated tracking image (`/tracking_image`)
- The keyframe ref. frames (`/kf_markers`)
- Odometry messages (`/odom`)

The implementation is based on [this](https://github.com/ozandmrz/orb_slam3_ros2_mono_publisher)
and [this](https://github.com/curryc/ros2_orbslam3) implementations.

Tested on:

- Ubuntu 24.04 LTS
- ROS2 Jazzy

## __Install__

1. Download and compile [ORB-SLAM 3](https://github.com/UZ-SLAMLab/ORB_SLAM3).
2. Install the required ROS2 dependencies:

- `ros-jazzy-image-transport`
- `ros-jazzy-cv-bridge`
- `ros-jazzy-vision-opencv`
- `ros-jazzy-message-filters`

3. Download or clone this repo under your __<ROS2_Workspace>/src__ folder. ( i.e: `/home/user/ros2_ws/src`)
4. Set the install path of your ROS2 distro in the `CMakeLists.txt` file:

   `set(ENV{PYTHONPATH} "/opt/ros/<ROS2_site_packages>")`

   In my case, the line should be set to:

   `set(ENV{PYTHONPATH} "/opt/ros/jazzy/lib/python3.12/site-packages")`

5. Set the root path of your ORB-SLAM3 folder in the `CMakeModules/FindORB_SLAM3.cmake` file:

   `set(ORB_SLAM3_ROOT_DIR "<path_to_your_ORBSLAM3_root_folder>")`

   In my case, the line should be set to:

   `set(ORB_SLAM3_ROOT_DIR "/home/bruno/Projects/ORB_SLAM3")`

6. Source your ROS2 installation:
   
   `$ source /opt/ros/<ros2_distro>/setup.bash`

7. Build the package:
   
   `$ cd /home/user/ros2_ws`

   `$ colcon build`

## __Launching__

1. Source your ROS2 workspace:
   
   `$ source /home/user/ros2_ws/install/setup.bash`

2. Launch the node:
   
   `$ ros2 run ros2_orbslam3 mono <path_to_vocabulary> <path_to_settings>` (monocular mode)

   or

   `$ ros2 run ros2_orbslam3 rgbd <path_to_vocabulary> <path_to_settings>` (RGB-D mode)