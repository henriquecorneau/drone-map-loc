# ORB-SLAM3 + ROS 2 (Docker)

This project packages **ORB-SLAM3** integrated with **ROS 2** inside a Docker container, allowing you to run monocular or RGB-D SLAM without manually installing dependencies on the host. The full environment (ROS 2, OpenCV, Pangolin, Eigen, ORB-SLAM3, and the integration workspace) is already set up inside the `orbslam3_dev` container.

## What this project does

- Provides a ready-to-use Docker environment with ORB-SLAM3 built and integrated into a ROS 2 workspace (`ros2_ws`).
- Exposes a ROS 2 node, `/ORBSLAM3_ROS2/`, that can run SLAM in two modes:
  - **Monocular** — subscribes to the image topic `/tello/camera`.
  - **RGB-D** — subscribes to `/depth/image_raw` and `/image_raw`.
- Publishes tracking and mapping results as ROS 2 topics in both modes (pose, map point clouds, tracking image, keyframe markers, and odometry — see [Published topics](#published-topics)).
- Allows quick testing with a demo rosbag available on Hugging Face.

## Prerequisites

- Docker and Docker Compose installed.
- (Optional, recommended) GPU/X11 support configured on the host if you want to visualize the map/viewer through Pangolin.

## How to run

### 1. Clone the repository

```bash
git clone https://github.com/henriquecorneau/orbslam3-ros2-docker.git<repository-url>
cd orbslam3-ros2-docker 
```

### 2. Build and start the container

```bash
docker compose build
docker compose up -d
```

### 3. Enter the container

```bash
docker exec -it orbslam3_dev bash
```

### 4. Build the ROS 2 workspace

Inside the container:

```bash
cd ros2_ws
colcon build
source install/setup.bash
```

> Run `source install/setup.bash` again every time you open a new terminal inside the container.

### 5. Run ORB-SLAM3

**Monocular mode:**

```bash
ros2 run orbslam3 mono path/to/vocabulary path/to/settings
```

**RGB-D mode:**

```bash
ros2 run orbslam3 rgbd path/to/vocabulary path/to/settings
```

Replace `path/to/vocabulary` with the ORB vocabulary file (e.g. `ORBvoc.txt`) and `path/to/settings` with the `.yaml` file containing the camera/sensor parameters.

## Atlas (map) settings

In the camera settings `.yaml` file, you need to include one of the following lines depending on whether you want to build a new map or reuse an existing one:

- To **build a new map and save the atlas** once SLAM finishes:

  ```yaml
  System.SaveAtlasToFile: "atlas_00"
  ```

- To **load a previously saved atlas** instead of mapping from scratch:

  ```yaml
  System.LoadAtlasFromFile: "atlas_00"
  ```

Only one of these should be active at a time, depending on whether you are creating or reusing a map.

## Testing with a demo rosbag

If you don't have a camera connected, or just want to validate that everything works, use the sample rosbag provided on Hugging Face:

1. Download the file from the [Hugging Face link](https://huggingface.co/datasets/rickmandioca/orb_bags_dataset).
2. Extract its contents.
3. Place the extracted folder inside `bags/` at the root of the project.
4. Play the bag in another terminal (inside or outside the container, depending on your ROS network setup) while the ORB-SLAM3 node is running:

```bash
ros2 bag play bags/<bag-name>
```

## Subscribed topics (input)

| Mode | Topics |
|------|--------|
| Monocular | `/tello/camera` |
| RGB-D | `/depth/image_raw`, `/image_raw` |

## Published topics

Both nodes (`mono` and `rgbd`) publish the same output topics, prefixed with the node name (`/ORBSLAM3_ROS2/...`):

| Topic | Message type | Description |
|-------|--------------|-------------|
| `/ORBSLAM3_ROS2/pose` | `geometry_msgs/msg/PoseStamped` | Estimated camera pose |
| `/ORBSLAM3_ROS2/tracked_mappoints` | `sensor_msgs/msg/PointCloud2` | Currently tracked map points |
| `/ORBSLAM3_ROS2/all_mappoints` | `sensor_msgs/msg/PointCloud2` | All map points |
| `/ORBSLAM3_ROS2/tracking_image` | `sensor_msgs/msg/Image` | Image with tracking overlay |
| `/ORBSLAM3_ROS2/kf_markers` | `visualization_msgs/msg/Marker` | Keyframe markers (for RViz) |
| `/ORBSLAM3_ROS2/odom` | `nav_msgs/msg/Odometry` | Estimated odometry |

## Expected project structure

```
.
├── Dockerfile
├── docker-compose.yml
├── ros2_ws/
│   └── src/...
├── bags/
│   └── (extracted rosbags here)
├── configs/
│   └── (config files here)
└── README.md
```

## Notes

- Make sure the vocabulary and settings paths passed to `ros2 run` are accessible inside the container (mount volumes in `docker-compose.yml` if the files live outside the workspace).
- To visualize pose, point cloud, and keyframes, point RViz2 to the topics listed above.