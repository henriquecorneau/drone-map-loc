# Drone Map Localization

This project uses **ORB-SLAM3 with ROS 2 Humble** to build a map from an
RGB-D camera and then localize a monocular drone camera against that saved
map. The long-term goal is autonomous drone navigation through a previously
mapped environment using **Nav2**.

The ROS 2 workspace and ORB-SLAM3 dependencies are built in Docker. The
container includes Pangolin for visualization. ORB-SLAM3 is built from
[henriquecorneau/ORB_SLAM3](https://github.com/henriquecorneau/ORB_SLAM3), a
fork of `huynhduc9905/ORB_SLAM3` that relocalizes after tracking loss in
localization-only mode. The commit is pinned in the `Dockerfile`
(`ORB_SLAM3_COMMIT`); bump it and run `docker compose build` to pick up fork
changes.

## Current capabilities

- Build an ORB-SLAM3 atlas using synchronized RGB and depth images.
- Save and load the atlas as an `.osa` file.
- Localize monocular images against a loaded atlas, including starting in
  localization-only mode.
- Play recorded ROS 2 bags for mapping and localization.

The monocular and RGB-D nodes publish pose, odometry, map-point clouds,
tracking images, and keyframe markers. The RGB-D node subscribes to
`/image_raw` and `/depth/image_raw`. The monocular node defaults to
`/tello/camera`; its topic can be changed with the `image_topic` ROS parameter.

## Requirements

- Linux with Docker Engine and the Docker Compose plugin.
- X11 available for the Pangolin viewer. Pangolin display forwarding may
  require host-specific X11 permissions/configuration.
- Enough disk space for the Docker image and downloaded rosbag files.

## 1. Clone and build the Docker image

```bash
git clone https://github.com/henriquecorneau/drone-map-loc.git
cd drone-map-loc
docker compose build
docker compose up -d
```

The Compose service is named `orbslam`. The project directory is mounted into
the container at `/home/dev`, including `ros2_ws`, `config`, and `bags`.

## 2. Download example bags

Download the RGB-D and drone bags from the project dataset:

<https://huggingface.co/datasets/rickmandioca/orb_bags_dataset>

You can download the files from the Hugging Face page, or use the Hugging Face
CLI on the host:

```bash
python3 -m pip install --user -U huggingface_hub
hf download rickmandioca/orb_bags_dataset \
  --repo-type dataset \
  --local-dir bags
```

If the dataset contains archives, extract each bag directory under `bags/`.
A ROS 2 bag directory should contain `metadata.yaml` and its storage file(s),
such as `.db3` files. Bag data and generated maps are intentionally excluded
from this Git repository.

## 3. Build the ROS 2 workspace

```bash
docker compose exec orbslam bash
```

Inside the container:

```bash
source /opt/ros/humble/setup.bash
cd ~/ros2_ws
colcon build --packages-select ros2_orbslam3
source install/setup.bash
```

Source the ROS and workspace setup files in every new container shell.

The vocabulary and settings files in this setup are:

```text
/home/dev/ORB_SLAM3/Vocabulary/ORBvoc.txt
/home/dev/config/kinect.yaml
/home/dev/config/tello.yaml
/home/dev/config/tello_kinect_scale.yaml
```

## 4. Build an RGB-D map

In a container shell, start the RGB-D node:

```bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
ros2 run ros2_orbslam3 rgbd \
  /home/dev/ORB_SLAM3/Vocabulary/ORBvoc.txt \
  /home/dev/config/kinect.yaml \
  --save-map /home/dev/bags/environment.osa
```

In a second container shell, play the RGB-D bag:

```bash
source /opt/ros/humble/setup.bash
ros2 bag play /home/dev/bags/<rgbd_bag_directory>
```

The RGB-D node expects synchronized color and depth topics named
`/image_raw` and `/depth/image_raw`. Check a bag's topics before running it:

```bash
ros2 bag info /home/dev/bags/<rgbd_bag_directory>
```

`kinect.yaml` uses the RGB intrinsics from the `/camera_info` recorded in
`bags/kinect` and `RGBD.DepthMapFactor: 1000`, because that bag stores depth as
16UC1 millimetres. Maps built before this fix (`map_rgbd_01` to `_03`, with the
TUM calibration and factor 5000) are 5x too small; `map_rgbd_04` is the
corrected map. The depth images in `bags/kinect` are not registered to the RGB
camera (`kinect_depth` frame, fx 587 vs RGB fx 527), so depth is slightly
misaligned with the RGB pixels, especially near the image borders.

When mapping is complete, stop the node with **Ctrl+C once** and allow
shutdown to finish. ORB-SLAM3 writes the requested `.osa` atlas during
shutdown. The project configs disable ORB-SLAM3's optional web viewer because
its blocking server accept can prevent clean shutdown on Linux.

## 5. Localize the drone against the RGB-D map

Start the monocular node with the saved atlas and localization-only mode:

```bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
ros2 run ros2_orbslam3 mono \
  /home/dev/ORB_SLAM3/Vocabulary/ORBvoc.txt \
  /home/dev/config/tello_kinect_scale.yaml \
  --load-map /home/dev/bags/environment.osa \
  --localize-only \
  --ros-args -p image_topic:=/tello/camera
```

In another shell, play the monocular drone bag:

```bash
source /opt/ros/humble/setup.bash
ros2 bag play /home/dev/bags/<drone_bag_directory>
```

Use a monocular settings file calibrated for the **drone camera**. The RGB-D
map may have been built with a different camera, but localization requires
the drone images to see visual features that overlap with the mapped
environment. Start the drone bag in a mapped area and move slowly while the
system relocalizes. The RGB-D `DepthMapFactor` is not used by the monocular
node.

When the map was built with the Kinect, use `tello_kinect_scale.yaml`. It uses
the `tello_02.yaml` calibration plus `Camera.newWidth/newHeight`, which resize
the Tello images so features have the same pixel scale as in the Kinect images
(Kinect fx 526.6, Tello fx ~921). ORB-SLAM3 predicts a map point's ORB pyramid
level from its distance only, so without the resize most matches are searched ~3 pyramid
levels off and rejected. On `tello_alpha` against `map_rgbd_03`, tracked
frames went from 66% (`tello.yaml`) to 85% with the resize, and to 89% (mean
~230 matched features) with the resize and the `tello_02` calibration. Against
the corrected `map_rgbd_04` it tracks 90% of the frames (mean ~236 matched).
Use plain `tello.yaml` for monocular maps built with the Tello itself.

The default monocular topic is `/tello/camera`; set `image_topic` to the bag's
actual image topic if it differs. For example:

```bash
--ros-args -p image_topic:=/image_raw
```

`--localize-only` requires `--load-map`. It starts localization mode when the
atlas is loaded instead of first creating a new map and switching modes in
Pangolin.

## Map and localization options

Both executables accept these options:

| Option | Purpose |
| --- | --- |
| `--save-map <file.osa>` | Save the atlas to this file when the node shuts down. |
| `--load-map <file.osa>` | Load an existing atlas at startup. |
| `--localize-only` | Start in localization-only mode; requires `--load-map`. |

The `mono` executable also accepts:

| Option | Purpose |
| --- | --- |
| `--features-csv <file.csv>` | Where to save per-frame feature stats on shutdown (default `/home/dev/maps/localization_features.csv`). |
| `--no-plot` | Save the CSV but do not open the feature plot. |

## Feature plot and localization map

During tracking, the `mono` node records for each frame the number of ORB
features **detected**, the number **matched to map points** (inliers used for
pose estimation), the tracking state and the camera pose. When you stop the
node with Ctrl+C it writes, next to `--features-csv <stats>.csv`:

| File | Content |
| --- | --- |
| `<stats>.csv` | Per frame: `stamp,detected,matched,state,map_id,x,y,z,qx,qy,qz,qw`. The pose is `T_world_camera` (camera position in the map frame, quaternion in xyzw order); empty when not tracked. |
| `<stats>_keyframes.csv` | Keyframes of every map in the atlas: `map_id,id,x,y,z,qx,qy,qz,qw` (`T_world_camera`). |
| `<stats>_map_points.csv` | Map points of every map in the atlas: `map_id,x,y,z`. |

It then shuts ORB-SLAM3 down and opens two plots, also saved as PNGs:

- `<stats>.png`: detected and matched features over time, with the frames
  where tracking was lost shaded.
- `<stats>_map.png`: top-down view of the map with the drone trajectory
  colored by matched features, the Kinect mapping keyframes, and markers where
  tracking was lost (last tracked pose) and relocalized. The ground plane is
  estimated from the mapping keyframes' camera down axis, assuming the Kinect
  was held roughly level; map points far above or below the camera heights
  (floor, ceiling) are hidden. Distances are in map units, which are metres
  only if the RGB-D map used the correct `RGBD.DepthMapFactor`.

Results for `tello_alpha`, with plots and an analysis of where and why
tracking is lost:

- [`results/tello_alpha_vs_map_rgbd_04`](results/tello_alpha_vs_map_rgbd_04/README.md):
  against the metric Kinect map, with the localization map.
- [`results/tello_alpha_vs_map_rgbd_03`](results/tello_alpha_vs_map_rgbd_03/README.md):
  against the earlier map (5x too small), one plot per localization fix.

To re-plot an existing run:

```bash
ros2 run ros2_orbslam3 plot_features.py /home/dev/maps/localization_features.csv
ros2 run ros2_orbslam3 plot_localization_map.py /home/dev/maps/localization_features.csv
```

## ROS topics

### Inputs

| Node | Topic | Data |
| --- | --- | --- |
| `rgbd` | `/image_raw` | RGB/color image |
| `rgbd` | `/depth/image_raw` | Registered depth image |
| `mono` | `/tello/camera` by default | Monocular image; configurable with `image_topic` |

### Outputs

The node name and namespace may affect the fully qualified topic names. The
nodes publish pose/odometry, tracked and reported map points, tracking image,
and keyframe markers. Inspect the live names with:

```bash
ros2 topic list
```

## Troubleshooting localization

If Pangolin reports `Track lost, trying to relocalize`, the atlas may have
loaded correctly while the current image failed to match its keyframes.
Check, in order:

1. The image topic is receiving the intended drone camera stream.
2. `tello.yaml` matches the drone camera calibration and actual image size,
   and `tello_kinect_scale.yaml` is used against Kinect-built maps.
3. The drone starts in a location and orientation represented in the map.
4. The image is sharp, well-lit, and contains textured features; avoid rapid
   motion, motion blur, blank surfaces, and large viewpoint changes.
5. The mapping route covers the area where localization is being attempted.

Loading visible map points in Pangolin confirms the atlas loaded; it does not
by itself mean the live camera has localized successfully.

## Project TODO: autonomous navigation with Nav2

The end goal is to navigate the drone autonomously through a pre-mapped
environment using Nav2. Planned work:

- [ ] Verify localization robustness across RGB-D mapping and drone-camera
  sessions, including camera extrinsics and coordinate frames.
- [ ] Publish a consistent TF tree for `map`, `odom`, `base_link`, and the
  drone camera frame.
- [ ] Bridge ORB-SLAM3 pose/localization and confidence into ROS 2 odometry
  and localization interfaces expected by Nav2.
- [ ] Integrate Nav2 with a suitable 3D-to-2D occupancy representation of
  the pre-mapped environment.
- [ ] Connect Nav2 velocity/path commands to the drone control interface.
- [ ] Add obstacle avoidance, failsafes, and simulation/real-flight tests.

## License and upstream components

This repository contains a ROS 2 wrapper and Docker setup. ORB-SLAM3,
Pangolin, ROS 2, and downloaded bag datasets have their own licenses and
usage terms; consult their upstream projects and the dataset page.
