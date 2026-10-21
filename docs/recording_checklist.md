# Recording checklist: Kinect map and drone bags

Checklist for recording new `bags/` so the drone localizes reliably against the
Kinect map. Each item comes from what went wrong with `bags/kinect` and
`bags/tello_alpha`; see
[`results/tello_alpha_vs_map_rgbd_04`](../results/tello_alpha_vs_map_rgbd_04/README.md).

Rule of thumb: the drone can only match what the Kinect keyframes saw, from
roughly the same viewpoint. Matches peak when the drone is within a few cm of
the mapping path and drop sharply ~0.3 m away from it.

## Before recording (both bags)

- [ ] Same scene for both bags: same furniture layout, lighting and time of
      day; no people walking through the view.
- [ ] Plan the drone route first (take-off, turns, landing), then map that
      route with the Kinect.
- [ ] Textureless or glass surfaces along the route (glass wall, plain
      cardboard): add posters or printed patterns if possible.
- [ ] Enough free disk space (raw images at 30 fps are ~1-2 GB/min).

## Kinect bag (mapping)

### Driver and topics

- [ ] **Depth registered to the RGB camera.** In `bags/kinect` the depth is in
      the `kinect_depth` frame (fx 587) while RGB is `kinect_rgb` (fx 527), so
      depth is misaligned by up to ~33 px. Enable depth registration in the
      driver if it has it (libfreenect supports a registered depth mode in mm).
- [ ] If registration is not available, record `/tf` and `/tf_static` (or note
      the depth-to-RGB extrinsic) so the depth can be registered offline.
- [ ] Record the camera info of both cameras.
- [ ] Header stamps are valid (non-zero) and RGB/depth are within a few ms of
      each other.

```bash
ros2 bag record -o /home/dev/bags/kinect_$(date +%Y%m%d_%H%M) \
  /image_raw /depth/image_raw /camera_info /depth/camera_info /tf /tf_static
```

### How to move

- [ ] Hold the Kinect at the drone's flight height, pointing where the drone
      camera will point.
- [ ] Walk the drone route 2-3 times: offset 0.3-0.5 m to each side, at
      slightly different heights, and once in the opposite direction.
- [ ] Start and end 1-2 m beyond the drone's take-off and landing points (the
      `tello_alpha` loss at 78 s happened where the map ends).
- [ ] Turn slowly; pause halfway through each turn.
- [ ] At the glass wall, aim at nearby textured objects (0.5-4.5 m, the Kinect
      depth range), not at the background through the glass.
- [ ] Stay 0.5-4.5 m from what you are looking at.
- [ ] Close the loop: return to the start pose, facing the start direction.
- [ ] Move slowly and steadily (no shaking, no fast rotations).

### After recording

- [ ] `ros2 bag info` shows all topics at ~30 Hz with similar message counts.
- [ ] Depth encoding and units match `config/kinect.yaml`: 16UC1 in mm means
      `RGBD.DepthMapFactor: 1000`; 32FC1 in metres means `1.0`.
- [ ] RGB intrinsics in `config/kinect.yaml` match the recorded RGB
      `camera_info` (`Camera1.fx/fy/cx/cy`, `k1`, `k2`, ...).
- [ ] Build the map (README section 4). In the mapping log, look for
      `Creation of new map`: more than one map means tracking was lost while
      mapping, so slow down in that stretch and record again.

## Drone bag (localization)

### Driver and topics

- [ ] **Valid header stamps.** `bags/tello_alpha` has `header.stamp = 0` on
      every image; make the driver stamp each frame with its capture time.
- [ ] Record the camera info.
- [ ] Record decoded images at the full 30 fps; do not recompress them.
- [ ] Keep the drone close to the computer on an uncongested Wi-Fi channel;
      transmission glitches cause the blurred and corrupted frames.

```bash
ros2 bag record -o /home/dev/bags/tello_$(date +%Y%m%d_%H%M) \
  /tello/camera <camera_info topic of the driver>
```

### Calibration

- [ ] Use the `tello_02` calibration (`config/tello_kinect_scale.yaml`), or
      recalibrate at 960x720 with many checkerboard images that cover the
      image corners. Reject calibrations with large `k2`/`k3`: the old
      `tello.yaml` (k3 = 10) moved the image corners by ~240 px.

### How to fly

- [ ] Take off inside the mapped area, facing the direction the Kinect faced at
      the start.
- [ ] Fly along the mapped route, at the mapping height, within ~0.3 m of it.
- [ ] Turn slowly; prefer yawing in place to fast turns while moving (the
      `tello_alpha` loss happened in a turn where the image changed twice as
      fast as on straight stretches).
- [ ] Stay at least 0.5 m from objects (at the end of `tello_alpha` the camera
      was against a chair).
- [ ] Land inside the mapped area.

### After recording

- [ ] `ros2 bag info`: ~30 Hz on the image topic, no long gaps.
- [ ] Localize against the new map (README section 5) and check the two plots:
  - [ ] Features plot: matched features stay above ~150, no gray (lost) bands.
  - [ ] Map plot: the drone trajectory follows the dotted Kinect keyframe path,
        and keyframes cover the whole route, including turns and the ends.
