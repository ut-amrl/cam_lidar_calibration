# Calibration from one ROS 2 bag

Record the raw LiDAR cloud, camera image (raw or JPEG), CameraInfo, and preferably
`/tf_static` in **one normal Jazzy MCAP/SQLite bag**. Hold the rig stationary on
flat ground; show the checkerboard at different distances, image positions,
pitches, and yaws. Finish with a few seconds of the board flat on the floor and
nobody in front of the sensors. No special recording format or separate bags.

## Run

From the memo workspace, using the existing `memo_ws` container (no new libraries):

```bash
scripts/calibrate_bag /data/arthur/bags/calibration/mission_20261107_150933_951207 \
  --background-start 181
```

Defaults: `/camera/rgb/image_raw/compressed`, `/camera/rgb/camera_info`,
`/velodyne_points`; **9 × 6 squares, 83.63 mm each**, with no outside border.
Output defaults to `data/calibration/results/<bag-directory-name>` in the memo
workspace; its full path is printed at completion. Override with `--output PATH`.
Use `--help` to change topics/board dimensions. Output must be empty.
Background defaults to three seconds before the end; timestamps are seconds
from bag start. JPEG decoding happens in memory. The input bag is unchanged.

The command fits camera intrinsics, registers LiDAR points to checkerboard
planes, fits the ground, and prints the selected intrinsics, transforms, and
errors. Recorded CameraInfo wins if the new fit is worse on held-out views.
Without a recorded static mounting chain, initialization assumes a forward
optical camera roughly 25 cm below the LiDAR; for other mounts supply
`--initial-extrinsic FILE.yaml` containing a 4×4 **T_camera_lidar** matrix.
This uses iterative target-plane registration, not ACFR's interactive edge solver.

## Outputs

| File | Contents |
| --- | --- |
| `camera_intrinsics.yaml` | Selected ROS camera-calibration-format intrinsics |
| `camera_intrinsics_candidate.yaml` | Newly fitted intrinsics, including when not selected |
| `lidar_camera.yaml` | Camera pose in LiDAR frame, inverse matrix, fit checks |
| `lidar_ground.yaml` | LiDAR pose in ground frame and ground fit checks |
| `calibration.yaml` | CameraInfo and transforms for the publisher |
| `report.yaml` | Stage status, source frames, initialization, held-out errors |
| `intrinsics_errors.csv` | Observed/predicted corners and pixel errors |
| `debug/intrinsics/` | Corner/reprojection error images |
| `debug/lidar_projection/` | LiDAR-on-image overlays |

Ground fixes the LiDAR origin at **x=0, y=0**, estimates height/roll/pitch, and
uses projected LiDAR +X as heading. Absolute yaw cannot come from a plane.
Transform YAMLs explicitly name parent/child and use meters and XYZW quaternions.
A failed stage returns exit code 2; inspect the report. Failed transforms are not
saved as successful calibrations. Initializer-only overlays are labeled separately.
Held-out residuals are consistency checks, not independent ground-truth accuracy.

## Publish later

```bash
scripts/publish_calibration /home/ros/memo_ws/data/calibration/results/mission_20261107 --dry-run
scripts/publish_calibration /home/ros/memo_ws/data/calibration/results/mission_20261107 \
  --camera-info-topic /calibration/camera_info
```

Publishes CameraInfo and `/tf_static` only when explicitly invoked and the report
passes. Stop conflicting camera/TF publishers before deployment. Ground parent
is `calibration_ground`; its child is `velodyne`, whose child is `kinect_color`
for the supplied bag. Nothing is published by the calibration command itself.
