# benchmark-SR-LIO-to-HDMapping

Runs the [SR-LIO](https://github.com/ZikangYuan/sr_lio) LiDAR-Inertial odometry
algorithm on a ROS 1 bag file and converts the output to an
[HDMapping](https://github.com/MapsHD/HDMapping) session.

SR-LIO is *LiDAR-Inertial Odometry with Sweep Reconstruction* by Zikang Yuan
et al. It reconstructs high-frequency sweeps from a spinning LiDAR and fuses
them with IMU in a continuous-time bundle-adjustment-style optimizer.

## Prerequisites

- Docker
- A ROS 1 bag containing a `sensor_msgs/PointCloud2` LiDAR topic and a
  `sensor_msgs/Imu` topic matching the topic names declared in the chosen
  SR-LIO config (ROS 2 bags are automatically converted to ROS 1 format).

## Step 1 — Clone with submodules

```bash
git clone https://github.com/MapsHD/benchmark-SR-LIO-to-HDMapping.git --recursive
cd benchmark-SR-LIO-to-HDMapping
```

## Step 2 — Build the Docker image

```bash
docker build -t sr-lio_noetic .
```

This installs:
- Ubuntu 20.04 + ROS 1 Noetic
- Eigen3, PCL, Boost, TBB, glog/gflags
- Ceres (system `libceres-dev`)
- SR-LIO (compiled from submodule; the `tessil` robin_map is vendored)
- catkin workspace with `sr_lio` and `srlio_to_hdmapping`

## Step 3 — Run the pipeline

```bash
chmod +x docker_session_run-ros1-sr-lio.sh
./docker_session_run-ros1-sr-lio.sh /path/to/input.bag /path/to/output/dir
```

Or with no arguments to use a GUI file selector (requires `zenity`):

```bash
./docker_session_run-ros1-sr-lio.sh
```

By default the script uses the `utbm` config. Pick a different one with the
`SENSOR` environment variable, e.g.:

```bash
SENSOR=nclt ./docker_session_run-ros1-sr-lio.sh /path/to/input.bag /path/to/output/dir
```

Available dataset configs (from the `sr_lio/config/` directory):

| `SENSOR`    | Config file          | LiDAR type | Config LiDAR topic  | Config IMU topic |
|-------------|----------------------|------------|---------------------|------------------|
| `utbm`      | `utbm.yaml`          | Velodyne   | `/velodyne_points`  | `/imu/data`      |
| `nclt`      | `nclt.yaml`          | Velodyne   | `/points_raw`       | `/imu_raw`       |
| `robosense` | `robosense.yaml`     | RoboSense  | `/rslidar_points`   | `/gps/gtimu`     |
| `ulhk_1`    | `ulhk_1.yaml`        | Velodyne   | `/velodyne_points_0`| `/imu/data`      |
| `ulhk_2`    | `ulhk_2.yaml`        | Velodyne   | `/velodyne_points`  | `/imu/data`      |
| `kaist`     | `kaist/urban06.yaml` | Velodyne   | `/velodyne_points`  | `/imu/data_raw`  |

If your bag uses different topic names than the config, remap them on playback
with `LIDAR_TOPIC` / `IMU_TOPIC` (the value is the name **in the bag**):

```bash
SENSOR=utbm LIDAR_TOPIC=/points_raw IMU_TOPIC=/imu/data_raw \
  ./docker_session_run-ros1-sr-lio.sh /path/to/input.bag /path/to/output/dir
```

**What happens:**

The script opens a Docker container with a tmux session containing five panes on
window 0 and a `control` window (window 1, the attach target):

| Pane | Role |
|------|------|
| 0 | `roscore` |
| 1 | `roslaunch sr_lio lio_bench.launch` — subscribes to the LiDAR + IMU topics, publishes `/Odometry_after_opt` + `/cloud_global_map` (+ RViz live view) |
| 2 | `rosbag record /Odometry_after_opt /cloud_global_map` — captures odometry and the registered world cloud |
| 3 | `rosbag play --clock` — plays your input bag |
| 4 | diagnostics — shows active topics and publishing rates |

When playback finishes, the control window stops the recorder, kills all nodes
and RViz, and exits tmux. A second Docker run then converts the recorded bag
into the HDMapping session format.

## Step 4 — Open in HDMapping

Output files appear in `<output_dir>/output_hdmapping-SR-LIO/`:

```
lio_initial_poses.reg
poses.reg
scan_lio_0.laz
scan_lio_1.laz
...
session.json
trajectory_lio_0.csv
trajectory_lio_1.csv
...
```

Open `session.json` with the
[multi_view_tls_registration_step_2](https://github.com/MapsHD/HDMapping)
application.

## Notes on SR-LIO

SR-LIO publishes:

| Topic | Type | Meaning |
|-------|------|---------|
| `/Odometry_after_opt` | `nav_msgs/Odometry` | the optimized 6-DoF body pose in the `camera_init` (world) frame |
| `/cloud_global_map` | `sensor_msgs/PointCloud2` | the sweep's new points, already registered into the `camera_init` (world) frame |

Both messages are stamped with the sensor sweep-end time, so the exported
trajectory keeps the dataset's own timestamps (correct alignment with ground
truth). Because `/cloud_global_map` is already in the world frame, the converter
does **not** re-apply the pose to the points — it only uses
`/Odometry_after_opt` to build the per-chunk trajectory files. The recorded
topics are tunable via `ODOM_TOPIC` / `CLOUD_TOPIC`.

The upstream per-dataset launch files (e.g. `lio_utbm.launch`) spawn a
`velodyne_pointcloud` node to decode raw Velodyne packets. This benchmark uses
its own `lio_bench.launch` instead, which assumes the LiDAR topic is already a
decoded `sensor_msgs/PointCloud2` and only runs the SR-LIO node.

## Contact

januszbedkowski@gmail.com
