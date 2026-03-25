# Multi-LiDAR Calibration — Run Guide

Calibrates `rslidarfront` and `rslidarback` to the `base_link` frame on the mecanum robot using GICP.

---

## Prerequisites

- ROS2 Humble installed and sourced
- `ros-humble-rosbag2-storage-mcap` installed (for MCAP bag playback)
- Workspace built with `colcon build --symlink-install`
- A recorded `.mcap` bag with both lidar topics and `/tf_static`

```bash
sudo apt-get install ros-humble-rosbag2-storage-mcap
```

---

## Step 1 — Build the workspace

```bash
cd /ros_ws
colcon build --symlink-install
source install/setup.bash
```

---

## Step 2 — Extract PCD files from the bag

This only needs to be done once per bag. The extracted PCDs are saved to `data/pcds/` and reused for all subsequent calibration runs.

**Terminal 1 — play the bag on loop:**
```bash
ros2 bag play <path_to_bag> --loop
```

**Terminal 2 — run the extractor:**
```bash
source /ros_ws/install/setup.bash
python3 /ros_ws/src/multi_lidar_calibration/scripts/extract_pcds.py
```

The script captures the first 10 frames from each lidar within a 2-second window, merges them into a single dense cloud per lidar, and saves:

```
data/pcds/rslidarfront.pcd
data/pcds/rslidarback.pcd
```

Stop the bag after the extractor exits.

---

## Step 3 — Set the TF table in params.yaml

The TF table gives the initial pose estimate for each lidar in `base_link`. Read the actual values from the bag's `/tf_static`:

```bash
# Terminal 1 — play bag
ros2 bag play <path_to_bag> --loop

# Terminal 2 — read tf_static
python3 /tmp/read_tf.py
```

Update `config/params.yaml` with the printed xyz/rpy values (in degrees if `table_degrees: true`):

```yaml
read_tf_from_table: true
table_degrees: true
rslidarfront: [1.312, -0.45, 0.345, 0.0, 0.0, -46.9825]
rslidarback:  [-1.312, 0.45, 0.345, 0.0, 0.0, 135.069]
```

---

## Step 4 — Review params.yaml

Key parameters for the calibration run:

```yaml
# Use saved PCD files — no bag needed at calibration time
read_pcds_from_file: true
pcd_directory: /../data/pcds/

# Calibrate all lidars to base_link
calibrate_to_base: true
calibrate_target: false
target_frame_id: rslidarfront
base_frame_id: base_link

# GICP tuning — tighter = better quality, lower fitness score
max_corresp_dist: 0.3      # max point-pair distance [m]
max_rotation_deg: 10.0     # rotation sanity cap — rejects GICP if delta > this
max_iterations: 200
rel_fitness: 1.0e-6
rel_rmse: 1.0e-6
epsilon: 0.0001
voxel_size: 0.05           # downsampling for normal estimation [m]
fitness_score_threshold: 0.3  # calibration is rejected if fitness <= this

# Visualize alignment before and after calibration
visualize: true
runs_count: 5              # number of independent calibration runs
```

---

## Step 5 — Run the calibration

```bash
source /ros_ws/install/setup.bash
ros2 launch multi_lidar_calibrator calibration.launch.py \
  parameter_file:=/ros_ws/src/multi_lidar_calibration/config/params.yaml
```

Two visualization windows will pop up per run:

| Window | What it shows |
|---|---|
| **Window 1** | Initial state — both lidars placed using TF table transforms. Should look aligned. |
| **Window 2** | GICP result — refined alignment. Should be equal or better than Window 1. |

Close each window to proceed to the next run.

---

## Step 6 — Read the results

Results are written to `output/results.txt` after each run. The terminal also prints per-run calibration info.

**Interpret the output:**

```
rslidarback to rslidarfront calibration
calibrated xyz = <x> <y> <z>       # translation of rslidarback in base_link [m]
calibrated rpy = <r> <p> <y>       # rotation in degrees (roll, pitch, yaw)
fitness: <value>                    # fraction of points with a valid correspondence (higher = better)
inlier_rmse: <value>               # mean distance of matched point pairs [m] (lower = better)
```

**Quality thresholds:**

| Metric | Good | Acceptable | Poor |
|---|---|---|---|
| fitness | > 0.5 | 0.3 – 0.5 | < 0.3 (rejected) |
| inlier_rmse | < 0.05 m | 0.05 – 0.15 m | > 0.15 m |

---

## Reference Result — 2026-03-25

Bag: `new_calibrated_rz_20260324_190930`
Platform: mecanum_bot
Environment: indoor, static scene

**TF table (initial guess from bag tf_static):**

| LiDAR | x [m] | y [m] | z [m] | roll [°] | pitch [°] | yaw [°] |
|---|---|---|---|---|---|---|
| rslidarfront | 1.312 | -0.45 | 0.345 | 0.0 | 0.0 | -46.9825 |
| rslidarback | -1.312 | 0.45 | 0.345 | 0.0 | 0.0 | 135.069 |

**GICP params used:**

| Param | Value |
|---|---|
| max_corresp_dist | 0.3 m |
| max_rotation_deg | 10° |
| max_iterations | 200 |
| voxel_size | 0.05 m |
| rel_fitness / rel_rmse | 1e-6 |

**Calibrated result — rslidarback → base_link:**

```
calibrated xyz = -1.2639  0.5497  0.2476   [m]
calibrated rpy = -1.222°  -0.359°  135.626°
fitness:         0.3001
inlier_rmse:     0.1229 m
```

**Transformation matrix:**
```
[[-0.71478  -0.69927  -0.01044  -1.26387]
 [ 0.69932  -0.71454  -0.01963   0.54968]
 [ 0.00626  -0.02133   0.99975   0.24760]
 [ 0.        0.        0.        1.     ]]
```

**Delta vs bag tf_static reference:**

| | Calibrated | tf_static | Delta |
|---|---|---|---|
| x | -1.264 m | -1.312 m | +48 mm |
| y | +0.550 m | +0.450 m | +100 mm |
| z | +0.248 m | +0.345 m | -97 mm |
| roll | -1.22° | 0° | -1.22° |
| pitch | -0.36° | 0° | -0.36° |
| yaw | 135.63° | 135.07° | +0.56° |

Notes:
- Yaw error is **0.56°** — well within mounting tolerance
- Pitch error is **0.36°** — good
- Y and Z translation offsets (~100 mm) are consistent across runs and may reflect real mounting offset vs the URDF reference
- Rotation sanity cap (`max_rotation_deg: 10°`) did **not** trigger — GICP converged without a yaw flip

---

## Output files

| File | Description |
|---|---|
| `output/results.txt` | Full calibration log for all runs |
| `output/stitched_initial.pcd` | Merged point cloud using TF initial guess (before GICP) |
| `output/stitched_transformed.pcd` | Merged point cloud after GICP calibration |

View PCD files in CloudCompare or with:
```bash
python3 -c "import open3d as o3d; o3d.visualization.draw_geometries([o3d.io.read_point_cloud('output/stitched_transformed.pcd')])"
```
