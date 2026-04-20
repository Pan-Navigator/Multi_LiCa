# Multi-LiDAR Calibration — Run Guide

Calibrates the mecanum robot's RoboSense LiDARs to `base_link` using GICP, with TEASER++ feature matching for coarse alignment.

Two recommended workflows:

- **Live (interactive)**: `calibrate_live` CLI reads initial guesses from the active URDF, runs calibration against live sensor topics, and writes the refined transforms back into the URDF after confirmation.
- **Offline (bag + PCD)**: record a bag, extract PCDs, run the calibrator node against the saved PCDs. Useful for iterating on parameters without tying up the robot.

---

## Prerequisites

- ROS 2 Humble installed and sourced
- `ros-humble-rosbag2-storage-mcap` for MCAP bag playback
- Python packages: `open3d`, `numpy 1.24.x`, `scipy <1.14`, `ros2_numpy`
- `teaserpp_python` built from source (see below)

### Python package pinning

Open3D's aarch64 wheel pins numpy to 1.24.2, so scipy must be held back to a version that supports numpy 1.x:

```bash
pip install "numpy==1.24.2" "scipy==1.11.4" --force-reinstall
pip install open3d ros2_numpy
```

### Building TEASER++

The Python bindings are not on PyPI — build from the submodule with tests disabled (the test suite has build errors on current compilers that don't affect the library):

```bash
cd utilities/Multi_LiCa/TEASER-plusplus
mkdir -p build && cd build
cmake .. -DTEASERPP_PYTHON_VERSION=3.10 -DBUILD_TEASER_FPFH=ON -DBUILD_TESTS=OFF
make -j$(nproc) teaserpp_python
cp python/teaserpp_python/teaserpp_python.cpython-310-*.so \
   $(python3 -c "import site; print(site.getsitepackages()[0])")/
python3 -c "import teaserpp_python; print('ok')"
```

Rebuild whenever the installed numpy version changes (ABI incompatibility causes segfaults).

### Workspace build

```bash
cd <workspace>
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-up-to multi_lidar_calibrator
source install/setup.bash
```

---

## Workflow A — Live calibration (`calibrate_live`)

The recommended workflow. Reads initial guesses from the active URDF (via the versioning system), runs the calibrator against live lidar topics, and writes results back to both `urdf/current/` and the robot-specific customer URDF.

```bash
calibrate_live                     # defaults: 1 frame per LiDAR, auto-detect URDF
calibrate_live --frames 3          # accumulate 3 frames per LiDAR (denser cloud)
calibrate_live --urdf /path.urdf   # override URDF path
```

Requires `AMR_CUSTOMER`, `AMR_LOCATION`, `AMR_MACHINE` env vars (or the `.env` file written by `apply.sh`) to locate the robot-specific write target. Without them, only `urdf/current/` is updated — the change is lost on next `apply.sh` run.

After the run, the script prints per-LiDAR fitness and xyz/rpy deltas, then prompts `[y/N]` before writing. Say N if any fitness is below your threshold or a transform looks wrong.

---

## Workflow B — Offline calibration from a bag

Use when you want to iterate on parameters without tying up the robot.

### Step 1 — Record a bag

From the robot (with lidars publishing):

```bash
ros2 bag record -s mcap -o calibrate_four_lidars_$(date +%Y%m%d_%H%M%S) \
  /lidar/front/rslidar_points \
  /lidar/back/rslidar_points \
  /lidar/left/rslidar_points \
  /lidar/right/rslidar_points \
  /tf_static /tf
```

10–20 seconds is enough for a static scene.

### Step 2 — Extract PCDs

**Terminal 1 — play the bag on loop:**
```bash
ros2 bag play <bag_dir> --loop
```

**Terminal 2 — run the extractor:**
```bash
source install/setup.bash
python3 src/mecanum-robot-ros2/utilities/Multi_LiCa/scripts/extract_pcds.py
```

Captures 10 frames per LiDAR within a 10-second window, merges them per LiDAR, saves to `utilities/Multi_LiCa/data/pcds_four/`.

Stop the bag (Ctrl+C in terminal 1) once the extractor exits.

### Step 3 — Run calibration

The package ships two configs you'll typically edit:

| Config | Lidars | When to use |
|---|---|---|
| `config/params.yaml` | front + back | Recommended for auto-calibration — only these two have enough overlap for GICP |
| `config/params_four.yaml` | front + back + left + right | Diagnostic runs; left/right won't auto-calibrate (see below) |

Key parameters to review before running:

```yaml
visualize: false                        # true requires X11; false for headless/robot
read_pcds_from_file: true
pcd_directory: /../data/pcds_four/      # relative to multi_lidar_calibrator.py dir
output_dir: /../output/four/

target_frame_id: rslidarfront
base_frame_id: base_link
calibrate_to_base: true

# Initial guess per lidar: [x, y, z, roll_deg, pitch_deg, yaw_deg]
rslidarfront: [1.3120, -0.4400, 0.3450, 0.0, 0.0, -50.0]
rslidarback:  [-1.2639, 0.5497, 0.2476, -1.222, -0.359, 135.647]

max_corresp_dist: 0.3                   # max GICP point-pair distance [m]
max_rotation_deg: 10.0                  # safety cap; if GICP exceeds this, falls back to TF-only
fitness_score_threshold: 0.3            # pairs below this are rejected (0.15 acceptable for low overlap)
voxel_size: 0.05                        # downsampling [m]
use_fitness_based_calibration: false    # true picks best pair chain automatically
```

Then launch:

```bash
ros2 launch multi_lidar_calibrator calibration.launch.py \
  parameter_file:=src/mecanum-robot-ros2/utilities/Multi_LiCa/config/params.yaml
```

### Step 4 — Read the results

Results land in `utilities/Multi_LiCa/output/` (or `output/four/` for the 4-lidar config):

| File | Description |
|---|---|
| `results.txt` / `results_four.txt` | Per-run calibration log (xyz, rpy, fitness, rmse) |
| `stitched_initial.pcd` | Merged cloud using the YAML initial guesses (pre-GICP) |
| `stitched_transformed.pcd` | Merged cloud after GICP refinement |

View the stitched PCDs in CloudCompare or:

```bash
python3 -c "import open3d as o3d; o3d.visualization.draw_geometries([o3d.io.read_point_cloud('output/stitched_transformed.pcd')])"
```

Quality bands (empirical):

| Metric | Good | Acceptable | Poor |
|---|---|---|---|
| fitness | > 0.5 | 0.3 – 0.5 | < 0.3 |
| inlier_rmse | < 0.05 m | 0.05 – 0.15 m | > 0.15 m |

---

## Known limitations

### Left/right LiDARs can't auto-calibrate to front

On the 4-LiDAR mecanum platform, left/right sensors face roughly 90° away from front/back and have near-zero overlap with front. GICP reports fitness below the threshold regardless of the initial guess quality. The `stitched_initial.pcd` will usually look visually well-aligned (because the URDF values are hand-measured and correct) — but the calibrator can't *verify* that automatically.

Consequence: `calibrate_live` with all 4 LiDARs will report left/right as "failed" even when the URDF is correct. Use Workflow B with `config/params.yaml` (front+back only) for automatic GICP refinement, and keep the hand-measured URDF values for left/right.

### GICP rotation cap

`max_rotation_deg` (default 10°) protects against GICP converging to a wrong local minimum when overlap is marginal. If GICP requests a larger delta, the calibrator logs a warning and falls back to the TF-only (initial guess) transform. Raising the cap above ~15° usually produces garbage on this sensor layout — tune down rather than up.

### Headless environments

Set `visualize: false` in the YAML when running on the robot (no X11). Otherwise Open3D crashes with a GLFW error during the initial cloud preview.

---

## Reference run — 2026-04-20

Bag: `calibrate_four_lidars_20260420_092640`
Config: `config/params.yaml` (front + back)
Platform: Rijk Zwaan Kwintsheul harvester_001

**Result — rslidarback → rslidarfront:**

```
calibrated xyz = -1.2128  0.6849  0.2737   [m]
calibrated rpy = -1.152°  -0.386°  132.367°
fitness:         0.3004
inlier_rmse:     0.1233 m
```

**Delta vs pre-run URDF (`-1.2399, 0.5189, 0.2321 / rpy -1.764°, 0.458°, 135.43°`):**

| | Calibrated | Pre-URDF | Delta |
|---|---|---|---|
| x | -1.213 m | -1.240 m | +27 mm |
| y | +0.685 m | +0.519 m | +166 mm |
| z | +0.274 m | +0.232 m | +42 mm |
| roll | -1.15° | -1.76° | +0.61° |
| pitch | -0.39° | +0.46° | -0.85° |
| yaw | 132.37° | 135.43° | -3.06° |

Applied to `urdf/customers/rijkzwaan/kwintsheul/harvester_001/mecanum_bot.xacro`.
