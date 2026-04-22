# LiDAR Calibration — Run Guide

Calibrates the mecanum AMR's LiDARs to `base_link` using GICP (and TEASER++ for coarse feature matching). Single entry point: `calibrate_live`, driven by a preset YAML plus the active URDF.

---

## Prerequisites

- ROS 2 Humble, workspace built with `multi_lidar_calibrator`.
- `ros-humble-rosbag2-storage-mcap` for bag playback.
- Python deps: `open3d`, `numpy==1.24.2`, `scipy==1.11.4`, `ros2_numpy`, `pyyaml`.
- `teaserpp_python` built from the vendored submodule (see below).

### Build TEASER++ (one-time)

```bash
cd utilities/Multi_LiCa/TEASER-plusplus
mkdir -p build && cd build
cmake .. -DTEASERPP_PYTHON_VERSION=3.10 -DBUILD_TEASER_FPFH=ON -DBUILD_TESTS=OFF
make -j$(nproc) teaserpp_python
cp python/teaserpp_python/teaserpp_python.cpython-310-*.so \
   $(python3 -c "import site; print(site.getsitepackages()[0])")/
python3 -c "import teaserpp_python; print('ok')"
```

Rebuild if the installed numpy version changes (ABI mismatch → segfaults).

### Build the workspace

```bash
cd <workspace>
source /opt/ros/humble/setup.bash
colcon build --packages-up-to multi_lidar_calibrator
source install/setup.bash
```

---

## The Workflow

```bash
# Live (robot connected, LiDARs publishing)
calibrate_live --preset horizontal_pair

# Offline (from a recorded bag)
calibrate_live --preset top_pair --bag /path/to/bag_dir

# Accumulate multiple frames for denser clouds
calibrate_live --preset horizontal_pair --frames 3

# Explicit URDF override
calibrate_live --preset horizontal_pair --urdf /path/to/mecanum_bot.urdf
```

Under the hood:

1. Loads the preset YAML (sensor list, target, thresholds).
2. Finds the active URDF — `<workspace>/src/mecanum-robot-ros2/amr-versioning-system/urdf/current/mecanum_bot.urdf` — and reads initial `xyz`/`rpy` for each preset joint.
3. Generates a flat ROS-params YAML in a timestamped `/tmp/` directory.
4. Optionally spawns `ros2 bag play --loop` for the duration of the run.
5. Launches `calibration.launch.py`; the node subscribes to each sensor's topic, accumulates the requested frames (NaN-filtered), runs GICP (with TEASER++ coarse matching), and writes refined transforms back into a temporary URDF.
6. Prints per-sensor deltas (mm / deg) vs the initial URDF.
7. Prompts `[y/N]` before copying the refined URDF over `urdf/current/` and the robot-specific `urdf/customers/<customer>/<location>/<machine>/mecanum_bot.urdf`.

---

## Presets

Preset YAMLs live in `config/presets/`. Each describes a **calibration scenario** — which sensors participate, which is the reference, and what thresholds are appropriate.

### Shipped presets

| File | Sensors | When to use |
|---|---|---|
| `horizontal_pair.yaml` | front + back | The only auto-cal-viable subset of the horizontal ring — enough overlap for GICP. Recommended default. |
| `horizontal_ring.yaml` | front + back + left + right | Diagnostic. Left/right have near-zero overlap with front — they usually fall back to URDF values. |
| `top_pair.yaml` | front_top + rear_top (P3) | Top-facing LiDARs that see the canopy. Relaxed `max_rotation_deg` because initial guesses can be off by tens of degrees. |

### Schema

```yaml
preset: my_scenario           # free-form name, shown in logs
sensors:
  - name: front               # short id used internally + in reports
    topic: /lidar/front/rslidar_points
    frame_id: rslidarfront    # what the driver publishes in PointCloud2.header
    joint: joint_rslidarfront # URDF joint to read initial xyz/rpy from
  - name: back
    topic: /lidar/back/rslidar_points
    frame_id: rslidarback
    joint: joint_rslidarback
target: front                 # reference sensor (not refined)

calibration:
  fitness_score_threshold: 0.3
  max_corresp_dist: 0.3       # GICP point-pair distance limit (m)
  max_rotation_deg: 10.0      # GICP rotation safety cap; falls back to URDF if exceeded
  voxel_size: 0.05            # downsample before GICP (m)
  remove_ground_flag: false   # strip ground plane before GICP (useful on open fields)
  calibrate_to_base: true     # RANSAC ground-fit to set z
  calibrate_target: false     # leave target frame fixed (recommended)
  use_fitness_based_calibration: false  # auto-pick best pair chain (slow but robust)
  runs_count: 1               # iterate N times for testing
  base_to_ground_z: 0.178     # base_link height above ground (m)
```

`frame_id` must match what the driver *actually publishes* — decoupled from `joint` so a driver publishing `rslidarfronttop` can still map to a URDF joint named `joint_rslidarfront_top`.

Add a new preset by dropping a YAML in `config/presets/` and rebuilding the workspace (the new YAML gets installed to `share/multi_lidar_calibrator/config/presets/`).

---

## Reading the Output

Each run writes to `/tmp/multi_lidar_calib_<preset>_<timestamp>/`:

| File | Description |
|---|---|
| `live_params.yaml` | Generated ROS params — the actual input to the node |
| `mecanum_bot_calibrated.urdf` | Temp URDF the node writes refined joints into |
| `results.txt` | Per-run calibration log (xyz, rpy, fitness, rmse) |
| `stitched_initial.pcd` | Merged cloud using URDF initial guesses (pre-GICP) |
| `stitched_transformed.pcd` | Merged cloud after refinement |

Quality bands (empirical):

| Metric | Good | Acceptable | Poor |
|---|---|---|---|
| fitness | > 0.5 | 0.3 – 0.5 | < 0.3 |
| inlier_rmse | < 0.05 m | 0.05 – 0.15 m | > 0.15 m |

View in CloudCompare, or:

```bash
python3 -c "import open3d as o3d; o3d.visualization.draw_geometries([o3d.io.read_point_cloud('/tmp/.../stitched_transformed.pcd')])"
```

High fitness can still hide drift on a single axis if the dominant geometry is a ground plane — always sanity-check the stitched cloud visually before accepting.

---

## Persisting Results

`calibrate_live` writes to two paths on confirmation:

- `urdf/current/mecanum_bot.urdf` — immediately active.
- `urdf/customers/<customer>/<location>/<machine>/mecanum_bot.urdf` — the tracked source of truth, so the next `apply.sh` doesn't revert.

The robot version is resolved from `AMR_CUSTOMER`, `AMR_LOCATION`, `AMR_MACHINE` env vars or from `config/current/.env` (written by `apply.sh`). If none are set, only `urdf/current/` is updated — commit the output manually or the change is lost on next apply.

Once happy, commit in the `amr-versioning-system` submodule.

---

## Known Limitations

**Left/right horizontals on the 4-LiDAR ring**: near-zero overlap with front means GICP usually reports `< 0.05` fitness regardless of initial guess. The `stitched_initial.pcd` looks aligned (because URDF values are typically hand-measured correctly), but the calibrator can't *prove* it automatically. Expected; keep the hand-measured URDF.

**GICP local minima over flat fields**: ground-dominant scenes can give high fitness while drifting on axes where features are sparse. Mitigate with `remove_ground_flag: true` or by reducing `voxel_size` to retain more structure.

**TEASER++ numpy ABI**: if calibration segfaults at startup after a numpy upgrade, rebuild `teaserpp_python` against the new version.
