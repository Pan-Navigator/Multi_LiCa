# LiDAR Calibration — Run Guide

Calibrates the mecanum AMR's LiDARs to `base_link` using GICP. One entry point: the `multi_lidar_calibrator` node, launched via `calibration.launch.py` with a flat ROS-params YAML.

> GICP refines **purely from the initial transforms you supply** — the coarse-registration code paths (TEASER++/FPFH/RANSAC) exist in `Calibration.py` but are disabled (`method = 'TF_ONLY'`). Priors more than ~10° off will be rejected by `max_rotation_deg` or converge to a wrong local minimum. Getting the priors right is most of the job.

---

## Prerequisites

- ROS 2 Humble, workspace built with `multi_lidar_calibrator`.
- `ros-humble-rosbag2-storage-mcap` for bag playback.
- Python deps: `open3d`, `numpy==1.24.2`, `scipy==1.11.4`, `ros2_numpy`, `pyyaml`.
- `teaserpp_python` built from the vendored submodule (still imported at module load even though coarse init is disabled — see below).

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
colcon build --packages-select multi_lidar_calibrator --symlink-install
source install/setup.bash
```

---

## The Workflow

### 1. Record or pick a bag

A short **stationary** recording (~5–10 s) containing the pair's PointCloud2 topics is enough. Verify before running:

```bash
ros2 bag info /path/to/bag_dir   # topics must match the params YAML exactly
```

(Live calibration works the same way — skip the bag and make sure the drivers are publishing.)

### 2. Author the params YAML

Copy a template from `config/ros_params/` and adjust. The critical part is the initial-transform table:

- **Take priors from the robot's live URDF, not a possibly-stale repo xacro.** On p01 the repo xacro was ~20° off in yaw and calibration failed (fitness 0.11) until the priors were corrected.
- With `table_degrees: true` the table is `[x, y, z, roll_deg, pitch_deg, yaw_deg]` — URDF rpy is **radians**, convert.
- `urdf_path:` must point at a **scratch copy** of the URDF — the node rewrites the matching joint origins in that file in place.
- For eyeballing priors against the bag clouds, use the standalone viewer in `tools/`.

Key params (validated values from the p01 front/back run):

```yaml
/**:
  ros__parameters:
    read_pcds_from_file: false     # subscribe to topics
    read_tf_from_table: true       # priors come from the table below
    table_degrees: true
    frame_count: 3                 # frames accumulated per lidar
    lidar_topics: [/lidar/front/rslidar_points, /lidar/back/rslidar_points]
    target_frame_id: rslidarfront  # held fixed; only the OTHER joints get refined
    base_frame_id: base_link
    calibrate_to_base: true
    calibrate_target: false
    urdf_path: /tmp/scratch/mecanum_bot_copy.xacro
    output_dir: /tmp/calib_output/
    max_corresp_dist: 0.3          # GICP correspondence limit (m)
    max_rotation_deg: 10.0         # reject GICP results rotating more than this
    voxel_size: 0.05
    fitness_score_threshold: 0.15  # 0.3 is unreachable for low-overlap pairs; see below
    base_to_ground_z: 0.17864325963808905

    rslidarfront: [1.312, -0.45, 0.345, 0.0, 0.0, -23.282]   # degrees!
    rslidarback:  [-1.264, 0.610, 0.167, -1.455, 0.018, 119.58]
    rslidarfront_joint: joint_rslidarfront
    rslidarback_joint:  joint_rslidarback
```

`<frame_id>` keys must match what the driver publishes in `PointCloud2.header.frame_id`; the `<frame_id>_joint` keys map them to URDF joint names.

### 3. Run

```bash
ros2 bag play /path/to/bag_dir --loop &
ros2 launch multi_lidar_calibrator calibration.launch.py \
  parameter_file:=/abs/path/to/params.yaml
# the node exits by itself when done; kill the bag play afterwards
```

---

## Quality gates — before touching any repo URDF

Each run writes to `output_dir`: `results.txt`, `stitched_initial.pcd`, `stitched_transformed.pcd`, plus the refined origins in the scratch `urdf_path` file.

| Metric | Good | Acceptable | Poor |
|---|---|---|---|
| fitness | > 0.5 | 0.3 – 0.5 | < 0.3 |
| inlier_rmse | < 0.05 m | 0.05 – 0.15 m | > 0.15 m |

Fitness is the fraction of source points with a correspondence within `max_corresp_dist` — for the front↔back pair (~20 % FOV overlap) **~0.2 is structural, not misalignment**. When fitness is band-poor, judge by these instead:

1. **Stitched-cloud improvement**: median nearest-neighbour distance between the two sensors' points must clearly drop from `stitched_initial.pcd` to `stitched_transformed.pcd` (p01: 150 → 98 mm).
2. **Degeneracy probe**: perturb the solution ±5/±15 cm per axis and ±1/±3° per rotation and re-evaluate. On p01, rotation was constrained to ~±1° but translation only to ~±5 cm — treat cm-level translation output with caution, especially along corridor axes.
3. **Two-prior consistency**: re-run from a second plausible prior; solutions should agree (p01: 17 mm / 0.3°).

High fitness can still hide drift on one axis when the scene is ground- or wall-dominant — always check the stitched cloud (CloudCompare, or `o3d.visualization.draw_geometries`).

---

## Persisting Results

Nothing is written to the repo automatically. Once the gates pass:

1. Copy the refined joint origin(s) from the scratch URDF into the machine's tracked xacro: `amr-versioning-system/urdf/customers/<customer>/<location>/<machine>/mecanum_bot.xacro`.
2. Make sure the **target** sensor's origin in that xacro equals the prior you held it at — the refined joints are relative to it.
3. Commit on a branch in the `amr-versioning-system` submodule; `apply.sh` materialises it to `urdf/current/`.

**Never source priors from or write results to `urdf/current/mecanum_bot.urdf` directly** — it may belong to a different machine (check `AMR_MACHINE` in `config/current/.env`) or contain a bad earlier write-back.

---

## Known Limitations

**No coarse registration**: `method = 'TF_ONLY'` is hardcoded — priors must be within ~10°/tens of cm. If a sensor was remounted with an unknown pose, eyeball a prior with the `tools/` viewer first.

**Left/right horizontals on the 4-LiDAR ring**: near-zero overlap with front means GICP can't calibrate them. Keep the hand-measured URDF values.

**GICP local minima over flat scenes**: ground-dominant geometry gives high fitness while drifting on feature-sparse axes. Mitigate with `remove_ground_flag: true` or a smaller `voxel_size`, and run the degeneracy probe.

**TEASER++ numpy ABI**: if the node segfaults at import after a numpy upgrade, rebuild `teaserpp_python`.
