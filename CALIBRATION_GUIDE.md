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

### 2. Materialize the params for the machine

Params live in the amr-versioning-system layer, not in this package:

- `config/base/sensors/multi_lidar_calibration.yaml` — shared algorithm tuning + I/O conventions.
- `config/customers/<customer>/<location>/<machine>/sensors/` — per-machine topic lists, `<frame_id>_joint` mappings, tuning overrides. Deep-merged over base by `apply.sh` into `config/current/sensors/`.

```bash
cd amr-versioning-system && ./utils/apply.sh <customer> <location> <machine>
```

**Priors come from the URDF, not from config.** With `read_tf_from_urdf: true` (the base default) the node reads each `<frame_id>`'s joint origin from `urdf_source_path` — `urdf/current/mecanum_bot.urdf`, which `apply.sh` generates from the machine's tracked xacro — then copies that file to the scratch `urdf_path` and writes refined origins **only there**. The xacro stays the single source of truth.

Because GICP has no coarse init, priors must be within ~10°. If a sensor was physically remounted, fix the machine's xacro first (eyeball against bag clouds with the standalone viewer in `tools/`), re-run `apply.sh`, then calibrate.

`<frame_id>` keys must match what the driver publishes in `PointCloud2.header.frame_id`; `<frame_id>_joint` params map them to URDF joint names when they differ from `joint_<frame_id>` (see the p03v2 top-pair file).

A manual `[x,y,z,roll,pitch,yaw]` table is still supported for experiments: set `read_tf_from_urdf: false`, `read_tf_from_table: true`, and per-frame `<frame_id>:` arrays (degrees when `table_degrees: true` — URDF rpy is radians, convert).

### 3. Run

```bash
ros2 bag play /path/to/bag_dir --loop &
ros2 launch multi_lidar_calibrator calibration.launch.py
# parameter_file:= defaults to amr-versioning-system/config/current/sensors/multi_lidar_calibration.yaml
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

**Before calibrating, make sure `urdf/current/` belongs to the machine you're calibrating** — check `AMR_MACHINE` in `config/current/.env` and re-run `apply.sh` if not. Never hand-edit `urdf/current/` or point the node's write-back (`urdf_path`) at it; it gets regenerated on every apply and edits are silently lost.

---

## Known Limitations

**No coarse registration**: `method = 'TF_ONLY'` is hardcoded — priors must be within ~10°/tens of cm. If a sensor was remounted with an unknown pose, eyeball a prior with the `tools/` viewer first.

**Left/right horizontals on the 4-LiDAR ring**: near-zero overlap with front means GICP can't calibrate them. Keep the hand-measured URDF values.

**GICP local minima over flat scenes**: ground-dominant geometry gives high fitness while drifting on feature-sparse axes. Mitigate with `remove_ground_flag: true` or a smaller `voxel_size`, and run the degeneracy probe.

**TEASER++ numpy ABI**: if the node segfaults at import after a numpy upgrade, rebuild `teaserpp_python`.
