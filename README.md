# multi_lidar_calibrator

Multi-LiDAR extrinsic calibration for the mecanum AMR platform. Refines sensor poses in a URDF against live topics or a recorded bag using GICP.

Originally forked from [TUM-AVS/Multi_LiCa](https://github.com/TUM-AVS/Multi_LiCa) — the upstream algorithm stays; the URDF write-back and the flat ROS-params workflow are specific to this repo.

## Quick start

```bash
# Params come from the amr-versioning-system layer; materialize for the active machine:
cd ../../amr-versioning-system && ./utils/apply.sh   # or pass <customer> <location> <machine>
ros2 bag play /path/to/bag_dir --loop &
ros2 launch multi_lidar_calibrator calibration.launch.py
# parameter_file:= defaults to amr-versioning-system/config/current/sensors/multi_lidar_calibration.yaml
```

The node accumulates `frame_count` frames per topic, registers each source LiDAR to the target with GICP, writes `results.txt` + stitched point clouds to `output_dir`, and rewrites the joint origins in the file given by `urdf_path` — point that at a scratch copy, never a repo URDF.

Full docs → [CALIBRATION_GUIDE.md](CALIBRATION_GUIDE.md).

## Params files

Calibration params follow the amr-versioning-system base/customers/current pattern:

- `amr-versioning-system/config/base/sensors/multi_lidar_calibration.yaml` — shared algorithm tuning and I/O conventions.
- `amr-versioning-system/config/customers/<customer>/<location>/<machine>/sensors/` — per-machine topic lists, `<frame_id>_joint` mappings, and tuning overrides (deep-merged over base by `apply.sh`).

Priors are **not** duplicated in config: with `read_tf_from_urdf: true` the node seeds them from the joint origins in `urdf_source_path` (`urdf/current/mecanum_bot.urdf`, machine-correct after `apply.sh`) and copies that file to the scratch `urdf_path` for write-back. GICP has no coarse init and needs priors within ~10° — if a sensor was remounted, fix the xacro first (eyeball with the `tools/` viewer).

| Pair | Notes |
|---|---|
| front + back | Only horizontal pair with enough overlap for automatic GICP |
| front_top + rear_top | P3 top-facing mast LiDARs |
| left / right | Near-zero overlap — keep hand-measured origins |

For eyeballing priors against bag clouds, use the standalone viewer in [tools/](tools/).

## License

LGPL-3.0 (inherited from upstream). See [LICENSE.md](LICENSE.md).
