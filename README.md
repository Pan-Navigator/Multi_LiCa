# multi_lidar_calibrator

Multi-LiDAR extrinsic calibration for the mecanum AMR platform. Refines sensor poses in a URDF against live topics or a recorded bag using GICP.

Originally forked from [TUM-AVS/Multi_LiCa](https://github.com/TUM-AVS/Multi_LiCa) — the upstream algorithm stays; the URDF write-back and the flat ROS-params workflow are specific to this repo.

## Quick start

```bash
# Author a params YAML from a template in config/ros_params/, then:
ros2 bag play /path/to/bag_dir --loop &
ros2 launch multi_lidar_calibrator calibration.launch.py \
  parameter_file:=/abs/path/to/params.yaml
```

The node accumulates `frame_count` frames per topic, registers each source LiDAR to the target with GICP, writes `results.txt` + stitched point clouds to `output_dir`, and rewrites the joint origins in the file given by `urdf_path` — point that at a scratch copy, never a repo URDF.

Full docs → [CALIBRATION_GUIDE.md](CALIBRATION_GUIDE.md).

## Params files

Per-robot flat ROS-params YAMLs live in [config/ros_params/](config/ros_params/). Initial transforms are `[x, y, z, roll, pitch, yaw]` per sensor frame — degrees when `table_degrees: true`. Take them from the robot's live URDF; GICP has no coarse init and needs priors within ~10°.

| Pair | Notes |
|---|---|
| front + back | Only horizontal pair with enough overlap for automatic GICP |
| front_top + rear_top | P3 top-facing mast LiDARs |
| left / right | Near-zero overlap — keep hand-measured origins |

For eyeballing priors against bag clouds, use the standalone viewer in [tools/](tools/).

## License

LGPL-3.0 (inherited from upstream). See [LICENSE.md](LICENSE.md).
