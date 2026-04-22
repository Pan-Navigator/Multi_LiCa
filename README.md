# multi_lidar_calibrator

Multi-LiDAR extrinsic calibration for the mecanum AMR platform. Refines sensor poses in a URDF against live topics or a recorded bag using GICP with TEASER++ feature matching.

Originally forked from [TUM-AVS/Multi_LiCa](https://github.com/TUM-AVS/Multi_LiCa) — the upstream algorithm stays; the wrapping workflow (URDF-driven initial guesses, preset YAMLs, amr-versioning-system write-back) is specific to this repo.

## Quick start

```bash
# One preset CLI, URDF is the single source of truth for initial guesses
calibrate_live --preset horizontal_pair                         # live
calibrate_live --preset top_pair --bag /path/to/bag_dir         # offline
```

Results land in `/tmp/multi_lidar_calib_<preset>_<timestamp>/`; the script prompts before writing the refined URDF back into `amr-versioning-system/urdf/current/` and the robot-specific customer path.

Full docs → [CALIBRATION_GUIDE.md](CALIBRATION_GUIDE.md).

## Presets

| Preset | Sensors | Notes |
|---|---|---|
| `horizontal_pair` | front + back | Only pair with enough overlap for automatic GICP |
| `horizontal_ring` | front + back + left + right | Diagnostic; left/right typically don't auto-cal |
| `top_pair` | front_top + rear_top | P3 top-facing mast LiDARs |

Add your own: drop a YAML in [config/presets/](config/presets/) and rebuild.

## License

LGPL-3.0 (inherited from upstream). See [LICENSE.md](LICENSE.md).
