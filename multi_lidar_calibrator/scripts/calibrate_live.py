#!/usr/bin/env python3
"""Preset-driven LiDAR calibration for the mecanum AMR platform.

Reads initial transforms from the versioning system URDF, runs calibration
against live topics (or a replayed bag), and writes refined transforms back
into the URDF after confirmation.

Usage:
    calibrate_live --preset horizontal_pair
    calibrate_live --preset top_pair --bag /path/to/bag_dir
    calibrate_live --preset-file /abs/path/to/custom.yaml --frames 3
"""
import argparse
import math
import os
import pathlib
import shutil
import signal
import subprocess
import sys
import time
import xml.etree.ElementTree as ET
from datetime import datetime

import yaml
from ament_index_python.packages import get_package_prefix, get_package_share_directory


def _versioning_system_root() -> pathlib.Path:
    """Locate amr-versioning-system in the workspace by walking up from install/."""
    start = pathlib.Path(get_package_prefix("multi_lidar_calibrator"))
    for parent in start.parents:
        candidate = parent / "src/mecanum-robot-ros2/amr-versioning-system"
        if candidate.is_dir():
            return candidate
    raise RuntimeError(
        "amr-versioning-system not found in any parent workspace.\n"
        "Pass --urdf /path/to/mecanum_bot.urdf explicitly."
    )


def _read_robot_version(vs_root: pathlib.Path) -> tuple:
    """Resolve AMR_CUSTOMER/LOCATION/MACHINE from env or apply.sh's .env."""
    c = os.environ.get("AMR_CUSTOMER")
    l = os.environ.get("AMR_LOCATION")
    m = os.environ.get("AMR_MACHINE")
    if c and l and m:
        return c, l, m
    env_file = vs_root / "config" / "current" / ".env"
    if env_file.exists():
        env = {}
        for line in env_file.read_text().splitlines():
            line = line.strip()
            if "=" in line and not line.startswith("#"):
                k, _, v = line.partition("=")
                env[k.strip()] = v.strip()
        return env.get("AMR_CUSTOMER"), env.get("AMR_LOCATION"), env.get("AMR_MACHINE")
    return None, None, None


def find_urdf_paths(override: str = None) -> tuple:
    """Return (read_path, write_path). write_path is None if robot version is unknown."""
    if override:
        p = pathlib.Path(override)
        if not p.exists():
            raise FileNotFoundError(f"URDF not found: {override}")
        return p, p
    vs_root = _versioning_system_root()
    read_path = vs_root / "urdf" / "current" / "mecanum_bot.urdf"
    if not read_path.exists():
        raise RuntimeError(
            f"Active URDF not found: {read_path}\n"
            "Run apply.sh first or pass --urdf explicitly."
        )
    customer, location, machine = _read_robot_version(vs_root)
    write_path = None
    if customer and location and machine:
        robot_dir = vs_root / "urdf" / "customers" / customer / location / machine
        if robot_dir.is_dir():
            write_path = robot_dir / "mecanum_bot.urdf"
    return read_path, write_path


def load_preset(name_or_path: str) -> tuple:
    """Load preset YAML by bare name or absolute path. Returns (dict, resolved_path)."""
    p = pathlib.Path(name_or_path)
    if p.is_absolute() and p.exists():
        path = p
    else:
        share = pathlib.Path(get_package_share_directory("multi_lidar_calibrator"))
        path = share / "config" / "presets" / f"{name_or_path}.yaml"
        if not path.exists():
            raise FileNotFoundError(f"Preset '{name_or_path}' not found at {path}")
    with open(path) as f:
        preset = yaml.safe_load(f)
    _validate_preset(preset, path)
    return preset, path


def _validate_preset(preset: dict, path: pathlib.Path) -> None:
    required_top = ["sensors", "target"]
    for k in required_top:
        if k not in preset:
            raise ValueError(f"Preset {path} missing key: {k}")
    if not preset["sensors"]:
        raise ValueError(f"Preset {path} has no sensors")
    for s in preset["sensors"]:
        for k in ("name", "topic", "frame_id", "joint"):
            if k not in s:
                raise ValueError(f"Preset {path} sensor missing '{k}': {s}")
    names = [s["name"] for s in preset["sensors"]]
    if preset["target"] not in names:
        raise ValueError(
            f"Preset {path} target '{preset['target']}' not in sensors {names}"
        )


def parse_urdf_transforms(urdf_path: pathlib.Path, joints: list) -> dict:
    """Extract xyz + rpy (degrees) for the given joint names from the URDF."""
    tree = ET.parse(urdf_path)
    root = tree.getroot()
    found = {}
    for joint in root.iter("joint"):
        jname = joint.attrib.get("name", "")
        if jname not in joints:
            continue
        origin = joint.find("origin")
        if origin is None:
            continue
        try:
            xyz = [float(v) for v in origin.attrib.get("xyz", "0 0 0").split()]
            rpy_rad = [float(v) for v in origin.attrib.get("rpy", "0 0 0").split()]
            found[jname] = xyz + [math.degrees(v) for v in rpy_rad]
        except ValueError as e:
            print(f"Warning: parse error for joint {jname}: {e}")
    missing = sorted(set(joints) - set(found.keys()))
    if missing:
        raise RuntimeError(f"URDF {urdf_path} missing joints: {missing}")
    return found


def build_ros_params(
    preset: dict,
    urdf_transforms: dict,
    temp_urdf: pathlib.Path,
    output_dir: pathlib.Path,
    frames: int,
) -> dict:
    """Translate preset + URDF initial guesses into a flat ROS params YAML dict."""
    sensors = preset["sensors"]
    target_sensor = next(s for s in sensors if s["name"] == preset["target"])
    cal = preset.get("calibration", {})

    lidar_table = {
        s["frame_id"]: urdf_transforms[s["joint"]] for s in sensors
    }
    joint_map = {
        s["frame_id"] + "_joint": s["joint"] for s in sensors
    }

    return {
        "/**": {
            "ros__parameters": {
                "read_pcds_from_file": False,
                "read_tf_from_table": True,
                "table_degrees": True,
                "frame_count": frames,
                "runs_count": cal.get("runs_count", 1),
                "visualize": False,
                "lidar_topics": [s["topic"] for s in sensors],
                "target_frame_id": target_sensor["frame_id"],
                "base_frame_id": "base_link",
                "calibrate_to_base": cal.get("calibrate_to_base", True),
                "calibrate_target": cal.get("calibrate_target", False),
                "use_fitness_based_calibration": cal.get("use_fitness_based_calibration", False),
                "urdf_path": str(temp_urdf),
                "output_dir": str(output_dir) + "/",
                "results_file": "results.txt",
                "max_corresp_dist": cal.get("max_corresp_dist", 0.3),
                "max_rotation_deg": cal.get("max_rotation_deg", 10.0),
                "voxel_size": cal.get("voxel_size", 0.05),
                "remove_ground_flag": cal.get("remove_ground_flag", False),
                "fitness_score_threshold": cal.get("fitness_score_threshold", 0.3),
                "max_iterations": cal.get("max_iterations", 200),
                "rel_fitness": cal.get("rel_fitness", 1.0e-6),
                "rel_rmse": cal.get("rel_rmse", 1.0e-6),
                "epsilon": cal.get("epsilon", 0.0001),
                "distance_threshold": cal.get("distance_threshold", 0.1),
                "ransac_n": cal.get("ransac_n", 10),
                "num_iterations": cal.get("num_iterations", 2000),
                "r_voxel_size": cal.get("r_voxel_size", 0.1),
                "r_runs": cal.get("r_runs", 10),
                "crop_cloud": cal.get("crop_cloud", 25),
                "base_to_ground_z": cal.get("base_to_ground_z", 0.17864325963808905),
                **lidar_table,
                **joint_map,
            }
        }
    }


def play_bag(bag_path: pathlib.Path) -> subprocess.Popen:
    """Spawn ros2 bag play --loop in its own process group for clean termination."""
    return subprocess.Popen(
        ["ros2", "bag", "play", str(bag_path), "--loop"],
        preexec_fn=os.setsid,
        stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL,
    )


def stop_bag(proc: subprocess.Popen) -> None:
    if proc.poll() is None:
        os.killpg(os.getpgid(proc.pid), signal.SIGINT)
        try:
            proc.wait(timeout=5)
        except subprocess.TimeoutExpired:
            os.killpg(os.getpgid(proc.pid), signal.SIGKILL)


def run_calibration(params_file: pathlib.Path) -> int:
    print(f"\n{'='*70}\nLaunching calibrator with {params_file}\n{'='*70}\n")
    return subprocess.run(
        [
            "ros2", "launch", "multi_lidar_calibrator", "calibration.launch.py",
            f"parameter_file:={params_file}",
        ],
        check=False,
    ).returncode


def parse_results(
    temp_urdf: pathlib.Path, preset: dict, read_urdf: pathlib.Path
) -> dict:
    """Extract per-sensor calibrated xyz/rpy_rad from the temp URDF (radians)."""
    tree = ET.parse(temp_urdf)
    root = tree.getroot()
    results = {}
    for s in preset["sensors"]:
        if s["name"] == preset["target"]:
            continue
        joint_name = s["joint"]
        for joint in root.iter("joint"):
            if joint.attrib.get("name") == joint_name:
                origin = joint.find("origin")
                if origin is None:
                    break
                try:
                    xyz = [float(v) for v in origin.attrib.get("xyz", "0 0 0").split()]
                    rpy = [float(v) for v in origin.attrib.get("rpy", "0 0 0").split()]
                    results[s["name"]] = {
                        "joint": joint_name,
                        "xyz": xyz,
                        "rpy_rad": rpy,
                        "frame_id": s["frame_id"],
                    }
                except ValueError:
                    pass
                break
    return results


def print_results(results: dict, preset: dict, urdf_transforms: dict) -> None:
    print(f"\n{'='*100}")
    print(f"Calibration results (target: {preset['target']})")
    print(f"{'='*100}\n")
    if not results:
        print("No results parsed from URDF — calibrator may have failed.")
        return

    print(f"{'Sensor':<14} {'xyz (m)':<30} {'rpy (rad)':<30} {'Δxyz mm':<20} {'Δrpy deg':<20}")
    print("-" * 110)
    for name, r in results.items():
        initial = urdf_transforms[r["joint"]]
        dxyz = [(r["xyz"][i] - initial[i]) * 1000 for i in range(3)]
        drpy = [math.degrees(r["rpy_rad"][i]) - initial[3 + i] for i in range(3)]
        xyz_s = f"{r['xyz'][0]:8.4f} {r['xyz'][1]:8.4f} {r['xyz'][2]:8.4f}"
        rpy_s = f"{r['rpy_rad'][0]:9.5f} {r['rpy_rad'][1]:9.5f} {r['rpy_rad'][2]:9.5f}"
        dxyz_s = f"{dxyz[0]:+6.1f} {dxyz[1]:+6.1f} {dxyz[2]:+6.1f}"
        drpy_s = f"{drpy[0]:+5.2f} {drpy[1]:+5.2f} {drpy[2]:+5.2f}"
        print(f"{name:<14} {xyz_s:<30} {rpy_s:<30} {dxyz_s:<20} {drpy_s:<20}")
    print(f"{'='*110}\n")


def confirm_apply(read_urdf: pathlib.Path, write_urdf: pathlib.Path) -> bool:
    print("Apply these transforms to URDF?")
    print(f"  Read: {read_urdf}")
    if write_urdf and write_urdf != read_urdf:
        print(f"  Write: {write_urdf} (robot-specific)")
    try:
        while True:
            r = input("[y/N] ").strip().lower()
            if r in ("y", "yes"):
                return True
            if r in ("n", "no", ""):
                return False
            print("Enter 'y' or 'n'.")
    except EOFError:
        print()
        return False


def main(args=None):
    parser = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    src = parser.add_mutually_exclusive_group(required=True)
    src.add_argument("--preset", help="Preset name (from config/presets/<name>.yaml)")
    src.add_argument("--preset-file", help="Absolute path to a preset YAML")
    parser.add_argument("--urdf", help="Override URDF path (auto-detected otherwise)")
    parser.add_argument("--bag", help="Play this bag in the background during calibration")
    parser.add_argument("--frames", type=int, default=1, help="Frames per lidar (default 1)")
    parser.add_argument("--output", help="Output dir (default /tmp/multi_lidar_calib_<preset>_<ts>)")
    parsed = parser.parse_args(args)

    # 1. Preset
    preset, preset_path = load_preset(parsed.preset_file or parsed.preset)
    print(f"Preset: {preset.get('preset', preset_path.stem)} ({preset_path})")

    # 2. URDF
    try:
        read_urdf, write_urdf = find_urdf_paths(parsed.urdf)
    except (FileNotFoundError, RuntimeError) as e:
        print(f"Error: {e}", file=sys.stderr)
        sys.exit(1)
    print(f"URDF read:  {read_urdf}")
    if write_urdf and write_urdf != read_urdf:
        print(f"URDF write: {write_urdf}")
    elif not write_urdf:
        print(
            "WARNING: robot version unknown; only urdf/current/ will be updated.\n"
            "  Set AMR_CUSTOMER/LOCATION/MACHINE or run apply.sh to persist results."
        )

    joints = [s["joint"] for s in preset["sensors"]]
    urdf_transforms = parse_urdf_transforms(read_urdf, joints)
    print("Initial transforms (from URDF):")
    for j, v in urdf_transforms.items():
        print(f"  {j}: xyz=[{v[0]:.4f}, {v[1]:.4f}, {v[2]:.4f}]  "
              f"rpy(deg)=[{v[3]:.3f}, {v[4]:.3f}, {v[5]:.3f}]")

    # 3. Output dir + temp URDF
    if parsed.output:
        output_dir = pathlib.Path(parsed.output)
    else:
        ts = datetime.now().strftime("%Y%m%d_%H%M%S")
        preset_name = preset.get("preset", preset_path.stem)
        output_dir = pathlib.Path(f"/tmp/multi_lidar_calib_{preset_name}_{ts}")
    output_dir.mkdir(parents=True, exist_ok=True)
    temp_urdf = output_dir / "mecanum_bot_calibrated.urdf"
    shutil.copy(read_urdf, temp_urdf)
    print(f"Output: {output_dir}")

    # 4. Generate ROS params
    ros_params = build_ros_params(preset, urdf_transforms, temp_urdf, output_dir, parsed.frames)
    params_file = output_dir / "live_params.yaml"
    with open(params_file, "w") as f:
        yaml.dump(ros_params, f, default_flow_style=False, sort_keys=False)

    # 5. Optionally play bag
    bag_proc = None
    if parsed.bag:
        bag_path = pathlib.Path(parsed.bag)
        if not bag_path.exists():
            print(f"Error: bag not found: {bag_path}", file=sys.stderr)
            sys.exit(1)
        print(f"Playing bag in background: {bag_path}")
        bag_proc = play_bag(bag_path)
        time.sleep(2)

    # 6. Run
    try:
        run_calibration(params_file)
    except KeyboardInterrupt:
        print("\nInterrupted.")
    finally:
        if bag_proc:
            stop_bag(bag_proc)

    # 7. Parse + display + apply
    results = parse_results(temp_urdf, preset, read_urdf)
    print_results(results, preset, urdf_transforms)
    if not results:
        sys.exit(1)

    if confirm_apply(read_urdf, write_urdf):
        shutil.copy(temp_urdf, read_urdf)
        print(f"Updated: {read_urdf}")
        if write_urdf and write_urdf != read_urdf:
            shutil.copy(temp_urdf, write_urdf)
            print(f"Updated: {write_urdf}")
            print("Commit amr-versioning-system to persist.")
    else:
        print(f"URDF unchanged. Calibrated URDF saved to: {temp_urdf}")


if __name__ == "__main__":
    main()
