#!/usr/bin/env python3
"""
Interactive live LiDAR calibration script for mecanum AMR.

Reads initial transform guesses from the versioning system URDF, runs calibration
on live sensor streams, and applies results after interactive confirmation.

Usage:
    calibrate_live                              # zero-argument convenience
    calibrate_live --frames 3                   # accumulate 3 frames per LiDAR
    calibrate_live --urdf /path/to/urdf.urdf   # explicit URDF path
"""

import argparse
import importlib.util
import math
import os
import pathlib
import re
import shutil
import subprocess
import sys
import xml.etree.ElementTree as ET
from datetime import datetime

import yaml
from ament_index_python.packages import get_package_prefix

LIDAR_FRAME_IDS = ["rslidarfront", "rslidarback", "rslidarleft", "rslidarright"]

LIDAR_TOPICS = [
    "/lidar/front/rslidar_points",
    "/lidar/back/rslidar_points",
    "/lidar/left/rslidar_points",
    "/lidar/right/rslidar_points",
]


def _versioning_system_root() -> pathlib.Path:
    """Locate the amr-versioning-system directory in the workspace.

    Walks upward from the install prefix to find the workspace root,
    which works regardless of whether the package is in src/Multi_LiCa
    or nested inside src/mecanum-robot-ros2/utilities/Multi_LiCa.
    """
    start = pathlib.Path(get_package_prefix("multi_lidar_calibrator"))
    for parent in start.parents:
        candidate = parent / "src/mecanum-robot-ros2/amr-versioning-system"
        if candidate.is_dir():
            return candidate
    raise RuntimeError(
        "amr-versioning-system not found in any parent workspace.\n"
        "Ensure mecanum-robot-ros2 is in <workspace>/src/ and submodules are initialized."
    )


def _read_robot_version(vs_root: pathlib.Path) -> tuple:
    """Read AMR_CUSTOMER/AMR_LOCATION/AMR_MACHINE from env or .env file.

    Returns (customer, location, machine) or (None, None, None).
    """
    customer = os.environ.get("AMR_CUSTOMER")
    location = os.environ.get("AMR_LOCATION")
    machine = os.environ.get("AMR_MACHINE")

    if customer and location and machine:
        return customer, location, machine

    # Fall back to .env written by apply.sh
    env_file = vs_root / "config" / "current" / ".env"
    if env_file.exists():
        env_vars = {}
        for line in env_file.read_text().splitlines():
            line = line.strip()
            if "=" in line and not line.startswith("#"):
                key, _, val = line.partition("=")
                env_vars[key.strip()] = val.strip()
        customer = env_vars.get("AMR_CUSTOMER")
        location = env_vars.get("AMR_LOCATION")
        machine = env_vars.get("AMR_MACHINE")
        if customer and location and machine:
            return customer, location, machine

    return None, None, None


def find_urdf_paths(urdf_override: str = None) -> tuple:
    """Find URDF read path (initial guesses) and write path (robot-specific).

    Returns (read_path, write_path). write_path may be None if the robot
    version cannot be determined.
    """
    if urdf_override:
        p = pathlib.Path(urdf_override)
        if not p.exists():
            raise FileNotFoundError(f"URDF not found: {urdf_override}")
        return p, p

    try:
        vs_root = _versioning_system_root()
    except RuntimeError as e:
        raise RuntimeError(
            f"{e}\nPass --urdf /path/to/mecanum_bot.urdf explicitly."
        )

    # Always read from current/ (the active merged URDF)
    read_path = vs_root / "urdf" / "current" / "mecanum_bot.urdf"
    if not read_path.exists():
        raise RuntimeError(
            f"Active URDF not found: {read_path}\n"
            "Run apply.sh first or pass --urdf explicitly."
        )

    # Try to resolve the robot-specific write path
    customer, location, machine = _read_robot_version(vs_root)
    write_path = None
    if customer and location and machine:
        robot_dir = (vs_root / "urdf" / "customers" /
                     customer / location / machine)
        if robot_dir.is_dir():
            # Write to .urdf in the customer dir (created if missing)
            write_path = robot_dir / "mecanum_bot.urdf"

    return read_path, write_path


def _node_file_dir() -> pathlib.Path:
    """Resolve the directory containing the installed multi_lidar_calibrator node module.

    The node prepends os.path.dirname(os.path.realpath(__file__)) to the
    output_dir parameter value, so we need this to compute a working
    relative path.
    """
    spec = importlib.util.find_spec("multi_lidar_calibrator.multi_lidar_calibrator")
    if spec is None or spec.origin is None:
        raise RuntimeError("multi_lidar_calibrator package not found on sys.path")
    return pathlib.Path(spec.origin).resolve().parent


def _relative_output_dir(output_dir: pathlib.Path) -> str:
    """Compute an output_dir param value that resolves correctly inside the node.

    The node constructs: os.path.dirname(__file__) + param_value
    So we return a relative path (e.g. "/../../../../../../tmp/foo/")
    that resolves to the desired absolute output_dir.
    """
    node_dir = _node_file_dir()
    rel = os.path.relpath(str(output_dir.resolve()), str(node_dir))
    return "/" + rel + "/"


def parse_urdf_transforms(urdf_path: pathlib.Path) -> dict:
    """Extract LiDAR joint origins from URDF.

    Only extracts the 4 calibration-relevant joints (front, back, left, right).
    Returns dict mapping frame_id -> [x, y, z, roll_deg, pitch_deg, yaw_deg].

    RPY is converted from URDF radians to degrees because the calibrator node
    hardcodes degrees=True in its Rotation() constructor (ignores table_degrees).
    """
    tree = ET.parse(urdf_path)
    root = tree.getroot()

    expected_joints = {f"joint_{fid}": fid for fid in LIDAR_FRAME_IDS}

    lidars = {}
    for joint in root.iter("joint"):
        joint_name = joint.attrib.get("name", "")
        if joint_name not in expected_joints:
            continue

        origin = joint.find("origin")
        if origin is None:
            continue

        frame_id = expected_joints[joint_name]
        xyz_str = origin.attrib.get("xyz", "0 0 0").split()
        rpy_str = origin.attrib.get("rpy", "0 0 0").split()

        try:
            xyz = [float(v) for v in xyz_str]
            rpy_deg = [math.degrees(float(v)) for v in rpy_str]
            lidars[frame_id] = xyz + rpy_deg
        except ValueError as e:
            print(f"Warning: Could not parse joint {joint_name}: {e}")

    return lidars


def generate_params_yaml(
    lidars: dict,
    frames: int,
    temp_urdf: pathlib.Path,
    output_dir: pathlib.Path,
) -> pathlib.Path:
    rel_output = _relative_output_dir(output_dir)

    params = {
        "/**": {
            "ros__parameters": {
                "read_pcds_from_file": False,
                "read_tf_from_table": True,
                "table_degrees": True,
                "frame_count": frames,
                "runs_count": 1,
                "visualize": False,
                "lidar_topics": LIDAR_TOPICS,
                "target_frame_id": "rslidarfront",
                "base_frame_id": "base_link",
                "calibrate_to_base": True,
                "calibrate_target": False,
                "use_fitness_based_calibration": False,
                "urdf_path": str(temp_urdf),
                "output_dir": rel_output,
                "results_file": "results.txt",
                "max_corresp_dist": 0.3,
                "fitness_score_threshold": 0.3,
                "voxel_size": 0.05,
                "max_iterations": 200,
                "rel_fitness": 1.0e-6,
                "rel_rmse": 1.0e-6,
                "epsilon": 0.0001,
                "distance_threshold": 0.1,
                "ransac_n": 10,
                "num_iterations": 2000,
                "r_voxel_size": 0.1,
                "r_runs": 10,
                "crop_cloud": 25,
                "base_to_ground_z": 0.17864325963808905,
                **lidars,
            }
        }
    }

    yaml_path = output_dir / "live_params.yaml"
    with open(yaml_path, "w") as f:
        yaml.dump(params, f, default_flow_style=False, sort_keys=False)

    return yaml_path


def run_calibration(params_yaml: pathlib.Pa
th) -> int:
    """Launch calibration and return the exit code.

    The calibrator node calls exit(0) from within rclpy.spin(), which may
    cause ros2 launch to report a non-zero exit code even on success.
    Callers should check for results files rather than trusting the return code.
    """
    print(
        f"\n{'='*70}\n"
        f"Listening on {len(LIDAR_TOPICS)} LiDAR topics...\n"
        f"Parameters: {params_yaml}\n"
        f"{'='*70}\n"
    )

    result = subprocess.run(
        [
            "ros2", "launch", "multi_lidar_calibrator", "calibration.launch.py",
            f"parameter_file:={params_yaml}",
        ],
        check=False,
    )
    return result.returncode


def parse_calibration_results(
    results_file: pathlib.Path, temp_urdf: pathlib.Path
) -> dict:
    """Parse calibration results from results.txt and temp URDF."""
    results = {}

    if not temp_urdf.exists():
        print(f"Temp URDF not found: {temp_urdf}")
        return results

    tree = ET.parse(temp_urdf)
    root = tree.getroot()
    expected_joints = {f"joint_{fid}": fid for fid in LIDAR_FRAME_IDS}

    for joint in root.iter("joint"):
        joint_name = joint.attrib.get("name", "")
        if joint_name not in expected_joints:
            continue

        origin = joint.find("origin")
        if origin is None:
            continue

        frame_id = expected_joints[joint_name]
        xyz_str = origin.attrib.get("xyz", "0 0 0").split()
        rpy_str = origin.attrib.get("rpy", "0 0 0").split()

        try:
            xyz = [float(v) for v in xyz_str]
            rpy = [float(v) for v in rpy_str]
            results[frame_id] = {"xyz": xyz, "rpy_rad": rpy, "fitness": None}
        except ValueError:
            pass

    if results_file.exists():
        content = results_file.read_text()
        for frame_id in results:
            # frame_id and "fitness:" are on different lines in results.txt.
            # A lidar may appear twice (initial + re-calibration); take the last match.
            pattern = rf"{re.escape(frame_id)}.*?fitness:\s*([\d.eE+\-]+)"
            matches = re.findall(pattern, content, re.DOTALL)
            if matches:
                results[frame_id]["fitness"] = float(matches[-1])

    return results


def print_results(results: dict, fitness_threshold: float) -> None:
    print(f"\n{'='*100}")
    print(f"CALIBRATION RESULTS  (target: rslidarfront)")
    print(f"{'='*100}\n")

    if not results:
        print("No results found.")
        return

    print(f"{'LiDAR':<16} {'xyz (m)':<26} {'rpy (rad)':<30} {'Fitness':<10}")
    print("-" * 100)

    low_fitness = []
    for frame_id in LIDAR_FRAME_IDS:
        if frame_id == "rslidarfront" or frame_id not in results:
            continue

        info = results[frame_id]
        xyz = info["xyz"]
        rpy = info["rpy_rad"]
        fitness = info["fitness"]

        xyz_str = f"{xyz[0]:7.4f} {xyz[1]:7.4f} {xyz[2]:7.4f}"
        rpy_str = f"{rpy[0]:8.5f} {rpy[1]:8.5f} {rpy[2]:8.5f}"
        fitness_str = f"{fitness:.3f}" if fitness is not None else "N/A"
        warn = "  << LOW" if fitness is not None and fitness <= fitness_threshold else ""

        print(f"{frame_id:<16} {xyz_str:<26} {rpy_str:<30} {fitness_str:<10}{warn}")

        if fitness is not None and fitness <= fitness_threshold:
            low_fitness.append((frame_id, fitness))

    print("-" * 100)
    print(f"Fitness threshold: {fitness_threshold}")

    if low_fitness:
        print(f"\nWARNING: Low-fitness LiDARs (limited point cloud overlap):")
        for fid, f in low_fitness:
            print(f"  {fid}: {f:.3f}")

    print(f"{'='*100}\n")


def confirm_apply(urdf_path: pathlib.Path) -> bool:
    print(f"Apply these transforms to URDF?")
    print(f"  Target: {urdf_path}")
    try:
        while True:
            response = input("[y/N] ").strip().lower()
            if response in ("y", "yes"):
                return True
            if response in ("n", "no", ""):
                return False
            print("Enter 'y' or 'n'.")
    except EOFError:
        print()
        return False


def main(args=None):
    parser = argparse.ArgumentParser(
        description="Interactive live LiDAR calibration for mecanum AMR",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="\n".join([
            "Examples:",
            "  calibrate_live                     # Use all defaults",
            "  calibrate_live --frames 3          # Accumulate 3 frames per LiDAR",
            "  calibrate_live --urdf /custom.urdf # Explicit URDF path",
        ]),
    )
    parser.add_argument(
        "--urdf", type=str, default=None,
        help="Path to mecanum_bot.urdf (auto-detected if omitted)",
    )
    parser.add_argument(
        "--frames", type=int, default=1,
        help="Number of frames to accumulate per LiDAR (default: 1)",
    )
    parser.add_argument(
        "--output", type=str, default=None,
        help="Output directory (default: /tmp/multi_lidar_calib_<timestamp>/)",
    )

    parsed_args = parser.parse_args(args)

    # --- Find URDF ---
    try:
        read_urdf, write_urdf = find_urdf_paths(parsed_args.urdf)
        print(f"URDF (read): {read_urdf}")
        if write_urdf and write_urdf != read_urdf:
            print(f"URDF (write): {write_urdf}")
        elif not write_urdf:
            print(
                "WARNING: Could not determine robot version.\n"
                "  Set AMR_CUSTOMER, AMR_LOCATION, AMR_MACHINE env vars,\n"
                "  or run apply.sh to populate config/current/.env.\n"
                "  Calibration will only update urdf/current/ (not robot-specific)."
            )
    except (FileNotFoundError, RuntimeError) as e:
        print(f"Error: {e}", file=sys.stderr)
        sys.exit(1)

    # --- Output directory ---
    if parsed_args.output:
        output_dir = pathlib.Path(parsed_args.output)
    else:
        ts = datetime.now().strftime("%Y%m%d_%H%M%S")
        output_dir = pathlib.Path(f"/tmp/multi_lidar_calib_{ts}")

    output_dir.mkdir(parents=True, exist_ok=True)
    print(f"Output: {output_dir}")

    # --- Parse URDF for initial guesses ---
    print("Parsing URDF for initial transform guesses...")
    lidars = parse_urdf_transforms(read_urdf)
    if not lidars:
        print("Error: No LiDAR joints found in URDF", file=sys.stderr)
        sys.exit(1)

    missing = [fid for fid in LIDAR_FRAME_IDS if fid not in lidars]
    if missing:
        print(f"Warning: Missing joints for: {', '.join(missing)}")

    for frame_id in LIDAR_FRAME_IDS:
        if frame_id in lidars:
            v = lidars[frame_id]
            print(f"  {frame_id}: xyz=[{v[0]:.4f}, {v[1]:.4f}, {v[2]:.4f}]  "
                  f"rpy(deg)=[{v[3]:.3f}, {v[4]:.3f}, {v[5]:.3f}]")

    # --- Copy URDF to temp (calibrator writes results here) ---
    temp_urdf = output_dir / "mecanum_bot_calibrated.urdf"
    shutil.copy(read_urdf, temp_urdf)

    # --- Generate params YAML ---
    print("Generating live calibration parameters...")
    params_yaml = generate_params_yaml(lidars, parsed_args.frames, temp_urdf, output_dir)
    print(f"  {params_yaml}")

    # --- Run calibration ---
    try:
        run_calibration(params_yaml)
    except KeyboardInterrupt:
        print("\nCalibration interrupted.")
        sys.exit(0)

    # --- Parse and display results ---
    # Check for results regardless of exit code (node exit(0) inside spin
    # can cause ros2 launch to report non-zero).
    results_file = output_dir / "results.txt"
    results = parse_calibration_results(results_file, temp_urdf)
    if not results:
        print("No calibration results produced. Check sensor topics.", file=sys.stderr)
        sys.exit(1)

    fitness_threshold = 0.3
    print_results(results, fitness_threshold)

    # --- Confirm and apply ---
    if confirm_apply(read_urdf):
        # Always update current/ (active URDF)
        shutil.copy(temp_urdf, read_urdf)
        print(f"\nUpdated: {read_urdf}")

        # Also update robot-specific URDF if we know the version
        if write_urdf and write_urdf != read_urdf:
            shutil.copy(temp_urdf, write_urdf)
            print(f"Updated: {write_urdf}")
            print("Commit amr-versioning-system when satisfied to persist.")
        elif not write_urdf:
            print(
                "\nNOTE: Only urdf/current/ was updated (robot version unknown).\n"
                "This will be overwritten on next apply.sh run.\n"
                "To persist, copy the calibrated URDF to the correct customer path:\n"
                f"  cp {temp_urdf} <versioning-system>/urdf/customers/<customer>/<location>/<machine>/mecanum_bot.urdf"
            )
    else:
        print(f"\nURDF unchanged. Calibrated URDF saved to: {temp_urdf}")


if __name__ == "__main__":
    main()
