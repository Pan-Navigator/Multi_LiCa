"""Launch the multi-LiDAR calibrator node.

The parameter file supplies all runtime config including `output_dir`. The
default points at the amr-versioning-system merged output for the active
machine (base/calibration/ + customer overrides, materialized by apply.sh).
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    params_declare = DeclareLaunchArgument(
        "parameter_file",
        default_value=(
            "/workspaces/isaac_ros-dev/amr_ws/src/mecanum-robot-ros2/"
            "amr-versioning-system/config/current/sensors/multi_lidar_calibration.yaml"
        ),
        description="Absolute path to the ROS 2 parameters YAML.",
    )

    return LaunchDescription([
        params_declare,
        Node(
            package="multi_lidar_calibrator",
            executable="multi_lidar_calibrator",
            name="multi_lidar_calibration_node",
            parameters=[LaunchConfiguration("parameter_file")],
            output="screen",
        ),
    ])
