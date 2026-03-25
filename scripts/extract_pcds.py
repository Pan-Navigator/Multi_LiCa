#!/usr/bin/env python3
"""
Extract point cloud frames from a ROS2 bag and save as PCD files.
Captures the first N frames from each lidar topic within the first 2 seconds.

Usage:
    Terminal 1: ros2 bag play <bag_path> --loop
    Terminal 2: python3 extract_pcds.py
"""

import os
import sys

import numpy as np
import open3d as o3d
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
import ros2_numpy as rnp

TOPICS = {
    "/lidar/front/rslidar_points": "rslidarfront",
    "/lidar/back/rslidar_points":  "rslidarback",
}
FRAME_COUNT = 10
OUT_DIR = os.path.join(os.path.dirname(os.path.realpath(__file__)), "../data/pcds/")


class PcdExtractor(Node):
    def __init__(self):
        super().__init__("pcd_extractor")
        self.frames = {name: [] for name in TOPICS.values()}
        self.start_time = None

        for topic, name in TOPICS.items():
            self.create_subscription(PointCloud2, topic, self._make_cb(name), 10)

        self.get_logger().info(f"Waiting for point clouds on {list(TOPICS.keys())}...")

    def _make_cb(self, lidar_name):
        def cb(msg):
            if self.start_time is None:
                self.start_time = self.get_clock().now()

            elapsed = (self.get_clock().now() - self.start_time).nanoseconds * 1e-9
            if elapsed > 2.0:
                return  # outside the 2-second window

            frames = self.frames[lidar_name]
            if len(frames) >= FRAME_COUNT:
                return

            t = rnp.numpify(msg)
            pcd = o3d.geometry.PointCloud(o3d.utility.Vector3dVector(t["xyz"]))
            frames.append(pcd)
            self.get_logger().info(f"{lidar_name}: {len(frames)}/{FRAME_COUNT} frames")

            if self._done():
                self._save()
                self.get_logger().info("Done. Shutting down.")
                raise SystemExit

        return cb

    def _done(self):
        return all(len(f) >= FRAME_COUNT for f in self.frames.values())

    def _save(self):
        os.makedirs(OUT_DIR, exist_ok=True)
        for lidar_name, frames in self.frames.items():
            merged = frames[0]
            for pcd in frames[1:]:
                merged += pcd
            path = os.path.join(OUT_DIR, f"{lidar_name}.pcd")
            o3d.io.write_point_cloud(path, merged)
            self.get_logger().info(f"Saved {path} ({len(merged.points)} points)")
        self.get_logger().info(f"Merged {FRAME_COUNT} frames per lidar into {OUT_DIR}")


def main():
    rclpy.init()
    node = PcdExtractor()
    try:
        rclpy.spin(node)
    except SystemExit:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
