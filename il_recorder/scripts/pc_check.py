import os
from functools import partial

import numpy as np
import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField

from ..pointcloud_utils import (
    dp3_preprocess_point_cloud,
    flat_pc_from_ros,
    idp3_preprocess_point_cloud,
)

"""

Script to debug your pointcloud configuration and preprocessing, publishing it to RViz 
to ensure that you see the correct output on your pointcloud before recording is started.

Usage: ros2 run il_recorder pc_check --ros-args -p robot_config:=<config>.yaml -p obs_config:=<config>.yaml

"""


class PCCheck(Node):
    def __init__(self):
        super().__init__("pc_check")

        pkg_name = "il_recorder"
        share_dir = get_package_share_directory(pkg_name)

        self.declare_parameter("robot_config", "wam7.yaml")
        self.declare_parameter("obs_config", "idp3.yaml")

        robot_filename = self.get_parameter("robot_config").value
        obs_filename = self.get_parameter("obs_config").value

        robot_cfg_path = os.path.join(share_dir, "configs", "robots", robot_filename)
        obs_cfg_path = os.path.join(share_dir, "configs", "observations", obs_filename)

        self.get_logger().info(f"Loading Robot Config: {robot_cfg_path}")
        self.get_logger().info(f"Loading Obs Config:   {obs_cfg_path}")

        with open(robot_cfg_path, "r") as f:
            self.robot_cfg = yaml.safe_load(f)
        with open(obs_cfg_path, "r") as f:
            self.obs_cfg = yaml.safe_load(f)

        self.pc_cfg = self.obs_cfg.get("pointcloud", {})
        self.proc_type = self.pc_cfg.get("processing_type", "dp3")

        self.get_logger().info(f"Processing Type: {self.proc_type}")

        # Dictionary to store a publisher for each camera key
        self.debug_pubs = {}

        # 2. Setup Subscribers and matching Debug Publishers
        for sub_cfg in self.robot_cfg["active_subscribers"]:
            key = sub_cfg["key"]

            # We only care about pointcloud inputs
            if "pointcloud" in key.lower():
                topic = sub_cfg["topic"]

                # Create the Subscriber
                self.create_subscription(
                    PointCloud2, topic, partial(self.pc_callback, key=key), 10
                )

                # Create a matching Debug Publisher
                # e.g., if key is "wrist_camera", topic becomes "/debug/wrist_camera"
                debug_topic = f"/debug/{key}"
                self.debug_pubs[key] = self.create_publisher(
                    PointCloud2, debug_topic, 10
                )
                self.get_logger().info(
                    f"Subscribed to {topic} -> Publishing debug to {debug_topic}"
                )

    def pc_callback(self, msg, key):
        # 1. Unpack (Robust Method)
        flat_pc = flat_pc_from_ros(msg, remove_nans=True)

        # 2. Process
        if self.proc_type == "idp3":
            processed = idp3_preprocess_point_cloud(
                flat_pc,
                num_points=self.pc_cfg.get("num_points", 4096),
                near=self.pc_cfg.get("near", 0.1),
                far=self.pc_cfg.get("far", 2.0),
            )
        else:
            processed = dp3_preprocess_point_cloud(
                flat_pc,
                num_points=self.pc_cfg.get("num_points", 1024),
                workspace=self.pc_cfg.get("workspace"),
                rpy=self.pc_cfg.get("rpy", [0, 0, 0]),
            )

        n_points = processed.shape[0]

        # 3. Prepare Data (Zero Dependency, Packed RGB)
        # Structure: x, y, z, rgb (all float32)
        dtype_spec = [
            ("x", np.float32),
            ("y", np.float32),
            ("z", np.float32),
            ("rgb", np.float32),
        ]

        cloud_arr = np.zeros((n_points,), dtype=dtype_spec)
        cloud_arr["x"] = processed[:, 0]
        cloud_arr["y"] = processed[:, 1]
        cloud_arr["z"] = processed[:, 2]

        if processed.shape[1] >= 6:
            r = processed[:, 3]
            g = processed[:, 4]
            b = processed[:, 5]

            # Normalize 0-1 -> 0-255 if needed
            if r.max() <= 1.01:
                r = (r * 255).astype(np.uint32)
                g = (g * 255).astype(np.uint32)
                b = (b * 255).astype(np.uint32)
            else:
                r = r.astype(np.uint32)
                g = g.astype(np.uint32)
                b = b.astype(np.uint32)

            # Pack: (R << 16) | (G << 8) | B
            rgb_uint32 = (r << 16) | (g << 8) | b
            cloud_arr["rgb"] = rgb_uint32.view(np.float32)
        else:
            # White default
            rgb_uint32 = np.array([0xFFFFFF] * n_points, dtype=np.uint32)
            cloud_arr["rgb"] = rgb_uint32.view(np.float32)

        # 4. Create Message
        out_msg = PointCloud2()
        out_msg.header.frame_id = msg.header.frame_id
        out_msg.header.stamp = self.get_clock().now().to_msg()
        out_msg.height = 1
        out_msg.width = n_points
        out_msg.point_step = 16
        out_msg.row_step = out_msg.point_step * out_msg.width
        out_msg.is_dense = True

        out_msg.fields = [
            PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(name="rgb", offset=12, datatype=PointField.FLOAT32, count=1),
        ]

        out_msg.data = cloud_arr.tobytes()

        # 5. Publish to the specific debug topic for this key
        if key in self.debug_pubs:
            self.debug_pubs[key].publish(out_msg)


def main():
    rclpy.init()
    rclpy.spin(PCCheck())
    rclpy.shutdown()


if __name__ == "__main__":
    main()
