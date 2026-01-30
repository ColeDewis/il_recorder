import importlib
import os
import queue
import time

import cv2
import h5py
import message_filters
import numpy as np
import rclpy
import yaml
from cv_bridge import CvBridge
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Joy

# Custom Imports
from .adapters import ADAPTERS
from .pointcloud_utils import (
    dp3_preprocess_point_cloud,
    flat_pc_from_ros,
    idp3_preprocess_point_cloud,
)


def get_msg_class(msg_type_str: str):
    try:
        parts = msg_type_str.split("/")
        if len(parts) != 3:
            raise ValueError(f"Invalid type format: {msg_type_str}")
        module = importlib.import_module(f"{parts[0]}.{parts[1]}")
        return getattr(module, parts[2])
    except (ImportError, AttributeError, ValueError) as e:
        print(f"Failed to import message type {msg_type_str}: {e}")
        return None


class ILRecorder(Node):
    def __init__(self):
        super().__init__("il_recorder")

        # --- 1. Parameters & Configs ---
        self.declare_parameter("robot_config", "config/robots/fr3.yaml")
        self.declare_parameter("obs_config", "config/observations/all.yaml")
        self.declare_parameter("base_path", "data")

        robot_cfg_path = (
            self.get_parameter("robot_config").get_parameter_value().string_value
        )
        obs_cfg_path = (
            self.get_parameter("obs_config").get_parameter_value().string_value
        )
        self.base_path = (
            self.get_parameter("base_path").get_parameter_value().string_value
        )

        with open(robot_cfg_path, "r") as f:
            self.robot_cfg = yaml.safe_load(f)
        with open(obs_cfg_path, "r") as f:
            self.obs_cfg = yaml.safe_load(f)

        self.robot_name = self.robot_cfg["robot_name"]
        self.adapter = ADAPTERS[self.robot_cfg.get("adapter_type", self.robot_name)]
        self.params = self.robot_cfg.get("params", {})
        self.params.update(self.obs_cfg)

        # --- 2. Concurrency Setup (ROS Native) ---
        # Queue for passing data between the Subscriber and the Processing Timer
        self.raw_queue = queue.Queue()
        self.processed_buffer = []
        self.is_recording = False

        # Callback groups
        self.subscriber_group = MutuallyExclusiveCallbackGroup()
        self.proc_group = MutuallyExclusiveCallbackGroup()
        self.control_group = ReentrantCallbackGroup()  # joy

        # Spawn a thread to periodically process data from the queue
        self.proc_timer = self.create_timer(
            0.01, self.processing_callback, callback_group=self.proc_group
        )

        # debug information callbacks
        self.stream_ready = False
        self.status_timer = self.create_timer(
            5.0, self.status_callback, callback_group=self.proc_group
        )

        # --- Dynamic Subscribers for data recording ---
        self.cv_bridge = CvBridge()
        self.sub_keys = []
        subs = []
        self.image_keys = []
        self.pc_keys = []

        rec_img = self.obs_cfg.get("images", {}).get("enabled", False)
        rec_pc = self.obs_cfg.get("pointcloud", {}).get("enabled", False)

        for sub_cfg in self.robot_cfg["active_subscribers"]:
            key = sub_cfg["key"]

            is_image = "image" in key.lower()
            is_pointcloud = "pointcloud" in key.lower()
            if is_image and not rec_img:
                continue
            if is_pointcloud and not rec_pc:
                continue

            if is_image:
                self.image_keys.append(key)
            if is_pointcloud:
                self.pc_keys.append(key)

            msg_class = get_msg_class(sub_cfg["type"])
            if msg_class:
                self.get_logger().info(
                    f"Subscribing to {sub_cfg['topic']} with type {msg_class} as {key}"
                )
                sub = message_filters.Subscriber(
                    self,
                    msg_class,
                    sub_cfg["topic"],
                    callback_group=self.subscriber_group,
                    qos_profile=qos_profile_sensor_data,
                )
                subs.append(sub)
                self.sub_keys.append(key)

        # Sync Policy
        self.ts = message_filters.ApproximateTimeSynchronizer(
            subs, queue_size=50, slop=0.1, allow_headerless=True
        )
        self.ts.registerCallback(self.unified_callback)

        # --- Joystick ---
        self.joy_sub = self.create_subscription(
            Joy, "/joy", self.joy_callback, 10, callback_group=self.control_group
        )
        self.last_joy_time = time.time()

        # IO Setup
        os.makedirs(self.base_path, exist_ok=True)
        existing = [
            int(n.split("_")[-1].split(".")[0])
            for n in os.listdir(self.base_path)
            if n.startswith("episode_") and n.endswith(".h5")
        ]
        self.episode_num = max(existing) + 1 if existing else 0

        self.get_logger().info(f"Recorder Online. Robot: {self.robot_name}")

    def status_callback(self):
        """Runs every 5 seconds to print recording health."""
        if self.is_recording:
            count = len(self.processed_buffer)
            if count > 0:
                self.get_logger().info(f"Recording... buffer length: {count}.")
            else:
                self.get_logger().warn(
                    "Recording ACTIVE but NO DATA buffered! Are all your requested topics publishing?"
                )

    def unified_callback(self, *msgs):
        """PRODUCER: Runs a synced callback for all subscribed msgs, and adds to queue."""
        if not self.stream_ready:
            self.get_logger().info(">>> All topics found - Recording is ready! <<<")
            self.stream_ready = True

        if not self.is_recording:
            return
        msg_dict = {key: msg for key, msg in zip(self.sub_keys, msgs)}
        self.raw_queue.put(msg_dict)

    def processing_callback(self):
        """CONSUMER: Runs in Processing Group via Timer"""

        # This runs really slow when I am recording, not good.
        try:
            # We fetch with NoWait so we don't block the Timer thread idly
            msg_dict = self.raw_queue.get_nowait()
        except queue.Empty:
            return

        try:
            frame_data = {}

            # 1. Low Dim
            frame_data["low_dim"] = self.adapter(msg_dict, self.params)

            # 2. Images
            for key in self.image_keys:
                if key in msg_dict:
                    cv_img = self.cv_bridge.imgmsg_to_cv2(msg_dict[key], "bgr8")

                    img_cfg = self.obs_cfg["images"]
                    resize_dim = img_cfg.get("resize")  # e.g., [224, 224]
                    if resize_dim:
                        # Ensure config provides integers
                        w, h = int(resize_dim[0]), int(resize_dim[1])
                        cv_img = cv2.resize(
                            cv_img, (w, h), interpolation=cv2.INTER_AREA
                        )

                    frame_data[key] = cv_img

            # 3. Point Clouds
            for key in self.pc_keys:
                if key in msg_dict:
                    pc_cfg = self.obs_cfg["pointcloud"]
                    proc_type = pc_cfg.get("processing_type", "raw")

                    flat_pc = flat_pc_from_ros(msg_dict[key])
                    if proc_type == "dp3":
                        frame_data[key] = dp3_preprocess_point_cloud(
                            flat_pc,
                            num_points=pc_cfg.get("num_points", 1024),
                            workspace=pc_cfg.get("workspace"),
                            rpy=pc_cfg.get("rpy", [0, 0, 0]),
                        )
                    elif proc_type == "idp3":
                        frame_data[key] = idp3_preprocess_point_cloud(
                            flat_pc,
                            num_points=pc_cfg.get("num_points", 1024),
                            near=pc_cfg.get("near", 0.0),
                            far=pc_cfg.get("far", 1.5),
                        )
                    elif proc_type == "raw":
                        frame_data[key] = flat_pc

            self.processed_buffer.append(frame_data)

            self.get_logger().info("")

        except Exception as e:
            self.get_logger().error(f"Processing Error: {e}")
        finally:
            self.raw_queue.task_done()

    def joy_callback(self, msg: Joy):
        """CONTROL: Runs in Reentrant Group"""
        if time.time() - self.last_joy_time < 0.5:
            return

        # Start (Button 7)
        if msg.buttons[7]:
            if not self.is_recording:
                self.get_logger().info(f"REC >>> Start Ep {self.episode_num}")
                self.processed_buffer = []
                with self.raw_queue.mutex:
                    self.raw_queue.queue.clear()
                self.is_recording = True
            else:
                self.is_recording = False
                self.save_episode_hdf5()
            self.last_joy_time = time.time()

        # Cancel (Button 8)
        elif msg.buttons[8]:
            self.get_logger().warn("Cancelled - Buffer Discarded")
            self.is_recording = False
            self.processed_buffer = []
            with self.raw_queue.mutex:
                self.raw_queue.queue.clear()
            self.last_joy_time = time.time()

    def save_episode_hdf5(self):
        self.get_logger().info("Recording Stopped. Draining queue...")

        # Wait for queue to be empty.
        last_log = time.time()
        while not self.raw_queue.empty():
            # Give feedback every 1.0 second
            now = time.time()
            if now - last_log > 1.0:
                remaining = self.raw_queue.qsize()
                self.get_logger().info(
                    f"  ... processing backlog: {remaining} frames remaining"
                )
                last_log = now

            # Sleep briefly to yield CPU to the processing thread
            time.sleep(0.1)

        self.get_logger().info(
            f"Queue empty. Saving {len(self.processed_buffer)} frames..."
        )

        if not self.processed_buffer:
            self.get_logger().warn("Buffer empty! Nothing saved.")
            return

        filename = os.path.join(self.base_path, f"episode_{self.episode_num}.h5")

        try:
            with h5py.File(filename, "w") as f:
                f.attrs["robot_name"] = self.robot_name
                f.attrs["timestamp"] = time.time()
                f.attrs["robot_config"] = str(self.robot_cfg)
                f.attrs["obs_config"] = str(self.obs_cfg)

                obs_group = f.create_group("observations")

                def save_recursive(group, name, data):
                    if not data:
                        return

                    # Recurse if dictionary
                    if isinstance(data[0], dict):
                        for key in data[0].keys():
                            # Extract list for this key across all frames
                            val_list = [d[key] for d in data]
                            save_recursive(group, f"{name}/{key}", val_list)
                    else:
                        # Base case: Save array
                        try:
                            arr = np.array(data)
                            if arr.dtype == "O":
                                self.get_logger().error(
                                    f"Ragged array at {name}, skipping."
                                )
                                return
                            group.create_dataset(name, data=arr, compression="gzip")
                        except Exception as e:
                            self.get_logger().error(f"Failed to save {name}: {e}")

                first = self.processed_buffer[0]

                # 1. Save Low Dim (Nested)
                low_dim_data = [d["low_dim"] for d in self.processed_buffer]
                if low_dim_data:
                    # We treat the top-level keys of low_dim as the start of recursion
                    for k in low_dim_data[0].keys():
                        save_recursive(obs_group, k, [d[k] for d in low_dim_data])

                # 2. Save Images & Clouds (Flat)
                for key in self.image_keys + self.pc_keys:
                    if key in self.processed_buffer[0]:
                        save_recursive(
                            obs_group, key, [d[key] for d in self.processed_buffer]
                        )

                self.get_logger().info(f"Success! Episode {self.episode_num} Saved.")
                self.episode_num += 1

        except Exception as e:
            self.get_logger().error(f"Failed to save episode to disk: {e}")


def main(args=None):
    rclpy.init(args=args)
    node = ILRecorder()

    executor = MultiThreadedExecutor(num_threads=8)
    executor.add_node(node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
