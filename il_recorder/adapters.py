from typing import TYPE_CHECKING, Any, Dict, TypedDict

import numpy as np
from scipy.spatial.transform import Rotation

"""

Define adapters to parse robot messages into a standard format for il_recorder.

Any adapter will take in a dictionary of ROS messages (msg_dict) and a dictionary of parameters from
the robot config file. It can then parse this data however it wants into a dictionary, which will then
be saved into the hdf5. The format is flexible to the downstream needs, and does not need to include
any more data than you require.

"""

# This block is ONLY seen by VSCode/Type Checkers.
if TYPE_CHECKING:
    from geometry_msgs.msg import PoseStamped, TwistStamped
    from sensor_msgs.msg import JointState

    # If you have custom robot msgs, add them here:


# Define a TypedDict for the msg_dict to hint the keys for development
class RobotMsgDict(TypedDict):
    joints: "JointState"
    joint_vels: "JointState"
    cartesian: "PoseStamped"
    cartesian_vels: "TwistStamped"
    gripper: Any


def parse_fr3(msg_dict: RobotMsgDict, params: Dict[str, Any]):

    joint_msg = msg_dict["joints"]
    cart_msg = msg_dict["cartesian"]
    gripper_msg = msg_dict["gripper"]

    rpy = Rotation.from_quat(
        [
            cart_msg.pose.orientation.x,
            cart_msg.pose.orientation.y,
            cart_msg.pose.orientation.z,
            cart_msg.pose.orientation.w,
        ]
    ).as_euler("xyz")

    return {
        "joints": {
            "position": np.array(joint_msg.position),
            "velocity": np.array(joint_msg.velocity),
        },
        "cartesian": {
            "position": np.array(
                [
                    cart_msg.pose.position.x,
                    cart_msg.pose.position.y,
                    cart_msg.pose.position.z,
                    rpy[0],
                    rpy[1],
                    rpy[2],
                ]
            )
            # TODO no velocity since I have to add it in franky bridge.
        },
        "gripper": np.array([gripper_msg.width]),
    }


def parse_wam(msg_dict: RobotMsgDict, params: Dict[str, Any]):
    joint_msg = msg_dict["joints"]
    cartesian_msg = msg_dict["cartesian"]
    cart_vel_msg = msg_dict.get("cartesian_vels", None)

    rpy = Rotation.from_quat(
        [
            cartesian_msg.pose.orientation.x,
            cartesian_msg.pose.orientation.y,
            cartesian_msg.pose.orientation.z,
            cartesian_msg.pose.orientation.w,
        ]
    ).as_euler("xyz")

    data = {
        "joints": {
            "position": np.array(joint_msg.position),
        },
        "cartesian": {
            "position": np.array(
                [
                    cartesian_msg.pose.position.x,
                    cartesian_msg.pose.position.y,
                    cartesian_msg.pose.position.z,
                    rpy[0],
                    rpy[1],
                    rpy[2],
                ]
            ),
            "velocity": np.array(
                [
                    cart_vel_msg.twist.linear.x,
                    cart_vel_msg.twist.linear.y,
                    cart_vel_msg.twist.linear.z,
                    cart_vel_msg.twist.angular.x,
                    cart_vel_msg.twist.angular.y,
                    cart_vel_msg.twist.angular.z,
                ]
            ),
        },
    }

    if params.get("has_gripper", False):
        # TODO figure out logic for wam gripper
        # "gripper": np.array([joint_msg.position[params["gripper_idx"]]]),
        pass

    if "joint_vels" in msg_dict:
        joint_vels_msg = msg_dict["joint_vels"]
        data["joints"]["velocity"] = np.array(joint_vels_msg.velocity)

    return data


ADAPTERS = {"fr3": parse_fr3, "wam": parse_wam}
