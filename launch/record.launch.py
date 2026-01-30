import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg_share = FindPackageShare("il_recorder")

    return LaunchDescription(
        [
            # Robot Config: looks in configs/robots/<name>.yaml
            DeclareLaunchArgument(
                "robot",
                default_value="wam7.yaml",
                description="Name of the robot config file (with .yaml) in configs/robots/",
            ),
            # Observation Config: looks in configs/observations/<name>.yaml
            DeclareLaunchArgument(
                "obs",
                default_value="idp3.yaml",
                description="Name of the observation config file (with .yaml) in configs/observations/",
            ),
            DeclareLaunchArgument("save_dir", default_value="data"),
            Node(
                package="il_recorder",
                executable="il_recorder",
                name="il_recorder",
                output="screen",
                parameters=[
                    {
                        "robot_config": PathJoinSubstitution(
                            [
                                pkg_share,
                                "configs",
                                "robots",
                                [LaunchConfiguration("robot")],
                            ]
                        ),
                        "obs_config": PathJoinSubstitution(
                            [
                                pkg_share,
                                "configs",
                                "observations",
                                [LaunchConfiguration("obs")],
                            ]
                        ),
                        "base_path": LaunchConfiguration("save_dir"),
                    }
                ],
            ),
        ]
    )
