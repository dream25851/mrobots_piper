"""Local robot_state_publisher + RViz for joint states produced by the RK3588S."""

from __future__ import annotations

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import xacro


def _nodes(context):
    share = get_package_share_directory("piper_description")
    arms = LaunchConfiguration("arms").perform(context)
    if arms not in ("left", "both"):
        raise RuntimeError("arms must be 'left' or 'both'")
    robot_description = xacro.process_file(
        os.path.join(
            share, "urdf", "piper_bimanual_manipulation.urdf.xacro"
        ),
        mappings={
            "enable_left": "true",
            "enable_right": str(arms == "both").lower(),
            "enable_left_gripper": "true",
            "enable_right_gripper": str(arms == "both").lower(),
            "use_fake_hardware": "true",
        },
    ).toprettyxml(indent="  ")
    return [
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            name="mrobot_remote_robot_state_publisher",
            output="screen",
            parameters=[{"robot_description": robot_description}],
            remappings=[("joint_states", "/joint_states")],
        ),
        Node(
            package="rviz2",
            executable="rviz2",
            name="mrobot_remote_rviz2",
            output="screen",
            arguments=["-d", os.path.join(share, "rviz", "visualize_piper.rviz")],
            condition=IfCondition(LaunchConfiguration("use_rviz")),
        ),
    ]


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("arms", default_value="left"),
            DeclareLaunchArgument("use_rviz", default_value="true"),
            OpaqueFunction(function=_nodes),
        ]
    )
