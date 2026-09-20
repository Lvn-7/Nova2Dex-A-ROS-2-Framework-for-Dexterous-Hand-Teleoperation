"""Launch Nova2 input, independent Inspire retargeting, and MuJoCo viewer(s)."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def _launch_nodes(context):
    mode = LaunchConfiguration("hand_mode").perform(context)
    inspire_share = get_package_share_directory("nova2_inspire_retarget")
    nova2_share = get_package_share_directory("nova2_glove_driver")
    mapping_config = LaunchConfiguration("mapping_config").perform(context)
    if not os.path.isabs(mapping_config):
        mapping_config = os.path.join(inspire_share, "config", mapping_config)
    nova2_config = LaunchConfiguration("nova2_config").perform(context)
    if not os.path.isabs(nova2_config):
        nova2_config = os.path.join(nova2_share, "config", nova2_config)

    actions = [
        Node(
            package="nova2_glove_driver",
            executable="nova2_udp_bridge.py",
            name="nova2_glove_driver",
            parameters=[
                nova2_config,
                {
                    "enable_left": mode in ("left", "both"),
                    "enable_right": mode in ("right", "both"),
                    "listen_host": LaunchConfiguration("listen_host"),
                    "udp_port": ParameterValue(LaunchConfiguration("udp_port"), value_type=int),
                },
            ],
            output="screen",
        ),
        Node(
            package="nova2_inspire_retarget",
            executable="nova2_inspire_retarget_node",
            name="nova2_inspire_retarget",
            parameters=[mapping_config, {"hand_mode": mode}],
            output="screen",
        ),
    ]
    if LaunchConfiguration("launch_viewer").perform(context).lower() in ("1", "true", "yes", "on"):
        for side in ("left", "right"):
            if mode in (side, "both"):
                actions.append(
                    Node(
                        package="nova2_inspire_retarget",
                        executable="inspire_mujoco_viewer",
                        name=f"inspire_{side}_mujoco_viewer",
                        parameters=[{"side": side}],
                        output="screen",
                    )
                )
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("hand_mode", default_value="right", choices=["left", "right", "both"]),
        DeclareLaunchArgument("mapping_config", default_value="direct_mapping.yaml"),
        DeclareLaunchArgument("nova2_config", default_value="nova2_mapping.yaml"),
        DeclareLaunchArgument("listen_host", default_value="0.0.0.0"),
        DeclareLaunchArgument("udp_port", default_value="15020"),
        DeclareLaunchArgument("launch_viewer", default_value="true"),
        OpaqueFunction(function=_launch_nodes),
    ])

