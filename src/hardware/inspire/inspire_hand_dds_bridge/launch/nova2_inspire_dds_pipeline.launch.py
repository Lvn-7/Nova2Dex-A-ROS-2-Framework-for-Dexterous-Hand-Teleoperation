"""Nova2 -> Inspire retarget -> Unitree DDS pipeline for G1-mounted hands."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def _selected_sides(mode):
    return ("left", "right") if mode == "both" else (mode,)


def _create_actions(context):
    mode = LaunchConfiguration("hand_mode").perform(context).strip().lower()
    if mode not in ("left", "right", "both"):
        raise ValueError("hand_mode must be left, right, or both")

    sides = _selected_sides(mode)
    nova2_share = get_package_share_directory("nova2_glove_driver")
    retarget_share = get_package_share_directory("nova2_inspire_retarget")
    bridge_share = get_package_share_directory("inspire_hand_dds_bridge")

    nova2_config = LaunchConfiguration("nova2_config").perform(context)
    if not os.path.isabs(nova2_config):
        nova2_config = os.path.join(nova2_share, "config", nova2_config)
    mapping_config = LaunchConfiguration("mapping_config").perform(context)
    if not os.path.isabs(mapping_config):
        mapping_config = os.path.join(retarget_share, "config", mapping_config)

    actions = [
        Node(
            package="nova2_glove_driver",
            executable="nova2_udp_bridge.py",
            name="nova2_glove_driver",
            parameters=[
                nova2_config,
                {
                    "enable_left": "left" in sides,
                    "enable_right": "right" in sides,
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

    bridges = []
    for side in sides:
        bridge_config = LaunchConfiguration(f"{side}_bridge_config").perform(context)
        if not os.path.isabs(bridge_config):
            bridge_config = os.path.join(bridge_share, "config", bridge_config)
        bridges.append(
            Node(
                package="inspire_hand_dds_bridge",
                executable="inspire_jointstate_to_dds",
                name=f"inspire_{side}_jointstate_to_dds",
                parameters=[
                    bridge_config,
                    {
                        "network_interface": LaunchConfiguration("network_interface"),
                        "command_speed": ParameterValue(
                            LaunchConfiguration("command_speed"), value_type=float
                        ),
                    },
                ],
                output="screen",
            )
        )
    actions.append(
        TimerAction(period=LaunchConfiguration("bridge_delay"), actions=bridges)
    )
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("hand_mode", default_value="right", choices=["left", "right", "both"]),
        DeclareLaunchArgument("nova2_config", default_value="nova2_mapping.yaml"),
        DeclareLaunchArgument("mapping_config", default_value="direct_mapping.yaml"),
        DeclareLaunchArgument("left_bridge_config", default_value="left_bridge.yaml"),
        DeclareLaunchArgument("right_bridge_config", default_value="right_bridge.yaml"),
        DeclareLaunchArgument("listen_host", default_value="0.0.0.0"),
        DeclareLaunchArgument("udp_port", default_value="15020"),
        DeclareLaunchArgument("network_interface", default_value=""),
        DeclareLaunchArgument("command_speed", default_value="0.5"),
        DeclareLaunchArgument("bridge_delay", default_value="1.0"),
        OpaqueFunction(function=_create_actions),
    ])

