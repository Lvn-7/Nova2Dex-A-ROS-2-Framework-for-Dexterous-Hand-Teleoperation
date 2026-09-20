import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def _selected_sides(hand_mode):
    if hand_mode == "both":
        return ("left", "right")
    return (hand_mode,)


def _create_actions(context, *args, **kwargs):
    del args, kwargs

    hand_mode = LaunchConfiguration("hand_mode").perform(context).strip().lower()
    if hand_mode not in ("left", "right", "both"):
        raise ValueError("hand_mode must be one of: left, right, both")

    nova2_share = get_package_share_directory("nova2_glove_driver")
    retarget_share = get_package_share_directory("manus_revo2_retarget")
    bridge_share = get_package_share_directory("brainco_hand_dds_bridge")

    nova2_config = LaunchConfiguration("nova2_config").perform(context)
    if not os.path.isabs(nova2_config):
        nova2_config = os.path.join(nova2_share, "config", nova2_config)

    sides = _selected_sides(hand_mode)
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
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(retarget_share, "launch", "pipeline_launch.py")
            ),
            launch_arguments={
                "hand_mode": hand_mode,
                "launch_manus_publisher": "false",
                "use_split_controller": "true",
                "controller_backend": "ros2_control",
                "control_config": LaunchConfiguration("control_config"),
                "retarget_config": LaunchConfiguration("retarget_config"),
            }.items(),
        ),
    ]

    bridge_nodes = []
    for side in sides:
        bridge_config = LaunchConfiguration(f"{side}_bridge_config").perform(context)
        if not os.path.isabs(bridge_config):
            bridge_config = os.path.join(bridge_share, "config", bridge_config)
        bridge_nodes.append(
            Node(
                package="brainco_hand_dds_bridge",
                executable="revo2_jointstate_to_brainco_dds",
                name=f"revo2_{side}_jointstate_to_brainco_dds",
                parameters=[
                    bridge_config,
                    {"network_interface": LaunchConfiguration("network_interface")},
                ],
                output="screen",
            )
        )

    actions.append(
        TimerAction(
            period=LaunchConfiguration("bridge_delay"),
            actions=bridge_nodes,
        )
    )
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            "hand_mode",
            default_value="right",
            choices=["left", "right", "both"],
            description="Which hand to run: left, right, or both.",
        ),
        DeclareLaunchArgument("nova2_config", default_value="nova2_mapping.yaml"),
        DeclareLaunchArgument("listen_host", default_value="0.0.0.0"),
        DeclareLaunchArgument("udp_port", default_value="15020"),
        DeclareLaunchArgument("control_config", default_value="retarget.yaml"),
        DeclareLaunchArgument("retarget_config", default_value=""),
        DeclareLaunchArgument("left_bridge_config", default_value="left_bridge.yaml"),
        DeclareLaunchArgument("right_bridge_config", default_value="right_bridge.yaml"),
        DeclareLaunchArgument("network_interface", default_value=""),
        DeclareLaunchArgument("bridge_delay", default_value="1.0"),
        OpaqueFunction(function=_create_actions),
    ])
