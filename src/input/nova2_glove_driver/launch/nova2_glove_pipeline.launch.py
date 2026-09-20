import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def _create_actions(context, *args, **kwargs):
    del args, kwargs

    nova2_share = get_package_share_directory("nova2_glove_driver")
    retarget_share = get_package_share_directory("manus_revo2_retarget")
    nova2_config = LaunchConfiguration("nova2_config").perform(context)
    if not os.path.isabs(nova2_config):
        nova2_config = os.path.join(nova2_share, "config", nova2_config)

    hand_mode = LaunchConfiguration("hand_mode")
    use_revo2_pipeline = LaunchConfiguration("launch_revo2_pipeline")

    actions = [
        Node(
            package="nova2_glove_driver",
            executable="nova2_udp_bridge.py",
            name="nova2_glove_driver",
            parameters=[
                nova2_config,
                {
                    "enable_left": ParameterValue(LaunchConfiguration("enable_left"), value_type=bool),
                    "enable_right": ParameterValue(LaunchConfiguration("enable_right"), value_type=bool),
                    "listen_host": LaunchConfiguration("listen_host"),
                    "udp_port": ParameterValue(LaunchConfiguration("udp_port"), value_type=int),
                },
            ],
            output="screen",
        )
    ]

    if LaunchConfiguration("launch_revo2_pipeline").perform(context).lower() in ("1", "true", "yes", "on"):
        actions.append(
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(retarget_share, "launch", "real_hand_pipeline_launch.py")
                ),
                launch_arguments={
                    "hand_mode": hand_mode,
                    "controller_backend": LaunchConfiguration("controller_backend"),
                    "if_sim": LaunchConfiguration("if_sim"),
                    "launch_manus_publisher": "false",
                    "launch_plot": LaunchConfiguration("launch_plot"),
                    "switch_controllers": LaunchConfiguration("switch_controllers"),
                    "launch_driver": LaunchConfiguration("launch_driver"),
                    "launch_retarget": use_revo2_pipeline,
                }.items(),
            )
        )

    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("hand_mode", default_value="right", choices=["left", "right", "both"]),
        DeclareLaunchArgument("nova2_config", default_value="nova2_mapping.yaml"),
        DeclareLaunchArgument("enable_left", default_value="true"),
        DeclareLaunchArgument("enable_right", default_value="true"),
        DeclareLaunchArgument("listen_host", default_value="0.0.0.0"),
        DeclareLaunchArgument("udp_port", default_value="15020"),
        DeclareLaunchArgument("launch_revo2_pipeline", default_value="true"),
        DeclareLaunchArgument("controller_backend", default_value="ros2_control"),
        DeclareLaunchArgument("if_sim", default_value="true"),
        DeclareLaunchArgument("launch_plot", default_value="false"),
        DeclareLaunchArgument("switch_controllers", default_value="true"),
        DeclareLaunchArgument("launch_driver", default_value="true"),
        OpaqueFunction(function=_create_actions),
    ])
