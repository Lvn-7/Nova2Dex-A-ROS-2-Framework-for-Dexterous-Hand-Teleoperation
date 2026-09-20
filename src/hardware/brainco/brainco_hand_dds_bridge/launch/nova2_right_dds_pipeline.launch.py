from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("nova2_config", default_value="nova2_mapping.yaml"),
        DeclareLaunchArgument("listen_host", default_value="0.0.0.0"),
        DeclareLaunchArgument("udp_port", default_value="15020"),
        DeclareLaunchArgument("control_config", default_value="retarget.yaml"),
        DeclareLaunchArgument("retarget_config", default_value=""),
        DeclareLaunchArgument("bridge_config", default_value="right_bridge.yaml"),
        DeclareLaunchArgument("network_interface", default_value=""),
        DeclareLaunchArgument("bridge_delay", default_value="1.0"),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution([
                FindPackageShare("brainco_hand_dds_bridge"),
                "launch",
                "nova2_dds_pipeline.launch.py",
            ])),
            launch_arguments={
                "hand_mode": "right",
                "nova2_config": LaunchConfiguration("nova2_config"),
                "listen_host": LaunchConfiguration("listen_host"),
                "udp_port": LaunchConfiguration("udp_port"),
                "control_config": LaunchConfiguration("control_config"),
                "retarget_config": LaunchConfiguration("retarget_config"),
                "right_bridge_config": LaunchConfiguration("bridge_config"),
                "network_interface": LaunchConfiguration("network_interface"),
                "bridge_delay": LaunchConfiguration("bridge_delay"),
            }.items(),
        ),
    ])
