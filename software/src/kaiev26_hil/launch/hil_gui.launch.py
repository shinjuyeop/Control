from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import os


def generate_launch_description():
    share = get_package_share_directory("kaiev26_hil")
    config = os.path.join(share, "config", "hil.yaml")
    return LaunchDescription([
        DeclareLaunchArgument("serial_port", default_value="/dev/serial/by-id/REPLACE_WITH_STM_TEST"),
        Node(package="kaiev26_hil", executable="stm_test_bridge", name="stm_test_bridge",
             output="screen", parameters=[config, {"serial_port": LaunchConfiguration("serial_port")}]),
        Node(package="kaiev26_hil", executable="command_generator", name="hil_command_generator",
             output="screen", parameters=[config]),
        Node(package="kaiev26_hil", executable="hil_gui", name="hil_gui", output="screen"),
    ])
