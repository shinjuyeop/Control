from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("motor_iface", default_value="vcan0"),
        DeclareLaunchArgument("steer_iface", default_value="vcan1"),
        DeclareLaunchArgument("no_gui", default_value="false"),
        Node(
            package="kaiev26_hil", executable="vcan_sim", name="kaiev26_vcan_sim",
            output="screen",
            arguments=[
                "--motor-iface", LaunchConfiguration("motor_iface"),
                "--steer-iface", LaunchConfiguration("steer_iface"),
                "--ros-no-gui", LaunchConfiguration("no_gui"),
            ],
        ),
    ])
