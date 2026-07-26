from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import os


def generate_launch_description():
    bridge_share = get_package_share_directory("kaiev26_actuator_command_bridge")
    bridge_config = os.path.join(
        bridge_share, "config", "actuator_command_bridge.yaml"
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="true",
                description="Use Gazebo/ROS simulation clock.",
            ),
            Node(
                package="kaiev26_actuator_command_bridge",
                executable="actuator_command_bridge_node",
                name="kaiev26_actuator_command_bridge",
                output="screen",
                parameters=[
                    bridge_config,
                    {"use_sim_time": LaunchConfiguration("use_sim_time")},
                ],
            ),
        ]
    )
