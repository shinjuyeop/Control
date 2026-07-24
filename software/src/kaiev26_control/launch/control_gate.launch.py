from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import os


def generate_launch_description():
    control_share = get_package_share_directory("kaiev26_control")
    control_config = os.path.join(control_share, "config", "control_command_gate.yaml")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="true",
                description="Use Gazebo/ROS simulation clock.",
            ),
            Node(
                package="kaiev26_control",
                executable="control_command_gate_node",
                name="kaiev26_control_command_gate",
                output="screen",
                parameters=[control_config, {"use_sim_time": LaunchConfiguration("use_sim_time")}],
            ),
        ]
    )
