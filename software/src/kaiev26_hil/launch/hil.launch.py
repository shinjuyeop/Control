from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
import os


def generate_launch_description():
    share = get_package_share_directory("kaiev26_hil")
    config = os.path.join(share, "config", "hil.yaml")
    scenario_file = LaunchConfiguration("scenario_file")
    return LaunchDescription([
        DeclareLaunchArgument("serial_port", default_value="/dev/hil_test"),
        DeclareLaunchArgument("scenario_file", default_value=""),
        DeclareLaunchArgument("results_dir", default_value="test_results"),
        Node(
            package="kaiev26_hil", executable="stm_test_bridge", name="stm_test_bridge",
            output="screen", parameters=[config, {"serial_port": LaunchConfiguration("serial_port")}],
        ),
        Node(
            package="kaiev26_hil", executable="command_generator", name="hil_command_generator",
            output="screen", parameters=[config],
        ),
        Node(
            package="kaiev26_hil", executable="scenario_runner", name="hil_scenario_runner",
            output="screen",
            parameters=[{"scenario_file": scenario_file, "results_dir": LaunchConfiguration("results_dir")}],
            condition=IfCondition(PythonExpression(["'", scenario_file, "' != ''"])),
        ),
    ])
