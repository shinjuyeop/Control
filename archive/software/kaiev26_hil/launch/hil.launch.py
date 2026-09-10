"""Headless STM_TEST bridge, command generator and governor; optional scenarios.

For the shared browser console use hil_web.launch.py. The generator defaults
and command/watchdog behavior are unchanged. Never duplicate this stack.
"""

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
import os


def generate_launch_description():
    share = get_package_share_directory("kaiev26_hil")
    config = os.path.join(share, "config", "hil.yaml")
    scenario_file = LaunchConfiguration("scenario_file")
    return LaunchDescription([
        DeclareLaunchArgument("serial_port", default_value="/dev/hil_test"),
        DeclareLaunchArgument(
            "scenario_file", default_value="",
            description="비어 있지 않으면 scenario runner를 함께 띄운다.",
        ),
        DeclareLaunchArgument("results_dir", default_value="test_results"),
        DeclareLaunchArgument("generator_enabled", default_value="true"),
        DeclareLaunchArgument("web_heartbeat_timeout_sec", default_value="0.0"),
        Node(
            package="kaiev26_hil", executable="stm_test_bridge", name="stm_test_bridge",
            output="screen",
            parameters=[config, {"serial_port": LaunchConfiguration("serial_port")}],
        ),
        Node(
            package="kaiev26_hil", executable="hil_command_generator",
            name="hil_command_generator",
            output="screen", parameters=[config, {
                "enabled": ParameterValue(LaunchConfiguration("generator_enabled"), value_type=bool),
                "web_heartbeat_timeout_sec": ParameterValue(
                    LaunchConfiguration("web_heartbeat_timeout_sec"), value_type=float),
            }],
        ),
        # /vehicle/command의 유일한 발행자. 자기 패키지의 config를 그대로 쓴다.
        Node(
            package="command_governor", executable="command_governor_node",
            name="command_governor", output="screen",
            parameters=[os.path.join(
                get_package_share_directory("command_governor"),
                "config", "command_governor.yaml",
            )],
        ),
        Node(
            package="kaiev26_hil", executable="hil_scenario_runner",
            name="hil_scenario_runner",
            output="screen",
            parameters=[{
                "scenario_file": scenario_file,
                "results_dir": LaunchConfiguration("results_dir"),
            }],
            condition=IfCondition(PythonExpression(["'", scenario_file, "' != ''"])),
        ),
    ])
