"""Whole HIL bench in one launch: both micro-ROS Agents plus bridge/generator/GUI.

This is the runbook's "현재 bench 실행" section as a single command. It is the HIL
bench stack only. It is not the integrated vehicle launch: `lateral_ctrl`,
`vehicle_state_publisher` and `visualization` stay out, because the bench has no
KEYA CAN2 transceiver yet and those nodes are started per the runbook when it
does.

Each Agent owns one serial device, so a device that is already open elsewhere
makes only that Agent fail. Run the parts separately when you need to read one
Agent's XRCE session log without the other two nodes interleaved into it.
"""

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import os


def _agent(name, port, baud):
    return Node(
        package="micro_ros_agent",
        executable="micro_ros_agent",
        name=name,
        output="screen",
        arguments=["serial", "--dev", port, "-b", baud],
        condition=IfCondition(LaunchConfiguration("agents")),
    )


def generate_launch_description():
    share = get_package_share_directory("kaiev26_hil")
    hil_launch = os.path.join(share, "launch", "hil.launch.py")
    baud = LaunchConfiguration("agent_baud")
    return LaunchDescription([
        DeclareLaunchArgument("serial_port", default_value="/dev/hil_test"),
        DeclareLaunchArgument("stm_a_port", default_value="/dev/hil_a"),
        DeclareLaunchArgument("stm_b_port", default_value="/dev/hil_b"),
        DeclareLaunchArgument("agent_baud", default_value="115200"),
        DeclareLaunchArgument(
            "agents", default_value="true",
            description="Start both micro-ROS Agents. Set false when they already run.",
        ),
        _agent("stm_a_agent", LaunchConfiguration("stm_a_port"), baud),
        _agent("stm_b_agent", LaunchConfiguration("stm_b_port"), baud),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(hil_launch),
            launch_arguments={
                "serial_port": LaunchConfiguration("serial_port"),
                "gui": "true",
            }.items(),
        ),
    ])
