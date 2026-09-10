"""Whole HIL bench in one launch: both micro-ROS Agents plus bridge/generator/
governor/GUI, optionally with steering and vehicle state.

This is the runbook's "현재 bench 실행" section as a single command. Defaults
keep the historical minimal scope (no `lateral_ctrl`, no
`vehicle_state_publisher`); the KEYA CAN2 transceiver has been installed since
2026-08-12, so `steering:=true` adds the steering stack on `can0` and
`state:=true` adds `/vehicle/state` fusion — replacing the manual runbook
steps. `visualization` stays out: observation is launched separately.

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
    steering_launch = os.path.join(
        get_package_share_directory("lateral_ctrl"),
        "launch", "steering_actuator.launch.py",
    )
    baud = LaunchConfiguration("agent_baud")
    return LaunchDescription([
        DeclareLaunchArgument("serial_port", default_value="/dev/hil_test"),
        DeclareLaunchArgument("stm_a_port", default_value="/dev/hil_a"),
        DeclareLaunchArgument("stm_b_port", default_value="/dev/hil_b"),
        DeclareLaunchArgument("agent_baud", default_value="460800"),
        DeclareLaunchArgument(
            "gui", default_value="true",
            description="HIL GUI를 함께 띄운다. headless/scenario 실행이면 false.",
        ),
        DeclareLaunchArgument(
            "gui_refresh_ms", default_value="200",
            description="GUI 갱신 주기(ms). 원격 X11 기준값이며 로컬 화면이면 100.",
        ),
        DeclareLaunchArgument(
            "agents", default_value="true",
            description="Start both micro-ROS Agents. Set false when they already run.",
        ),
        DeclareLaunchArgument(
            "steering", default_value="false",
            description="can0의 KEYA 조향 스택을 함께 띄운다 (runbook 10절 대체).",
        ),
        DeclareLaunchArgument(
            "state", default_value="false",
            description="vehicle_state_publisher를 함께 띄운다 (runbook 11절 대체).",
        ),
        DeclareLaunchArgument(
            "scenario_file", default_value="",
            description="비어 있지 않으면 scenario runner를 함께 띄운다 (hil.launch.py로 통과).",
        ),
        DeclareLaunchArgument("results_dir", default_value="test_results"),
        DeclareLaunchArgument("generator_enabled", default_value="true"),
        DeclareLaunchArgument("web_heartbeat_timeout_sec", default_value="0.0"),
        _agent("stm_a_agent", LaunchConfiguration("stm_a_port"), baud),
        _agent("stm_b_agent", LaunchConfiguration("stm_b_port"), baud),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(hil_launch),
            launch_arguments={
                "serial_port": LaunchConfiguration("serial_port"),
                "gui": LaunchConfiguration("gui"),
                "gui_refresh_ms": LaunchConfiguration("gui_refresh_ms"),
                "scenario_file": LaunchConfiguration("scenario_file"),
                "results_dir": LaunchConfiguration("results_dir"),
                "generator_enabled": LaunchConfiguration("generator_enabled"),
                "web_heartbeat_timeout_sec": LaunchConfiguration("web_heartbeat_timeout_sec"),
            }.items(),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(steering_launch),
            condition=IfCondition(LaunchConfiguration("steering")),
        ),
        Node(
            package="vehicle_state_publisher",
            executable="vehicle_state_publisher_node",
            name="vehicle_state_publisher",
            output="screen",
            condition=IfCondition(LaunchConfiguration("state")),
        ),
    ])
