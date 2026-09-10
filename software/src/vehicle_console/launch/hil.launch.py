"""HIL console and optional bench; web:=false selects the headless tools.

Web starts without hardware by default. With start_bench:=true, Web operation
always starts the generator disarmed with its 3 s heartbeat watchdog. Headless
operation preserves the former backend defaults and supports a scenario file.
"""

import os
from pathlib import Path
import sys

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    OpaqueFunction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def _launch_actions(context):
    web = IfCondition(LaunchConfiguration("web")).evaluate(context)
    share = Path(get_package_share_directory("vehicle_console"))
    results_dir = LaunchConfiguration("results_dir").perform(context) or (
        str(Path.home() / ".local/state/kaiev26_hil/web_results") if web else "test_results"
    )
    actions = []
    if IfCondition(LaunchConfiguration("start_bench")).evaluate(context):
        if IfCondition(LaunchConfiguration("agents")).evaluate(context):
            for name, port in (("stm_a_agent", "stm_a_port"), ("stm_b_agent", "stm_b_port")):
                actions.append(Node(
                    package="micro_ros_agent", executable="micro_ros_agent", name=name,
                    output="screen",
                    arguments=["serial", "--dev", LaunchConfiguration(port),
                               "-b", LaunchConfiguration("agent_baud")],
                ))
        config = str(share / "config/hil.yaml")
        actions.extend([
            Node(
                package="vehicle_console", executable="stm_test_bridge", name="stm_test_bridge",
                output="screen", parameters=[config, {"serial_port": LaunchConfiguration("serial_port")}],
            ),
            Node(
                package="vehicle_console", executable="hil_command_generator", name="hil_command_generator",
                output="screen", parameters=[config, {
                    # Web's ARM/heartbeat contract cannot be bypassed by the
                    # options retained for the headless command source.
                    "enabled": False if web else ParameterValue(
                        LaunchConfiguration("generator_enabled"), value_type=bool),
                    "web_heartbeat_timeout_sec": 3.0 if web else ParameterValue(
                        LaunchConfiguration("web_heartbeat_timeout_sec"), value_type=float),
                }],
            ),
            Node(
                package="command_governor", executable="command_governor_node", name="command_governor",
                output="screen", parameters=[os.path.join(
                    get_package_share_directory("command_governor"), "config", "command_governor.yaml")],
            ),
        ])
        scenario_file = LaunchConfiguration("scenario_file").perform(context)
        if scenario_file:
            actions.append(Node(
                package="vehicle_console", executable="hil_scenario_runner", name="hil_scenario_runner",
                output="screen", parameters=[{"scenario_file": scenario_file, "results_dir": results_dir}],
            ))
        if IfCondition(LaunchConfiguration("steering")).evaluate(context):
            actions.append(IncludeLaunchDescription(PythonLaunchDescriptionSource(os.path.join(
                get_package_share_directory("lateral_ctrl"), "launch", "steering_actuator.launch.py"))))
        if IfCondition(LaunchConfiguration("state")).evaluate(context):
            actions.append(Node(
                package="vehicle_state_publisher", executable="vehicle_state_publisher_node",
                name="vehicle_state_publisher", output="screen",
            ))
    if web:
        actions.append(ExecuteProcess(
            cmd=[
                LaunchConfiguration("web_python"), "-m", "vehicle_console.hil.hil_web_bridge",
                ["--port=", LaunchConfiguration("port")],
                ["--bind=", LaunchConfiguration("bind")],
                ["--allowed-hosts=", LaunchConfiguration("allowed_hosts")],
                ["--update-hz=", LaunchConfiguration("update_hz")],
                ["--dashboard-port=", LaunchConfiguration("dashboard_port")],
                "--results-dir=" + results_dir,
            ],
            output="screen",
        ))
    return actions


def generate_launch_description():
    arguments = {
        "web": "true",
        "start_bench": "false",
        "web_python": os.path.join(os.environ["VIRTUAL_ENV"], "bin/python") if os.environ.get("VIRTUAL_ENV") else sys.executable,
        "port": "8765",
        "bind": "127.0.0.1",
        "allowed_hosts": "100.112.72.27,100.112.72.26",
        "update_hz": "5.0",
        "dashboard_port": "",
        "serial_port": "/dev/hil_test",
        "stm_a_port": "/dev/hil_a",
        "stm_b_port": "/dev/hil_b",
        "agents": "true",
        "agent_baud": "460800",
        "steering": "false",
        "state": "false",
        "scenario_file": "",
        "results_dir": "",
        "generator_enabled": "true",
        "web_heartbeat_timeout_sec": "0.0",
    }
    descriptions = {
        "web": "Start the HIL Web server; false keeps only the selected bench tools.",
        "results_dir": "Empty selects ~/.local/state/kaiev26_hil/web_results for Web, test_results for headless.",
        "generator_enabled": "Headless only; Web always starts disarmed.",
        "web_heartbeat_timeout_sec": "Headless only; Web always uses a 3 s heartbeat watchdog.",
        "scenario_file": "Optional standalone scenario runner; use with web:=false start_bench:=true.",
    }
    return LaunchDescription([
        *[DeclareLaunchArgument(name, default_value=value, description=descriptions.get(name, ""))
          for name, value in arguments.items()],
        OpaqueFunction(function=_launch_actions),
    ])
