"""React console; opt in to starting the complete hardware bench.

The default starts only the Web bridge, without touching USB/CAN devices.
Use start_bench:=true for the Web-controlled bench (generator starts disarmed).
"""

import sys
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    share = Path(get_package_share_directory("kaiev26_hil"))
    arguments = {
        "start_bench": "false",
        "web_python": sys.executable,
        "port": "8765",
        "dashboard_port": "",
        "serial_port": "/dev/hil_test",
        "stm_a_port": "/dev/hil_a",
        "stm_b_port": "/dev/hil_b",
        "agents": "true",
        "agent_baud": "460800",
        "steering": "false",
        "state": "false",
        "results_dir": str(Path.home() / ".local/state/kaiev26_hil/web_results"),
    }
    return LaunchDescription(
        [
            *[
                DeclareLaunchArgument(name, default_value=value)
                for name, value in arguments.items()
            ],
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    str(share / "launch/hil_bench.launch.py")
                ),
                condition=IfCondition(LaunchConfiguration("start_bench")),
                launch_arguments={
                    **{
                        key: LaunchConfiguration(key)
                        for key in (
                            "serial_port",
                            "stm_a_port",
                            "stm_b_port",
                            "agents",
                            "agent_baud",
                            "steering",
                            "state",
                        )
                    },
                    "gui": "false",
                    "generator_enabled": "false",
                    "web_heartbeat_timeout_sec": "3.0",
                }.items(),
            ),
            ExecuteProcess(
                cmd=[
                    LaunchConfiguration("web_python"),
                    "-m",
                    "kaiev26_hil.hil_web_bridge",
                    "--port",
                    LaunchConfiguration("port"),
                    "--dashboard-port",
                    LaunchConfiguration("dashboard_port"),
                    "--results-dir",
                    LaunchConfiguration("results_dir"),
                ],
                output="screen",
            ),
        ]
    )
