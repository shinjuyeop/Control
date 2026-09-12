"""Observe an existing ROS graph. Starts no vehicle controllers or HIL machinery."""

import sys
import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    arguments = {
        "web_python": os.path.join(os.environ["VIRTUAL_ENV"], "bin/python") if os.environ.get("VIRTUAL_ENV") else sys.executable,
        "bind": "127.0.0.1", "port": "8766", "allowed_hosts": "",
        "update_hz": "5.0",
        "command_stale_ms": "300", "drive_stale_ms": "300",
        "brake_stale_ms": "300", "steering_stale_ms": "500",
    }
    return LaunchDescription([
        *[DeclareLaunchArgument(name, default_value=value) for name, value in arguments.items()],
        ExecuteProcess(cmd=[LaunchConfiguration("web_python"), "-m", "vehicle_console.vehicle",
                            *[["--" + name.replace("_", "-") + "=", LaunchConfiguration(name)]
                              for name in arguments if name != "web_python"]],
                       output="screen"),
    ])
