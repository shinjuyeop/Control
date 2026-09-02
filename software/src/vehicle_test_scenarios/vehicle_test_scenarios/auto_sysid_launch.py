"""Shared no-argument launch composition for one AUTO sysid run."""

from __future__ import annotations

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, LogInfo
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


CAMPAIGN_NAME = "260825_auto_sysid"


def auto_sysid_launch_description(
    config_filename: str,
    session_name: str,
    scenario_summary: str,
) -> LaunchDescription:
    scenario_share = get_package_share_directory("vehicle_test_scenarios")
    vehicle_launch = os.path.join(
        get_package_share_directory("control_launch"),
        "launch",
        "vehicle.launch.py",
    )
    logging_launch = os.path.join(
        get_package_share_directory("lateral_ctrl"),
        "launch",
        "mcap_logging.launch.py",
    )
    scenario_config = os.path.join(
        scenario_share,
        "config",
        config_filename,
    )
    return LaunchDescription(
        [
            LogInfo(
                msg=(
                    f"AUTO sysid: {scenario_summary}; "
                    "MCAP starts now; AUTO detection starts the YAML countdown"
                )
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(vehicle_launch),
            ),
            Node(
                package="vehicle_test_scenarios",
                executable="vehicle_test_scenario_runner",
                name="vehicle_test_scenario_runner",
                output="screen",
                parameters=[scenario_config],
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(logging_launch),
                launch_arguments={
                    "campaign_name": CAMPAIGN_NAME,
                    "session_name": session_name,
                    "require_external_storage": "false",
                }.items(),
            ),
        ]
    )
