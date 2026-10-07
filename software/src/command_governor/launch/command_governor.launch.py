"""command_governor 단독 실행. /planning/command -> /vehicle/command 감속 slew.

이 노드가 /vehicle/command의 유일한 발행자여야 한다. hil_command_generator를
/vehicle/command로 직접 발행하도록 되돌리거나 steering_bench_test_node와 함께
띄우면 발행자가 둘이 되어 명령이 서로 덮어쓴다.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from control_tuning.storage import default_settings_path
from control_tuning.presets import load_launch_overrides
from control_tuning.conditions import CONDITION_KEYS


def _make_governor(context, config):
    return [Node(
        package="command_governor", executable="command_governor_node",
        name="command_governor", output="screen",
        parameters=[config, load_launch_overrides(
            LaunchConfiguration("control_tuning_file").perform(context), "command_governor",
            LaunchConfiguration("tuning_preset").perform(context), governor_config=config)],
    )]


def generate_launch_description():
    share = get_package_share_directory("command_governor")
    config = os.path.join(share, "config", "command_governor.yaml")
    return LaunchDescription([
        DeclareLaunchArgument("control_tuning_file", default_value=str(default_settings_path())),
        DeclareLaunchArgument("tuning_preset", default_value="driving", choices=list(CONDITION_KEYS)),
        OpaqueFunction(function=_make_governor, kwargs={"config": config}),
    ])
