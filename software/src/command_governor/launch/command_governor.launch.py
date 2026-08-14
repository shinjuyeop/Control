"""command_governor 단독 실행. /planning/command -> /vehicle/command 패스스루.

이 노드가 /vehicle/command의 유일한 발행자여야 한다. hil_command_generator를
/vehicle/command로 직접 발행하도록 되돌리거나, steering_bench_test_node 또는
visualization test_publisher와 함께 띄우면 발행자가 둘이 되어 명령이 서로
덮어쓴다.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    share = get_package_share_directory("command_governor")
    config = os.path.join(share, "config", "command_governor.yaml")
    return LaunchDescription([
        Node(
            package="command_governor",
            executable="command_governor_node",
            name="command_governor",
            output="screen",
            parameters=[config],
        ),
    ])
