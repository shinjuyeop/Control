from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package='visualization',
                executable='vehicle_dashboard',
                name='vehicle_dashboard',
                output='screen',
            ),
        ]
    )
