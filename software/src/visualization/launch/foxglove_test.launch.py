from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package='visualization',
                executable='test_publisher',
                name='test_publisher',
                output='screen',
            ),
            Node(
                package='visualization',
                executable='gps_waypoint_publisher',
                name='gps_waypoint_publisher',
                output='screen',
            ),
            Node(
                package='visualization',
                executable='vehicle_dashboard',
                name='vehicle_dashboard',
                output='screen',
            ),
        ]
    )
