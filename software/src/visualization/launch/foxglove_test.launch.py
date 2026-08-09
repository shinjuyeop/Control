import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    share = get_package_share_directory('visualization')
    bridge_launch = os.path.join(share, 'launch', 'foxglove_bridge.launch.py')
    return LaunchDescription(
        [
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(bridge_launch),
            ),
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
