# pyright: reportMissingImports=false
from launch import LaunchDescription
from launch_ros.actions import Node

"""        
        # steering_command_publisher_node 필요 시 아래에 추가
        Node(
            package='steering_control',
            executable='steering_command_publisher_node',
            name='steering_command_publisher',
            output='screen',
        ),
"""

def generate_launch_description():
    return LaunchDescription([
        # 터미널 1: steering_control_node
        Node(
            package='steering_control',
            executable='steering_control_node',
            name='steering_control',
            output='screen',
            arguments=['can1'],
        ),

        # 터미널 2: steering_feedback_node
        Node(
            package='steering_control',
            executable='steering_feedback_node',
            name='steering_feedback',
            output='screen',
            arguments=['can1'],
        ),

        # 터미널 3: steering_monitor_node
        Node(
            package='steering_monitor_cpp',
            executable='steering_monitor_node',
            name='steering_monitor',
            output='screen',
        ),
    ])
