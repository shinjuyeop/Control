from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # 터미널 1: steering_control_node
        Node(
            package='steering_control',
            executable='steering_control_node',
            name='steering_control',
            output='screen'
        ),

        # 터미널 2: steering_feedback_node
        Node(
            package='steering_control',
            executable='steering_feedback_node',
            name='steering_feedback',
            output='screen'
        ),
    ])
