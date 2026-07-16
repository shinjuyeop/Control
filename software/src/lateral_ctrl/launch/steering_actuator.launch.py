from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import os


def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('lateral_ctrl'), 'config', 'steering_actuator.yaml')
    return LaunchDescription([
        DeclareLaunchArgument('hardware_can_enabled', default_value='false'),
        DeclareLaunchArgument('actuator_output_enabled', default_value='false'),
        Node(
            package='lateral_ctrl',
            executable='steering_actuator_node',
            name='steering_actuator_node',
            output='screen',
            parameters=[
                config,
                {
                    'hardware_can_enabled': LaunchConfiguration('hardware_can_enabled'),
                    'actuator_output_enabled': LaunchConfiguration('actuator_output_enabled'),
                    'test_input_injection_enabled': False,
                    'tracking_control_enabled': False,
                },
            ],
        ),
    ])
