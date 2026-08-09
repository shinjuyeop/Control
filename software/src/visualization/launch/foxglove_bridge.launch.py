from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument('port', default_value='8765'),
            DeclareLaunchArgument('address', default_value='0.0.0.0'),
            Node(
                package='foxglove_bridge',
                executable='foxglove_bridge',
                name='foxglove_bridge',
                output='screen',
                parameters=[
                    {
                        'port': ParameterValue(
                            LaunchConfiguration('port'), value_type=int
                        ),
                        'address': LaunchConfiguration('address'),
                        'send_buffer_limit': 100000000,
                    }
                ],
            ),
        ]
    )
