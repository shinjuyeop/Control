from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument('port', default_value='8765'),
            DeclareLaunchArgument('address', default_value='0.0.0.0'),
            DeclareLaunchArgument(
                'diagnostics', default_value='true',
                description='status 3종을 /diagnostics로 요약하는 노드를 함께 띄운다.',
            ),
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
            # 관찰 계열은 이 launch에 모은다. 실차 vehicle.launch.py는 구동
            # 노드만 띄우므로 Foxglove Diagnostics 패널의 소스가 여기 있어야
            # 관찰을 켤 때 함께 올라온다.
            Node(
                package='visualization',
                executable='diagnostics_publisher',
                name='diagnostics_publisher',
                output='screen',
                condition=IfCondition(LaunchConfiguration('diagnostics')),
            ),
        ]
    )
