from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    # bridge만 띄운다. /diagnostics는 vehicle_state_publisher package가
    # control_launch/vehicle.launch.py를 통해 독립 노드로 발행한다.
    # bridge는 PC당 하나만 실행한다 — 하나가 ROS graph 전체를 서빙하고
    # 클라이언트 여러 대가 동시 접속할 수 있다.
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
