from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                'ui_update_hz',
                default_value='4.0',
                description=(
                    'GUI 갱신 주기(Hz). 매 tick마다 두 plot을 전부 다시 그리므로 '
                    '원격 X11에서는 이 값이 대역폭을 그대로 결정한다. 로컬 화면이면 10.'
                ),
            ),
            Node(
                package='visualization',
                executable='vehicle_dashboard',
                name='vehicle_dashboard',
                output='screen',
                parameters=[
                    {
                        'ui_update_hz': ParameterValue(
                            LaunchConfiguration('ui_update_hz'), value_type=float
                        ),
                    }
                ],
            ),
        ]
    )
