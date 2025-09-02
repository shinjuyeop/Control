# pyright: reportMissingImports=false
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

"""
        # speed_publisher 필요 시 아래에 추가
        Node(
            package='motor_control_cpp',
            executable='speed_publisher',
            name='speed_publisher',
            output='screen',
            parameters=[{'rpm': LaunchConfiguration('rpm')}],
        ),
"""

def generate_launch_description():
    rpm_arg = DeclareLaunchArgument(
        'rpm',
        default_value='250',
        description='Initial RPM for motor_control_cpp/speed_publisher'
    )

    can0 = 'can0'

    return LaunchDescription([
        rpm_arg,



        Node(
            package='throttle_serial_py',
            executable='throttle_serial_node',
            name='throttle_serial',
            output='screen',
        ),

        Node(
            package='throttle_serial_py',
            executable='rpm_mux',
            name='rpm_mux',
            output='screen',
        ),

        # can0로 고정 (argv 방식)
        Node(
            package='motor_control_cpp',
            executable='can_speed_sender',
            name='can_speed_sender',
            output='screen',
            arguments=[can0],
        ),
        Node(
            package='motor_control_cpp',
            executable='can_speed_receiver',
            name='can_speed_receiver',
            output='screen',
            arguments=[can0],
        ),

        Node(
            package='speed_monitor_cpp',
            executable='speed_monitor_node',
            name='speed_monitor',
            output='screen',
        ),
    ])