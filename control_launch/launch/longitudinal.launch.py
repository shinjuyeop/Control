from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    rpm_arg = DeclareLaunchArgument(
        'rpm',
        default_value='250',
        description='Initial RPM for motor_control_cpp/speed_publisher'
    )

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
        Node(
            package='motor_control_cpp',
            executable='can_speed_sender',
            name='can_speed_sender',
            output='screen'
        ),
        Node(
            package='motor_control_cpp',
            executable='can_speed_receiver',
            name='can_speed_receiver',
            output='screen'
        ),
        Node(
            package='speed_monitor_cpp',
            executable='speed_monitor_node',
            name='speed_monitor',
            output='screen'
        ),
        # ✅ 추가: speed_publisher 노드 (rpm 인자 전달)
        Node(
            package='motor_control_cpp',
            executable='speed_publisher',
            name='speed_publisher',
            output='screen',
            parameters=[{'rpm': LaunchConfiguration('rpm')}]
        ),
    ])