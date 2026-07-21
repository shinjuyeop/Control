from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import os


def generate_launch_description():
    share = get_package_share_directory('lateral_ctrl')
    actuator_config = os.path.join(share, 'config', 'steering_actuator.yaml')
    bench_config = os.path.join(share, 'config', 'steering_bench_test.yaml')
    actuator = Node(
        package='lateral_ctrl',
        executable='steering_actuator_node',
        name='steering_actuator_node',
        output='screen',
        parameters=[
            actuator_config,
            {
                'can_interface': LaunchConfiguration('can_interface'),
                'hardware_can_enabled': LaunchConfiguration('hardware_can_enabled'),
                'actuator_output_enabled': LaunchConfiguration('actuator_output_enabled'),
                'test_input_injection_enabled': True,
                'max_steering_rate_deg_s': 5.0,
            },
        ],
    )
    bench = Node(
        package='lateral_ctrl',
        executable='steering_bench_test_node',
        name='steering_bench_test_node',
        output='screen',
        parameters=[
            bench_config,
            {
                'drive_mode_publish_enabled': LaunchConfiguration(
                    'drive_mode_publish_enabled'),
                'steer_sequence_publish_enabled': LaunchConfiguration(
                    'steer_sequence_publish_enabled'),
                'safety_injection_enabled': LaunchConfiguration(
                    'safety_injection_enabled'),
            },
        ],
    )
    return LaunchDescription([
        DeclareLaunchArgument('can_interface', default_value='can0'),
        DeclareLaunchArgument('hardware_can_enabled', default_value='true'),
        DeclareLaunchArgument('actuator_output_enabled', default_value='true'),
        DeclareLaunchArgument('drive_mode_publish_enabled', default_value='true'),
        DeclareLaunchArgument('steer_sequence_publish_enabled', default_value='true'),
        DeclareLaunchArgument('safety_injection_enabled', default_value='false'),
        actuator,
        bench,
        RegisterEventHandler(
            OnProcessExit(
                target_action=bench,
                on_exit=[EmitEvent(event=Shutdown(
                    reason='bench node exited; stopping actuator node'))],
            )
        ),
    ])
