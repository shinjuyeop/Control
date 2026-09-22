from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    EmitEvent,
    IncludeLaunchDescription,
    RegisterEventHandler,
)
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import os


def generate_launch_description():
    share = get_package_share_directory('control_tools')
    lateral_share = get_package_share_directory('lateral_ctrl')
    bench_config = os.path.join(share, 'config', 'steering_bench_test.yaml')
    actuator = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(lateral_share, 'launch', 'steering_actuator.launch.py')
        ),
        launch_arguments={
            'can_interface': LaunchConfiguration('can_interface'),
            'actuation_mode': LaunchConfiguration('actuation_mode'),
            'steering_enabled': LaunchConfiguration('steering_enabled'),
            'hardware_can_enabled': LaunchConfiguration('hardware_can_enabled'),
            'actuator_output_enabled': LaunchConfiguration('actuator_output_enabled'),
            'test_input_injection_enabled': LaunchConfiguration(
                'safety_injection_enabled'),
        }.items(),
    )
    bench = Node(
        package='control_tools',
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
        DeclareLaunchArgument('actuation_mode', default_value='', choices=['', 'position', 'torque']),
        DeclareLaunchArgument('steering_enabled', default_value=''),
        DeclareLaunchArgument('hardware_can_enabled', default_value='true'),
        DeclareLaunchArgument('actuator_output_enabled', default_value='true'),
        DeclareLaunchArgument('drive_mode_publish_enabled', default_value='true'),
        DeclareLaunchArgument('steer_sequence_publish_enabled', default_value='true'),
        DeclareLaunchArgument('safety_injection_enabled', default_value='true'),
        DeclareLaunchArgument(
            'shutdown_on_process_exit',
            default_value='true',
            description=(
                'Stop the full launch when either bench process exits. Set false '
                'only when an external staged-shutdown owner monitors both nodes.'
            ),
        ),
        actuator,
        bench,
        RegisterEventHandler(
            OnProcessExit(
                target_action=bench,
                on_exit=[EmitEvent(event=Shutdown(
                    reason='bench node exited; stopping actuator node'))],
            ),
            condition=IfCondition(LaunchConfiguration('shutdown_on_process_exit')),
        ),
        RegisterEventHandler(
            OnProcessExit(
                target_action=lambda action: getattr(
                    action, 'node_executable', None) == 'steering_actuator_node',
                on_exit=[EmitEvent(event=Shutdown(
                    reason='actuator node exited; stopping bench command source'))],
            ),
            condition=IfCondition(LaunchConfiguration('shutdown_on_process_exit')),
        ),
    ])
