from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    joy_defaults = {
        'device_id': 0,
        'deadzone': 0.05,
        'autorepeat_rate': 60.0,
        'coalesce_interval_ms': 1,
    }
    teleop_defaults = {
        'max_steering_deg': 30.0,
        'max_speed_kph': 20.0,
        'steering_step_deg': 1.0,
        'regen_max_a': 1.0,
        'left_steering_axis': 0,
        'right_steering_axis': 3,
        'lt_axis': 2,
        'rt_axis': 5,
        'dpad_horizontal_axis': 6,
    }
    arguments = [
        DeclareLaunchArgument(name, default_value=str(default))
        for name, default in {**joy_defaults, **teleop_defaults}.items()
    ]
    arguments.extend([
        DeclareLaunchArgument('joy_topic', default_value='/joy'),
        DeclareLaunchArgument('command_topic', default_value='/planning/command'),
        DeclareLaunchArgument('current_topic', default_value='/vehicle/drive_current_command'),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
    ])
    clock_parameter = {
        'use_sim_time': ParameterValue(LaunchConfiguration('use_sim_time'), value_type=bool)
    }

    return LaunchDescription(arguments + [
        Node(
            package='joy',
            executable='joy_node',
            name='joy_node',
            output='screen',
            parameters=[{
                name: ParameterValue(LaunchConfiguration(name), value_type=type(default))
                for name, default in joy_defaults.items()
            }, clock_parameter],
            remappings=[('joy', LaunchConfiguration('joy_topic'))],
        ),
        Node(
            package='control_launch',
            executable='joystick_teleop_node',
            name='joystick_teleop',
            output='screen',
            parameters=[{
                name: ParameterValue(LaunchConfiguration(name), value_type=type(default))
                for name, default in teleop_defaults.items()
            }, clock_parameter],
            remappings=[
                ('/joy', LaunchConfiguration('joy_topic')),
                ('/planning/command', LaunchConfiguration('command_topic')),
                ('/vehicle/drive_current_command', LaunchConfiguration('current_topic')),
            ],
        ),
    ])
