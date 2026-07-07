# pyright: reportMissingImports=false
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # ── PID gains ─────────────────────────────────────────────────────
    lon_kp = LaunchConfiguration('lon_kp')
    lon_ki = LaunchConfiguration('lon_ki')
    lon_kd = LaunchConfiguration('lon_kd')
    lon_out_max = LaunchConfiguration('lon_out_max')
    lon_i_limit = LaunchConfiguration('lon_i_limit')
    wheel_radius = LaunchConfiguration('wheel_radius')

    str_kp = LaunchConfiguration('str_kp')
    str_ki = LaunchConfiguration('str_ki')
    str_kd = LaunchConfiguration('str_kd')
    str_out_max = LaunchConfiguration('str_out_max')
    str_i_limit = LaunchConfiguration('str_i_limit')

    # ── Test setpoint ─────────────────────────────────────────────────
    sp_rpm = LaunchConfiguration('sp_rpm')
    sp_steer_deg = LaunchConfiguration('sp_steer_deg')

    return LaunchDescription([
        # Longitudinal PID
        DeclareLaunchArgument('lon_kp',       default_value='0.5'),
        DeclareLaunchArgument('lon_ki',       default_value='0.05'),
        DeclareLaunchArgument('lon_kd',       default_value='0.02'),
        DeclareLaunchArgument('lon_out_max',  default_value='3.0'),
        DeclareLaunchArgument('lon_i_limit',  default_value='50.0'),
        DeclareLaunchArgument('wheel_radius', default_value='0.1'),

        # Steering PID
        DeclareLaunchArgument('str_kp',      default_value='1.0'),
        DeclareLaunchArgument('str_ki',      default_value='0.0'),
        DeclareLaunchArgument('str_kd',      default_value='0.1'),
        DeclareLaunchArgument('str_out_max', default_value='1.5'),
        DeclareLaunchArgument('str_i_limit', default_value='1.0'),

        # Test setpoints
        DeclareLaunchArgument('sp_rpm',       default_value='300.0',
                              description='Target RPM for longitudinal channel'),
        DeclareLaunchArgument('sp_steer_deg', default_value='0.0',
                              description='Target heading angle [deg] for steering channel'),

        Node(
            package='pid_gazebo',
            executable='pid_gazebo_node',
            name='pid_gazebo',
            output='screen',
            parameters=[{
                'lon_kp':       lon_kp,
                'lon_ki':       lon_ki,
                'lon_kd':       lon_kd,
                'lon_out_max':  lon_out_max,
                'lon_i_limit':  lon_i_limit,
                'wheel_radius': wheel_radius,
                'str_kp':       str_kp,
                'str_ki':       str_ki,
                'str_kd':       str_kd,
                'str_out_max':  str_out_max,
                'str_i_limit':  str_i_limit,
                'control_hz':   20.0,
                'target_timeout_s': 1.0,
            }],
        ),

        Node(
            package='pid_gazebo',
            executable='setpoint_publisher',
            name='setpoint_publisher',
            output='screen',
            parameters=[{
                'rpm':       sp_rpm,
                'steer_deg': sp_steer_deg,
                'hz':        10.0,
            }],
        ),
    ])
