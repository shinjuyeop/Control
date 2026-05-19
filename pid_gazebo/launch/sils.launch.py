from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    # RPM 인자 선언
    rpm_arg = DeclareLaunchArgument('rpm', default_value='30.0')
    
    # 회차 번호 인자 선언
    run_id_arg = DeclareLaunchArgument('run_id', default_value='1')

    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/cmd_vel@geometry_msgs/msg/Twist]ignition.msgs.Twist',
            '/odom@nav_msgs/msg/Odometry[ignition.msgs.Odometry',
        ],
        output='screen'
    )

    pid_node = Node(
        package='pid_gazebo',
        executable='pid_gazebo_node',
        parameters=[{
            'wheel_radius': 0.4,
            #'lon_kp': 0.1,
            #'lon_ki': 0.0,
            #'lon_kd': 0.0,
        }],
        output='screen'
    )

    setpoint = Node(
        package='pid_gazebo',
        executable='setpoint_publisher',
        parameters=[{
            #'rpm': 30.0,
            'rpm': LaunchConfiguration('rpm'),  # 외부 인자 받음
            'steer_deg': 0.0,
            'hz': 10.0,
        }],
        output='screen'
    )

    bag_record = ExecuteProcess(
        cmd=[
            'ros2', 'bag', 'record',
            '--storage', 'mcap',
            '/cmd_vel',
            '/odom',
            '/setpoint/rpm',
            '/setpoint/steer_deg',
            '-o', ['/home/junhyeon/workspace/Control/pid_gazebo/sils_bag_run_', LaunchConfiguration('run_id')],  # 절대경로 + 회차 번호
        ],
        output='screen'
    )

    return LaunchDescription([rpm_arg, run_id_arg, bridge, pid_node, setpoint, bag_record])