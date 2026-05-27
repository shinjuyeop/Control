from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
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
            'wheel_radius': 0.3,
            'lon_i_limit': 5.0,   # 적분 누적 한계 축소 (50 → 5)
            'lon_out_max': 1.5,   # 출력 제한 축소 (3.0 → 1.5)
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
            '-o', ['/home/junhyeon/workspace/Control/pid_gazebo/sils_bag_run_', LaunchConfiguration('run_id')],
        ],
        output='screen'
    )

    return LaunchDescription([run_id_arg, bridge, pid_node, bag_record])