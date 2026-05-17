from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
import datetime

def generate_launch_description():
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
            'lon_kp': 0.1,
            'lon_ki': 0.0,
            'lon_kd': 0.0,
        }],
        output='screen'
    )

    setpoint = Node(
        package='pid_gazebo',
        executable='setpoint_publisher',
        parameters=[{
            'rpm': 30.0,
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
            '-o', f'sils_bag_{datetime.datetime.now().strftime("%Y%m%d_%H%M%S")}',
        ],
        output='screen'
    )

    return LaunchDescription([bridge, pid_node, setpoint, bag_record])
