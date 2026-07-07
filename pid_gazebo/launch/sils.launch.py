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
            'wheel_radius': 0.3,
            'lon_i_limit': 5.0,   # 적분 누적 한계 축소 (50 → 5)
            'lon_out_max': 1.5,   # 출력 제한 축소 (3.0 → 1.5)
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

    # launch 파일 종료 시 mcap record 함께 종료 
    bag_record = ExecuteProcess(
        cmd=[
            'ros2', 'bag', 'record',
            '--storage', 'mcap',
            # ── SILS 토픽 ──
            '/cmd_vel',
            '/odom',
            '/setpoint/rpm',
            '/setpoint/steer_deg',
            # ── 하드웨어 토픽 (연결 시 자동 수집) ──
            '/cmd/speed',
            '/current/speed',
            '/target/rpm',
            '/cmd/steer',
            '/current/steer',
            '/gear',
            '/mode',
            '/aeb',
            '/estop/status',
            '/estop/rf',
            '/pedal',
            '/brake/target',
            '/brake/current',
            '-o', f'/home/seo/KAI_26/workspace/Control/pid_gazebo/bags/sils_bag_{datetime.datetime.now().strftime("%Y%m%d_%H%M%S")}',
        ],
        output='screen'
    )

    return LaunchDescription([bridge, pid_node, setpoint, bag_record])
