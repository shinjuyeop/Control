"""longitudinal_actuator_node 단독 실행.

/vehicle/command와 /drive/status를 읽어 /drive/torque_command를 발행한다.
torque_control_enabled가 false이면 torque_mode=false만 발행하므로 STM_A는 기존
EZkontrol speed control을 유지한다. 실차 통합은 control_launch/vehicle.launch.py가
이 launch를 포함한다.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _make_node(context):
    optional = {}
    for name, cast in (('torque_control_enabled', lambda v: v.lower() == 'true'),
                       ('speed_source', str)):
        value = LaunchConfiguration(name).perform(context)
        if value:
            optional[name] = cast(value)
    return [Node(
        package='longitudinal_ctrl',
        executable='longitudinal_actuator_node',
        name='longitudinal_actuator_node',
        output='screen',
        parameters=[
            LaunchConfiguration('params_file'),
            {**optional,
             'tuning_preset': LaunchConfiguration('tuning_preset'),
             'use_sim_time': LaunchConfiguration('use_sim_time')},
        ],
    )]


def generate_launch_description():
    share = get_package_share_directory('longitudinal_ctrl')
    config = os.path.join(share, 'config', 'longitudinal_actuator.yaml')
    return LaunchDescription([
        DeclareLaunchArgument('params_file', default_value=config),
        DeclareLaunchArgument(
            'torque_control_enabled', default_value='',
            description='Empty follows the YAML; true/false overrides torque control at start.'),
        DeclareLaunchArgument(
            'speed_source', default_value='', choices=['', 'drive_status', 'gnss'],
            description='Empty follows the YAML; drive_status uses /drive/status, gnss uses /gnss/fix_velocity.'),
        DeclareLaunchArgument('tuning_preset', default_value='unspecified'),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        OpaqueFunction(function=_make_node),
    ])
