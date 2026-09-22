"""Run one unloaded T0-T2 torque SYSID experiment and record its evidence."""

from datetime import datetime
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    LogInfo,
    OpaqueFunction,
    TimerAction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


ARTIFACT_ROOT = os.path.expanduser('~/Storage/ROS2_Workspace_offload')
DEFAULT_RECORDS_DIR = os.path.join(ARTIFACT_ROOT, 'Control', 'torque_sysid')
RECORD_TOPICS = ('/steering/status', '/deadzone/start', '/deadzone/stop', '/rosout')


def _presets(test, direction, level):
    common = {'ramp_direction': direction, 'ramp_period_ms': 20, 'auto_start': True}
    if test == 't0':
        return 'T0_sign_check', {
            **common, 'sweep_mode': False, 'start_cmd': 300, 'ramp_step': 300,
            'torque_limit': 600, 'step_hold_ms': 1500, 'max_angle_delta': 20.0,
        }
    if test == 't1':
        return 'T1_friction_sweep', {
            **common, 'sweep_mode': True, 'start_cmd': 60, 'ramp_step': 10,
            'torque_limit': 400, 'step_hold_ms': 300, 'max_angle_delta': 90.0,
        }
    if test == 't2':
        return f'T2_inertia_step_lvl{level}', {
            **common, 'sweep_mode': False, 'start_cmd': level, 'ramp_step': 1,
            'torque_limit': level + 1, 'step_hold_ms': 300, 'max_angle_delta': 90.0,
        }
    raise RuntimeError(f'test must be t0, t1 or t2: {test!r}')


def _recording_directory(requested, label):
    artifact_root = os.path.realpath(ARTIFACT_ROOT)
    if not os.path.isdir(artifact_root):
        raise RuntimeError(f'external ROS artifact root is unavailable: {ARTIFACT_ROOT}')
    if os.stat(artifact_root).st_dev == os.stat('/').st_dev:
        raise RuntimeError(f'external ROS artifact root is not mounted separately: {ARTIFACT_ROOT}')
    records_dir = os.path.realpath(os.path.expanduser(requested))
    if os.path.commonpath((artifact_root, records_dir)) != artifact_root:
        raise RuntimeError(f'records_dir must stay under {ARTIFACT_ROOT}: {requested}')
    os.makedirs(records_dir, exist_ok=True)
    available_gib = os.statvfs(records_dir).f_bavail * os.statvfs(records_dir).f_frsize / 1024 ** 3
    run_dir = os.path.join(
        records_dir, datetime.now().strftime('%Y%m%d_%H%M%S_%f') + '_' + label)
    os.makedirs(run_dir, exist_ok=False)
    return run_dir, available_gib


def _make_actions(context):
    test = LaunchConfiguration('test').perform(context).strip().lower()
    direction = int(LaunchConfiguration('direction').perform(context))
    level = int(LaunchConfiguration('level').perform(context))
    if direction not in (-1, 1):
        raise RuntimeError('direction must be 1 or -1')
    if not 1 <= level <= 7999:
        raise RuntimeError('level must be within 1..7999')
    label, parameters = _presets(test, direction, level)
    label += '_pos' if direction > 0 else '_neg'
    can_interface = LaunchConfiguration('can_interface').perform(context)
    actions = [
        LogInfo(msg=(f'[{label}] read-only steering + deadzone_measure_node; '
                     'torque starts after the first valid angle')),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(
                get_package_share_directory('lateral_ctrl'), 'launch',
                'steering_actuator.launch.py')),
            launch_arguments={
                'can_interface': can_interface,
                'actuation_mode': 'torque',
                'steering_enabled': 'false',
                'hardware_can_enabled': 'true',
                'actuator_output_enabled': 'false',
            }.items(),
        ),
    ]
    if LaunchConfiguration('record').perform(context).lower() in ('true', '1', 'yes'):
        run_dir, available_gib = _recording_directory(
            LaunchConfiguration('records_dir').perform(context), label)
        actions += [
            LogInfo(msg=f'[{label}] MCAP: {run_dir}/rosbag ({available_gib:.1f} GiB free)'),
            ExecuteProcess(
                cmd=['ros2', 'bag', 'record', '-s', 'mcap', '-o',
                     os.path.join(run_dir, 'rosbag'), *RECORD_TOPICS],
                output='screen',
            ),
        ]
    delay = float(LaunchConfiguration('start_delay_sec').perform(context))
    actions.append(TimerAction(period=delay, actions=[Node(
        package='control_tools', executable='deadzone_measure_node',
        name='deadzone_measure_node', output='screen', arguments=[can_interface],
        parameters=[parameters],
    )]))
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('test', description='t0 sign | t1 friction | t2 inertia'),
        DeclareLaunchArgument('direction', default_value='1', description='1 (LEFT+) or -1'),
        DeclareLaunchArgument('level', default_value='300', description='T2 torque step raw'),
        DeclareLaunchArgument('can_interface', default_value='can0'),
        DeclareLaunchArgument('record', default_value='true'),
        DeclareLaunchArgument('records_dir', default_value=DEFAULT_RECORDS_DIR),
        DeclareLaunchArgument('start_delay_sec', default_value='3.0'),
        OpaqueFunction(function=_make_actions),
    ])
