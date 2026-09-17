# ============================================================================
# torque_sysid_bench.launch.py
#   docs/Experiment_RUNBOOK.md T0–T2(무부하 토크 SYSID)를 명령 하나로 실행한다.
#
#   ros2 launch lateral_ctrl torque_sysid_bench.launch.py test:=t0 direction:=1
#   ros2 launch lateral_ctrl torque_sysid_bench.launch.py test:=t1 direction:=-1
#   ros2 launch lateral_ctrl torque_sysid_bench.launch.py test:=t2 level:=500
#
#   띄우는 것: 읽기 전용 steering_actuator_node(조회만, 출력 잠금)
#             + deadzone_measure_node(시험별 프리셋, 첫 유효 각도 수신 후 자동 시작)
#             + MCAP 녹화(~/KAI_ws/records/<시간>_<label>/rosbag, /rosout 포함)
#   종료: 측정이 끝나면(DEADZONE DETECTED / SWEEP COMPLETE / ANGLE LIMIT ABORT) Ctrl+C.
#
#   전제: KEYA 0020=2(Torque)+Burn, 바퀴 띄움, E-stop 준비.
#   vehicle.launch.py(조향 armed)와 동시에 실행하지 않는다. 한 모터에 송신원이 둘이 된다.
# ============================================================================
from datetime import datetime
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, LogInfo,
    OpaqueFunction, TimerAction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


RECORD_TOPICS = ('/steering/status', '/deadzone/start', '/deadzone/stop', '/rosout')


def _presets(test, direction, level):
    """deadzone_measure_node parameters per runbook test. The node disables as soon as
    |cmd| reaches torque_limit, so torque_limit sits one step above the held torque."""
    common = {'ramp_direction': direction, 'ramp_period_ms': 20, 'auto_start': True}
    if test == 't0':
        # Hold +/-300 (~3x breakaway) for 1.5 s; the sign check needs only a few degrees.
        return 'T0_sign_check', {**common, 'sweep_mode': False, 'start_cmd': 300,
                                 'ramp_step': 300, 'torque_limit': 600,
                                 'step_hold_ms': 1500, 'max_angle_delta': 20.0}
    if test == 't1':
        # Staircase from below breakaway (~106); SWEEP log lines go to /rosout in the MCAP.
        return 'T1_friction_sweep', {**common, 'sweep_mode': True, 'start_cmd': 60,
                                     'ramp_step': 10, 'torque_limit': 400,
                                     'step_hold_ms': 300, 'max_angle_delta': 90.0}
    if test == 't2':
        return f'T2_inertia_step_lvl{level}', {**common, 'sweep_mode': False,
                                               'start_cmd': level, 'ramp_step': 1,
                                               'torque_limit': level + 1,
                                               'step_hold_ms': 300, 'max_angle_delta': 90.0}
    raise RuntimeError(f'test must be t0, t1 or t2: {test!r}')


def _make_actions(context):
    test = LaunchConfiguration('test').perform(context).strip().lower()
    direction = int(LaunchConfiguration('direction').perform(context))
    level = int(LaunchConfiguration('level').perform(context))
    if direction not in (-1, 1):
        raise RuntimeError('direction must be 1 or -1')
    if not 1 <= level <= 7999:
        raise RuntimeError('level must be within 1..7999 (measurement node caps raw at 8000)')
    label, parameters = _presets(test, direction, level)
    label += '_pos' if direction > 0 else '_neg'
    can_interface = LaunchConfiguration('can_interface').perform(context)

    actions = [
        LogInfo(msg=(f'[{label}] read-only steering + deadzone_measure_node '
                     f'{parameters} — torque starts after the first valid angle')),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(
                get_package_share_directory('lateral_ctrl'), 'launch',
                'steering_actuator.launch.py')),
            launch_arguments={
                'can_interface': can_interface,
                'hardware_can_enabled': 'true',
                'actuator_output_enabled': 'false',
            }.items(),
        ),
    ]
    if LaunchConfiguration('record').perform(context).lower() in ('true', '1', 'yes'):
        root = os.path.expanduser(LaunchConfiguration('records_dir').perform(context))
        run_dir = os.path.join(root, datetime.now().strftime('%Y%m%d_%H%M%S_%f') + '_' + label)
        os.makedirs(run_dir, exist_ok=False)
        actions += [
            LogInfo(msg=f'[{label}] MCAP: {run_dir}/rosbag'),
            ExecuteProcess(
                cmd=['ros2', 'bag', 'record', '-s', 'mcap', '-o',
                     os.path.join(run_dir, 'rosbag'), *RECORD_TOPICS],
                output='screen'),
        ]
    # Give the read-only node and the recorder time to come up before any torque.
    delay = float(LaunchConfiguration('start_delay_sec').perform(context))
    actions.append(TimerAction(period=delay, actions=[
        Node(
            package='lateral_ctrl',
            executable='deadzone_measure_node',
            name='deadzone_measure_node',
            output='screen',
            arguments=[can_interface],
            parameters=[parameters],
        ),
    ]))
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('test', description='t0 부호 | t1 마찰 sweep | t2 관성 step'),
        DeclareLaunchArgument('direction', default_value='1', description='1(LEFT+) | -1'),
        DeclareLaunchArgument('level', default_value='300', description='t2 계단 토크 raw'),
        DeclareLaunchArgument('can_interface', default_value='can0'),
        DeclareLaunchArgument('record', default_value='true'),
        DeclareLaunchArgument('records_dir', default_value='~/KAI_ws/records'),
        DeclareLaunchArgument('start_delay_sec', default_value='3.0'),
        OpaqueFunction(function=_make_actions),
    ])
