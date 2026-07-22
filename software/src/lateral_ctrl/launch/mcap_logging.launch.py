# ============================================================================
# mcap_logging.launch.py
#   모든 ros2 topic을 mcap으로 자동 녹화하는 로깅 전용 launch.
#   실험 launch(steering_bench_test 등)와 별개로, 이거 하나만 띄우면
#   그 시점부터 발견되는 모든 토픽이 기록된다.
#
#   ros2 launch lateral_ctrl mcap_logging.launch.py
#   ros2 launch lateral_ctrl mcap_logging.launch.py session_name:=deadzone_sweep
#
#   출력: ~/KAI_26/logs/rosbag/<session_name>_<YYYYmmdd_HHMMSS>/  (repo 밖)
#   종료: Ctrl+C (rosbag2가 SIGINT를 받아 mcap을 정상 마감함)
#   CSV 변환: ros2 run lateral_ctrl mcap_to_csv.py <bag 디렉토리>
# ============================================================================
import datetime
import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction
from launch.substitutions import LaunchConfiguration


def _make_record_process(context):
    session = LaunchConfiguration('session_name').perform(context)
    base_dir = os.path.expanduser(
        LaunchConfiguration('output_base_dir').perform(context))
    stamp = datetime.datetime.now().strftime('%Y%m%d_%H%M%S')
    bag_dir = os.path.join(base_dir, f'{session}_{stamp}')
    os.makedirs(base_dir, exist_ok=True)

    return [ExecuteProcess(
        cmd=[
            'ros2', 'bag', 'record',
            '--all',                      # 모든 토픽 (record 중 새로 생기는 토픽 포함)
            '--storage', 'mcap',
            '--output', bag_dir,
        ],
        output='screen',
    )]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'session_name', default_value='session',
            description='bag 디렉토리 이름 접두어 (예: deadzone_sweep, bench)'),
        DeclareLaunchArgument(
            'output_base_dir', default_value='~/KAI_26/logs/rosbag',
            description='bag 디렉토리들이 쌓일 상위 경로'),
        OpaqueFunction(function=_make_record_process),
    ])
