"""Record independently using the shared Control profile and parameter snapshot."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_prefix
import os


def _record(context):
    executable = os.path.join(get_package_prefix('control_tools'), 'lib/control_tools/record_mcap')
    cmd = [executable, '--output-base-dir', LaunchConfiguration('output_base_dir').perform(context)]
    for name, flag, enabled_value in (
        ('require_external_storage', '--require-external-storage', True),
        ('capture_parameters', '--no-capture-parameters', False),
        ('require_longitudinal', '--no-require-longitudinal', False),
    ):
        enabled = LaunchConfiguration(name).perform(context).lower() in ('true', '1', 'yes')
        if enabled == enabled_value:
            cmd.append(flag)
    return [ExecuteProcess(cmd=cmd, output='screen', sigterm_timeout='10', sigkill_timeout='5')]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('output_base_dir', default_value=os.path.expanduser('~/KAI_ws/records'),
                              description='MCAP 세션과 파라미터 snapshot을 저장할 부모 경로'),
        DeclareLaunchArgument('require_external_storage', default_value='false',
                              description='외장 artifact root의 별도 mount·하위 경로를 요구'),
        DeclareLaunchArgument('capture_parameters', default_value='true',
                              description='녹화 시작 시 실제 조향·거버너 파라미터를 조회'),
        DeclareLaunchArgument('require_longitudinal', default_value='true',
                              description='파라미터 수집 시 fresh한 STM_A readback도 요구'),
        OpaqueFunction(function=_record),
    ])
