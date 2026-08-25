# ============================================================================
# mcap_logging.launch.py
#   실차 sysid에 필요한 ros2 topic만 mcap으로 녹화하는 로깅 전용 launch.
#   카메라와 LiDAR를 포함하는 --all은 쓰지 않는다.
#
#   ros2 launch lateral_ctrl mcap_logging.launch.py
#   ros2 launch lateral_ctrl mcap_logging.launch.py session_name:=deadzone_sweep
#
#   출력: ~/Storage/ROS2_Workspace_offload/gazebo_sitl/vehicle_sysid/
#         <campaign_name>/<session_name>_<YYYYmmdd_HHMMSS>/rosbag/
#   종료: Ctrl+C (rosbag2가 SIGINT를 받아 mcap을 정상 마감함)
#   CSV 변환: ros2 run lateral_ctrl mcap_to_csv.py <bag 디렉토리>
# ============================================================================
import datetime
import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, LogInfo, OpaqueFunction
from launch.substitutions import LaunchConfiguration


ARTIFACT_ROOT = os.path.expanduser("~/Storage/ROS2_Workspace_offload")
DEFAULT_OUTPUT_BASE_DIR = os.path.join(
    ARTIFACT_ROOT, "gazebo_sitl", "vehicle_sysid"
)

SYSID_TOPICS = (
    "/planning/command",
    "/vehicle/command",
    "/steering/status",
    "/steering/angle",
    "/drive/status",
    "/brake/status",
    "/vehicle/state",
    "/imu/data",
    "/imu/data_raw",
    "/ebimu/status",
    "/gnss/fix",
    "/gnss/fix_velocity",
    "/gnss/status",
    "/gnss/status_verbose",
    "/localization/kinematic_state",
    "/localization/kinematic_state_gnss",
    "/left/imu/data",
    "/right/imu/data",
    "/tf",
    "/tf_static",
    "/diagnostics",
    "/parameter_events",
)


def _path_segment(value, argument_name):
    if not value or os.path.basename(value) != value or value in {".", ".."}:
        raise RuntimeError(
            f"{argument_name} must be one non-empty directory name: {value!r}"
        )
    return value


def _output_base(value, require_external_storage):
    output_base = os.path.realpath(os.path.expanduser(value))
    if not require_external_storage:
        return output_base

    artifact_root = os.path.realpath(ARTIFACT_ROOT)
    if not os.path.isdir(artifact_root):
        raise RuntimeError(
            f"external ROS artifact root is unavailable: {ARTIFACT_ROOT}"
        )
    if os.stat(artifact_root).st_dev == os.stat("/").st_dev:
        raise RuntimeError(
            f"external ROS artifact root is not mounted separately: {ARTIFACT_ROOT}"
        )

    if os.path.commonpath((artifact_root, output_base)) != artifact_root:
        raise RuntimeError(
            f"output_base_dir must stay under {ARTIFACT_ROOT}: {value}"
        )
    return output_base


def _make_record_process(context):
    campaign = _path_segment(
        LaunchConfiguration("campaign_name").perform(context), "campaign_name"
    )
    session = _path_segment(
        LaunchConfiguration("session_name").perform(context), "session_name"
    )
    require_external_storage = (
        LaunchConfiguration("require_external_storage").perform(context).lower()
        in {"true", "1", "yes"}
    )
    base_dir = _output_base(
        LaunchConfiguration("output_base_dir").perform(context),
        require_external_storage,
    )
    stamp = datetime.datetime.now().strftime('%Y%m%d_%H%M%S')
    run_dir = os.path.join(base_dir, campaign, f"{session}_{stamp}")
    bag_dir = os.path.join(run_dir, "rosbag")
    os.makedirs(run_dir, exist_ok=False)

    filesystem = os.statvfs(base_dir)
    available_gib = filesystem.f_bavail * filesystem.f_frsize / (1024 ** 3)

    return [
        LogInfo(
            msg=(
                f"vehicle sysid run: {run_dir} "
                f"(external storage available: {available_gib:.1f} GiB)"
            )
        ),
        ExecuteProcess(
            cmd=[
                "ros2", "bag", "record",
                "--storage", "mcap",
                "--output", bag_dir,
                *SYSID_TOPICS,
            ],
            output="screen",
        ),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            "campaign_name", default_value="commissioning",
            description="시험 campaign 디렉토리 이름"),
        DeclareLaunchArgument(
            'session_name', default_value='session',
            description='bag 디렉토리 이름 접두어 (예: deadzone_sweep, bench)'),
        DeclareLaunchArgument(
            'output_base_dir', default_value=DEFAULT_OUTPUT_BASE_DIR,
            description='ROS 외장 artifact root 아래 sysid 출력 경로'),
        DeclareLaunchArgument(
            'require_external_storage', default_value='true',
            description=(
                'true면 외장 artifact root와 별도 mount를 강제한다. '
                '실차 제어컴의 고정 local records launch만 false를 사용한다.')),
        OpaqueFunction(function=_make_record_process),
    ])
