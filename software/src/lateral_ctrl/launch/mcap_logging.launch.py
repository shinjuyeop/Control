# ============================================================================
# mcap_logging.launch.py
#   실차 sysid, 압축 카메라 영상, Ouster 원시 패킷을 MCAP으로 녹화한다.
#
#   ros2 launch lateral_ctrl mcap_logging.launch.py
#
#   출력: /home/kai/KAI_ws/records/
#         rosbag2_YYYY_MM_DD-HH_MM_SS/
#   종료: Ctrl+C (rosbag2가 SIGINT를 받아 mcap을 정상 마감함)
#   CSV 변환: ros2 run lateral_ctrl mcap_to_csv.py <bag 디렉토리>
# ============================================================================
import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, LogInfo, OpaqueFunction
from launch.substitutions import LaunchConfiguration


ARTIFACT_ROOT = os.path.expanduser("~/Storage/ROS2_Workspace_offload")
DEFAULT_OUTPUT_BASE_DIR = os.path.expanduser("/home/kai/KAI_ws/records")

SYSID_TOPICS = (
    "/planning/command",
    "/planning/route_context",
    "/planning/target_path",
    "/planning/target_speed",
    "/vehicle/command",
    "/sysid/scinario",
    "/steering/status",
    "/steering/angle",
    "/drive/status",
    "/brake/status",
    "/vehicle/state",
    "/vehicle/wheel_twist",
    "/imu/data",
    "/imu/data_raw",
    "/ebimu/status",
    "/gnss/fix",
    "/gnss/fix_velocity",
    "/gnss/status",
    "/gnss/status_verbose",
    "/localization/odometry",
    "/localization/odometry_gnss",
    "/perception/camera/left/source/image_raw/compressed",
    "/perception/camera/left/source/camera_info",
    "/perception/camera/right/source/image_raw/compressed",
    "/perception/camera/right/source/camera_info",
    "/ouster/lidar_packets",
    "/ouster/imu_packets",
    "/ouster/metadata",
    "/tf",
    "/tf_static",
    "/diagnostics",
    "/parameter_events",
)


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
    require_external_storage = (
        LaunchConfiguration("require_external_storage").perform(context).lower()
        in {"true", "1", "yes"}
    )
    base_dir = _output_base(
        LaunchConfiguration("output_base_dir").perform(context),
        require_external_storage,
    )
    os.makedirs(base_dir, exist_ok=True)

    filesystem = os.statvfs(base_dir)
    available_gib = filesystem.f_bavail * filesystem.f_frsize / (1024 ** 3)

    return [
        LogInfo(
            msg=(
                f"MCAP output directory: {base_dir} "
                f"(storage available: {available_gib:.1f} GiB)"
            )
        ),
        ExecuteProcess(
            cmd=[
                "ros2", "bag", "record",
                "--storage", "mcap",
                *SYSID_TOPICS,
            ],
            cwd=base_dir,
            output="screen",
        ),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'output_base_dir', default_value=DEFAULT_OUTPUT_BASE_DIR,
            description='MCAP 출력 경로'),
        DeclareLaunchArgument(
            'require_external_storage', default_value='false',
            description=(
                'true면 외장 artifact root와 별도 mount를 강제한다. '
                '실차 제어컴의 고정 local records launch만 false를 사용한다.')),
        OpaqueFunction(function=_make_record_process),
    ])
