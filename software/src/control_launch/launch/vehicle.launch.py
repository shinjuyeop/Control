"""
실차 구동 스택 통합 launch.

구동에 필요한 것만 띄운다: 차량·INS 기준 정적 TF, STM_A/B micro-ROS Agent,
command_governor, steering_actuator와 vehicle_state_publisher package의 상태 합성,
diagnostics_publisher, auto_dashboard_summary를 실행한다.
관측 화면은 별도 실행한다: `ros2 launch vehicle_console vehicle.launch.py` 또는
`ros2 launch foxglove foxglove_bridge.launch.py`.

Planning은 포함하지 않는다. 판단 계층은 별도로 실행되어 /planning/command를
발행한다. Planning 없이 이 launch만 켜면 command_governor가 중계할 입력이
없어 /vehicle/command가 발행되지 않고, STM_A는 500 ms Host timeout으로
HALTED, 조향은 WAIT_CMD로 대기한다 — 이것이 안전한 기본 상태다.
hil_command_generator는 Planning 대역 시험 노드이므로 실차 launch에 절대
넣지 않는다.

조향은 기본 armed다(hardware_can_enabled=true, actuator_output_enabled=true).
launch가 뜨면 바로 구동 가능한 상태가 되므로, 전원 인가 전에 KEYA 배선과
E-stop을 먼저 확인한다.

Torque 조향의 invalid/stale feedback과 control stall은 `FEEDBACK_LOST`로 정지한다.
정상 feedback과 차단 뒤 받은 새 `/vehicle/command`가 확인되면 노드 재시작 없이
자동 복구한다. `FAULT`는 KEYA bit0(Disable)을 제외한 실제 fault bit에만 사용한다.

can0는 launch 전에 올라와 있어야 한다. 조향 노드는 SocketCAN을 시작 시 한
번만 열고 실패해도 재시도하지 않으므로, can0가 늦으면 조향만 조용히 CAN이
죽은 채 뜬다. host/systemd/의 networkd 설정을 설치하면 부팅·USB 재연결 시
자동으로 준비된다(HIL_RUNBOOK 10.1).

command_governor는 /vehicle/command의 유일한 발행자라서 죽으면 명령이
끊긴다(안전망은 위 timeout들). 노드가 stateless라 재시작만 하면 그대로
복구되므로 respawn을 걸어 둔다. 반복 크래시는 로그와 /diagnostics의 명령
두절로 드러난다.

이 launch는 HIL bench(2026-08-14)에서 동형 구조로 검증했고, 실차 장치
(/dev/stm_a, /dev/stm_b)에서의 최종 확인은 아직이다.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from control_tuning.storage import default_settings_path
from control_tuning.presets import load_launch_overrides
from control_tuning.conditions import CONDITION_KEYS


def _governor(context, config):
    return [Node(
        package="command_governor", executable="command_governor_node",
        name="command_governor", output="screen",
        parameters=[config, load_launch_overrides(
            LaunchConfiguration("control_tuning_file").perform(context), "command_governor",
            LaunchConfiguration("tuning_preset").perform(context), governor_config=config)],
        respawn=True, respawn_delay=2.0,
    )]


def _longitudinal(context):
    return [Node(package='control_tuning', executable='longitudinal_calibration',
                 name='longitudinal_calibration', output='screen',
                 parameters=[load_launch_overrides(
                     LaunchConfiguration('control_tuning_file').perform(context),
                     'stm_a_longitudinal', LaunchConfiguration('tuning_preset').perform(context))])]


def _agent(name, port, baud):
    return Node(
        package="micro_ros_agent",
        executable="micro_ros_agent",
        name=name,
        output="screen",
        arguments=["serial", "--dev", port, "-b", baud],
        condition=IfCondition(LaunchConfiguration("agents")),
    )


def _robot_state_publisher():
    vehicle_xacro = os.path.join(
        get_package_share_directory("kaiev26_description"),
        "urdf", "kaiev26.urdf.xacro",
    )
    return Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="vehicle_robot_state_publisher",
        output="screen",
        parameters=[{
            "robot_description": ParameterValue(
                Command([
                    FindExecutable(name="xacro"),
                    " ", vehicle_xacro,
                    " lidar_output_mode:=none",
                    " lidar_frame_model:=hesai",
                    " debug_driver_camera_enabled:=false",
                ]),
                value_type=str,
            ),
        }],
    )


def generate_launch_description():
    governor_config = os.path.join(
        get_package_share_directory("command_governor"),
        "config", "command_governor.yaml",
    )
    steering_launch = os.path.join(
        get_package_share_directory("lateral_ctrl"),
        "launch", "steering_actuator.launch.py",
    )
    baud = LaunchConfiguration("agent_baud")
    return LaunchDescription([
        DeclareLaunchArgument("stm_a_port", default_value="/dev/stm_a"),
        DeclareLaunchArgument("stm_b_port", default_value="/dev/stm_b"),
        DeclareLaunchArgument("agent_baud", default_value="460800"),
        DeclareLaunchArgument("control_tuning_file", default_value=str(default_settings_path())),
        DeclareLaunchArgument("tuning_preset", default_value="driving", choices=list(CONDITION_KEYS),
                              description="주행 / 공중 제어 프리셋"),
        DeclareLaunchArgument(
            "agents", default_value="true",
            description="STM_A/B micro-ROS Agent를 함께 띄운다. 이미 따로 실행 중이면 false.",
        ),
        DeclareLaunchArgument(
            "steering", default_value="true",
            description="KEYA 조향 스택을 함께 띄운다. 조향 없이 종방향만 시험할 때 false.",
        ),
        DeclareLaunchArgument("actuation_mode", default_value="", choices=["", "position", "torque"]),
        DeclareLaunchArgument("steering_enabled", default_value=""),
        DeclareLaunchArgument(
            "can_interface", default_value="can0",
            description="KEYA SocketCAN 인터페이스.",
        ),
        # kaiev26_description의 차량·INS·LiDAR·카메라 고정 TF 전체를 발행한다.
        _robot_state_publisher(),
        _agent("stm_a_agent", LaunchConfiguration("stm_a_port"), baud),
        _agent("stm_b_agent", LaunchConfiguration("stm_b_port"), baud),
        OpaqueFunction(function=_longitudinal),
        # /vehicle/command의 유일한 발행자. stateless라 respawn으로 복구한다.
        OpaqueFunction(function=_governor, kwargs={"config": governor_config}),
        # 조향 armed 기본값을 launch 인자로 명시해, lateral_ctrl 쪽 기본값이
        # 바뀌어도 실차 launch는 즉시 구동 가능 상태를 유지한다.
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(steering_launch),
            launch_arguments={
                "can_interface": LaunchConfiguration("can_interface"),
                "control_tuning_file": LaunchConfiguration("control_tuning_file"),
                "tuning_preset": LaunchConfiguration("tuning_preset"),
                "actuation_mode": LaunchConfiguration("actuation_mode"),
                "steering_enabled": LaunchConfiguration("steering_enabled"),
                "hardware_can_enabled": "true",
                "actuator_output_enabled": "true",
            }.items(),
            condition=IfCondition(LaunchConfiguration("steering")),
        ),
        Node(
            package="vehicle_state_publisher",
            executable="vehicle_state_publisher_node",
            name="vehicle_state_publisher",
            output="screen",
        ),
        # status 3종 → /diagnostics 요약. Foxglove 없이도 발행되어야 bag
        # 기록과 나중에 붙는 관찰 클라이언트가 바로 쓸 수 있다.
        Node(
            package="vehicle_state_publisher",
            executable="diagnostics_publisher",
            name="diagnostics_publisher",
            output="screen",
        ),
        # 실제 수신 freshness와 액추에이터 명령/응답 관계를 display 전용
        # 고정 payload로 요약한다. STM_B가 /dashboard/auto_status를 CAN에 중계한다.
        Node(
            package='vehicle_state_publisher',
            executable='auto_dashboard_summary',
            name='auto_dashboard_summary',
            output='screen',
        ),
    ])
