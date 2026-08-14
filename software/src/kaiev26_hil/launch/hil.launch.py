"""STM_TEST bridge, command generator와 command governor 위에 GUI 또는
scenario runner를 얹는다.

명령 체인은 실차와 동일하다: hil_command_generator(Planning 대역)가
/planning/command를 내고 command_governor가 /vehicle/command로 재발행한다.
governor를 빼면 /vehicle/command 발행자가 사라져 STM_A Host command 미수신,
STM_B command age 증가, 조향 MODE_TIMEOUT로 진단이 온통 고장으로 보인다.

`gui`와 `scenario_file`은 서로 독립이다. 기본값(`gui:=true`, scenario 없음)은
운영자가 GUI로 직접 주입하는 평소 HIL 실행이고, `gui:=false`에 `scenario_file`을
주면 자동 scenario 실행이 된다. bridge가 serial 장치를 단독으로 소유하므로 이
launch를 두 번 띄우면 node 중복과 serial 소유권 충돌이 생긴다.
"""

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
import os


def generate_launch_description():
    share = get_package_share_directory("kaiev26_hil")
    config = os.path.join(share, "config", "hil.yaml")
    scenario_file = LaunchConfiguration("scenario_file")
    return LaunchDescription([
        DeclareLaunchArgument("serial_port", default_value="/dev/hil_test"),
        DeclareLaunchArgument(
            "gui", default_value="true",
            description="HIL GUI를 함께 띄운다. scenario만 돌릴 때는 false.",
        ),
        DeclareLaunchArgument(
            "scenario_file", default_value="",
            description="비어 있지 않으면 scenario runner를 함께 띄운다.",
        ),
        DeclareLaunchArgument("results_dir", default_value="test_results"),
        DeclareLaunchArgument(
            "gui_refresh_ms", default_value="200",
            description="GUI 갱신 주기(ms). 원격 X11 기준값이며 로컬 화면이면 100.",
        ),
        Node(
            package="kaiev26_hil", executable="stm_test_bridge", name="stm_test_bridge",
            output="screen",
            parameters=[config, {"serial_port": LaunchConfiguration("serial_port")}],
        ),
        Node(
            package="kaiev26_hil", executable="command_generator", name="hil_command_generator",
            output="screen", parameters=[config],
        ),
        # /vehicle/command의 유일한 발행자. 자기 패키지의 config를 그대로 쓴다.
        Node(
            package="command_governor", executable="command_governor_node",
            name="command_governor", output="screen",
            parameters=[os.path.join(
                get_package_share_directory("command_governor"),
                "config", "command_governor.yaml",
            )],
        ),
        Node(
            package="kaiev26_hil", executable="hil_gui", name="hil_gui", output="screen",
            parameters=[{
                "refresh_interval_ms": ParameterValue(
                    LaunchConfiguration("gui_refresh_ms"), value_type=int
                ),
            }],
            condition=IfCondition(LaunchConfiguration("gui")),
        ),
        Node(
            package="kaiev26_hil", executable="scenario_runner", name="hil_scenario_runner",
            output="screen",
            parameters=[{
                "scenario_file": scenario_file,
                "results_dir": LaunchConfiguration("results_dir"),
            }],
            condition=IfCondition(PythonExpression(["'", scenario_file, "' != ''"])),
        ),
    ])
