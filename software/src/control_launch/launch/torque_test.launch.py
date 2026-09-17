"""
토크 조향 T3 시험용 실차 구동 스택 (docs/Experiment_RUNBOOK.md T3).

    ros2 launch control_launch torque_test.launch.py

vehicle.launch.py를 torque 모드로 띄운다. 조향은 비활성으로 시작하고, 실차 시험
TUI(`ros2 run vehicle_test_scenarios vehicle_test_scenario_runner`)가 Run 때
steering_enabled와 메뉴의 P/D를 적용한다. 토크 한계는 bring-up 값(peak 2000,
sustained 1000)으로 시작하며 인자로 바꿀 수 있다.

전제: KEYA 0020=2(Torque)+Burn, 바퀴 띄움, E-stop 준비. 조향은 launch 즉시 armed다.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    vehicle_launch = os.path.join(
        get_package_share_directory("control_launch"), "launch", "vehicle.launch.py")
    return LaunchDescription([
        DeclareLaunchArgument("can_interface", default_value="can0"),
        DeclareLaunchArgument(
            "torque_peak_limit_raw", default_value="2000",
            description="bring-up peak. 실험 중 8000 이하 유지"),
        DeclareLaunchArgument(
            "torque_sustained_limit_raw", default_value="1000",
            description="peak 이하"),
        LogInfo(msg=(
            "TORQUE TEST STACK: actuation_mode=torque, steering disabled until the TUI Run. "
            "Next: ros2 run vehicle_test_scenarios vehicle_test_scenario_runner")),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(vehicle_launch),
            launch_arguments={
                "actuation_mode": "torque",
                "steering_enabled": "false",
                "can_interface": LaunchConfiguration("can_interface"),
                "torque_peak_limit_raw": LaunchConfiguration("torque_peak_limit_raw"),
                "torque_sustained_limit_raw": LaunchConfiguration("torque_sustained_limit_raw"),
            }.items(),
        ),
    ])
