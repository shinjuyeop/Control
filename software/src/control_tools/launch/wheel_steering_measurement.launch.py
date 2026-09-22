import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource


def include_launch(package_name, launch_file, launch_arguments=None):
    launch_path = os.path.join(
        get_package_share_directory(package_name),
        "launch",
        launch_file,
    )
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(launch_path),
        launch_arguments=(launch_arguments or {}).items(),
    )


def _compose(_context):
    return [
        include_launch("ebimu_driver", "ebimu_dual.launch.py"),
        include_launch(
            "lateral_ctrl",
            "steering_actuator.launch.py",
            {
                "can_interface": "can0",
                "hardware_can_enabled": "true",
                "actuator_output_enabled": "false",
                "test_input_injection_enabled": "false",
                "use_sim_time": "false",
            },
        ),
    ]


def generate_launch_description():
    return LaunchDescription([OpaqueFunction(function=_compose)])
