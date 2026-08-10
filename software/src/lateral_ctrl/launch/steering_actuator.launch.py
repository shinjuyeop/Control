from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import os
import yaml


CALIBRATION_PARAMETER_NAMES = (
    "count_scale",
    "equivalent_k1_deg",
    "equivalent_k3_deg",
    "equivalent_k5_deg",
    "command_count_limit",
    "counts_per_rev",
)


def load_steering_motor_calibration(path):
    with open(path, "r", encoding="utf-8") as stream:
        document = yaml.safe_load(stream)
    if not isinstance(document, dict):
        raise RuntimeError(f"steering calibration must be a mapping: {path}")
    try:
        parameters = {name: document[name] for name in CALIBRATION_PARAMETER_NAMES}
    except KeyError as error:
        raise RuntimeError(
            f"steering calibration is missing {error.args[0]!r}: {path}"
        ) from error
    return parameters


def generate_launch_description():
    share = get_package_share_directory("lateral_ctrl")
    config = os.path.join(share, "config", "steering_actuator.yaml")
    calibration_config = os.path.join(
        share, "config", "steering_motor_calibration.yaml"
    )
    calibration_parameters = load_steering_motor_calibration(
        calibration_config
    )
    return LaunchDescription([
        DeclareLaunchArgument('can_interface', default_value='can0'),
        DeclareLaunchArgument('hardware_can_enabled', default_value='true'),
        DeclareLaunchArgument('actuator_output_enabled', default_value='true'),
        DeclareLaunchArgument('test_input_injection_enabled', default_value='false'),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        Node(
            package='lateral_ctrl',
            executable='steering_actuator_node',
            name='steering_actuator_node',
            output='screen',
            parameters=[
                config,
                calibration_parameters,
                {
                    'can_interface': LaunchConfiguration('can_interface'),
                    'hardware_can_enabled': LaunchConfiguration('hardware_can_enabled'),
                    'actuator_output_enabled': LaunchConfiguration('actuator_output_enabled'),
                    'test_input_injection_enabled': LaunchConfiguration(
                        'test_input_injection_enabled'),
                    'use_sim_time': LaunchConfiguration('use_sim_time'),
                    'tracking_control_enabled': False,
                },
            ],
        ),
    ])
