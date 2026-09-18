"""No-argument A11 launch.

Available AUTO sysid launches in this package:
  auto_longitudinal_speed_step.launch.py  configured low/high speeds (default 0), 5 s, 4 cycles
  auto_lateral_steering_step.launch.py    center/+1/center/-1 deg, 5 s, 3 cycles
  auto_lateral_steering_sine.launch.py    center +/-1 deg, period 8 s, 6 cycles
  auto_lateral_steering_square.launch.py  center +/-1 deg square, period 8 s, 6 cycles

This launch starts the complete physical vehicle stack, the fixed YAML scenario,
and MCAP logging. It accepts no launch arguments.
"""

from vehicle_test_scenarios.auto_sysid_launch import auto_sysid_launch_description


def generate_launch_description():
    return auto_sysid_launch_description(
        config_filename="auto_lateral_steering_step.yaml",
        scenario_summary="configured speed (default 0 km/h), center/+1/center/-1 deg, 5 s, 3 cycles",
    )
