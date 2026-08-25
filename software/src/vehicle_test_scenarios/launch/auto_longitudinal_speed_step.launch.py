"""No-argument A02 launch.

Available AUTO sysid launches in this package:
  auto_longitudinal_speed_step.launch.py  0->1 then 1<->2 m/s, 5 s, 4 cycles
  auto_lateral_steering_step.launch.py    center/+1/center/-1 deg, 5 s, 3 cycles
  auto_lateral_steering_sine.launch.py    center +/-1 deg, period 8 s, 6 cycles

This launch starts the complete physical vehicle stack, the fixed YAML scenario,
and MCAP logging. It accepts no launch arguments.
"""

from vehicle_test_scenarios.auto_sysid_launch import auto_sysid_launch_description


def generate_launch_description():
    return auto_sysid_launch_description(
        config_filename="auto_longitudinal_speed_step.yaml",
        session_name="A02_speed_step_10_20",
        scenario_summary="10 s zero baseline, then 0->1 and 1<->2 m/s steps",
    )
