#!/usr/bin/env python3
"""Temporary user1 entrypoint for AUTO SYSID on Gazebo."""

import os
from datetime import datetime
from pathlib import Path
import shlex

from auto_sysid import overlay_environment


if __name__ == '__main__':
    if os.environ.get('AUTO_SYSID_GAZEBO_ENV') != '1':
        env = overlay_environment()
        env['AUTO_SYSID_GAZEBO_ENV'] = '1'
        command = '\n'.join([
            'source /opt/ros/humble/setup.bash',
            'source /home/user1/ROS2_Workspace/ros2_ws/install/local_setup.bash',
            'source /home/user1/ROS2_Workspace/gazebo_sitl/install/local_setup.bash',
            'exec /usr/bin/python3 ' + shlex.quote(str(Path(__file__).resolve())),
        ])
        os.execvpe('/bin/bash', ['bash', '-c', command], env)
    from vehicle_test_scenarios.sysid_tui import main

    raise SystemExit(main(
        use_sim_time=True,
        require_ezkontrol_status=False,
        records_root=(
            Path('/home/user1/ROS2_Workspace/ros2_ws/src/vehicle_sysid_analysis/test_bag')
            / (datetime.now().strftime('%y%m%d') + '_gazebo_sysid')
        ),
    ))
