#!/usr/bin/env python3
"""Temporary user1 entrypoint for AUTO SYSID on Gazebo."""

import os
from datetime import datetime
from pathlib import Path
import shlex

from auto_sysid import overlay_environment


if __name__ == '__main__':
    here = Path(__file__).resolve()
    ros2_workspace = next((
        ancestor / 'ros2_ws'
        for ancestor in reversed(here.parents)
        if (ancestor / 'ros2_ws/install/local_setup.bash').is_file()
    ), None)
    gazebo_workspace = next((
        ancestor
        for ancestor in here.parents
        if (ancestor / 'install/local_setup.bash').is_file()
    ), None)
    if ros2_workspace is None or gazebo_workspace is None:
        raise SystemExit('ros2_ws / Gazebo workspace install을 찾지 못했습니다.')

    if os.environ.get('AUTO_SYSID_GAZEBO_ENV') != '1':
        env = overlay_environment()
        env['AUTO_SYSID_GAZEBO_ENV'] = '1'
        command = '\n'.join([
            'source /opt/ros/humble/setup.bash',
            'source ' + shlex.quote(str(ros2_workspace / 'install/local_setup.bash')),
            'source ' + shlex.quote(str(gazebo_workspace / 'install/local_setup.bash')),
            'exec /usr/bin/python3 ' + shlex.quote(str(here)),
        ])
        os.execvpe('/bin/bash', ['bash', '-c', command], env)
    from vehicle_test_scenarios.sysid_tui import main

    raise SystemExit(main(
        use_sim_time=True,
        require_ezkontrol_status=False,
        records_root=(
            ros2_workspace / 'src/vehicle_sysid_analysis/test_bag'
            / (datetime.now().strftime('%y%m%d') + '_gazebo_sysid')
        ),
    ))
