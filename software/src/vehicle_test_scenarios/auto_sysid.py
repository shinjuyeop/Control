#!/usr/bin/env python3
"""Start the field SYSID interface from a sourced shell or a ROS workspace."""

from pathlib import Path
import os
import shlex
import sys


def overlay_environment():
    # Colcon's DSV hooks do not reorder paths already inherited from the shell.
    # Rebuild the overlay paths from the selected setup files instead.
    env = os.environ.copy()
    for name in ('PYTHONPATH', 'LD_LIBRARY_PATH', 'AMENT_PREFIX_PATH',
                 'CMAKE_PREFIX_PATH', 'COLCON_PREFIX_PATH',
                 'COLCON_CURRENT_PREFIX', 'AMENT_CURRENT_PREFIX'):
        env.pop(name, None)
    return env


def main():
    here = Path(__file__).resolve()
    if len(sys.argv) > 1:
        print('Usage: python3 auto_sysid.py\n'
              'Rich TUI: arrow keys + Enter to select/edit, '
              'R Run, S Stop, Q Quit')
        return 0 if sys.argv[1:] in (['--help'], ['-h']) else 2
    if os.environ.get('AUTO_SYSID_ROS_ENV') != '1':
        setups = []
        humble = Path('/opt/ros/humble/setup.bash')
        if humble.is_file():
            setups.append(humble)
        # A field checkout has one install; the development checkout also uses ros2_ws.
        for ancestor in reversed(here.parents):
            sibling = ancestor / 'ros2_ws/install/local_setup.bash'
            if sibling.is_file():
                setups.append(sibling)
        for ancestor in here.parents:
            local = ancestor / 'install/local_setup.bash'
            if local.is_file():
                setups.append(local)
                break
        if not setups:
            print('ROS 2 Humble / workspace install을 찾지 못했습니다.', file=sys.stderr)
            return 1
        env = overlay_environment()
        env['AUTO_SYSID_ROS_ENV'] = '1'
        command = '\n'.join('source ' + shlex.quote(str(path)) for path in dict.fromkeys(setups))
        command += '\nexec /usr/bin/python3 ' + shlex.quote(str(here))
        os.execvpe('/bin/bash', ['bash', '-c', command], env)
    sys.path.insert(0, str(here.parent))
    from vehicle_test_scenarios.sysid_tui import main as run
    return run()


if __name__ == '__main__':
    raise SystemExit(main())
