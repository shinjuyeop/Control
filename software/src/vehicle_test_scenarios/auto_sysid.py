#!/usr/bin/env python3
"""Start the field SYSID interface from a sourced shell or a ROS workspace."""

from pathlib import Path
import os
import shlex
import sys


def main():
    here = Path(__file__).resolve()
    if len(sys.argv) > 1:
        print('Usage: python3 auto_sysid.py\n'
              'Rich TUI: 1-4 scenario, V maximum target speed, R Run, S Stop, Q Quit')
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
        env = os.environ.copy()
        env['AUTO_SYSID_ROS_ENV'] = '1'
        command = '\n'.join('source ' + shlex.quote(str(path)) for path in dict.fromkeys(setups))
        command += '\nexec /usr/bin/python3 ' + shlex.quote(str(here))
        os.execvpe('/bin/bash', ['bash', '-c', command], env)
    sys.path.insert(0, str(here.parent))
    from vehicle_test_scenarios.sysid_tui import main as run
    return run()


if __name__ == '__main__':
    raise SystemExit(main())
