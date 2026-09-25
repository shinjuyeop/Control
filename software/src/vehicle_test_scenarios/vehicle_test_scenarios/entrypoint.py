"""Choose the interactive field console or the headless ROS scenario node."""

import sys


def main():
    # ros2 launch supplies --ros-args and params files. Keep that established
    # headless contract, while a direct terminal invocation opens the TUI.
    # --sim-time follows the Gazebo /clock, like the headless use_sim_time parameter.
    if sys.stdin.isatty() and '--ros-args' not in sys.argv[1:]:
        from .sysid_tui import main as tui_main
        return tui_main(use_sim_time='--sim-time' in sys.argv[1:])

    from .scenario_runner import main as node_main
    return node_main()
