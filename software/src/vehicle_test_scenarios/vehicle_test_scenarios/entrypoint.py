"""Choose the interactive field console or the headless ROS scenario node."""

import sys


def main():
    # ros2 launch supplies --ros-args and params files. Keep that established
    # headless contract, while a direct terminal invocation opens the TUI.
    if sys.stdin.isatty() and '--ros-args' not in sys.argv[1:]:
        from .sysid_tui import main as tui_main
        return tui_main()

    from .scenario_runner import main as node_main
    return node_main()
