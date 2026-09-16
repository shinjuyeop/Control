#!/usr/bin/env bash
set -euo pipefail

tool_dir=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)

# This machine's graphical session runs on :1 rather than the SSH shell's
# (unset) DISPLAY, and remote GL acceleration isn't available, so Qt Quick
# needs the software renderer.
export DISPLAY=:1
export XAUTHORITY="$HOME/.Xauthority"
export QT_QUICK_BACKEND=software

# A sourced ROS/Gazebo shell can put the system Qt ahead of PySide6's bundled Qt.
# This tool has no ROS dependency, so give Qt a clean dynamic-library search path.
if command -v uv >/dev/null 2>&1; then
    exec env -u LD_LIBRARY_PATH uv run --isolated \
        --with-requirements "$tool_dir/requirements-gui.txt" \
        python "$tool_dir/keya_v43_gui.py" "$@"
fi

exec env -u LD_LIBRARY_PATH python3 "$tool_dir/keya_v43_gui.py" "$@"
