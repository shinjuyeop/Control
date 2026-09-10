# Archived kaiev26_hil package

Snapshot of the former standalone HIL package at `d8b3089624424e4eb4e836f76ab25fa12e3df81a`.
The maintained backend now lives in `software/src/vehicle_console/vehicle_console/hil`.
Use `ros2 launch vehicle_console hil.launch.py` for the browser profile,
`hil_backend.launch.py` / `hil_bench.launch.py` for headless operation, and
`ros2 run vehicle_console <tool>` for the migrated CLI tools.

This snapshot preserves Python source, launch/config/scenarios, tests and the old
package metadata. `ORIGINAL_README.md` and `WEB_CONSOLE.md` document that revision;
their commands/imports are historical and are not supported with the new package.
Both this directory and its parent are excluded by `COLCON_IGNORE`.

The local running legacy bench may still reference this snapshot through an ignored
source-path symlink. This does not make the archive an active colcon package.
Virtual environments, old built Web assets and native libraries are ignored local
artifacts, not a second maintained frontend or backend.
