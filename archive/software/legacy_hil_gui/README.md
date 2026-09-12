# Legacy Qt HIL GUI

Archived after the shared KAI Vehicle Console HIL profile passed Web/ROS and
browser regression tests. This is only the former GUI surface; the active
`software/src/vehicle_console/vehicle_console/hil` now owns serial, protocol, commands, scenarios,
emulators, watchdog, control ownership and Virtual Touch.

`hil_gui.py` and `test_hil_gui_smoke.py` preserve the original source/tests.
`hil.launch.py`, `hil_bench.launch.py` and `ORIGINAL_README.md` preserve their
former context. They are historical examples, not current launch commands.
The module originally lived at `kaiev26_hil.hil_gui`; its relative imports need
that package context when examining/running the historical version. Its Qt helper
is preserved in `../control_gui_common`. Nothing here is a colcon build target.

Parity and the deliberate replacement of Qt quick presets with existing Web
controls are documented in [Console parity](../../../software/src/vehicle_console/CONSOLE_PARITY.md).
