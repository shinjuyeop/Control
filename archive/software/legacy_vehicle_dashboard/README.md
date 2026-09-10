# Legacy Qt Vehicle Dashboard

`vehicle_dashboard.py`, `ros_state_cache.py`, their original tests, launch file
and README are retained here after Vehicle Web parity was checked against the
canonical message definitions and exercised with ROS sample publishers and Chrome.
The original module names were `visualization.vehicle_dashboard` and
`visualization.ros_state_cache`. These files are historical context, not an active
ROS package or launch surface. The shared Qt helper is in `../control_gui_common`.

Use `ros2 launch vehicle_console vehicle.launch.py`. The active
`vehicle_state_publisher` package supplies diagnostics and `foxglove` supplies the
engineering bridge/layout. See
[Console parity](../../../software/src/vehicle_console/CONSOLE_PARITY.md) for tested semantics and limits.
