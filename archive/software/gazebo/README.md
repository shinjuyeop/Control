# Archived PID / Gazebo SILS experiments

Moved from `software/src/gazebo` on 2026-09-11 after retiring the legacy PID/SILS
bundle from the active Control workspace. Source baseline: `f4a68398ab978c656bf22d988f747f2dbe055c59`.
This directory is excluded from colcon discovery by `COLCON_IGNORE`.

The original files are preserved:

- `pid_gazebo`: two-channel RPM/heading PID, odometry feedback, setpoint publisher,
  PID/SILS launch files, gain CSV, bag analysis and MATLAB tuning scripts.
- `assets`: SDF models and the tracked vehicle STL mesh.

The controller consumes `/odom`, `/setpoint/rpm`, `/setpoint/steer_deg` and produces
`/cmd_vel`. It is an older differential-drive experiment, not the canonical
`/planning/command` -> governor -> `/vehicle/command` vehicle/HIL path.
No active Control package or launch referenced `pid_gazebo` at retirement.

The old automatic experiment script includes process termination, removal of
previous experiment bags/CSV and overwriting PID gains. It is preserved as
historical source and was not executed for this archive operation. Restoring it
requires reviewing those behaviors and its Gazebo/ROS bridge environment first.

No bag/recording directories were present in this source subtree when moved.
The retired `~/KAI_ws/build/pid_gazebo` and `install/pid_gazebo` generated package
artifacts were removed. Other experiment records and workspaces were not moved.

The separate Gazebo SYSID entrypoint is maintained in
`Simulation/kaiev26_gazebo_bringup` and does not depend on this package.
The Console's `vehicle_console.hil.vcan_sim` also remains active.
