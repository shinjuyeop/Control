# Steering actuator control

`lateral_ctrl` owns the KEYA steering CAN interface through one production node,
`steering_actuator_node`. Do not run the legacy `steering_control_node` or
`steering_feedback_node` on the same CAN interface at the same time.

## Safety state machine

States are `INIT`, `DISABLED`, `WAIT_CMD`, `ACTIVE`, `ESTOP`, `AEB`, `CAN_LOST`,
`FEEDBACK_LOST`, and `FAULT`.

- `DISABLED`: `/mode` was not received, timed out, is MANUAL, or is invalid.
- `WAIT_CMD`: AUTO and all safety/communication checks are valid, but the motor remains
  disabled and no position frame or tracking update is performed.
- `ACTIVE`: the only state in which the motor may be enabled.
- `ESTOP`, `AEB`, `CAN_LOST`, `FEEDBACK_LOST`, and `FAULT`: motor disabled and all
  current and pending steering commands discarded.

E-STOP, AEB, MANUAL/invalid mode, mode timeout, command timeout, CAN loss, position
feedback loss, motor fault, startup, and shutdown discard the movement target, pending
target, pending flag/generation, command timestamp, tracking integral, and tracking
correction. Recovery never restores an old target.

After recovery the node enters `WAIT_CMD`. A command received before that recovery
boundary is not eligible. A new `/cmd/steer` received afterwards is consumed only by the
control timer after every inhibit is checked again. Activation is:

1. Initialize commanded/corrected angle from valid actual wheel angle.
2. Set the new target and reset tracking control.
3. Send Enable and require successful transmission.
4. Send one position command for the current actual angle and require success.
5. Enter `ACTIVE`; start rate-limited target motion on the next control tick.

`/cmd/steer` callbacks never transmit CAN frames or change state. While `ACTIVE`, a
command age above `steer_command_timeout_ms` (default 200 ms for an expected 50 Hz
controller) immediately disables the motor and returns to `WAIT_CMD`. No-command waiting
in `WAIT_CMD` is normal and is not a timeout fault.

`/mode` has a default timeout of 1000 ms. The STM_A firmware currently publishes `/mode`
on change and periodically every 1000 ms. Therefore 1000 ms has no scheduling or transport
jitter margin and must be measured/tuned before vehicle use. Use:

```bash
ros2 topic hz /mode
ros2 topic delay /mode
```

Record a rosbag and choose a timeout at least 5--10 times the normal publish period and
above the observed maximum gap. If firmware changes to publish only on state changes, mode
timeout is incompatible until periodic publication is restored.

## Parameters and CAN

The default interface is `can0`; the vehicle bus is configured at 250000 bit/s.
The C++ and YAML safety defaults for `hardware_can_enabled` and
`actuator_output_enabled` remain false. The production launch deliberately overrides both
to true. Tracking control defaults false. Existing wheel limit, 60 deg/s rate limiter,
angle converter, and optional tracking controller remain in place. A command governor,
acceleration/jerk limiter, speed-dependent limits, and new PID tuning are future work.

The required version-controlled configuration files are:

- `config/steering_actuator.yaml`
- `config/steering_bench_test.yaml`

Personal overrides should use `*.local.yaml` or `*.local.yml` and stay untracked.

## Running

Configure physical CAN first:

```bash
sudo ip link set can0 down
sudo ip link set can0 type can bitrate 250000
sudo ip link set can0 up
ip -details link show can0
candump -L can0
```

Production (hardware and output enabled by the launch):

```bash
ros2 launch lateral_ctrl steering_actuator.launch.py
```

Bench (actuator and bench node, AUTO at 10 Hz, absolute sequence, and 5 deg/s rate limit):

```bash
ros2 launch lateral_ctrl steering_bench_test.launch.py
```

AUTO without an automatic steering sequence:

```bash
ros2 launch lateral_ctrl steering_bench_test.launch.py \
  steer_sequence_publish_enabled:=false
```

Use an external STM32 `/mode` publisher:

```bash
ros2 launch lateral_ctrl steering_bench_test.launch.py \
  drive_mode_publish_enabled:=false
```

The bench sequence is absolute wheel angle `[0, 5, 0, -5, 0]` degrees, begins after 3 s,
uses a 3 s interval, and does not repeat. `bench_test_enabled` no longer exists: starting
the bench launch is the explicit test action. Before running, lift the wheels, clear all
people from the mechanism, prepare the physical E-STOP, and verify the steering is near
mechanical center. The first absolute 0-degree command can cause a large movement when it
is not centered. Test E-STOP/AEB injection remains disabled by default.

On normal bench-node shutdown it best-effort publishes MANUAL and clears test E-STOP/AEB.
The launch shuts down the actuator if the bench node exits; actuator destruction sends a
best-effort Disable.

Status and CAN inspection:

```bash
ros2 topic echo /steering/status
candump -L can0
```

## vcan verification

```bash
sudo modprobe vcan
sudo ip link add dev vcan0 type vcan
sudo ip link set up vcan0
ros2 launch lateral_ctrl steering_actuator.launch.py can_interface:=vcan0
```

If `vcan0` already exists, omit the add command. Inject valid KEYA feedback while observing
`candump -L vcan0`. Verify startup Disable; AUTO plus feedback produces `WAIT_CMD` without
Enable/position frames; a new steer command produces one Enable followed by the actual-angle
position command; later ticks move toward the target; 200 ms command loss disables; a new
command can reactivate; E-STOP clear and CAN/feedback recovery remain disabled until another
new command; stopping `/mode` reaches `DISABLED`; shutdown sends Disable. There must be no
periodic Enable ping.

## Build and test

The repository also contains an archived package with the same name, so constrain discovery
to the active workspace source tree:

```bash
source /opt/ros/humble/setup.bash
colcon build --base-paths software/src --packages-select lateral_ctrl --symlink-install
source install/setup.bash
colcon test --base-paths software/src --packages-select lateral_ctrl
colcon test-result --verbose
ros2 pkg executables lateral_ctrl
ros2 launch lateral_ctrl steering_actuator.launch.py --show-args
ros2 launch lateral_ctrl steering_bench_test.launch.py --show-args
```

Current `main` has been merged into `Shin`. The legacy `steering_control_node` and
`steering_feedback_node` sources/targets were removed from the integrated result so that
`SteeringActuatorNode` remains the sole CAN owner. If `main` changes again, resolve future
conflicts file by file and rerun build, unit, vcan, and physical bench tests before merge.
