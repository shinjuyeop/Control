# KEYA Steering Actuator Control

## 1. Scope

This package is the actuator layer between the decision/planning command `/cmd/steer` and the
KEYA steering motor. It does not contain Pure Pursuit, Stanley, MPC, or any path-tracking logic.
The decision layer publishes an absolute road-wheel angle; this package applies safety gating,
rate limiting, optional wheel-angle tracking correction, mechanical conversion, and SocketCAN I/O.

Naming is deliberate:

- **Node**: ROS I/O, execution flow, and state management.
- **Controller**: the optional external wheel-angle closed loop.
- **Driver**: KEYA SocketCAN communication.
- **Converter**: wheel/motor/count conversion.

## 2. Structure

```text
/cmd/steer -> clamp -> rate limiter -> WheelAngleTrackingController
           -> SteeringAngleConverter -> KeyaCanDriver -> KEYA motor
                                             |
/steering/status <- safety/state/feedback <- CAN query and heartbeat
```

Files:

- `SteeringActuatorNode`: ROS subscriptions/publications, safety logic, and state machine.
- `WheelAngleTrackingController`: P/PI/PID correction with anti-windup and clamps.
- `SteeringAngleConverter`: one shared calibration for command and feedback.
- `KeyaCanDriver`: non-blocking extended-frame SocketCAN driver and documented parsing.
- `SteeringBenchTestNode`: explicitly armed absolute-angle bench sequence and test safety input.

`archive/steering_control` was analyzed but is not built. Its unconditional startup Enable,
periodic Enable ping, incorrect Disable payload, duplicated conversions, and unsafe target reuse
are not carried forward.

The generated firmware copy of `control_msgs/SteeringControllerStatus` was also inspected. It is
the generic ros2_control steering-joint status (arrays of traction/steering positions and commands),
not an actuator safety/diagnostic message, and its original source definition is absent from this
workspace. Therefore it is not modified or misused; the host-only
`lateral_ctrl/SteeringActuatorStatus` message carries the required safety and KEYA fields.

## 3. Confirmed repository conventions

| Topic | Type | Meaning |
|---|---|---|
| `/cmd/steer` | `std_msgs/Float32` | Absolute wheel angle in degrees; left negative, right positive |
| `/mode` | `std_msgs/UInt8` | `0=MANUAL`, `1=AUTO` |
| `/estop` | `std_msgs/UInt8` | nonzero is active |
| `/aeb` | `std_msgs/UInt8` | nonzero is active |
| `/ctrl/steer` | `std_msgs/Float32` | legacy actual-angle compatibility output |

Archived `ThrottleData.asms=true` means MANUAL. The archived serial node ORs hardware E-STOP
with AEB into `ThrottleData.estop`. `throttle_msgs` is not an active package under `software/src`,
so this implementation deliberately uses the unambiguous direct inputs. Setting
`legacy_throttle_input_enabled=true` produces an error explaining this limitation instead of
silently guessing or adding an archive dependency.

## 4. Hardware output opt-ins

No physical CAN output is possible with default configuration. Both parameters must be explicitly
true:

```text
hardware_can_enabled && actuator_output_enabled
```

Bench publishing has separate opt-ins:

```text
bench_test_enabled && drive_mode_publish_enabled
bench_test_enabled && steer_sequence_publish_enabled
bench_test_enabled && safety_injection_enabled
```

Production launch never enables test input injection and never starts the bench node.

## 5. Enable, Disable, and safety

KEYA Disable keeps driver power and CAN communication alive but disables active motor torque.
Enable activates the internal position controller and may create holding torque.

Policy:

- startup, missing mode, MANUAL, invalid mode: Disable;
- E-STOP, AEB, motor fault, feedback invalid, or CAN timeout: Disable;
- AUTO becomes enabled only after valid accumulated encoder feedback;
- startup and every safety recovery hold the current measured angle;
- a new `/cmd/steer` received after `AUTO_READY` is required before movement;
- shutdown sends best-effort Disable.

Test and real safety inputs are monotonic ORs:

```text
effective_estop = real_estop || test_estop
effective_aeb   = real_aeb   || test_aeb
```

A false test value can never clear a true real input. E-STOP/AEB rising edges attempt immediate
Disable without waiting for the 50 Hz control tick. No extra latch or reset topic is introduced.

Disable reason priority:

1. E-STOP
2. CAN timeout
3. motor fault
4. AEB
5. mode not received
6. MANUAL
7. invalid mode
8. feedback invalid
9. none

States are `INIT`, `DISABLED`, `AUTO_READY`, `ACTIVE`, `ESTOP`, `AEB`, `CAN_LOST`, and `FAULT`.
Enable/Disable frames are emitted on state boundaries, not as a periodic Enable ping. Position
commands repeat while enabled so the motor watchdog and position hold remain serviced.

## 6. KEYA CAN

The supplied KY170DJD01005-08G-V1 manual specifies:

- normal CAN 2.0 extended frames;
- factory baud rate 250 kbit/s;
- command ID `0x06000000 + node_id`;
- response ID `0x05800000 + node_id`;
- heartbeat ID `0x07000000 + node_id`;
- default node ID 1;
- watchdog approximately 1000 ms;
- absolute position scale 10,000 counts/revolution.

Payloads:

```text
Enable:   23 0D 20 01 00 00 00 00
Disable:  23 0C 20 01 00 00 00 00
Position: 23 02 20 01 XX XX XX XX  (signed int32, big-endian)
```

Queries implemented exactly as documented:

| Value | Query |
|---|---|
| current | `40 00 21 01 00 00 00 00` |
| speed | `40 03 21 01 00 00 00 00` |
| accumulated encoder | `40 04 21 02 00 00 00 00` |
| fault | `40 12 21 01 00 00 00 00` |
| bus voltage | `40 0D 21 02 00 00 00 00` |
| temperature | `40 0F 21 01 00 00 00 00` |

The accumulated 10,000-count encoder query is authoritative for steering angle. Heartbeat position
is not interpreted as steering position because its 360-count semantics and multi-turn behavior are
not precise enough for this conversion. Heartbeat still provides communication, speed, current,
and fault evidence.

Socket write success updates only `enabled_commanded`. The manual does not define a trustworthy
actual Enable-status query for this protocol, so `enabled_feedback_valid=false` and
`enabled_feedback` is not guessed. Physical CAN loss may also prevent the Disable frame from
arriving; the motor's internal watchdog is the final defense.

## 7. Angle conversion and tracking

For motor magnitude `x`, road-wheel magnitude is:

```text
y = A*x^2 + B*x + C
A=0.0002, B=0.2159, C=0.1612
```

The mapping is odd-symmetric and `direction_sign=-1` preserves the confirmed left-negative,
right-positive wheel convention against the motor direction. Both command and feedback use the
same converter. `counts_per_rev=10000`, so one count is 0.036 motor degree. Wheel commands clamp
to ±30 degrees.

The production rate limit defaults to 60 deg/s and requires vehicle tuning. Bench launch overrides
it to 5 deg/s. On AUTO entry and every safety/CAN/fault recovery, the rate limiter origin is reset
to measured angle.

`WheelAngleTrackingController` is separate from the KEYA internal motor position loop. Default
`tracking_control_enabled=false` still measures tracking error but produces zero correction. When
enabled, P/PI/PID has integral anti-windup, invalid-dt handling, correction clamp, finite-value
checks, and reset on every inhibit. Gains default to zero because no vehicle data was supplied.

## 8. Outputs and visualization

| Topic | Type | Description |
|---|---|---|
| `/steering/angle` | `Float32` | estimated actual wheel angle |
| `/ctrl/steer` | `Float32` | legacy-compatible estimated actual wheel angle |
| `/steering/enable_commanded` | `Bool` | node requested Enable |
| `/steering/enabled` | `Bool` | confirmed Enable; false while validity is unavailable |
| `/steering/status` | `lateral_ctrl/SteeringActuatorStatus` | complete state for Foxglove/LCD |

Plot target, commanded, corrected, actual, tracking error, and correction together. Plot motor
current/speed/steering rate separately, and voltage/temperature separately. Indicators should show
state, disable reason, commanded/feedback Enable, communication, and tracking flags. LCD rendering
is intentionally outside this package.

## 9. Important parameters

| Parameter | Default | Meaning |
|---|---:|---|
| `hardware_can_enabled` | false | first physical-output opt-in |
| `actuator_output_enabled` | false | second physical-output opt-in |
| `can_interface` | can1 | confirmed steering CAN interface |
| `node_id` | 1 | KEYA node address |
| `control_period_ms` | 20 | central 50 Hz period |
| `can_timeout_ms` | 500 | feedback timeout |
| `wheel_limit_deg` | 30 | absolute wheel limit |
| `max_steering_rate_deg_s` | 60 | production rate limit |
| `counts_per_rev` | 10000 | absolute encoder command/query scale |
| `direction_sign` | -1 | motor/wheel direction relation |
| `test_input_injection_enabled` | false | subscribe to test safety topics |
| `tracking_control_enabled` | false | external tracking correction |
| `tracking_kp/ki/kd` | 0 | untuned external loop gains |
| `tracking_integral_limit` | 1 | anti-windup limit |
| `tracking_correction_limit_deg` | 0.5 | correction clamp |

All topics and remaining warning/fault thresholds are in `config/steering_actuator.yaml`.

## 10. Production and bench launch

The repository contains an archived package with the same name, so build and test from the active
software tree explicitly:

```bash
source /opt/ros/humble/setup.bash
colcon build --base-paths software/src --packages-select lateral_ctrl
source install/setup.bash
colcon test --base-paths software/src --packages-select lateral_ctrl
colcon test-result --verbose --test-result-base build/lateral_ctrl
```

Disarmed production:

```bash
ros2 launch lateral_ctrl steering_actuator.launch.py
```

Physical output requires both explicit launch arguments:

```bash
ros2 launch lateral_ctrl steering_actuator.launch.py \
  hardware_can_enabled:=true actuator_output_enabled:=true
```

Before doing this, verify 250 kbit/s configuration, node ID, wiring, mechanical center, wheel
clearance, E-STOP, and a person ready at the power disconnect.

Disarmed bench:

```bash
ros2 launch lateral_ctrl steering_bench_test.launch.py
```

The default absolute sequence is exactly `[0, +1, 0, -1, 0]` degrees. It does not add the current
angle. The sequence waits three seconds after opt-in; AUTO first holds current feedback, and only
the actual sequence applies absolute zero. If the wheels are not near zero, the first zero command
can cause a large move.

After a successful ±1 degree test, set `[0,2,0,-2,0]`, then `[0,5,0,-5,0]`. Do not skip stages.
Typical explicit bench launch:

```bash
ros2 launch lateral_ctrl steering_bench_test.launch.py \
  hardware_can_enabled:=true actuator_output_enabled:=true \
  bench_test_enabled:=true drive_mode_publish_enabled:=true \
  steer_sequence_publish_enabled:=true
```

Test E-STOP/AEB injection additionally requires `safety_injection_enabled:=true`. Test false values
are published only to `/steering/test/*`, never to real safety topics. When the bench node exits,
the launch shuts down the actuator node; both perform best-effort safe shutdown actions.

## 11. vcan integration test

The repository normally uses `vcan1` for lateral control; the following uses the requested `vcan0`
procedure and overrides `can_interface`:

```bash
sudo ip link add dev vcan0 type vcan
sudo ip link set up vcan0
candump vcan0
ros2 run lateral_ctrl steering_actuator_node --ros-args \
  -p can_interface:=vcan0 -p hardware_can_enabled:=true \
  -p actuator_output_enabled:=true -p test_input_injection_enabled:=true
```

Simulate heartbeat and accumulated position feedback:

```bash
cansend vcan0 07000001#0000000000000000
cansend vcan0 05800001#6004210200000000
ros2 topic pub --once /mode std_msgs/msg/UInt8 '{data: 0}'
ros2 topic pub --once /mode std_msgs/msg/UInt8 '{data: 1}'
ros2 topic pub --once /cmd/steer std_msgs/msg/Float32 '{data: 1.0}'
ros2 topic echo /steering/status
ros2 topic echo /steering/angle
```

Exercise test safety:

```bash
ros2 topic pub --once /steering/test/aeb std_msgs/msg/UInt8 '{data: 1}'
ros2 topic pub --once /steering/test/aeb std_msgs/msg/UInt8 '{data: 0}'
ros2 topic pub --once /steering/test/estop std_msgs/msg/UInt8 '{data: 1}'
ros2 topic pub --once /steering/test/estop std_msgs/msg/UInt8 '{data: 0}'
```

Verify with `candump`: startup Disable; no Enable without mode and position feedback; MANUAL
Disable; AUTO current-position hold; new command ACTIVE; safety immediate Disable; stopped
feedback becomes `CAN_LOST`; restored heartbeat plus encoder becomes `AUTO_READY` without replaying
the old target; actual angle remains published in MANUAL; shutdown sends Disable. Runtime bench
parameters can be changed with `ros2 param set /steering_bench_test_node ...`.

## 12. Troubleshooting and limitations

- No Enable: confirm both hardware opt-ins, AUTO mode, current safety inputs, accumulated position
  response, CAN ID/node ID, and timeout.
- Wrong direction: stop immediately; verify mechanical linkage and `direction_sign`.
- Incorrect center: perform the manual's mechanical-zero procedure before an absolute bench test.
- CAN disconnect fault: commands must remain within the motor's watchdog interval; inspect bitrate,
  termination, power sag, and harness.
- Undervoltage/current faults: the manual calls for appropriate supply current and cable gauge.

Known limitations:

- no `/mode` timeout and no `/cmd/steer` timeout, by requirement;
- actual Enable feedback is unavailable in the documented normal-CAN protocol;
- actual wheel angle is a motor-encoder-based estimate, not an independent road-wheel sensor;
- mechanical backlash and linkage hysteresis are not compensated;
- external PID is not tuned;
- tracking error fault is reported but does not add an unrequested hard inhibit;
- archived `ThrottleData` input is not linked into the active workspace;
- absolute bench testing requires physical center verification.

Future work: independent wheel-angle sensor, PI/PID tuning, hardened tracking-error fault policy,
current-based stall detection, speed-dependent limits/rates, HIL, rosbag analysis, mode heartbeat
timeout, and steer-command timeout.

No firmware message definition or generated micro-ROS header was changed. The new status message is
host-side only, so micro-ROS interface regeneration is not required unless firmware later consumes
`lateral_ctrl/SteeringActuatorStatus`.
