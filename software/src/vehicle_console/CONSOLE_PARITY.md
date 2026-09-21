# Legacy GUI parity and archive decision

> 이 문서는 Console 통합 당시의 판단·검증 기록이다. 날짜·테스트 수·당시 조건은
> 이력을 유지하며, 현재 설치·실행 방법은 [README](README.md)와 [HIL Web](HIL_WEB.md)을 따른다.

Baseline: `Shin` `f8a0b766eaa1ab227eec6d574e696024c0ef8ab0`, including shared HIL
control takeover, RuntimeStats and UNO Virtual Touch. The working tree was clean.
The comparison uses source contracts, existing Qt tests, Web/ROS tests and the
built browser UI. No firmware, CAN or actuator behavior is changed.

| Qt Vehicle Dashboard behavior | Shared Console replacement / verification |
|---|---|
| One glance at vehicle command, mode/gear, drive, steering and brake | Vehicle Overview: StateRail, target/actual readings, active conditions, ECU summaries; desktop and mobile Chrome |
| Command/drive/brake 300 ms; steering 500 ms stale defaults | Same Vehicle defaults, configurable at startup; ROS sample tests exercise LIVE → STALE and reconnect |
| Missing data and invalid feedback never look like zero | NO DATA, STALE and source age; invalid Part I/II and steering feedback gated in shared telemetry components |
| Active conditions name the source field | Shared ActiveConditions: brake reasons, drive timeout/stop/fault, steering state/reason/fault |
| KEYA policy comes from Host | Raw bits remain visible; `steering/safety` diagnostics provide all decoded bits, classes, recovery and reconnect state. The UI does not classify raw bits as faults. |
| Nominal flag polarity; unsupported feedback is informational | Shared drive/steering flag definitions; brake actual false → NOT WIRED; enable feedback valid false remains informational |
| Detailed power, temperature, command age, steering pipeline and motor observations | Drive, Steering and Safety pages reuse the former HIL telemetry components |
| Read-only subscriptions | VehicleObserver has no command publishers; only the fixed steering HARD-latch Trigger client is allowed. All inbound WebSocket messages remain rejected; same-origin POST performs acknowledgment; parameter services disabled |
| ROS m/s and rad, operator km/h and degrees | Same unit conversion; Overview retains ROS speed beside operator speed; source messages unchanged |
| Qt's stale fields remained visible beside stale badges | Web hides stale current readings and retains explicitly labeled historical graphs/diagnostics; browser source age keeps advancing during network silence |

| Qt HIL GUI behavior | Shared Console HIL replacement / verification |
|---|---|
| Mode/gear/pedal and DASH/SIDE/RF E-stop | Existing Control Dock and browser action tests |
| AUTO+Forward, pedal 10/30%, Dashboard E-stop quick presets | Same values through existing controls; the redundant one-click shortcuts are not copied |
| EZ RUN/control mode/RPM/current/Plant target, brake PWM, KEYA enable/target | Bench, Drive, Steering, Safety and Diagnostics |
| Closed-loop / direct override; speed/RPM, voltage/current/temperature, shaft position | Existing EmulatorControls, unchanged accepted values and command routes |
| CAN enable, TX drop, controller faults, fault clear | Existing Fault Injection and Web/ROS tests |
| Safe state and status refresh | Existing Bench Reset / refresh; serial-offline reset can queue desired defaults after taking control |
| Production command source | Existing ARM confirmation and common Control Dock; existing Web lease/watchdog retained |
| Original Qt did not disarm on window close | Existing Web policy remains: disconnect/expired lease/takeover disarms and fences old actions; no new change to this policy |
| Qt had no scenario UI, ECU Runtime or UNO mirror/touch | Existing Web scenarios, RuntimeStats, Virtual LCD and Virtual Touch preserved; browser and backend regression tests |

Decision: archive both Qt surfaces after the first profile tests and targeted
colcon build passed. Only legacy GUI code/tests/launch context moved; active HIL
backend and vcan SIL remain. Vehicle diagnostics now belong to
`vehicle_state_publisher`, while the Foxglove bridge/layout belong to `foxglove`.
Archives are under
`archive/software/legacy_hil_gui`, `legacy_vehicle_dashboard` and
`control_gui_common`; `archive/software/COLCON_IGNORE` excludes all three.

The two GUI packages were the only active consumers of `control_gui_common` and
PyQt5. Their active dependencies were removed. `python3-tk` and
`python3-matplotlib` remain because active `vehicle_console.hil.vcan_sim.hmi` imports them.

Physical vehicle operation, actuator movement, LCD panel output and physical
FT6336U touch were not repeated by these software tests. Historical board evidence
in the HIL/UNO runbooks remains historical evidence, not proof of this refactor.

## HIL backend package consolidation

After the initial frontend/Qt migration, the user chose one Console system.
The active HIL implementation is now `vehicle_console.hil`, shared infrastructure
is `vehicle_console.common`, and the Vehicle entrypoint only adds explicit steering HARD-latch acknowledgment without
initializing HIL modules. The former complete package is archived separately at
`archive/software/kaiev26_hil`; its archived code is excluded from colcon discovery.
Existing HIL node names, commands, watchdog, scenario and virtual-HMI behavior are
preserved. Headless launch and CLI tools remain available from `vehicle_console`.
