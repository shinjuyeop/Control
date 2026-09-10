# KAI Vehicle Console validation — 2026-09-11

## Initial frontend promotion (historical)

Starting branch/HEAD: `Shin`, `f8a0b766eaa1ab227eec6d574e696024c0ef8ab0`.
Initial worktree was clean and matched freshly fetched `origin/Shin`.
The existing Virtual Touch work (`dbcbb25`) and physical-touch/navigation firmware
were included in the baseline and preserved.

## Software results

| Surface | Completed result |
|---|---|
| `vehicle_console/test` | 29 passed; actual ROS pub/sub, Vehicle read-only WebSocket boundary, Host/Origin, diagnostics identity/age, RuntimeStats, Chrome pages/reconnect/responsive |
| Active `kaiev26_hil/test` after Qt archive | 91 passed; command ownership/takeover, revision fencing, heartbeat/watchdog, scenarios/faults, Virtual LCD/Touch, runtime acknowledgment and offline behavior |
| `visualization/test` | 1 passed; canonical status → diagnostics including E-stop and no command publisher |
| Legacy Qt tests before archive | 26 HIL Qt and 18 Vehicle Qt/cache tests passed before the source/tests were archived |
| TypeScript domain tests | 3 passed using Node's test runner and the existing TypeScript compiler |
| TypeScript | Typecheck and additional unused import/variable check passed |
| Frontend | Vite production build passed; one maintained source tree |
| Targeted colcon | `vehicle_console`, `kaiev26_hil`, `visualization`, `control_launch` built and installed |
| Package discovery | Active Console/HIL/visualization found; archived `control_gui_common` excluded |
| Installed resources | Vehicle/HIL launch files, shared frontend assets and Foxglove `control.json` installed; obsolete Qt entrypoints/launch links removed from the local generated install |

Python tests used isolated localhost domains 193/194/196. Browser tests ran with
installed Chrome/Playwright, not skipped. Vehicle widths 1920/1280/768/390 and the
existing HIL responsive widths were exercised with no horizontal page overflow.
The HIL profile label remains visible on mobile. No browser profile switch exists;
both backend profiles rejected attempts to mutate the launched profile.

The environment reports deprecation warnings from Starlette/httpx/anyio. They did
not fail the tests. No dependency upgrade was performed for those warnings.

## Live observations and limits

At the initial frontend promotion, the pre-existing HIL launch PID `464338` and Web PID `464353` were owned by the
user and left running. Read-only observation found STM_TEST connected/fresh,
UNO USB open/fresh, Virtual Touch supported/ready, and no control owner/ARM.
No control ownership was acquired and no test command, reset, fault or touch was
sent to this live bench. The new HIL backend was also started through the installed
launch on isolated domain 195 without serial/bench hardware.

The **new installed Vehicle profile** subscribed to the existing board graph on
domain 0. Drive, steering, brake, runtime A/B and diagnostics were LIVE. There was
no `/vehicle/command` reception during the measurement; the Console kept it at
NO DATA. The test did not create a command publisher to fill that gap. STM_A runtime
was OK, STM_B showed deadline MISS HISTORY from the real observations.

Chrome loaded the installed Vehicle Overview against that live graph, displayed
`VEHICLE · READ ONLY` and live drive source age, with no page errors or horizontal
overflow. The full seven-topic path, including command observation, was exercised
with isolated ROS sample publishers. This is not a moving real-vehicle road test.

Vehicle Console (8766) and Foxglove (8768 for this check, since the user's old HIL
already occupied 8765) accepted WebSocket connections concurrently. The installed
Foxglove build uses `foxglove.sdk.v1`; its handshake succeeded with that protocol.
Foxglove's repository default remains 8765, distinct from Vehicle's new 8766.
Its message plots/layout and diagnostics implementation were retained.

All agent-owned launch groups, ROS nodes, browser instances and temporary runtime
logs were released and checked after use. The user's existing HIL runtime and
Tailscale Serve configuration were preserved. Its old ignored frontend `dist`
was retained for that already-running process; it is not a second maintained
frontend source and is not installed by the new HIL package.

No firmware, dashboard CAN, actuator control, command_governor, micro-ROS message
or safety semantics changed. No board flash, actuator movement or physical
FT6336U/LCD revalidation was performed. Physical HIL controls on the new server and
an actual mobile-tethering/Tailscale client session remain operational follow-up.

## WebSocket bandwidth

30 actual WebSocket text frames per measurement, compression disabled, excluding
startup/discovery frames. Rates use elapsed monotonic receive time.

| Source | Update rate | Mean JSON payload | Approx. bytes/sec | Approx. Mbps |
|---|---:|---:|---:|---:|
| Live board graph, 6 received topics | 4.993 Hz | 8398.1 B | 41931.7 B/s | 0.3355 |
| Isolated ROS sample graph, all 7 topics | 4.977 Hz | 8315.6 B | 41387.0 B/s | 0.3311 |

These are per-browser uncompressed application payload estimates; TCP/TLS/
Tailscale overhead is excluded. Diagnostic text, number of diagnostic sources and
runtime values change the size, so the mock row is not a maximum. HIL catalogs,
desired state, ownership, event history and LCD PNG are absent from Vehicle frames.
Duplicate raw RuntimeStats and unrendered runtime counters are also excluded.

Architecture and archive decisions: [CONSOLE_PARITY.md](CONSOLE_PARITY.md).
Launch/security instructions: [vehicle_console README](../software/src/vehicle_console/README.md).

## Unified backend package consolidation

Starting branch/HEAD: `Shin`, `d8b3089624424e4eb4e836f76ab25fa12e3df81a`.
The user changed the architecture to a single Console system. HIL's backend,
headless tools, launch/config/scenarios and native renderer installation now belong
to `vehicle_console`. Common Python modules are in `vehicle_console.common`, the
Vehicle entrypoint remains `vehicle_console.vehicle`, and HIL is `vehicle_console.hil`.

- Migrated Python suite: **120 passed** (29 Vehicle/common + 91 HIL), with actual
  isolated ROS pub/sub and Chrome browser tests, including takeover, watchdog,
  scenarios, Virtual LCD/Touch, stale/reconnect and responsive rendering.
- After archiving, the affected Vehicle/HIL Web tests ran again: **42 passed**.
- TypeScript: **3 passed**, typecheck and Vite production build passed.
- Targeted colcon: **vehicle_console, visualization, control_launch** built and
  installed. Source discovery includes the new package and excludes the archived
  `kaiev26_hil`, including when its local compatibility symlink was present.
- Installed launch smoke: `vehicle_console hil.launch.py start_bench:=true` on
  localhost domain 195 with `agents:=false`, no steering/state stack and a temporary
  nonexistent serial path. Bridge, generator, governor and Web started; generator
  reported **enabled=false, web_heartbeat_timeout_sec=3.0**, no owner and no ARM.
- Installed Vehicle launch on isolated domain 196 started with NO DATA and rejected
  an inbound claim with WebSocket **1008**. The fresh-process test constructs the
  Vehicle observer and confirms no `vehicle_console.hil` or old HIL module loaded.
- Installed native dashboard library rendered a **480×320 PNG**; both installed
  profiles served the common index and current JS/CSS assets successfully.
- All smoke-test launch groups and children shut down cleanly and were absent in
  the post-check. No physical USB/CAN device was opened by those test launches.

The previously running user HIL was observed read only after the archive move:
STM_TEST remained connected and UNO display reports stayed fresh. No owner was
acquired and no command/reset/fault/touch was sent to that bench. At consolidation, the local old
source path pointed to the ignored archive to preserve its imports, scenario paths,
Python environment and already-served assets. The old ROS/colcon package
registrations were retired so new invocations use `vehicle_console`.

The default results directory remains `~/.local/state/kaiev26_hil/web_results` to
preserve access to prior scenario runs. This is a retained data path, not an active
package dependency. No telemetry schema/cadence, runtime-health interpretation,
firmware, vehicle command/safety logic or dashboard CAN behavior changed in this
consolidation. Bandwidth numbers above are from the preceding frontend promotion;
this package relocation did not remeasure a mobile/Tailscale link or physical HIL
control/actuator behavior.

## Legacy runtime and launch cleanup

Starting HEAD: `78510e57971882be49089af842609bd9ae502c26`.
After the user authorized legacy HIL shutdown, the recorded launcher, Web server
and child PIDs were already absent, with no remaining member of launch group
464338. No signal needed to be sent to that old runtime.

Removed the old source compatibility symlink, the retired workspace HIL
build/install directories, and the obsolete nested Control build/install prefix.
The package archive, source/configuration history, run results and Tailscale
forwarding configuration remain intact.

Active launch files are now `hil.launch.py`, `vehicle.launch.py` and
`vcan_sim.launch.py`. The former `hil_backend.launch.py` and `hil_bench.launch.py`
were intermediate includes; their functions were absorbed into `hil.launch.py`
before removing them and their installed/generated links. Headless operation uses
`web:=false start_bench:=true`; `agents:=false` selects the minimal backend.

Scoped validation after the launch change:

- Targeted `vehicle_console` colcon build/install passed in `~/KAI_ws`.
- Installed Web + bench launch on isolated localhost domain 195, no Agents or
  hardware: generator remained **enabled=false / watchdog=3.0** even when the
  headless-only overrides requested enabled=true / watchdog=0.0.
- Installed Web-only launch on domain 197: HIL profile served without a bridge or
  generator observation; no hardware backend was requested.
- Installed headless launch on domain 194: bridge/generator/governor appeared,
  no HIL Web node appeared, and generator defaults remained **enabled=true /
  watchdog=0.0**, matching the retired headless entrypoints.
- A separate headless scenario launch started the existing scenario runner and
  preserved the supplied scenario file and results directory parameters.
- All test launches used a temporary nonexistent serial path with Agents,
  steering and state options off. Their exact process groups and children were
  absent after shutdown. No physical actuator or USB/CAN test was performed.

No frontend, HIL domain control/watchdog logic, Vehicle backend or firmware code
changed in this cleanup; the earlier broader test results above were not rerun
or counted as new validation for this launch-only change.
