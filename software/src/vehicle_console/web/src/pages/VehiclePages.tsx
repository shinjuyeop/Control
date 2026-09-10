import { Chart, Fact, Panel, Source, fmt } from "../components/instruments";
import { HealthIndicator } from "../components/RuntimeInstruments";
import { StateRail, TargetActual } from "../components/SignalInstruments";
import { ActiveConditions, CommandTelemetry, DriveTelemetry, received, SteeringTelemetry } from "../components/VehicleTelemetry";
import { CurrentPlot, DiagnosticObservations, DriveComparison, SteeringComparison, WheelPlot } from "../components/VehicleObservations";
import { boolean, feedbackColor, targetColor } from "../domain/format";
import { reception, steeringState } from "../domain/presentation";
import { sourceTopics } from "../domain/profile";
import { runtimeBoards, runtimeView } from "../domain/runtime";
import { brakeReasons, enumText, speedText, degrees } from "../domain/vehicle";
import { mpsToKmh, radToDeg } from "../domain/units";
import { useTelemetry } from "../hooks/useTelemetry";

export function VehicleOverview() {
  const { state, online } = useTelemetry();
  const command = received(state, online, "/vehicle/command"), drive = received(state, online, "/drive/status"),
    brake = received(state, online, "/brake/status"), steering = received(state, online, "/steering/status");
  const live = sourceTopics(state).filter(topic => online && state.topics[topic]?.fresh).length;
  return <div className="vehicle-overview">
    <StateRail label="차량 핵심 상태" items={[
      { label: "ROS DATA", value: `${live} / 7 LIVE`, detail: "명령 · 구동 · 조향 · 제동 · ECU A/B · 진단", status: { label: online ? "WEB LIVE" : "WEB OFFLINE", tone: online ? "good" : "warning" } },
      { label: "MODE / GEAR", value: `${enumText(["MANUAL", "AUTO"], drive.mode)} / ${enumText(["FWD", "REV"], drive.gear)}`, status: reception(state, online, "/drive/status"), detail: `RUN ${boolean(drive.tractive_commanded)} · pedal ${fmt(drive.pedal_percent, 0)}%` },
      { label: "BRAKE REQUEST", value: boolean(brake.brake_commanded, "ENGAGED", "RELEASED"), status: reception(state, online, "/brake/status"), detail: brakeReasons(brake.stop_reason_bits) },
      { label: "E-STOP · STM_B", value: typeof brake.stop_reason_bits === "number" ? brake.stop_reason_bits & 4 ? "ACTIVE" : "CLEAR" : "UNKNOWN", detail: "brake.stop_reason_bits · 보드 보고", status: { label: typeof brake.stop_reason_bits === "number" ? brake.stop_reason_bits & 4 ? "ESTOP" : "REPORTED CLEAR" : "UNKNOWN", tone: typeof brake.stop_reason_bits !== "number" ? "muted" : brake.stop_reason_bits & 4 ? "danger" : "good" } },
      ...runtimeBoards.map(name => {
        const view = runtimeView(state.runtime?.[name], online);
        return { label: name + " RUNTIME", value: view.label, detail: view.detail, status: { label: view.age, tone: view.tone as "good" | "warning" | "muted" } };
      }),
    ]} />
    <ActiveConditions state={state} online={online} />
    <div className="overview-instruments">
      <Panel title="Drive" eyebrow="GOVERNOR → STM_A" className="overview-drive">
        <TargetActual label="주행 속도 비교" target={typeof command.speed_target_mps === "number" ? mpsToKmh(command.speed_target_mps) : undefined}
          actual={drive.part1_valid ? mpsToKmh(drive.speed_mps) : undefined} unit="km/h" targetLabel="목표" actualLabel="관측" note="AUTO 목표 · Part I 관측" />
        <div className="inline-facts"><Fact label="목표 / 관측 (ROS)" value={`${fmt(command.speed_target_mps, 2)} / ${drive.part1_valid ? fmt(drive.speed_mps, 2) : "—"}`} unit="m/s" />
          <Fact label="Bus" value={drive.part1_valid ? fmt(drive.bus_voltage_v) : "—"} unit="V" /><Fact label="Motor" value={drive.part1_valid ? fmt(drive.motor_rpm, 0) : "—"} unit="RPM" /></div>
        <Source name="/drive/status" sample={state.topics["/drive/status"]} online={online} />
      </Panel>
      <Panel title="Steering" eyebrow="ROAD WHEEL · LEFT POSITIVE" className="overview-steering">
        <TargetActual label="조향각 비교" target={typeof steering.target_wheel_angle_rad === "number" ? radToDeg(steering.target_wheel_angle_rad) : undefined}
          actual={steering.feedback_valid ? radToDeg(steering.actual_wheel_angle_rad) : undefined} unit="deg"
          targetLabel="목표 로드휠" actualLabel="관측 로드휠" note="엔코더 환산 · 좌회전 +" />
        <div className="inline-facts"><Fact label="제어 상태" value={steeringState(steering).label} />
          <Fact label="명령 / 보정" value={`${degrees(steering.commanded_wheel_angle_rad)} / ${degrees(steering.corrected_wheel_angle_rad)}`} /></div>
        <Source name="/steering/status" sample={state.topics["/steering/status"]} online={online} />
      </Panel>
    </div>
    <div className="overview-bottom">
      <Panel title="Vehicle command" eyebrow="GOVERNOR OUTPUT">
        <Fact label="Speed" value={speedText(command.speed_target_mps)} /><Fact label="Steering" value={degrees(command.steering_target_rad)} />
        <Fact label="Brake request" value={boolean(command.brake_engage)} />
        <Source name="/vehicle/command" sample={state.topics["/vehicle/command"]} online={online} />
      </Panel>
      {runtimeBoards.map(name => {
        const board = state.runtime?.[name], view = runtimeView(board, online);
        return <Panel key={name} title={name} eyebrow="ECU RUNTIME" aside={<HealthIndicator board={board} online={online} />}>
          <p className={`runtime-health-line ${view.tone}`}>{view.detail}</p>
          <div className="inline-facts"><Fact label="Heap used" value={fmt(view.data?.heap.percent, 0)} unit="%" /><Fact label="Peak stack" value={fmt(view.data?.worst_stack?.stack.percent, 0)} unit="%" /><Fact label="Miss history" value={view.data?.deadline_misses ?? "—"} /></div>
          <Source name={board?.topic ?? (name === "STM_A" ? "/stm_a/runtime_stats" : "/stm_b/runtime_stats")} sample={board ? state.topics[board.topic] : undefined} online={online} />
        </Panel>;
      })}
    </div>
  </div>;
}

export function VehicleDrive() {
  const { state, online, points } = useTelemetry();
  return <><DriveComparison /><Panel title="주행 목표와 관측 속도" eyebrow="GOVERNOR → STM_A">
    <Chart points={points} traces={[{ key: "targetSpeed", label: "속도 목표", color: targetColor }, { key: "speed", label: "STM_A 속도", color: feedbackColor }]} unit="km/h" />
    <Source name="/vehicle/command" sample={state.topics["/vehicle/command"]} online={online} /><Source name="/drive/status" sample={state.topics["/drive/status"]} online={online} />
  </Panel><CurrentPlot /><DriveTelemetry state={state} online={online} /></>;
}

export function VehicleSteering() {
  const { state, online } = useTelemetry();
  return <><SteeringComparison /><WheelPlot /><SteeringTelemetry state={state} online={online} /></>;
}

export function VehicleSafety() {
  const { state, online } = useTelemetry();
  const drive = received(state, online, "/drive/status"), brake = received(state, online, "/brake/status");
  return <><ActiveConditions state={state} online={online} /><CommandTelemetry state={state} online={online} />
    <Panel title="구동 정지 관련 관측" eyebrow="STM_A / STM_B">
      <Fact label="구동 정지 요청" value={boolean(drive.stop_request)} /><Fact label="RUN 명령" value={boolean(drive.tractive_commanded)} />
      <Fact label="Host timeout" value={boolean(drive.command_timeout)} /><Fact label="제동 사유" value={brakeReasons(brake.stop_reason_bits)} />
      <Fact label="STM_A 상전류 명령" value={fmt(drive.commanded_phase_current_a)} unit="A" />
      <p className="muted">브레이크 위치 피드백은 미배선입니다. 보드의 요청 상태를 표시합니다.</p>
      <Source name="/drive/status" sample={state.topics["/drive/status"]} online={online} />
    </Panel></>;
}

export function VehicleDiagnostics() {
  return <DiagnosticObservations />;
}
