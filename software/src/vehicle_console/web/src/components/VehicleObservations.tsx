import type { Trace } from "./instruments";
import type { ReactNode } from "react";
import { Badge, Chart, Details, Fact, Panel, Source } from "./instruments";
import { SteeringArc, TargetActual } from "./SignalInstruments";
import { FrameHealth, PedalMetric, received, ReceptionTable, SteeringSummary } from "./VehicleTelemetry";
import { steeringState } from "../domain/presentation";
import { auxiliaryColor, feedbackColor, targetColor } from "../domain/format";
import { mpsToKmh, radToDeg } from "../domain/units";
import { useTelemetry } from "../hooks/useTelemetry";
import type { Data } from "../domain/types";

const wheelTraces: Trace[] = [
  { key: "targetWheel", label: "로드휠 명령", color: targetColor },
  { key: "wheel", label: "엔코더 환산 관측", color: feedbackColor },
];

const currentTraces: Trace[] = [
  { key: "bus", label: "Bus current", color: auxiliaryColor },
  { key: "phase", label: "Phase current", color: feedbackColor },
];

export function DriveComparison() {
  const { state, online } = useTelemetry();
  const drive = received(state, online, "/drive/status");
  const target = received(state, online, "/vehicle/command").speed_target_mps;
  const actual = drive.part1_valid ? drive.speed_mps : undefined;
  return <div className="instrument-pair">
    <TargetActual label="주행 속도 비교" target={target === undefined ? undefined : mpsToKmh(target)}
      actual={actual === undefined ? undefined : mpsToKmh(actual)} unit="km/h"
      targetLabel="주행 목표 · Governor" actualLabel="관측 속도 · STM_A"
      note="Host 목표는 AUTO 명령 · 피드백은 Part I 유효값" />
    <div className="instrument-companion"><PedalMetric state={state} online={online} /><FrameHealth state={state} online={online} /></div>
  </div>;
}

export function SteeringComparison({ motorTargetDeg }: { motorTargetDeg?: number }) {
  const { state, online } = useTelemetry();
  const steering = received(state, online, "/steering/status");
  const target = steering.target_wheel_angle_rad ?? received(state, online, "/vehicle/command").steering_target_rad;
  return <div className="instrument-pair">
    <SteeringArc status={steeringState(steering)} target={typeof target === "number" ? radToDeg(target) : undefined}
      actual={steering.feedback_valid ? radToDeg(steering.actual_wheel_angle_rad) : undefined}
      motorAngleDeg={steering.feedback_valid ? steering.motor_angle_deg : undefined} motorTargetDeg={motorTargetDeg} />
    <SteeringSummary state={state} online={online} />
  </div>;
}

export function WheelPlot({ aside }: { aside?: ReactNode }) {
  const { state, online, points } = useTelemetry();
  return <Panel title="로드휠 위치 추종" eyebrow="LATERAL CONTROLLER" aside={aside}>
    <Chart points={points} traces={wheelTraces} unit="deg" />
    <Source name="/steering/status" sample={state.topics["/steering/status"]} online={online} />
  </Panel>;
}

export function CurrentPlot() {
  const { points } = useTelemetry();
  return <Panel title="전류 응답" eyebrow="STM_A / PART I">
    <Chart points={points} traces={currentTraces} unit="A" height={190} />
    <p className="muted">배터리 버스 전류와 모터 상전류는 서로 다른 값입니다.</p>
  </Panel>;
}

export function DiagnosticObservations() {
  const { state, online } = useTelemetry();
  const sample = state.topics["/diagnostics"];
  const statuses = sample?.data.status as Data[] | undefined;
  return <Panel title="ROS 수신 상태" eyebrow="TELEMETRY SOURCES">
    <ReceptionTable state={state} online={online} />
    <div className="diagnostic-observations">
      {statuses?.map((item, i) => {
        const fresh = online && sample.fresh && item.fresh !== false;
        const label = fresh ? ["OK", "WARN", "ERROR", "STALE"][item.level] ?? "UNKNOWN" : "STALE";
        return <div className="diagnostic-item" key={`${item.hardware_id}:${item.name}:${i}`}>
          <h3><Badge tone={fresh ? item.level === 0 ? "good" : item.level === 2 ? "danger" : "warning" : "muted"}>{label}</Badge> {item.name}</h3>
          <small>{item.hardware_id} · {Math.round((item.age_ms ?? sample.age_ms))} ms 전</small>
          <p>{fresh ? item.message : "마지막 진단 보고 · 현재 상태 확인 불가"}</p>
          <Details title={fresh ? "진단 상세" : "마지막 수신 기록 (STALE)"}>
            {!fresh && <p>{item.message}</p>}
            {item.values?.map((v: Data) => <Fact key={v.key} label={v.key} value={v.value} />)}
          </Details>
        </div>;
      }) ?? <p className="muted">NO DATA · diagnostics 수신 대기</p>}
    </div>
  </Panel>;
}
