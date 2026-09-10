import type { Data, Snapshot } from "../domain/types";
import { LinearMeter } from "./SignalInstruments";
import { sourceTopics } from "../domain/profile";
import { mpsToKmh } from "../domain/units";
import {
  Badge,
  Details,
  Fact,
  fmt,
  Metric,
  Panel,
  Source,
} from "./instruments";

import {
  states,
  reasons,
  enumText,
  bitHex,
  boolText,
  degrees,
  speedText,
  brakeReasons,
  driveFlags,
  steeringFlags,
  type Flag,
} from "../domain/vehicle";
export { brakeReasons, boolText } from "../domain/vehicle";
function Flags({
  data,
  flags,
  prefix,
}: {
  data: Data;
  flags: Flag[];
  prefix: string;
}) {
  return (
    <div className="flag-grid">
      {flags.map(([field, label, polarity]) => {
        const value = data[field];
        const alarm =
          typeof value === "boolean" &&
          (polarity === "bad_false"
            ? !value
            : (polarity === "bad_true" || polarity === "warn_true") && value);
        return (
          <span
            key={field}
            title={`${prefix}.${field} = ${String(value ?? "unavailable")}`}
            className={`flag-chip ${alarm ? (polarity === "warn_true" ? "warning" : "danger") : ""}`}
          >
            <span>{label}</span>
            <b>{boolText(value)}</b>
          </span>
        );
      })}
    </div>
  );
}
export function received(state: Snapshot, online: boolean, topic: string): Data {
  return online && state.topics[topic]?.fresh ? state.topics[topic].data : {};
}
export function PedalMetric({
  state,
  online,
}: {
  state: Snapshot;
  online: boolean;
}) {
  const drive = received(state, online, "/drive/status");
  // STM_A PA0 is a 12-bit ADC with nominal 3.3 V reference (firmware adc.c).
  const voltage =
    typeof drive.pedal_adc_raw === "number"
      ? (drive.pedal_adc_raw * 3.3) / 4095
      : undefined;
  return (
    <div className="metric pedal-metric" role="group" aria-label="페달 관측">
      <span>페달 관측 · STM_A</span>
      <LinearMeter label="입력 비율" value={drive.pedal_percent} />
      <p
        title={`STM_A ADC raw ${fmt(drive.pedal_adc_raw, 0)} · 3.3 V / 4095 환산`}
      >
        <strong>{fmt(voltage, 2)} V</strong> · ADC 환산
      </p>
    </div>
  );
}
export function VehicleSummary({
  state,
  online,
}: {
  state: Snapshot;
  online: boolean;
}) {
  const drive = received(state, online, "/drive/status"),
    cmd = received(state, online, "/vehicle/command"),
    steer = received(state, online, "/steering/status"),
    brake = received(state, online, "/brake/status");
  const target = cmd.speed_target_mps,
    actual = drive.part1_valid ? drive.speed_mps : undefined;
  const error =
    typeof target === "number" && typeof actual === "number"
      ? target - actual
      : undefined;
  return (
    <>
      <div className="metrics-grid operation-metrics">
        <PedalMetric state={state} online={online} />
        <Metric
          label="주행 목표"
          value={fmt(mpsToKmh(target), 2)}
          unit="km/h"
          note={typeof target === "number" ? "Governor 출력" : "명령 수신 대기"}
        />
        <Metric
          label="관측 속도"
          value={fmt(mpsToKmh(actual), 2)}
          unit="km/h"
          tone="accent"
          note={
            typeof actual === "number"
              ? "STM_A 피드백"
              : "유효한 Part I 수신 대기"
          }
        />
        <Metric
          label="속도 오차 · 목표 − 관측"
          value={fmt(error === undefined ? undefined : mpsToKmh(error), 2)}
          unit="km/h"
          note="목표 − 관측"
        />
        <Metric
          label="조향 상태"
          value={enumText(states, steer.state)}
          note={enumText(reasons, steer.disable_reason)}
        />
        <Metric
          label="STM_B 제동 요청"
          value={
            typeof brake.brake_commanded === "boolean"
              ? brake.brake_commanded
                ? "제동"
                : "해제"
              : "—"
          }
          note={brakeReasons(brake.stop_reason_bits)}
        />
      </div>
      <ActiveConditions state={state} online={online} />
    </>
  );
}
export function ActiveConditions({
  state,
  online,
}: {
  state: Snapshot;
  online: boolean;
}) {
  const d = received(state, online, "/drive/status"),
    s = received(state, online, "/steering/status"),
    b = received(state, online, "/brake/status");
  const conditions: { text: string; source: string; tone: string }[] = [];
  for (const topic of [
    "/vehicle/command",
    "/drive/status",
    "/brake/status",
    "/steering/status",
  ]) {
    if (!online || !state.topics[topic])
      conditions.push({ text: "수신 대기", source: topic, tone: "muted" });
    else if (!state.topics[topic].fresh)
      conditions.push({ text: "수신 지연", source: topic, tone: "warning" });
  }
  if (b.stop_reason_bits)
    conditions.push({
      text: brakeReasons(b.stop_reason_bits),
      source: "brake.stop_reason_bits",
      tone: "warning",
    });
  for (const field of ["command_timeout", "stop_request"])
    if (d[field])
      conditions.push({
        text: field === "command_timeout" ? "COMMAND TIMEOUT" : "STOP REQUEST",
        source: `drive.${field}`,
        tone: "warning",
      });
  if (d.fault_bits_valid && d.fault_bits)
    conditions.push({
      text: `FAULT ${bitHex(d.fault_bits, 6)}`,
      source: "drive.fault_bits",
      tone: "danger",
    });
  if (typeof s.state === "number" && s.state !== 3)
    conditions.push({
      text: enumText(states, s.state),
      source: "steering.state",
      tone: s.state === 8 ? "danger" : "muted",
    });
  if (typeof s.disable_reason === "number" && s.disable_reason !== 0)
    conditions.push({
      text: enumText(reasons, s.disable_reason),
      source: "steering.disable_reason",
      tone: "muted",
    });
  if (typeof s.fault_bits === "number" && s.fault_bits & ~1)
    conditions.push({
      text: `FAULT ${bitHex(s.fault_bits & ~1)}`,
      source: "steering.fault_bits (bit0 제외)",
      tone: "danger",
    });
  return (
    <div className="reported-conditions" aria-label="메시지가 보고한 상태">
      <span className="eyebrow">보드가 보고한 상태</span>
      <div>
        {conditions.length ? (
          conditions.map((c, i) => (
            <span className={`condition ${c.tone}`} key={i}>
              {c.text}
              <code>{c.source}</code>
            </span>
          ))
        ) : (
          <Badge tone="good">추가 사유 없음</Badge>
        )}
      </div>
    </div>
  );
}
export function FrameHealth({
  state,
  online,
}: {
  state: Snapshot;
  online: boolean;
}) {
  const d = received(state, online, "/drive/status");
  return (
    <div
      className="metric frame-summary"
      role="group"
      aria-label="EZkontrol CAN 수신"
    >
      <span>EZkontrol 수신</span>
      {[1, 2].map((part) => {
        const valid = d[`part${part}_valid`],
          age = d[`part${part}_age_ms`];
        const label = `Part ${part === 1 ? "I" : "II"}`;
        return (
          <div
            key={part}
            className={`frame-card ${typeof valid !== "boolean" ? "waiting" : valid ? "" : "invalid"}`}
            role="group"
            aria-label={`EZkontrol ${label}`}
            title={`${part === 1 ? "0x1801D0EF · 전압 / 전류 / RPM" : "0x1802D0EF · 온도 / RUN / Fault"} · STM_A 보고 시점의 마지막 CAN 수신 이후`}
          >
            <strong>{label}</strong>
            <span className="frame-age">
              {fmt(age, 0)}
              <small> ms</small>
            </span>
            <Badge
              tone={
                typeof valid !== "boolean"
                  ? "muted"
                  : valid
                    ? "good"
                    : "warning"
              }
            >
              {typeof valid !== "boolean"
                ? "수신 대기"
                : valid
                  ? "VALID"
                  : "INVALID"}
            </Badge>
          </div>
        );
      })}
    </div>
  );
}
export function ReceptionTable({
  state,
  online,
}: {
  state: Snapshot;
  online: boolean;
}) {
  return (
    <div
      className="reception-table"
      role="table"
      aria-label="ROS 토픽 수신 주기"
    >
      <div className="reception-row reception-header" role="row">
        <span>ROS 토픽</span>
        <span>수신 상태</span>
        <span>수신 Hz</span>
        <span>평균 주기</span>
        <span>마지막 수신 이후</span>
        <span>수신 누적</span>
      </div>
      {sourceTopics(state).map((topic) => {
        const sample = state.topics[topic],
          exists = online && sample;
        return (
          <div className="reception-row" role="row" key={topic}>
            <code>{topic}</code>
            <Badge
              tone={
                exists && sample.fresh ? "good" : exists ? "warning" : "muted"
              }
            >
              {exists ? (sample.fresh ? "수신 중" : "지연") : "수신 대기"}
            </Badge>
            <span>
              {exists ? fmt(sample.rx_hz, 1) : "—"} <small>Hz</small>
            </span>
            <span>
              {exists ? fmt(sample.period_ms, 1) : "—"} <small>ms</small>
            </span>
            <span>
              {exists ? fmt(sample.age_ms, 0) : "—"} <small>ms</small>
            </span>
            <span>
              {exists && typeof sample.rx_count === "number"
                ? sample.rx_count.toLocaleString()
                : "—"}
            </span>
          </div>
        );
      })}
      <p className="muted">
        PC의 ROS 콜백 수신 기준 · 최근 5초의 간격 평균 · 서버의 토픽별 freshness 기준 이후 0 Hz. ECU 상태에서 snapshot 진행과
        task rate를 확인합니다. CAN 주파수나 Web 갱신률(5 Hz)과는 별개입니다.
      </p>
    </div>
  );
}
export function CommandTelemetry({
  state,
  online,
}: {
  state: Snapshot;
  online: boolean;
}) {
  const d = received(state, online, "/drive/status"),
    b = received(state, online, "/brake/status"),
    c = received(state, online, "/vehicle/command");
  const source = (topic: string) => (
    <Source name={topic} sample={state.topics[topic]} online={online} />
  );
  return (
    <Panel title="명령 · 스위치 · 브레이크" eyebrow="GOVERNOR / STM_A / STM_B">
      <Fact
        label="STM_A 운전 모드 / 기어"
        value={`${enumText(["MANUAL", "AUTO"], d.mode)} / ${enumText(["FWD", "REV"], d.gear)}`}
      />
      <Fact
        label="STM_A 페달 / ADC raw"
        value={`${fmt(d.pedal_percent, 0)} % / ${fmt(d.pedal_adc_raw, 0)}`}
      />
      <Fact label="Governor 목표 속도" value={speedText(c.speed_target_mps)} />
      <Fact
        label="Governor 목표 조향각"
        value={degrees(c.steering_target_rad)}
      />
      <Fact label="Governor 브레이크 요청" value={boolText(c.brake_engage)} />
      <Fact
        label="보드가 보고한 E-stop"
        value={
          typeof b.stop_reason_bits === "number"
            ? boolText(!!(b.stop_reason_bits & 4))
            : "—"
        }
      />
      <Fact label="브레이크 commanded" value={boolText(b.brake_commanded)} />
      <Fact
        label="브레이크 command age"
        value={fmt(b.command_age_ms, 0)}
        unit="ms"
      />
      <Fact
        label="브레이크 실제 피드백"
        value={
          typeof b.actual_valid === "boolean"
            ? b.actual_valid
              ? "SUPPORTED"
              : "NOT WIRED · 미배선"
            : "—"
        }
      />
      <Fact label="정지 사유" value={brakeReasons(b.stop_reason_bits)} />
      <Fact label="정지 사유 원본 bits" value={bitHex(b.stop_reason_bits, 2)} />
      {source("/vehicle/command")}
      {source("/brake/status")}
    </Panel>
  );
}
export function DriveTelemetry({
  state,
  online,
}: {
  state: Snapshot;
  online: boolean;
}) {
  const d = received(state, online, "/drive/status");
  const source = (topic: string) => (
    <Source name={topic} sample={state.topics[topic]} online={online} />
  );
  const num = (value: unknown, valid = true, n = 1) =>
    valid ? fmt(value, n) : "—";
  return (
    <Panel
      className="board-telemetry"
      title="구동 보드 관측"
      eyebrow="STM_A / DRIVE STATUS"
    >
      <Fact
        label="모터 RPM"
        value={num(d.motor_rpm, !!d.part1_valid)}
        unit="RPM"
      />
      <Fact
        label="컨트롤러 제어 모드"
        value={
          d.part2_valid
            ? enumText(["TORQUE", "SPEED"], d.controller_control_mode)
            : "—"
        }
      />
      <Fact
        label="STM_A 명령 RPM"
        value={fmt(d.commanded_motor_rpm, 0)}
        unit="RPM"
      />
      <Fact
        label="STM_A 명령 상전류"
        value={fmt(d.commanded_phase_current_a)}
        unit="A"
      />
      <Fact
        label="Host command age"
        value={fmt(d.command_age_ms, 0)}
        unit="ms"
      />
      <div className="telemetry-group">
        <span className="eyebrow">PART I · 전력</span>
        <Fact
          label="배터리 버스 전압"
          value={num(d.bus_voltage_v, !!d.part1_valid)}
          unit="V"
        />
        <Fact
          label="배터리 버스 전류"
          value={num(d.bus_current_a, !!d.part1_valid)}
          unit="A"
        />
        <Fact
          label="모터 상전류"
          value={num(d.phase_current_a, !!d.part1_valid)}
          unit="A"
        />
      </div>
      <div className="telemetry-group">
        <span className="eyebrow">PART II · 온도 / 고장</span>
        <Fact
          label="컨트롤러 온도"
          value={num(d.controller_temperature_c, !!d.part2_valid)}
          unit="°C"
        />
        <Fact
          label="모터 온도"
          value={num(d.motor_temperature_c, !!d.part2_valid)}
          unit="°C"
        />
        <Fact
          label="모터 Fault bits"
          value={d.fault_bits_valid ? bitHex(d.fault_bits, 6) : "—"}
        />
      </div>
      <Details title="구동 상태 플래그">
        <Flags data={d} flags={driveFlags} prefix="drive" />
      </Details>
      {source("/drive/status")}
    </Panel>
  );
}
export function SteeringTelemetry({
  state,
  online,
}: {
  state: Snapshot;
  online: boolean;
}) {
  const s = received(state, online, "/steering/status");
  const source = (topic: string) => (
    <Source name={topic} sample={state.topics[topic]} online={online} />
  );
  const num = (value: unknown, valid = true, n = 1) =>
    valid ? fmt(value, n) : "—";
  return (
    <Panel
      className="board-telemetry"
      title="조향 보드 관측"
      eyebrow="LATERAL CONTROLLER / KEYA"
    >
      <Fact label="제어 상태" value={enumText(states, s.state)} />
      <Fact label="해제 사유" value={enumText(reasons, s.disable_reason)} />
      <Details title="KEYA 모터 · 피드백 · 수신 상태">
        <Fact
          label="모터 위치 count"
          value={num(s.motor_position_count, !!s.feedback_valid, 0)}
        />
        <Fact
          label="모터축 각도"
          value={num(s.motor_angle_deg, !!s.feedback_valid, 2)}
          unit="deg"
        />
        <Fact
          label="모터 속도"
          value={num(s.motor_speed_rpm, !!s.feedback_valid)}
          unit="RPM"
        />
        <Fact
          label="모터 전류"
          value={num(s.motor_current_a, !!s.feedback_valid, 2)}
          unit="A"
        />
        <Fact label="공급 전압" value={fmt(s.bus_voltage_v, 2)} unit="V" />
        <Fact
          label="드라이버 온도"
          value={fmt(s.driver_temperature_c)}
          unit="°C"
        />
        <Fact label="피드백 age" value={fmt(s.feedback_age_ms, 0)} unit="ms" />
        <Fact label="모드 age" value={fmt(s.mode_age_ms, 0)} unit="ms" />
        <Fact
          label="조향 command age"
          value={fmt(s.steer_command_age_ms, 0)}
          unit="ms"
        />
        <Fact
          label="조향에서 수신한 주행 모드"
          value={enumText(["MANUAL", "AUTO"], s.drive_mode)}
        />
      </Details>
      <div className="telemetry-group">
        <span className="eyebrow">KEYA · 상태 비트</span>
        <Fact label="KEYA Fault bits 원본" value={bitHex(s.fault_bits)} />
        <Fact
          label="Fault bits · bit0 제외"
          value={
            typeof s.fault_bits === "number" ? bitHex(s.fault_bits & ~1) : "—"
          }
        />
        <Fact
          label="bit0 · Enable 상태 비트"
          value={
            typeof s.fault_bits === "number" ? String(s.fault_bits & 1) : "—"
          }
        />
      </div>
      <Details title="조향 상태 플래그">
        <Flags data={s} flags={steeringFlags} prefix="steering" />
        <p className="muted">
          Enable 피드백 미지원은 이 하드웨어의 정상 계약이며 고장으로 표시하지
          않습니다.
        </p>
      </Details>
      {source("/steering/status")}
    </Panel>
  );
}

export function SteeringSummary({
  state,
  online,
}: {
  state: Snapshot;
  online: boolean;
}) {
  const s = received(state, online, "/steering/status");
  const c = received(state, online, "/vehicle/command");
  return (
    <section className="steering-summary" aria-label="로드휠 각도 경로">
      <div className="steering-state">
        <strong>{enumText(states, s.state)}</strong>
        <span>{enumText(reasons, s.disable_reason)}</span>
      </div>
      <div className="telemetry-group">
        <span className="eyebrow">
          로드휠 각도 · TARGET → COMMAND → CORRECTION → FEEDBACK
        </span>
        <Fact
          label="목표 각도"
          value={degrees(s.target_wheel_angle_rad ?? c.steering_target_rad)}
        />
        <Fact
          label="제한 적용 명령각"
          value={degrees(s.commanded_wheel_angle_rad)}
        />
        <Fact
          label="보정 후 명령각"
          value={degrees(s.corrected_wheel_angle_rad)}
        />
        <Fact
          label="피드백 각도"
          value={s.feedback_valid ? degrees(s.actual_wheel_angle_rad) : "—"}
        />
        <Fact
          label="추종 오차"
          value={s.feedback_valid ? degrees(s.tracking_error_rad) : "—"}
        />
        <Fact label="추종 보정량" value={degrees(s.tracking_correction_rad)} />
      </div>
    </section>
  );
}
