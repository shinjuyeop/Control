import { targetColor, feedbackColor } from "../domain/format";
import { useWorkbench } from "../hooks/useWorkbench";
import {
  Chart,
  Choice,
  Details,
  Fact,
  fmt,
  Notice,
  Numeric,
  Panel,
  Source,
} from "./instruments";
import { boolText } from "./VehicleTelemetry";

export function TractionFeedback() {
  const { hil, disabled, command, commands } = useWorkbench();
  const numeric = (
    field: string,
    label: string,
    key: string,
    min: number,
    max: number,
    unit: string,
    step = 0.1,
  ) => (
    <Numeric
      key={field}
      label={label}
      value={hil[key]}
      min={min}
      max={max}
      unit={unit}
      step={step}
      disabled={disabled}
      onApply={(v) => command(`SET ${field} ${v}`)}
    />
  );
  return (
    <Panel title="에뮬레이터 응답 설정" eyebrow="STM_TEST → CAN FEEDBACK">
      <Choice
        label="응답 방식"
        options={[
          { value: "closed", label: "폐루프 시뮬레이션" },
          { value: "direct", label: "직접 피드백" },
        ]}
        value={
          typeof hil.plant === "boolean"
            ? hil.plant
              ? "closed"
              : "direct"
            : null
        }
        disabled={disabled || hil.protocol < 2}
        onChange={(v) =>
          v === "direct"
            ? commands(["SET PLANT OFF", `SET RPM ${hil.rpm ?? 0}`])
            : command("SET PLANT ON")
        }
      />
      <p className="muted">
        폐루프에서는 STM_A 명령에 따라 모터 응답이 계산됩니다.
      </p>
      <fieldset disabled={disabled || hil.plant !== false}>
        <Numeric
          label="속도 피드백"
          value={hil.speed}
          min={-200}
          max={200}
          step={0.1}
          unit="km/h"
          disabled={disabled || hil.plant !== false}
          onApply={(v) => command(`SET SPEED ${v}`)}
        />
        <Numeric
          label="RPM 피드백"
          value={hil.rpm}
          min={-32000}
          max={32000}
          step={1}
          unit="RPM"
          disabled={disabled || hil.plant !== false}
          onApply={(v) => command(`SET RPM ${v}`)}
        />
      </fieldset>
      <Details title="전압 · 전류 · 온도 주입">
        {numeric("VOLTAGE", "DC 전압", "voltage", 0, 300, "V")}
        {numeric("CURRENT", "모터 전류 피드백", "current", -3200, 3200, "A")}
        {numeric("MOTOR_TEMP", "모터 온도", "motor_temp", -40, 210, "°C")}
      </Details>
    </Panel>
  );
}

export function ShaftFeedback() {
  const { hil, disabled, state, online, points, command } = useWorkbench();
  const source = (topic: string) => (
    <Source name={topic} sample={state.topics[topic]} online={online} />
  );
  const numeric = (
    field: string,
    label: string,
    key: string,
    min: number,
    max: number,
    unit: string,
    step = 0.1,
  ) => (
    <Numeric
      key={field}
      label={label}
      value={hil[key]}
      min={min}
      max={max}
      unit={unit}
      step={step}
      disabled={disabled}
      onApply={(v) => command(`SET ${field} ${v}`)}
    />
  );
  return (
    <Panel title="KEYA 모터축 피드백" eyebrow="STM_TEST / MOTOR SHAFT">
      {numeric(
        "STEER_POS",
        "모터축 초기 위치",
        "steer",
        -1800,
        1800,
        "deg",
        0.01,
      )}
      <Notice>
        STM_TEST가 모사한 KEYA 모터축 회전각입니다. 로드휠 조향각과는 다른
        물리량이며 두 각도는 보정식으로 연결됩니다. Enable 상태에서는 수신한
        위치 명령을 다시 추종합니다.
      </Notice>
      <Fact label="KEYA Enable 명령" value={boolText(hil.keya_enabled)} />
      <Fact
        label="모터축 관측 / 수신 목표"
        value={`${fmt(hil.steer)} / ${fmt(hil.steer_target)}`}
        unit="deg"
      />
      <Chart
        points={points}
        traces={[
          { key: "targetSteer", label: "수신 모터축 목표", color: targetColor },
          { key: "steer", label: "에뮬레이터 응답", color: feedbackColor },
        ]}
        unit="motor deg"
      />
      {source("/hil/status")}
    </Panel>
  );
}
