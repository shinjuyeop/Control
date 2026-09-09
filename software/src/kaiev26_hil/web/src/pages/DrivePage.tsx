import { TractionFeedback } from "../components/EmulatorControls";
import {
  Chart,
  Fact,
  fmt,
  Metric,
  Panel,
  Source,
} from "../components/instruments";
import { ProductionCommand } from "../components/ProductionCommand";
import { DriveTelemetry, FrameHealth } from "../components/VehicleTelemetry";
import { age, amber, blue, boolean, cyan } from "../domain/format";
import { mpsToKmh } from "../domain/units";
import { useWorkbench } from "../hooks/useWorkbench";
export function DrivePage() {
  const { state, online, hil, drive, observed, points, linkBadge } =
    useWorkbench();
  const target = observed("/vehicle/command").speed_target_mps;
  const actual = drive.part1_valid ? drive.speed_mps : undefined;
  const error =
    typeof target === "number" && typeof actual === "number"
      ? target - actual
      : undefined;
  return (
    <>
      <ProductionCommand />
      <div className="metrics-grid drive-metrics">
        <Metric
          label="주행 목표"
          value={fmt(mpsToKmh(target), 2)}
          unit="km/h"
          note="Governor output"
        />
        <Metric
          label="관측 속도"
          value={fmt(mpsToKmh(actual), 2)}
          unit="km/h"
          note="STM_A feedback"
        />
        <Metric
          label="속도 오차"
          value={fmt(error === undefined ? undefined : mpsToKmh(error), 2)}
          unit="km/h"
          note="목표 − 관측"
        />
      </div>
      <FrameHealth state={state} online={online} />
      <Panel
        title="속도 명령과 피드백"
        eyebrow="GOVERNOR → STM_A"
        aside={linkBadge("ez")}
      >
        <Chart
          points={points}
          traces={[
            { key: "targetSpeed", label: "Governor target", color: blue },
            { key: "speed", label: "STM_A feedback", color: cyan },
          ]}
          unit="km/h"
        />
        <Source
          name="/drive/status"
          sample={state.topics["/drive/status"]}
          online={online}
        />
      </Panel>
      <div className="grid-two">
        <Panel title="EZ 명령 · 모터 응답" eyebrow="STM_A → STM_TEST">
          <Chart
            points={points}
            traces={[
              { key: "targetRpm", label: "Plant target", color: blue },
              { key: "rpm", label: "Emulator output", color: cyan },
            ]}
            unit="RPM"
            height={190}
          />
          <div className="inline-facts">
            <Fact label="RUN" value={boolean(hil.ez_running)} />
            <Fact
              label="제어 방식"
              value={boolean(hil.ez_speed_control, "SPEED", "TORQUE")}
            />
            <Fact label="수신 명령 RPM" value={fmt(hil.ez_command_rpm, 0)} />
            <Fact
              label="수신 상전류 명령"
              value={fmt(hil.ez_command_current)}
              unit="A"
            />
            <Fact
              label="RX / TX"
              value={`${fmt(hil.ez_rx, 0)} / ${fmt(hil.ez_tx, 0)}`}
            />
            <Fact label="RX age" value={age(hil.ez_rx_age_ms)} />
          </div>
        </Panel>
        <Panel title="전류 응답" eyebrow="STM_A / PART I">
          <Chart
            points={points}
            traces={[
              { key: "bus", label: "Bus current", color: amber },
              { key: "phase", label: "Phase current", color: cyan },
            ]}
            unit="A"
            height={190}
          />
          <p className="muted">
            배터리 버스 전류와 모터 상전류는 서로 다른 값입니다.
          </p>
        </Panel>
      </div>
      <div className="grid-two details-grid">
        <DriveTelemetry state={state} online={online} />
        <TractionFeedback />
      </div>
    </>
  );
}
