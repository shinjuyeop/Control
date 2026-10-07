import type { Trace } from "../components/instruments";
import { DriveComparison, CurrentPlot } from "../components/VehicleObservations";
import { DriveFlow } from "../components/SubsystemSignals";
import { StateRail } from "../components/SignalInstruments";
import { reception, reported } from "../domain/presentation";
import { TractionFeedback } from "../components/EmulatorControls";
import { Chart, Fact, fmt, Panel, Source } from "../components/instruments";
import {
  DriveTelemetry,
} from "../components/VehicleTelemetry";
import {
  age,
  auxiliaryColor,
  targetColor,
  boolean,
  feedbackColor,
} from "../domain/format";
import { useWorkbench } from "../hooks/useWorkbench";
const driveTraces: Trace[] = [
  { key: "targetSpeed", label: "속도 목표", color: targetColor },
  { key: "speed", label: "STM_A 속도", color: feedbackColor },
  {
    key: "targetRpm",
    label: "Plant 목표",
    color: auxiliaryColor,
    axis: "right",
    dashed: true,
  },
  {
    key: "rpm",
    label: "모터 응답",
    color: "#206449",
    axis: "right",
    dashed: true,
  },
];

export function DrivePage() {
  const { state, online, hil, drive, points, linkBadge } =
    useWorkbench();
  return (
    <>
      <StateRail
        label="구동 주요 상태"
        items={[
          {
            label: "STM_A RUN COMMAND",
            value: boolean(drive.tractive_commanded, "RUN", "STOP"),
            status: reception(state, online, "/drive/status"),
            detail: drive.command_timeout
              ? "Host command timeout"
              : "보드가 보고한 명령 상태",
          },
          {
            label: "MOTOR RPM",
            value: drive.part1_valid ? fmt(drive.motor_rpm, 0) : "—",
            detail: "EZkontrol Part I",
          },
          {
            label: "CONTROL MODE",
            value: boolean(hil.ez_speed_control, "SPEED", "TORQUE"),
            status: reported(hil.ez_fault, "FAULT", "NO FAULT", "danger"),
            detail: "STM_TEST 수신 명령",
          },
        ]}
      />
      <DriveComparison />
      <DriveFlow />
      <Panel
        title="구동 명령과 응답"
        eyebrow="GOVERNOR → STM_A → STM_TEST"
        aside={linkBadge("ez")}
      >
        <Chart
          points={points}
          traces={driveTraces}
          unit="km/h"
          rightUnit="RPM"
          height={280}
        />
        <p className="muted">
          왼쪽: 속도 km/h · 오른쪽: 모터 RPM · 축별 자동 범위. RPM 점선은
          STM_TEST의 Plant 목표와 에뮬레이터 응답입니다.
        </p>
        <Source
          name="/drive/status"
          sample={state.topics["/drive/status"]}
          online={online}
        />
        <Source
          name="/hil/status"
          sample={state.topics["/hil/status"]}
          online={online}
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
      <CurrentPlot />
      <div className="grid-two details-grid">
        <DriveTelemetry state={state} online={online} />
        <TractionFeedback />
      </div>
    </>
  );
}
