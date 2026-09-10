import { SteeringArc } from "../components/SignalInstruments";
import { SteeringFlow } from "../components/SubsystemSignals";
import { radToDeg } from "../domain/units";
import { ShaftFeedback } from "../components/EmulatorControls";
import { Chart, Fact, fmt, Panel, Source } from "../components/instruments";
import {
  SteeringSummary,
  SteeringTelemetry,
} from "../components/VehicleTelemetry";
import { age, targetColor, boolean, feedbackColor } from "../domain/format";
import { useWorkbench } from "../hooks/useWorkbench";
import { steeringState } from "../domain/presentation";
export function SteeringPage() {
  const { state, online, hil, steering, points, linkBadge, observed } =
    useWorkbench();
  const target =
    steering.target_wheel_angle_rad ??
    observed("/vehicle/command").steering_target_rad;
  return (
    <>
      <div className="instrument-pair">
        <SteeringArc
          status={steeringState(steering)}
          target={typeof target === "number" ? radToDeg(target) : undefined}
          actual={
            steering.feedback_valid
              ? radToDeg(steering.actual_wheel_angle_rad)
              : undefined
          }
          motorAngleDeg={
            steering.feedback_valid ? steering.motor_angle_deg : undefined
          }
          motorTargetDeg={hil.steer_target}
        />
        <SteeringSummary state={state} online={online} />
      </div>
      <SteeringFlow />
      <div className="grid-two steering-charts">
        <Panel
          title="로드휠 위치 추종"
          eyebrow="LATERAL CONTROLLER"
          aside={linkBadge("keya")}
        >
          <Chart
            points={points}
            traces={[
              { key: "targetWheel", label: "로드휠 명령", color: targetColor },
              { key: "wheel", label: "엔코더 환산 관측", color: feedbackColor },
            ]}
            unit="deg"
          />
          <Source
            name="/steering/status"
            sample={state.topics["/steering/status"]}
            online={online}
          />
        </Panel>
        <ShaftFeedback />
      </div>
      <div className="grid-two details-grid">
        <SteeringTelemetry state={state} online={online} />
        <Panel title="KEYA command reception" eyebrow="STM_TEST / CAN 2">
          <Fact label="마지막 RX" value={age(hil.keya_rx_age_ms)} />
          <Fact
            label="RX / TX"
            value={`${fmt(hil.keya_rx, 0)} / ${fmt(hil.keya_tx, 0)}`}
          />
          <Fact
            label="Peripheral 초기화"
            value={boolean(hil.keya_ok, "OK", "FAILED")}
          />
        </Panel>
      </div>
    </>
  );
}
