import { SteeringComparison, WheelPlot } from "../components/VehicleObservations";
import { SteeringFlow } from "../components/SubsystemSignals";
import { ShaftFeedback } from "../components/EmulatorControls";
import { Fact, fmt, Panel } from "../components/instruments";
import {
  SteeringTelemetry,
} from "../components/VehicleTelemetry";
import { age, boolean } from "../domain/format";
import { useWorkbench } from "../hooks/useWorkbench";
export function SteeringPage() {
  const { state, online, hil, linkBadge } =
    useWorkbench();
  return (
    <>
      <SteeringComparison motorTargetDeg={hil.steer_target} />
      <SteeringFlow />
      <div className="grid-two steering-charts">
        <WheelPlot aside={linkBadge("keya")} />
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
