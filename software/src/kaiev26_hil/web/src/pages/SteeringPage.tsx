import { ShaftFeedback } from "../components/EmulatorControls";
import { Chart, Fact, fmt, Panel, Source } from "../components/instruments";
import { ProductionCommand } from "../components/ProductionCommand";
import {
  SteeringSummary,
  SteeringTelemetry,
} from "../components/VehicleTelemetry";
import { age, blue, boolean, cyan } from "../domain/format";
import { useWorkbench } from "../hooks/useWorkbench";
export function SteeringPage() {
  const { state, online, hil, points, linkBadge } = useWorkbench();
  return (
    <>
      <ProductionCommand />
      <SteeringSummary state={state} online={online} />
      <Panel
        title="로드휠 위치 추종"
        eyebrow="LATERAL CONTROLLER"
        aside={linkBadge("keya")}
      >
        <Chart
          points={points}
          traces={[
            { key: "targetWheel", label: "Commanded", color: blue },
            { key: "wheel", label: "Actual feedback", color: cyan },
          ]}
          unit="deg"
        />
        <Source
          name="/steering/status"
          sample={state.topics["/steering/status"]}
          online={online}
        />
      </Panel>
      <div className="grid-two details-grid">
        <SteeringTelemetry state={state} online={online} />
        <div className="panel-stack">
          <ShaftFeedback />
          <Panel title="KEYA command reception" eyebrow="STM_TEST / CAN 2">
            <Fact label="Enable 명령" value={boolean(hil.keya_enabled)} />
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
      </div>
    </>
  );
}
