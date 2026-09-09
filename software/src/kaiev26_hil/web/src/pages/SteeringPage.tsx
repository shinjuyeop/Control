import { ShaftFeedback } from "../components/EmulatorControls";
import { Chart, Fact, fmt, Panel, Source } from "../components/instruments";
import {
  SteeringSummary,
  SteeringTelemetry,
} from "../components/VehicleTelemetry";
import { age, targetColor, boolean, feedbackColor } from "../domain/format";
import { useWorkbench } from "../hooks/useWorkbench";
export function SteeringPage() {
  const { state, online, hil, points, linkBadge } = useWorkbench();
  return (
    <>
      <SteeringSummary state={state} online={online} />
      <Panel
        title="로드휠 위치 추종"
        eyebrow="LATERAL CONTROLLER"
        aside={linkBadge("keya")}
      >
        <p className="panel-intro">
          타이어 방향을 나타내는 로드휠 등가 조향각입니다. 관측각은 KEYA
          엔코더를 보정식으로 환산한 값이며 별도 바퀴 각도 센서값이 아닙니다.
        </p>
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
