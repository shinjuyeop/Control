import type { Trace } from "../components/instruments";
import { ArrowRight } from "lucide-react";
import { EventList } from "../components/EventList";
import { Badge, Chart, Fact, Panel, fmt } from "../components/instruments";
import { VehicleSummary } from "../components/VehicleTelemetry";
import { HealthIndicator } from "../components/RuntimeInstruments";
import { runtimeBoards } from "../domain/runtime";
import { targetColor, feedbackColor } from "../domain/format";
import { useWorkbench } from "../hooks/useWorkbench";
import {
  SystemState,
  DriveFlow,
  SteeringFlow,
  BenchSafetyFlow,
} from "../components/SubsystemSignals";
const speedTraces: Trace[] = [
  {
    key: "targetSpeed",
    label: "Governor target",
    color: targetColor,
  },
  { key: "speed", label: "STM_A observed", color: feedbackColor },
];

const wheelTraces: Trace[] = [
  { key: "targetWheel", label: "Commanded", color: targetColor },
  { key: "wheel", label: "Feedback", color: feedbackColor },
];

export function BenchPage() {
  const { state, online, fresh, hil, points, navigate } = useWorkbench();
  return (
    <>
      <SystemState />
      <Panel
        title="차량 제어 경로"
        eyebrow="SYSTEM OVERVIEW"
        className="pipeline-panel"
        aside={
          <Badge tone={fresh ? "good" : "warning"}>
            {fresh ? "BENCH LIVE" : "BENCH STALE"}
          </Badge>
        }
      >
        <div className="pipeline-input-note">
          <span className="eyebrow">OPERATOR INPUT</span>
          <strong>
            {hil.mode ?? "—"} / {hil.gear ?? "—"}
          </strong>
          <span>Pedal {fmt(hil.pedal, 0)}% · STM_TEST → STM_A ADC / GPIO</span>
        </div>
        <DriveFlow />
        <SteeringFlow />
        <BenchSafetyFlow />
        <div className="runtime-links" aria-label="ECU runtime diagnostic">
          {runtimeBoards.map((name) => (
            <button key={name} onClick={() => navigate("runtime")}>
              <span>{name} RUNTIME</span>
              <HealthIndicator board={state.runtime?.[name]} online={online} />
              <ArrowRight size={13} />
            </button>
          ))}
        </div>
      </Panel>
      <div className="secondary-metrics">
        <VehicleSummary state={state} online={online} />
      </div>
      <div className="grid-two">
        <Panel
          title="구동 응답"
          aside={
            <button className="text-button" onClick={() => navigate("drive")}>
              Drive <ArrowRight size={13} />
            </button>
          }
        >
          <Chart
            points={points}
            traces={speedTraces}
            unit="km/h"
            height={190}
          />
        </Panel>
        <Panel
          title="조향 응답"
          aside={
            <button
              className="text-button"
              onClick={() => navigate("steering")}
            >
              Steering <ArrowRight size={13} />
            </button>
          }
        >
          <Chart
            points={points}
            traces={wheelTraces}
            unit="deg"
            height={190}
          />
        </Panel>
      </div>
      <Panel
        title="최근 시험과 이벤트"
        aside={
          <button
            className="text-button"
            onClick={() => navigate("diagnostics")}
          >
            로그 <ArrowRight size={13} />
          </button>
        }
      >
        <Fact
          label="최근 시나리오"
          value={
            state.scenario
              ? `${state.scenario.scenario} · ${state.scenario.result}`
              : "실행 기록 없음"
          }
        />
        <EventList events={state.events.slice(0, 5)} />
      </Panel>
    </>
  );
}
