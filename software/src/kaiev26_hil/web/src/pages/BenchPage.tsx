import { ArrowRight } from "lucide-react";
import { EventList } from "../components/EventList";
import { Badge, Chart, Fact, Panel, fmt } from "../components/instruments";
import { VehicleSummary } from "../components/VehicleTelemetry";
import { targetColor, boolean, feedbackColor } from "../domain/format";
import { useWorkbench } from "../hooks/useWorkbench";
export function BenchPage() {
  const {
    state,
    online,
    fresh,
    hil,
    drive,
    brake,
    points,
    linkBadge,
    navigate,
  } = useWorkbench();
  const connections = [
    ["STM_TEST", fresh, "/hil/status"],
    ["STM_A", !!drive.header, "/drive/status"],
    ["STM_B", !!brake.header, "/brake/status"],
    ...(state.display?.enabled
      ? [["UNO R4", online && state.display.fresh, "USB display report"]]
      : []),
  ];
  return (
    <>
      <div className="connection-strip" aria-label="보드 연결 상태">
        {connections.map(([label, ok, topic]) => (
          <div key={String(label)}>
            <strong>{label}</strong>
            <Badge tone={ok ? "good" : "muted"}>
              {ok ? "RECEIVING" : "NO DATA"}
            </Badge>
            <code>{topic}</code>
          </div>
        ))}
      </div>
      <VehicleSummary state={state} online={online} />
      <Panel title="입력 → 명령 → 피드백" eyebrow="BENCH SIGNAL PATH">
        <div className="signal-path">
          <div>
            <span>01 · STM_TEST 입력</span>
            <strong>
              {hil.mode ?? "—"} / {hil.gear ?? "—"}
            </strong>
            <p>
              Pedal {fmt(hil.pedal, 0)}% · E-stop{" "}
              {typeof hil.estop_dash === "boolean"
                ? boolean(hil.estop_dash || hil.estop_side || hil.estop_rf)
                : "—"}
            </p>
          </div>
          <ArrowRight size={17} />
          <div>
            <span>02 · 보드 출력 관측</span>
            <strong>EZ {boolean(hil.ez_running, "RUN", "STOP")}</strong>
            <p>
              {linkBadge("ez")} {linkBadge("keya")}
            </p>
          </div>
          <ArrowRight size={17} />
          <div>
            <span>03 · 에뮬레이터 피드백</span>
            <strong>{fmt(hil.rpm, 0)} RPM</strong>
            <p>
              PWM {hil.brake ?? "—"} · KEYA {fmt(hil.steer, 1)}°
            </p>
          </div>
        </div>
      </Panel>
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
            traces={[
              {
                key: "targetSpeed",
                label: "Governor target",
                color: targetColor,
              },
              { key: "speed", label: "STM_A observed", color: feedbackColor },
            ]}
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
            traces={[
              { key: "targetWheel", label: "Commanded", color: targetColor },
              { key: "wheel", label: "Feedback", color: feedbackColor },
            ]}
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
