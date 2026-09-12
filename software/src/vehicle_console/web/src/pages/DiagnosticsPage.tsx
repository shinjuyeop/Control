import { DiagnosticObservations } from "../components/VehicleObservations";
import { Download } from "lucide-react";
import { EventList } from "../components/EventList";
import { Fact, fmt, Panel } from "../components/instruments";
import { download } from "../domain/download";
import { age, boolean } from "../domain/format";
import { useWorkbench } from "../hooks/useWorkbench";
import { StateRail } from "../components/SignalInstruments";
import { canLink } from "../domain/format";
import { reception } from "../domain/presentation";
export function DiagnosticsPage() {
  const {
    state,
    hil,
    online,
    fresh,
    logFilter,
    setLogFilter,
    linkBadge,
  } = useWorkbench();
  const samples = Object.values(state.topics);
  const receiving = online
    ? samples.filter((sample) => sample.fresh).length
    : 0;
  const errors = state.events.filter((event) => event.kind === "error").length;
  return (
    <>
      <StateRail
        label="진단 요약"
        items={[
          {
            label: "STM_TEST LINK",
            value: reception(state, online, "/hil/status").label,
            status: reception(state, online, "/hil/status"),
            detail: "Serial bridge → ROS",
          },
          {
            label: "CAN / EZ · KEYA",
            value: `${canLink(hil, fresh, "ez").text} / ${canLink(hil, fresh, "keya").text}`,
            detail: "각 CAN 경로의 현재 수신 상태",
          },
          {
            label: "ROS RECEPTION",
            value: `${receiving} / ${samples.length}`,
            detail: "수신 중 / 관측된 토픽",
          },
          {
            label: "ERROR HISTORY",
            value: errors,
            detail: "보관 중인 명령·연결 로그의 오류 수",
          },
        ]}
      />
      <div className="grid-two">
        {(["ez", "keya"] as const).map((bus) => (
          <Panel
            key={bus}
            title={`${bus.toUpperCase()} CAN 진단`}
            eyebrow={bus === "ez" ? "TRACTION" : "STEERING"}
            aside={linkBadge(bus)}
          >
            <Fact
              label="Peripheral 초기화"
              value={boolean(hil[`${bus}_ok`], "OK", "실패")}
            />
            <Fact
              label="RX / TX"
              value={`${fmt(hil[`${bus}_rx`], 0)} / ${fmt(hil[`${bus}_tx`], 0)}`}
            />
            <Fact label="RX age" value={age(hil[`${bus}_rx_age_ms`])} />
            <Fact
              label="TEC / REC"
              value={
                hil.protocol >= 3
                  ? `${hil[`${bus}_tec`]} / ${hil[`${bus}_rec`]}`
                  : "미지원"
              }
            />
            <Fact
              label="Bus-off / Error passive"
              value={
                hil.protocol >= 3
                  ? `${boolean(hil[`${bus}_busoff`])} / ${boolean(hil[`${bus}_errpass`])}`
                  : "미지원"
              }
            />
            <Fact
              label="Error warning"
              value={hil.protocol >= 3 ? boolean(hil[`${bus}_warn`]) : "미지원"}
            />
          </Panel>
        ))}
      </div>
      <DiagnosticObservations />
      <Panel
        title="명령 · 응답 로그"
        eyebrow="EVENT STREAM"
        aside={
          <div className="actions">
            <select
              aria-label="로그 필터"
              value={logFilter}
              onChange={(e) => setLogFilter(e.target.value)}
            >
              <option value="all">전체</option>
              <option value="command">명령</option>
              <option value="response">응답</option>
              <option value="error">오류</option>
            </select>
            <button
              className="icon-button"
              aria-label="로그 다운로드"
              onClick={() =>
                download({ events: state.events }, "hil-events.json")
              }
            >
              <Download size={16} />
            </button>
          </div>
        }
      >
        <EventList
          events={state.events.filter(
            (e) => logFilter === "all" || e.kind === logFilter,
          )}
        />
      </Panel>
    </>
  );
}
