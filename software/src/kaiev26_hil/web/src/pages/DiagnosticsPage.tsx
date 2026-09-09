import { Download } from "lucide-react";
import { EventList } from "../components/EventList";
import { Details, Fact, fmt, Panel } from "../components/instruments";
import { ReceptionTable } from "../components/VehicleTelemetry";
import { download } from "../domain/download";
import { age, boolean } from "../domain/format";
import type { Data } from "../domain/types";
import { useWorkbench } from "../hooks/useWorkbench";
export function DiagnosticsPage() {
  const { state, hil, online, observed, logFilter, setLogFilter, linkBadge } =
    useWorkbench();
  return (
    <>
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
      <Panel title="ROS 수신 상태" eyebrow="TELEMETRY SOURCES">
        <ReceptionTable state={state} online={online} />
        <Details title="Bridge와 제어기 diagnostics">
          {observed("/diagnostics").status?.map((item: Data, i: number) => (
            <div className="diagnostic-item" key={i}>
              <h3>{item.name}</h3>
              <p>{item.message}</p>
              {item.values?.map((v: Data) => (
                <Fact key={v.key} label={v.key} value={v.value} />
              ))}
            </div>
          )) ?? <p className="muted">diagnostics 수신 대기</p>}
        </Details>
      </Panel>
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
