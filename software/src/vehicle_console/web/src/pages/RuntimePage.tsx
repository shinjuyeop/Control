import { Activity } from "lucide-react";
import { Details, Fact, Panel, Source, fmt } from "../components/instruments";
import { HealthIndicator, RuntimeTaskRow, UtilizationGauge } from "../components/RuntimeInstruments";
import { bytes, delta, freshnessSeconds, kib, runtimeBoards, runtimeNotes, runtimeView, uptime, type RuntimeBoard } from "../domain/runtime";
import { useTelemetry } from "../hooks/useTelemetry";
import { capabilities } from "../domain/profile";

export function BoardSummary({ name, board, online }: { name: string; board?: RuntimeBoard; online: boolean }) {
  const view = runtimeView(board, online), data = view.data;
  const worst = data?.worst_stack;
  return (
    <Panel title={name} eyebrow={board?.role ?? "ECU RUNTIME"} className="runtime-summary" aside={<HealthIndicator board={board} online={online} />}>
      <p className={`runtime-health-line ${view.tone}`}><Activity size={14} />{view.detail}</p>
      <div className="runtime-meters">
        <UtilizationGauge label="FREERTOS HEAP" capacity={data?.heap} unit="KiB" note="heap_4 used / reserved" detail={runtimeNotes.heap} />
        <UtilizationGauge label="WORST STACK" capacity={worst?.stack} note={worst ? `${worst.name} · ${bytes(worst.stack.free_bytes)} margin` : "Present task · 최대 관측 사용량"} detail={runtimeNotes.stack} invalid={view.stackInvalid} />
      </div>
      <div className="runtime-memory-facts" title={runtimeNotes.heap}>
        <span>Free <b>{kib(data?.heap.free_bytes)}</b></span>
        <span>Min free <b>{kib(data?.heap.minimum_free_bytes)}</b></span>
        <small>Heap_4 only · Stack: observed max / allocated</small>
      </div>
      <div className="runtime-facts">
        <Fact label={`Deadline misses · ${data?.deadline_reset_uptime_ms != null ? "리셋 이후" : "부팅 이후"}`} value={<>{data?.deadline_misses_since_reset ?? data?.deadline_misses ?? "—"}<small>{delta(data?.deadline_misses_since_reset_delta ?? data?.deadline_misses_delta)}</small>{data?.deadline_reset_uptime_ms != null && <small>부팅 누적 {data.deadline_misses}</small>}</>} />
        <Fact label="Instrumentation errors" value={<>{data?.instrumentation_errors ?? "—"}<small>{delta(data?.instrumentation_errors_delta)}</small></>} />
        <Fact label="Timing source" value={view.timing} />
        <Fact label="Uptime" value={uptime(data?.uptime_ms)} />
        <Fact label="Snapshot age" value={view.age} />
        <Fact label="CPU load" value={view.cpu} />
      </div>
      {name === "STM_B" && <div className="runtime-latency" title={runtimeNotes.estop}>
        <div><span className="eyebrow">ESTOP DISPATCH MAX</span><p>EXTI callback → EStop task</p></div>
        <strong>{fmt(data?.estop.max_us, 0)}<small> µs</small></strong>
        <span>{data ? `${data.estop.samples} samples` : "— samples"}{data && data.estop.max_us == null && <small>미계측</small>}</span>
      </div>}
      {view.issues.length > 0 && <div className="inline-error">{view.issues.map((issue, i) => <p key={i}>{issue}</p>)}</div>}
    </Panel>
  );
}

export function RuntimePage() {
  const { state, online } = useTelemetry();
  return (
    <div className="runtime-page">
      <p className="runtime-purpose">ECU runtime diagnostic · 실행 계측과 메모리 여유를 관찰합니다. 차량 safety state와 별개입니다.</p>
      <div className="runtime-boards">
        {runtimeBoards.map(name => <BoardSummary key={name} name={name} board={state.runtime?.[name]} online={online} />)}
      </div>
      {runtimeBoards.map(name => {
        const board = state.runtime?.[name], view = runtimeView(board, online);
        const tasks = view.data?.tasks.filter(task => task.present) ?? [];
        return <Panel key={name} title={`${name} · Task runtime`} eyebrow="TASK INSTRUMENTATION" className="runtime-task-panel" aside={<HealthIndicator board={board} online={online} />}>
          <p className="panel-intro">Rate: snapshot 간 Δ · Stack / wall / jitter: boot 이후 관측 · Misses: {view.data?.deadline_reset_uptime_ms != null ? "Bench Reset 이후" : "boot 이후"}</p>
          {tasks.length > 0 ? <div className="runtime-task-table" role="table" aria-label={`${name} task runtime`}>
            <div className="runtime-task-row runtime-task-header" role="row">
              {["TASK", "RATE", "PEAK STACK", "WALL MAX / AVG", "JITTER", "MISSES"].map(label => <span key={label} role="columnheader">{label}</span>)}
            </div>
            {tasks.map(task => <RuntimeTaskRow key={task.task_id} task={task} sinceReset={view.data?.deadline_reset_uptime_ms != null} />)}
          </div> : <p className="runtime-empty">{view.detail} · 유효한 present task가 없습니다.</p>}
          {board && <Source name={board.topic} sample={state.topics[board.topic]} online={online} />}
        </Panel>;
      })}
      <Panel title="측정값 해석" eyebrow="MEASUREMENT NOTES" className="runtime-notes">
        <p>Peak stack은 현재 사용량이 아닌 최대 관측 사용량입니다. Wall max는 formal WCET가 아니며 jitter 해상도는 1 ms입니다.</p>
        <Details title="측정 범위 · freshness · 용량 표시">
          {Object.values(runtimeNotes).map(note => <p key={note}>{note}</p>)}
          <p>약 1 Hz RuntimeStats 수신 또는 snapshot 진행이 {freshnessSeconds(state.runtime?.STM_A)}초 이상 없으면 STALE입니다. 지난 값으로 live 게이지나 rate를 유지하지 않습니다.</p>
          <p>Deadline misses는 jitter를 계측하는 task의 누적값입니다. +N 최근은 직전 유효 snapshot 이후 증가량이며, 기준점에서는 증가 여부를 판정하지 않습니다.</p>
          {capabilities(state).control && <p>Bench Reset을 누르면 마지막 유효 runtime snapshot을 기준으로 miss 표시와 경고를 새로 시작합니다. 부팅 누적값은 남으며, heap·stack·wall·jitter·계측 오류는 초기화하지 않습니다. 이 기준은 Web backend 재시작 시 해제됩니다.</p>}
        </Details>
      </Panel>
    </div>
  );
}
