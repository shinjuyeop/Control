import { Badge, fmt } from "./instruments";
import { runtimeRowState } from "../domain/presentation";
import {
  bytes,
  capacityRatio,
  delta,
  duration,
  jitterRange,
  runtimeNotes,
  runtimeView,
  type Capacity,
  type RuntimeBoard,
  type RuntimeTask,
} from "../domain/runtime";

export function HealthIndicator({
  board,
  online,
}: {
  board?: RuntimeBoard;
  online: boolean;
}) {
  const view = runtimeView(board, online);
  return (
    <span title={`ECU runtime diagnostic · ${view.detail}`}>
      <Badge tone={view.tone}>{view.label}</Badge>
    </span>
  );
}

export function UtilizationGauge({
  label,
  capacity,
  unit = "B",
  note,
  detail,
  invalid = false,
}: {
  label: string;
  capacity?: Capacity | null;
  unit?: "B" | "KiB";
  note: string;
  detail: string;
  invalid?: boolean;
}) {
  const value = capacity?.percent;
  return (
    <div
      className={`utilization-gauge ${capacity?.tone ?? "muted"}`}
      role="group"
      aria-label={label}
      title={`${detail} ${runtimeNotes.capacity}`}
    >
      <span className="eyebrow">{label}</span>
      <div className="gauge-face">
        <svg viewBox="0 0 200 116" aria-hidden="true">
          <path
            className="gauge-track"
            d="M22 102 A78 78 0 0 1 178 102"
            pathLength="100"
          />
          {value != null && (
            <path
              className="gauge-fill"
              d="M22 102 A78 78 0 0 1 178 102"
              pathLength="100"
              strokeDasharray={`${value} 100`}
            />
          )}
          {Array.from({ length: 11 }, (_, i) => {
            const angle = Math.PI + (Math.PI * i) / 10;
            return (
              <line
                key={i}
                className="gauge-tick"
                x1={100 + 87 * Math.cos(angle)}
                y1={102 + 87 * Math.sin(angle)}
                x2={100 + (i % 5 === 0 ? 95 : 91) * Math.cos(angle)}
                y2={102 + (i % 5 === 0 ? 95 : 91) * Math.sin(angle)}
              />
            );
          })}
        </svg>
        <div className="gauge-value">
          {invalid || capacity?.valid === false ? (
            <span className="gauge-invalid">INVALID</span>
          ) : (
            <>
              {fmt(value, 0)}
              <small>%</small>
            </>
          )}
        </div>
        <span className="gauge-zero">0</span>
        <span className="gauge-full">100</span>
      </div>
      <strong className="gauge-ratio">{capacityRatio(capacity, unit)}</strong>
      <span className="gauge-note">{note}</span>
    </div>
  );
}

export function UtilizationBar({ capacity }: { capacity: Capacity }) {
  return (
    <div
      className={`utilization-bar ${capacity.tone}`}
      title={`${runtimeNotes.stack} ${runtimeNotes.capacity}`}
    >
      <div className="stack-track" aria-hidden="true">
        {capacity.valid && <span style={{ width: `${capacity.percent}%` }} />}
      </div>
      <strong>{capacityRatio(capacity)}</strong>
      <small>
        {capacity.valid
          ? `${fmt(capacity.percent, 0)}% · ${bytes(capacity.free_bytes)} margin`
          : `할당 ${bytes(capacity.allocated_bytes)}`}
      </small>
    </div>
  );
}

export function RuntimeTaskRow({
  task,
  sinceReset = false,
}: {
  task: RuntimeTask;
  sinceReset?: boolean;
}) {
  const issue = runtimeRowState(task);
  return (
    <div className={`runtime-task-row ${issue?.tone ?? ""}`} role="row">
      <strong role="rowheader" aria-label={task.name}>
        {task.name}
        {issue && <span className="runtime-row-issue">{issue.label}</span>}
      </strong>
      <div role="cell" data-label="RATE" title={runtimeNotes.rate}>
        <strong>
          {fmt(task.rate_hz, 1)} <small>Hz</small>
        </strong>
        {task.rate_hz == null && <small>기준점 대기</small>}
      </div>
      <div role="cell" data-label="PEAK STACK">
        <UtilizationBar capacity={task.stack} />
      </div>
      <div role="cell" data-label="WALL MAX / AVG" title={runtimeNotes.wall}>
        <strong>
          {duration(task.wall?.max_us)} <small>max</small>
        </strong>
        <small>{duration(task.wall?.average_us)} avg</small>
      </div>
      <div role="cell" data-label="JITTER" title={runtimeNotes.jitter}>
        <strong>{jitterRange(task)}</strong>
        <small>min / max</small>
      </div>
      <div role="cell" data-label="MISSES">
        <strong>
          {task.jitter
            ? (task.deadline_misses_since_reset ?? task.deadline_misses)
            : "미계측"}
        </strong>
        {task.jitter && (
          <small>
            {delta(
              task.deadline_misses_since_reset_delta ??
                task.deadline_misses_delta,
            )}
          </small>
        )}
        {task.jitter && sinceReset && (
          <small>부팅 누적 {task.deadline_misses}</small>
        )}
      </div>
    </div>
  );
}
