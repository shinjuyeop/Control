import { ArrowRight, ChevronRight } from "lucide-react";
import type { ReactNode } from "react";
import { Badge, fmt } from "./instruments";
import type { Indicator } from "../domain/presentation";
import { STEERING_SCALE_DEG } from "../domain/presentation";

export type StateItem = {
  label: string;
  value: ReactNode;
  detail?: ReactNode;
  status?: Indicator;
  onSelect?: () => void;
};

export function StateRail({
  label,
  items,
}: {
  label: string;
  items: StateItem[];
}) {
  return (
    <div className="state-rail" role="group" aria-label={label}>
      {items.map((item) => {
        const statusIsValue =
          typeof item.value === "string" && item.value === item.status?.label;
        const contents = (
          <>
            <div className="state-tile-label">
              <span>{item.label}</span>
              {item.status && !statusIsValue && (
                <Badge tone={item.status.tone}>{item.status.label}</Badge>
              )}
            </div>
            <strong className="state-tile-value">
              {statusIsValue && (
                <i
                  className={`instrument-led ${item.status?.tone}`}
                  aria-hidden="true"
                />
              )}
              {item.value}
            </strong>
            {item.detail && <small>{item.detail}</small>}
            {item.onSelect && (
              <ChevronRight className="state-tile-link" size={14} />
            )}
          </>
        );
        return item.onSelect ? (
          <button
            key={item.label}
            onClick={item.onSelect}
            className={`state-tile ${item.status?.tone ?? ""}`}
          >
            {contents}
          </button>
        ) : (
          <div
            key={item.label}
            className={`state-tile ${item.status?.tone ?? ""}`}
          >
            {contents}
          </div>
        );
      })}
    </div>
  );
}

export function SignalFlow({
  label,
  stages,
  note,
}: {
  label: string;
  stages: StateItem[];
  note?: string;
}) {
  return (
    <div className="signal-flow" role="group" aria-label={label}>
      <div className="signal-flow-label">
        <span>{label}</span>
        {note && <small>{note}</small>}
      </div>
      <ol className="signal-flow-stages">
        {stages.map((stage, index) => (
          <li key={stage.label} className={stage.status?.tone ?? ""}>
            <div className="signal-stage-heading">
              <span className="signal-stage-index">
                {String(index + 1).padStart(2, "0")}
              </span>
              <strong>{stage.label}</strong>
              {stage.status && stage.status.label !== stage.value && (
                <Badge tone={stage.status.tone}>{stage.status.label}</Badge>
              )}
            </div>
            <span className="signal-stage-value">
              {stage.status && stage.status.label === stage.value && (
                <i
                  className={`instrument-led ${stage.status.tone}`}
                  aria-hidden="true"
                />
              )}
              {stage.value}
            </span>
            {stage.detail && <small>{stage.detail}</small>}
            {index < stages.length - 1 && (
              <ArrowRight
                className="signal-stage-arrow"
                size={16}
                aria-hidden="true"
              />
            )}
          </li>
        ))}
      </ol>
    </div>
  );
}

export function LinearMeter({
  label,
  value,
  unit = "%",
  low = 0,
  high = 100,
}: {
  label: string;
  value: number | undefined;
  unit?: string;
  low?: number;
  high?: number;
}) {
  const position =
    value == null
      ? null
      : Math.max(0, Math.min(100, ((value - low) / (high - low)) * 100));
  return (
    <div className="linear-meter" role="group" aria-label={label}>
      <div>
        <span>{label}</span>
        <strong>
          {fmt(value, 0)}
          <small>{unit}</small>
        </strong>
      </div>
      <div className="linear-meter-track" aria-hidden="true">
        {position != null && <span style={{ width: `${position}%` }} />}
      </div>
      <div className="instrument-scale">
        <span>{low}</span>
        <span>
          {high} {unit}
        </span>
      </div>
    </div>
  );
}

export function TargetActual({
  label,
  target,
  actual,
  unit,
  targetLabel = "목표",
  actualLabel = "관측",
  note,
}: {
  label: string;
  target?: number;
  actual?: number;
  unit: string;
  targetLabel?: string;
  actualLabel?: string;
  note: string;
}) {
  const error = target != null && actual != null ? target - actual : undefined;
  return (
    <div className="target-actual" role="group" aria-label={label}>
      <div className="instrument-title">
        <div>
          <span className="eyebrow">TARGET / FEEDBACK</span>
          <h2>{label}</h2>
        </div>
      </div>
      <div className="paired-readings">
        <div className="target-reading">
          <span>
            <i />
            {targetLabel}
          </span>
          <strong>
            {fmt(target, 2)}
            <small>{unit}</small>
          </strong>
        </div>
        <ArrowRight size={22} aria-hidden="true" />
        <div className="actual-reading">
          <span>
            <i />
            {actualLabel}
          </span>
          <strong>
            {fmt(actual, 2)}
            <small>{unit}</small>
          </strong>
        </div>
      </div>
      <div className="comparison-footer">
        <span>
          목표 − 관측{" "}
          <b>
            {fmt(error, 2)} {unit}
          </b>
        </span>
        <small>{note}</small>
      </div>
    </div>
  );
}

export function SteeringArc({
  target,
  actual,
  status,
}: {
  target?: number;
  actual?: number;
  status?: Indicator;
}) {
  const limit = STEERING_SCALE_DEG;
  const point = (value: number, radius: number) => {
    const angle =
      ((Math.max(-limit, Math.min(limit, value)) / limit) * 70 * Math.PI) / 180;
    return {
      x: 150 - Math.sin(angle) * radius,
      y: 126 - Math.cos(angle) * radius,
    };
  };
  const outOfScale = [target, actual].some(
    (value) => value != null && Math.abs(value) > limit,
  );
  const left = point(limit, 95),
    right = point(-limit, 95);
  return (
    <div
      className="steering-arc"
      role="group"
      aria-label="조향각 목표와 피드백 계기"
    >
      <div className="instrument-title">
        <div>
          <span className="eyebrow">ROAD WHEEL · EQUIVALENT ANGLE</span>
          <h2>조향각 비교</h2>
        </div>
        {status && <Badge tone={status.tone}>{status.label}</Badge>}
      </div>
      <svg
        viewBox="0 0 300 155"
        role="img"
        aria-label={`목표 ${fmt(target, 2)}도, 피드백 ${fmt(actual, 2)}도. 양수는 좌회전.`}
      >
        <path
          d={`M${left.x} ${left.y} A95 95 0 0 1 ${right.x} ${right.y}`}
          className="angle-track"
        />
        {[-30, -20, -10, 0, 10, 20, 30].map((value) => {
          const a = point(value, 95),
            b = point(value, value === 0 ? 109 : 102),
            text = point(value, 120);
          return (
            <g key={value}>
              <line
                x1={a.x}
                y1={a.y}
                x2={b.x}
                y2={b.y}
                className={value === 0 ? "angle-zero" : "angle-tick"}
              />
              <text x={text.x} y={text.y + 4} textAnchor="middle">
                {value > 0 ? `+${value}` : value}
              </text>
            </g>
          );
        })}
        {actual != null && (
          <g className="angle-actual">
            <line
              x1="150"
              y1="126"
              x2={point(actual, 86).x}
              y2={point(actual, 86).y}
            />
            <circle cx={point(actual, 86).x} cy={point(actual, 86).y} r="4" />
          </g>
        )}
        {target != null && (
          <rect
            className="angle-target"
            x={point(target, 107).x - 4}
            y={point(target, 107).y - 4}
            width="8"
            height="8"
          />
        )}
        <circle cx="150" cy="126" r="5" className="angle-pivot" />
        <text x="18" y="146">
          LEFT +
        </text>
        <text x="282" y="146" textAnchor="end">
          − RIGHT
        </text>
      </svg>
      <div className="angle-readings">
        <span className="target-reading">
          □ 목표 <b>{fmt(target, 2)}°</b>
        </span>
        <span className="actual-reading">
          ● 피드백 <b>{fmt(actual, 2)}°</b>
        </span>
      </div>
      <small>
        {outOfScale
          ? "표시 눈금 밖 · 정확한 각도는 숫자를 확인하세요"
          : "엔코더 환산각 · 표시 눈금 ±30°"}
      </small>
    </div>
  );
}
