import { useEffect, useId, useState, type ReactNode } from "react";
import { Activity, ChevronDown, Send, CircleHelp } from "lucide-react";
import type { Point, Sample } from "./connection";
export const fmt = (value: unknown, decimals = 1) =>
  typeof value === "number" && Number.isFinite(value)
    ? value.toFixed(decimals)
    : "—";
export function Badge({
  children,
  tone = "muted",
}: {
  children: ReactNode;
  tone?: string;
}) {
  return (
    <span className={`badge ${tone}`}>
      <i />
      {children}
    </span>
  );
}
export function Panel({
  title,
  eyebrow,
  children,
  aside,
  className = "",
}: {
  title: string;
  eyebrow?: string;
  children: ReactNode;
  aside?: ReactNode;
  className?: string;
}) {
  return (
    <section className={`panel ${className}`}>
      <div className="panel-heading">
        <div>
          {eyebrow && <span className="eyebrow">{eyebrow}</span>}
          <h2>{title}</h2>
        </div>
        {aside}
      </div>
      {children}
    </section>
  );
}
export function Metric({
  label,
  value,
  unit = "",
  note,
  tone = "",
}: {
  label: string;
  value: ReactNode;
  unit?: string;
  note?: string;
  tone?: string;
}) {
  return (
    <div className={`metric ${tone}`}>
      <span>{label}</span>
      <div className="metric-number">
        {value}
        <small>{unit}</small>
      </div>
      {note && <p>{note}</p>}
    </div>
  );
}
export function Fact({
  label,
  value,
  unit = "",
}: {
  label: string;
  value: ReactNode;
  unit?: string;
}) {
  return (
    <div className="fact">
      <span>{label}</span>
      <strong>
        {value} <small>{unit}</small>
      </strong>
    </div>
  );
}
export function Source({
  name,
  sample,
  online,
}: {
  name: string;
  sample?: Sample;
  online: boolean;
}) {
  return (
    <div className="source">
      <Activity size={13} />
      <code>{name}</code>
      <span>
        {online && sample
          ? `${fmt(sample.rx_hz, 1)} Hz · ${fmt(sample.period_ms, 1)} ms 주기 · 마지막 ${fmt(sample.age_ms, 0)} ms 전${sample.fresh ? "" : " · 지연"}`
          : "수신 대기"}
      </span>
    </div>
  );
}
export function Choice({
  label,
  options,
  value,
  disabled,
  onChange,
}: {
  label: string;
  options: { value: string; label: string }[];
  value: unknown;
  disabled: boolean;
  onChange: (value: string) => void;
}) {
  return (
    <div className="control-row">
      <span>{label}</span>
      <div className="segmented" role="group" aria-label={label}>
        {options.map((o) => (
          <button
            key={o.value}
            disabled={disabled}
            aria-pressed={value === o.value}
            className={value === o.value ? "selected" : ""}
            onClick={() => onChange(o.value)}
          >
            {o.label}
          </button>
        ))}
      </div>
    </div>
  );
}
export function Switch({
  label,
  description,
  value,
  disabled,
  onChange,
  danger = false,
}: {
  label: string;
  description?: string;
  value: unknown;
  disabled: boolean;
  onChange: (value: boolean) => void;
  danger?: boolean;
}) {
  return (
    <div className="switch-row">
      <div>
        <strong>{label}</strong>
        {description && <p>{description}</p>}
      </div>
      <button
        className={`switch ${danger ? "danger" : ""}`}
        role="switch"
        aria-label={label}
        aria-checked={value === true}
        disabled={disabled}
        onClick={() => onChange(value !== true)}
      >
        <span />
      </button>
    </div>
  );
}
export function Numeric({
  label,
  value,
  min,
  max,
  step = 1,
  unit = "",
  disabled,
  onApply,
}: {
  label: string;
  value: unknown;
  min: number;
  max: number;
  step?: number;
  unit?: string;
  disabled: boolean;
  onApply: (v: number) => void;
}) {
  const [draft, setDraft] = useState("");
  const [dirty, setDirty] = useState(false);
  const id = useId();
  useEffect(() => {
    if (!dirty && typeof value === "number") setDraft(String(value));
  }, [value, dirty]);
  return (
    <form
      className="numeric"
      onSubmit={(e) => {
        e.preventDefault();
        if (draft !== "" && Number.isFinite(Number(draft))) {
          onApply(Number(draft));
          setDirty(false);
        }
      }}
    >
      <label htmlFor={id}>{label}</label>
      <div className="input-with-unit">
        <input
          id={id}
          type="number"
          value={draft}
          min={min}
          max={max}
          step={step}
          placeholder="—"
          disabled={disabled}
          required
          onChange={(e) => {
            setDirty(true);
            setDraft(e.target.value);
          }}
        />
        <span>{unit}</span>
      </div>
      <button
        className="icon-button"
        aria-label={`${label} 적용`}
        title="값 적용"
        disabled={disabled || draft === ""}
      >
        <Send size={16} />
      </button>
    </form>
  );
}
export function Details({
  title,
  children,
}: {
  title: string;
  children: ReactNode;
}) {
  return (
    <details>
      <summary>
        {title}
        <ChevronDown size={16} />
      </summary>
      <div className="detail-content">{children}</div>
    </details>
  );
}
export function Notice({ children }: { children: ReactNode }) {
  return (
    <div className="notice">
      <CircleHelp size={17} />
      <span>{children}</span>
    </div>
  );
}
export type Trace = {
  key: keyof Omit<Point, "time">;
  label: string;
  color: string;
};
export function Chart({
  points,
  traces,
  unit,
  height = 210,
}: {
  points: Point[];
  traces: Trace[];
  unit: string;
  height?: number;
}) {
  const [hover, setHover] = useState<number | null>(null);
  const values = points
    .flatMap((p) => traces.map((t) => p[t.key]))
    .filter((n): n is number => typeof n === "number");
  let low = Math.min(0, ...values),
    high = Math.max(1, ...values);
  const pad = (high - low) * 0.12;
  low -= pad;
  high += pad;
  const width = 740,
    left = 48,
    right = 16,
    top = 18,
    bottom = 30;
  const end = points.at(-1)?.time ?? 0,
    start = end - 60;
  const x = (time: number) =>
    left + ((time - start) / 60) * (width - left - right);
  const y = (v: number) =>
    top + ((high - v) / (high - low)) * (height - top - bottom);
  const nearest =
    hover === null
      ? null
      : points.reduce<Point | null>(
          (best, p) =>
            !best ||
            Math.abs(x(p.time) - hover) < Math.abs(x(best.time) - hover)
              ? p
              : best,
          null,
        );
  return (
    <div className="chart">
      <div className="chart-legend">
        {traces.map((t) => (
          <span key={t.key}>
            <i style={{ background: t.color }} />
            {t.label}
            {nearest && <b>{fmt(nearest[t.key])}</b>}
          </span>
        ))}
        <small>{unit} · 최근 60초</small>
      </div>
      <svg
        viewBox={`0 0 ${width} ${height}`}
        role="img"
        aria-label={`${traces.map((t) => t.label).join(", ")} ${unit} 실시간 그래프`}
        onMouseMove={(e) =>
          setHover(
            ((e.clientX - e.currentTarget.getBoundingClientRect().left) /
              e.currentTarget.getBoundingClientRect().width) *
              width,
          )
        }
        onMouseLeave={() => setHover(null)}
      >
        {[0, 1, 2, 3].map((i) => {
          const yy = top + ((height - top - bottom) * i) / 3;
          return (
            <g key={i}>
              <line
                x1={left}
                y1={yy}
                x2={width - right}
                y2={yy}
                className="gridline"
              />
              <text x={left - 9} y={yy + 4} textAnchor="end">
                {fmt(high - ((high - low) * i) / 3, 1)}
              </text>
            </g>
          );
        })}
        {[-60, -45, -30, -15, 0].map((t) => (
          <text
            key={t}
            x={x(end + t)}
            y={height - 5}
            textAnchor={t === 0 ? "end" : "start"}
          >
            {t === 0 ? "now" : `${t}s`}
          </text>
        ))}
        {traces.map((t) => {
          let pen = false;
          const d = points
            .map((p) => {
              const v = p[t.key];
              if (v === null) {
                pen = false;
                return "";
              }
              const s = `${pen ? "L" : "M"}${x(p.time).toFixed(1)},${y(v).toFixed(1)}`;
              pen = true;
              return s;
            })
            .join(" ");
          return (
            <path
              key={t.key}
              d={d}
              fill="none"
              stroke={t.color}
              strokeWidth="2.3"
              vectorEffect="non-scaling-stroke"
            />
          );
        })}
        {nearest && (
          <line
            x1={x(nearest.time)}
            y1={top}
            x2={x(nearest.time)}
            y2={height - bottom}
            className="cursorline"
          />
        )}
        {values.length === 0 && (
          <text
            x={width / 2}
            y={height / 2}
            textAnchor="middle"
            className="chart-empty"
          >
            유효한 관측값을 기다리고 있습니다
          </text>
        )}
      </svg>
    </div>
  );
}
