import type { Data } from "./types";
export const feedbackColor = "#0756b4",
  targetColor = "#a45b20",
  auxiliaryColor = "#7047a5";
export const modes = [
  { value: "MANUAL", label: "MANUAL" },
  { value: "AUTO", label: "AUTO" },
];
export const gears = [
  { value: "FWD", label: "FWD" },
  { value: "REV", label: "REV" },
];
export const boolean = (value: unknown, yes = "ON", no = "OFF") =>
  typeof value === "boolean" ? (value ? yes : no) : "—";
export const age = (value: unknown) =>
  typeof value === "number" && Number.isFinite(value) && value !== 4294967295
    ? `${value.toFixed(0)} ms`
    : "—";
export const scenarioDescriptions: Record<string, string> = {
  smoke_test: "구동·조향 기본 통신과 피드백",
  estop_test: "정상 제동 해제 → E-stop → 제동 요청",
  ez_can_timeout: "정상 EZ 피드백 → TX 차단 → 유효성 해제",
  keya_can_timeout: "정상 KEYA 통신 → TX 차단 → timeout",
};
export function canLink(hil: Data, fresh: boolean, bus: "ez" | "keya") {
  if (!fresh) return { text: "NO DATA", tone: "muted" };
  if ((hil.protocol ?? 1) < 2) return { text: "RX AGE N/A", tone: "muted" };
  if (hil[`${bus}_busoff`]) return { text: "BUS OFF", tone: "danger" };
  if (!hil[`${bus}_can`]) return { text: "CAN OFF", tone: "warning" };
  if (!hil[`${bus}_rx`] || hil[`${bus}_rx_age_ms`] === 4294967295)
    return { text: "NO RX", tone: "muted" };
  return hil[`${bus}_rx_age_ms`] < 500
    ? { text: "ACTIVE", tone: "good" }
    : { text: "STALE RX", tone: "warning" };
}

export const fmt = (value: unknown, decimals = 1) =>
  typeof value === "number" && Number.isFinite(value)
    ? value.toFixed(decimals)
    : "—";
