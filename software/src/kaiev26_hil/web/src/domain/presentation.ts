/** Presentation only: consume existing freshness/validity and reported states. */
import type { Data, Snapshot } from "./types";
import type { RuntimeTask } from "./runtime";
import { states } from "./vehicle";

export type Tone = "muted" | "good" | "warning" | "danger" | "command";
export type Indicator = { label: string; tone: Tone };

export function reception(
  state: Snapshot,
  online: boolean,
  topic: string,
): Indicator {
  const sample = state.topics[topic];
  if (!online) return { label: "OFFLINE", tone: "muted" };
  if (!sample) return { label: "NO DATA", tone: "muted" };
  return sample.fresh
    ? { label: "LIVE", tone: "good" }
    : { label: "STALE", tone: "warning" };
}

export function reported(
  value: unknown,
  active: string,
  clear: string,
  tone: Tone = "warning",
): Indicator {
  return typeof value !== "boolean"
    ? { label: "UNKNOWN", tone: "muted" }
    : value
      ? { label: active, tone }
      : { label: clear, tone: "muted" };
}

export function steeringState(data: Data): Indicator {
  return {
    label: states[data.state] ?? "UNKNOWN",
    tone:
      data.state === 3
        ? "good"
        : data.state === 4 || data.state === 8
          ? "danger"
          : data.state >= 5
            ? "warning"
            : "muted",
  };
}

// Display scale, not a command limit. The checked-in steering calibration maps
// +/-3279 counts to approximately -27.8/+25.9 degrees (positive = left).
// Out-of-scale feedback keeps its numeric value and is labelled explicitly.
export const STEERING_SCALE_DEG = 30;

export function runtimeRowState(task: RuntimeTask): Indicator | null {
  if (!task.stack.valid) return { label: "STACK INVALID", tone: "danger" };
  const misses = task.deadline_misses_since_reset ?? task.deadline_misses;
  if (task.jitter && misses !== "0") return { label: "MISS", tone: "warning" };
  if (task.stack.tone === "warning" || task.stack.tone === "danger")
    return { label: "CAPACITY", tone: task.stack.tone };
  return null;
}

export function sequenceStep(
  run: Data | null,
  definition: Data | undefined,
  index: number,
): Indicator {
  if (!run || run.scenario !== definition?.name)
    return { label: "WAITING", tone: "muted" };
  const record = run.records.find(
    (r: Data) => r.kind === "step" && r.index === index,
  );
  if (record) return { label: "SENT", tone: "command" }; // Sending is not an expectation PASS.
  if (run.result === "RUNNING" && run.step_index === index)
    return { label: "NEXT", tone: "warning" };
  return {
    label: run.result === "RUNNING" ? "WAITING" : "NOT SENT",
    tone: "muted",
  };
}
