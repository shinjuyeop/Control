import { fmt } from "./format";

export type Capacity = {
  valid: boolean;
  used_bytes: number | null;
  free_bytes: number;
  allocated_bytes: number;
  percent: number | null;
  tone: string;
};
export type RuntimeTask = {
  task_id: number;
  present: true;
  name: string;
  stack: Capacity;
  rate_hz: number | null;
  wall: { max_us: number; average_us: number } | null;
  jitter: { min_us: number; max_us: number } | null;
  deadline_misses: string;
  deadline_misses_delta: string | null;
};
export type RuntimeData = {
  generation: number;
  uptime_ms: number;
  core_clock_hz: number;
  timing_available: boolean;
  heap: Capacity & { minimum_free_bytes: number };
  worst_stack: RuntimeTask | null;
  tasks: (RuntimeTask | { task_id: number; present: false })[];
  deadline_misses: string;
  deadline_misses_delta: string | null;
  instrumentation_errors: number;
  instrumentation_errors_delta: number | null;
  cpu_load_percent: number | null;
  estop: { samples: number; max_us: number | null };
  rate_state: "baseline" | "measured";
};
export type RuntimeBoard = {
  name: string;
  role: string;
  topic: string;
  status: "waiting" | "stale" | "invalid" | "timing_unavailable" | "check" | "history" | "ok";
  age_ms: number | null;
  progress_age_ms: number | null;
  stale_after_ms: number;
  issues: string[];
  data: RuntimeData | null;
};

export const runtimeBoards = ["STM_A", "STM_B"] as const;
export const runtimeNotes = {
  capacity: "색상은 용량 여유에 대한 UI advisory입니다. 차량 safety threshold가 아닙니다.",
  heap: "FreeRTOS heap_4 내부 사용량입니다. 전체 SRAM이나 linker RAM footprint가 아닙니다. Min free는 boot 이후 minimum-ever free heap입니다.",
  stack: "Stack high-water mark는 task 생성 이후 관측된 최소 free stack입니다. 최대 관측 사용량 = 할당량 − HWM. 현재 사용량이 아닙니다.",
  wall: "Boot 이후 누적 observed wall max / avg. Interrupt, preemption, 내부 blocking을 포함하며 formal WCET가 아닙니다.",
  jitter: "Scheduled release 대비 min / max. RTOS tick 기준 1 ms 해상도. 표본이 없으면 미계측입니다.",
  rate: "연속 snapshot의 activation count 차이 / 보드 uptime 차이. 재시작·counter reset·generation wrap·수신 중단 후 첫 표본은 기준점입니다.",
  estop: "EXTI callback → EStop task 처리 시작. Boot 이후 최대 관측값이며 physical-edge interrupt latency가 아닙니다.",
};
const health = {
  waiting: { label: "NO DATA", tone: "muted", detail: "Runtime snapshot 수신 대기" },
  stale: { label: "STALE", tone: "warning", detail: "Runtime 수신 또는 snapshot 진행이 멈췄습니다" },
  invalid: { label: "INVALID", tone: "danger", detail: "Runtime 데이터와 allocation metadata 확인 필요" },
  timing_unavailable: { label: "TIMING N/A", tone: "warning", detail: "DWT timing source를 사용할 수 없습니다" },
  check: { label: "CHECK", tone: "warning", detail: "Instrumentation 오류 또는 최근 deadline miss 증가" },
  history: { label: "MISS HISTORY", tone: "warning", detail: "누적 deadline miss 있음 · 최근 증가 미관측" },
  ok: { label: "RUNTIME OK", tone: "good", detail: "수신·timing 정상 · 오류 및 deadline miss 미관측" },
};

export function runtimeView(board: RuntimeBoard | undefined, online: boolean) {
  const status = !online ? (board?.data ? "stale" : "waiting") : board?.status ?? "waiting";
  const data = online && status !== "stale" ? board?.data ?? null : null;
  return {
    ...health[status],
    data,
    stackInvalid: data?.tasks.some(task => task.present && !task.stack.valid) ?? false,
    timing: data ? data.timing_available ? `DWT · ${fmt(data.core_clock_hz / 1e6, 0)} MHz` : "TIMING N/A" : "—",
    cpu: data ? data.cpu_load_percent == null ? "미지원" : `${fmt(data.cpu_load_percent, 1)}%` : "—",
    age: online ? `${fmt(board?.progress_age_ms, 0)} ms` : "—",
    issues: board?.issues ?? [],
  };
}

export const freshnessSeconds = (board?: RuntimeBoard) => fmt(board ? board.stale_after_ms / 1000 : null, 0);

export const bytes = (n: number | null | undefined) => n == null ? "—" : `${fmt(n, 0)} B`;
export const kib = (n: number | null | undefined) => n == null ? "—" : `${fmt(n / 1024, 1)} KiB`;
export function capacityRatio(c: Capacity | null | undefined, unit: "B" | "KiB" = "B") {
  if (!c) return "— / —";
  if (!c.valid) return `INVALID · free ${bytes(c.free_bytes)}`;
  const scale = unit === "KiB" ? 1024 : 1;
  const precision = unit === "KiB" ? 1 : 0;
  return `${fmt(c.used_bytes! / scale, precision)} / ${fmt(c.allocated_bytes / scale, precision)} ${unit}`;
}
export const duration = (us: number | null | undefined) => us == null ? "미계측" :
  Math.abs(us) >= 1000 ? `${fmt(us / 1000, 2)} ms` : `${fmt(us, 0)} µs`;
export const jitterRange = (task: RuntimeTask) => task.jitter == null ? "미계측" :
  `${fmt(task.jitter.min_us / 1000, 0)} / ${fmt(task.jitter.max_us / 1000, 0)} ms`;
export function uptime(ms: number | undefined) {
  if (ms == null) return "—";
  const seconds = Math.floor(ms / 1000);
  return `${Math.floor(seconds / 3600)}h ${String(Math.floor(seconds / 60) % 60).padStart(2, "0")}m ${String(seconds % 60).padStart(2, "0")}s`;
}
export const delta = (value: string | number | null | undefined) => value == null ? "기준점" : `+${value} 최근`;
