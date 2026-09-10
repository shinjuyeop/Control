import type { Snapshot } from "./types";

/** Continue source age locally during a silent or interrupted WAN connection.
 * Uses elapsed monotonic browser time, never compares clocks on different PCs.
 */
export function ageSnapshot(state: Snapshot, elapsedMs: number): Snapshot {
  const topics = Object.fromEntries(Object.entries(state.topics).map(([topic, sample]) => {
    const age = sample.age_ms + elapsedMs;
    const fresh = sample.fresh && age < (sample.stale_after_ms ?? Infinity);
    const data = topic === "/diagnostics" && sample.data.status
      ? { status: sample.data.status.map((item: Record<string, any>) => ({
          ...item, age_ms: (item.age_ms ?? sample.age_ms) + elapsedMs,
          fresh: item.fresh !== false && (item.age_ms ?? sample.age_ms) + elapsedMs < (sample.stale_after_ms ?? Infinity),
        })) }
      : sample.data;
    return [topic, { ...sample, data, age_ms: age, fresh, rx_hz: fresh ? sample.rx_hz : 0 }];
  }));
  const runtime = Object.fromEntries(Object.entries(state.runtime ?? {}).map(([name, board]) => {
    const age = board.age_ms == null ? null : board.age_ms + elapsedMs;
    const progress = board.progress_age_ms == null ? null : board.progress_age_ms + elapsedMs;
    return [name, { ...board, age_ms: age, progress_age_ms: progress,
      status: Math.max(age ?? 0, progress ?? 0) >= board.stale_after_ms ? "stale" as const : board.status }];
  }));
  return { ...state, topics, runtime };
}
