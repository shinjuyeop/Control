"""HIL Bench Reset policy over the shared runtime observations."""

from vehicle_console.runtime_health import RuntimeHealth as SharedRuntimeHealth
from vehicle_console.runtime_config import BOARDS, RUNTIME_STALE_SECONDS


class RuntimeHealth(SharedRuntimeHealth):
    def reset_deadline_history(self, now):
        """Acknowledge the latest fresh observations without changing MCU counters."""
        cleared = []
        for topic, board in self.boards.items():
            if board.get("invalid") or board["issues"] or (
                now - self.progress_at[topic] >= RUNTIME_STALE_SECONDS
            ):
                continue
            self.deadline_baselines[topic] = {
                "uptime_ms": board["uptime_ms"],
                "misses": {t["task_id"]: t["deadline_misses"]
                           for t in board["tasks"] if t["present"]},
            }
            cleared.append(BOARDS[topic]["name"])
        return cleared
