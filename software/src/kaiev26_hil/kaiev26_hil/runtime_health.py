"""ROS-independent RuntimeStats normalization; consumes decoded ROS dictionaries.

One instance owns both boards' previous snapshots. Feed once per ROS reception,
then read snapshots at any display cadence. Counters stay Python integers until
after subtraction; uint64 task counts are decimal strings on the JSON boundary.
This module can also serve a diagnostics publisher or another dashboard adapter.
"""

from .runtime_config import (
    BOARDS, CAPACITY_DANGER_PERCENT, CAPACITY_WARNING_PERCENT,
    HEAP_RESERVED_BYTES, RUNTIME_STALE_SECONDS,
)


def integer(data, key, *, signed=False):
    value = data[key]
    if type(value) is not int or (not signed and value < 0):
        raise ValueError(f"{key}: invalid integer")
    return value


def boolean(data, key):
    value = data[key]
    if type(value) is not bool:
        raise ValueError(f"{key}: invalid boolean")
    return value


def capacity(free, allocated):
    # Never clamp a contradictory HWM or heap reading into a plausible gauge.
    valid = free <= allocated
    used = allocated - free if valid else None
    percent = used * 100 / allocated if valid else None
    tone = "muted" if not valid else (
        "danger" if percent >= CAPACITY_DANGER_PERCENT else
        "warning" if percent >= CAPACITY_WARNING_PERCENT else "normal"
    )
    return dict(valid=valid, free_bytes=free, allocated_bytes=allocated,
                used_bytes=used, percent=percent, tone=tone)


def normalize_runtime(data, config):
    """Decode/validate consumed fields once at the external message boundary."""
    if integer(data, "board_id") != config["board_id"]:
        raise ValueError("board_id does not match topic")
    board = {key: integer(data, key) for key in (
        "generation", "uptime_ms", "core_clock_hz", "instrumentation_errors",
    )}
    timing = board["timing_available"] = boolean(data, "timing_available")
    if timing and not board["core_clock_hz"]:
        raise ValueError("timing available with zero core clock")
    issues = board["issues"] = []
    heap = board["heap"] = capacity(integer(data, "free_heap_bytes"), HEAP_RESERVED_BYTES)
    heap["minimum_free_bytes"] = integer(data, "minimum_free_heap_bytes")
    if heap["minimum_free_bytes"] > heap["free_bytes"]:
        heap.update(valid=False, used_bytes=None, percent=None, tone="muted")
    if not heap["valid"]:
        issues.append("Heap free/min free가 예약량 또는 minimum-ever 관계와 모순됩니다.")
    cpu_valid = boolean(data, "cpu_load_valid")
    cpu = integer(data, "cpu_load_permille") if cpu_valid else None
    if cpu is not None and cpu > 1000:
        raise ValueError("cpu_load_permille exceeds 1000")
    board["cpu_load_percent"] = cpu / 10 if cpu is not None else None
    tasks = {}
    for raw in data["tasks"]:
        task_id = integer(raw, "task_id")
        if task_id in tasks:
            raise ValueError(f"duplicate task_id {task_id}")
        present = boolean(raw, "present")
        if not present:
            tasks[task_id] = dict(task_id=task_id, present=False)
            continue
        if task_id not in config["tasks"]:
            raise ValueError(f"no allocation metadata for present task {task_id}")
        name, allocated = config["tasks"][task_id]
        task = {key: integer(raw, key) for key in (
            "activations", "completed", "timed_samples", "wall_us_total",
            "wall_us_max", "wall_us_average", "jitter_samples", "deadline_misses",
        )}
        if not task["timed_samples"] <= task["completed"] <= task["activations"]:
            raise ValueError(f"{name}: inconsistent activation/completion counts")
        if task["timed_samples"] and task["wall_us_average"] > task["wall_us_max"]:
            raise ValueError(f"{name}: wall average > max")
        task.update(task_id=task_id, name=name, present=True, rate_hz=None,
                    deadline_misses_delta=None)
        task["stack"] = capacity(integer(raw, "stack_free_bytes"), allocated)
        if not task["stack"]["valid"]:
            issues.append(f"{name}: stack free {task['stack']['free_bytes']} B > allocated {allocated} B")
        task["wall"] = (
            dict(max_us=task["wall_us_max"], average_us=task["wall_us_average"])
            if timing and task["timed_samples"] else None
        )
        task["jitter"] = None
        if task["jitter_samples"]:
            low = integer(raw, "jitter_us_min", signed=True)
            high = integer(raw, "jitter_us_max", signed=True)
            if low > high:
                raise ValueError(f"{name}: jitter min > max")
            task["jitter"] = dict(min_us=low, max_us=high)
        tasks[task_id] = task
    if any(task_id not in tasks for task_id in config["tasks"]):
        raise ValueError("missing configured task observation")
    board["tasks"] = [tasks[task_id] for task_id in config["tasks"]]
    board["deadline_misses"] = sum(
        task["deadline_misses"] for task in board["tasks"]
        if task["present"] and task["jitter"] is not None
    )
    board["deadline_misses_delta"] = None
    board["instrumentation_errors_delta"] = None
    valid_stacks = [task for task in board["tasks"] if task["present"] and task["stack"]["valid"]]
    board["worst_stack"] = max(valid_stacks, key=lambda t: t["stack"]["percent"], default=None)
    if any(task["present"] and not task["stack"]["valid"] for task in board["tasks"]):
        board["worst_stack"] = None  # An invalid task cannot be silently excluded from ranking.
    estop_valid = boolean(data, "estop_valid")
    samples = integer(data, "estop_dispatch_samples")
    if estop_valid and (config["board_id"] != 2 or not timing or not samples):
        raise ValueError("estop_valid contradicts board/timing/samples")
    board["estop"] = dict(
        samples=samples,
        max_us=integer(data, "estop_dispatch_us_max") if estop_valid else None,
    )
    return board


class RuntimeHealth:
    def __init__(self):
        self.boards = {}
        self.previous = {}
        self.progress_at = {}
        self.received_at = {}
        self.deadline_baselines = {}

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

    def receive(self, topic, data, now):
        self.received_at[topic] = now
        try:
            board = normalize_runtime(data, BOARDS[topic])
        except (KeyError, TypeError, ValueError) as exc:
            self.previous.pop(topic, None)
            self.deadline_baselines.pop(topic, None)
            self.boards[topic] = dict(issues=[str(exc)], invalid=True)
            self.progress_at[topic] = now
            return
        previous = self.previous.get(topic)
        same = previous and all(board[k] == previous[k] for k in ("generation", "uptime_ms"))
        if same and not board["issues"] and not previous["issues"]:
            return  # Repeated ROS delivery is not a new firmware snapshot.
        reset = previous is None or now - self.progress_at[topic] >= RUNTIME_STALE_SECONDS
        reset |= bool(board["issues"] or (previous and previous["issues"]))
        discontinuity = False
        if previous:
            discontinuity = board["generation"] <= previous["generation"] or board["uptime_ms"] <= previous["uptime_ms"]
            discontinuity |= board["instrumentation_errors"] < previous["instrumentation_errors"]
            for task, old in zip(board["tasks"], previous["tasks"]):
                if task["present"] and old["present"]:
                    discontinuity |= any(task[k] < old[k] for k in (
                        "activations", "completed", "timed_samples", "wall_us_total",
                        "jitter_samples", "deadline_misses",
                    ))
        baseline = self.deadline_baselines.get(topic)
        if baseline:
            discontinuity |= any(
                t["present"] and t["deadline_misses"] < baseline["misses"].get(t["task_id"], 0)
                for t in board["tasks"]
            )
        reset |= discontinuity
        if discontinuity or board["issues"]:
            self.deadline_baselines.pop(topic, None)
        if not reset and not board["issues"] and not previous["issues"]:
            elapsed = (board["uptime_ms"] - previous["uptime_ms"]) / 1000
            for task, old in zip(board["tasks"], previous["tasks"]):
                if task["present"] and old["present"]:
                    task["rate_hz"] = (task["activations"] - old["activations"]) / elapsed
                    if task["jitter"] is not None and old["jitter"] is not None:
                        task["deadline_misses_delta"] = task["deadline_misses"] - old["deadline_misses"]
            board["deadline_misses_delta"] = sum(t["deadline_misses_delta"] or 0 for t in board["tasks"] if t["present"])
            board["instrumentation_errors_delta"] = board["instrumentation_errors"] - previous["instrumentation_errors"]
        board["rate_state"] = "baseline" if reset else "measured"
        self.previous[topic] = board
        self.boards[topic] = board
        self.progress_at[topic] = now

    def snapshot(self, now):
        result = {}
        for topic, config in BOARDS.items():
            board = self.boards.get(topic)
            age = (now - self.received_at[topic]) * 1000 if board else None
            progress_age = (now - self.progress_at[topic]) * 1000 if board else None
            baseline = self.deadline_baselines.get(topic)
            misses_since_reset = 0
            if board and not board.get("invalid"):
                misses_since_reset = sum(
                    self.misses_since_reset(t, baseline) for t in board["tasks"]
                    if t["present"] and t["jitter"] is not None
                )
            status = "waiting"
            if board:
                if max(age, progress_age) >= RUNTIME_STALE_SECONDS * 1000:
                    status = "stale"
                elif board.get("invalid") or board["issues"]:
                    status = "invalid"
                elif not board["timing_available"]:
                    status = "timing_unavailable"
                elif board["instrumentation_errors"] or (misses_since_reset and board["deadline_misses_delta"]):
                    status = "check"
                elif misses_since_reset:
                    status = "history"
                else:
                    status = "ok"
            public = None
            if board and not board.get("invalid"):
                public = {**board, "tasks": [self.public_task(t, baseline) for t in board["tasks"]]}
                worst = board["worst_stack"]
                public["worst_stack"] = self.public_task(worst, baseline) if worst else None
                public["deadline_misses"] = str(board["deadline_misses"])
                delta = board["deadline_misses_delta"]
                public["deadline_misses_delta"] = str(delta) if delta is not None else None
                public["deadline_reset_uptime_ms"] = baseline["uptime_ms"] if baseline else None
                public["deadline_misses_since_reset"] = str(misses_since_reset)
                public["deadline_misses_since_reset_delta"] = (
                    str(min(delta, misses_since_reset)) if delta is not None else None
                )
            result[config["name"]] = dict(
                name=config["name"], role=config["role"], topic=topic,
                status=status, age_ms=age, progress_age_ms=progress_age,
                stale_after_ms=RUNTIME_STALE_SECONDS * 1000,
                issues=board["issues"] if board else [], data=public,
            )
        return result

    @staticmethod
    def misses_since_reset(task, baseline):
        return task["deadline_misses"] - (
            baseline["misses"].get(task["task_id"], 0) if baseline else 0
        )

    @staticmethod
    def public_task(task, baseline=None):
        if not task["present"]:
            return task
        public = {key: str(value) if key in (
            "activations", "completed", "timed_samples", "wall_us_total",
            "jitter_samples", "deadline_misses", "deadline_misses_delta",
        ) and value is not None else value for key, value in task.items()}
        misses = RuntimeHealth.misses_since_reset(task, baseline)
        public["deadline_misses_since_reset"] = str(misses)
        delta = task["deadline_misses_delta"]
        public["deadline_misses_since_reset_delta"] = (
            str(min(delta, misses)) if delta is not None else None
        )
        return public
