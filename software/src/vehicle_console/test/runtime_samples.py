from vehicle_console.common.runtime_config import BOARDS

A = "/stm_a/runtime_stats"
B = "/stm_b/runtime_stats"

def runtime_data(board_id=1, generation=10, uptime_ms=10000):
    config = BOARDS[A if board_id == 1 else B]
    tasks = []
    for task_id in range(9):
        allocated = config["tasks"].get(task_id, ("", 0))[1]
        tasks.append(dict(
            task_id=task_id, present=bool(allocated),
            activations=uptime_ms // 20, completed=uptime_ms // 20,
            timed_samples=uptime_ms // 20, wall_us_total=uptime_ms * 50,
            wall_us_max=3207, wall_us_average=1000,
            jitter_samples=0 if task_id in (2, 3, 5, 6) else uptime_ms // 20,
            jitter_us_min=0, jitter_us_max=1000, deadline_misses=0,
            stack_free_bytes=564 if task_id == 4 and board_id == 1 else allocated * 3 // 4,
        ))
    return dict(board_id=board_id, generation=generation, uptime_ms=uptime_ms,
                core_clock_hz=170000000, window_ms=1000, timing_available=True,
                instrumentation_errors=0, free_heap_bytes=52560,
                minimum_free_heap_bytes=52000, cpu_load_valid=False,
                cpu_load_permille=0, tasks=tasks, estop_valid=board_id == 2,
                estop_callback_count=3 if board_id == 2 else 0,
                estop_coalesced_count=0, estop_dispatch_samples=3 if board_id == 2 else 0,
                estop_dispatch_us_max=6 if board_id == 2 else 0)
