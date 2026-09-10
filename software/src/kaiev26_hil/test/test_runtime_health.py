"""Runtime interpretation, independently of ROS, browsers and physical boards."""
from copy import deepcopy

import pytest

from kaiev26_hil.runtime_config import BOARDS
from kaiev26_hil.runtime_health import RuntimeHealth

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


def test_delta_rate_uses_uptime_and_preserves_uint64_precision():
    health = RuntimeHealth()
    first = runtime_data()
    first["tasks"][4]["activations"] = 2**60
    health.receive(A, first, 10)
    assert health.snapshot(10)["STM_A"]["data"]["worst_stack"]["rate_hz"] is None
    second = runtime_data(generation=12, uptime_ms=12000)
    second["tasks"][4]["activations"] = 2**60 + 100
    health.receive(A, second, 12)
    task = health.snapshot(12)["STM_A"]["data"]["worst_stack"]
    assert task["rate_hz"] == 50
    assert task["activations"] == str(2**60 + 100)
    # Rendering repeatedly or receiving the same firmware snapshot preserves the delta.
    health.receive(A, second, 12.8)
    assert health.snapshot(12.8)["STM_A"]["data"]["worst_stack"]["rate_hz"] == 50
    assert health.snapshot(15)["STM_A"]["status"] == "stale"


@pytest.mark.parametrize("change", ["uptime", "generation", "wrap", "counter", "gap"])
def test_discontinuity_resets_then_recovers_rate(change):
    health = RuntimeHealth()
    first = runtime_data(generation=2**32 - 1 if change == "wrap" else 10)
    health.receive(A, first, 10)
    second = runtime_data(generation=11, uptime_ms=11000)
    if change == "uptime": second["uptime_ms"] = 1
    if change in ("generation", "wrap"): second["generation"] = 0
    if change == "counter": second["tasks"][4]["activations"] = 499
    if change == "counter": second["tasks"][4]["completed"] = second["tasks"][4]["timed_samples"] = 499
    now = 14 if change == "gap" else 11
    health.receive(A, second, now)
    assert health.snapshot(now)["STM_A"]["data"]["worst_stack"]["rate_hz"] is None
    third = deepcopy(second)
    third["generation"] += 1
    third["uptime_ms"] += 1000
    third["tasks"][4]["activations"] += 50
    health.receive(A, third, now + 1)
    assert health.snapshot(now + 1)["STM_A"]["data"]["worst_stack"]["rate_hz"] == 50


def test_memory_semantics_validity_flags_and_board_mapping():
    health = RuntimeHealth()
    a, b = runtime_data(), runtime_data(2)
    a["tasks"][1]["present"] = False
    health.receive(A, a, 10)
    health.receive(B, b, 10)
    result = health.snapshot(11.5)
    assert result["STM_A"]["status"] == result["STM_B"]["status"] == "ok"
    a = result["STM_A"]["data"]
    assert a["heap"]["used_bytes"] == 12976
    assert a["heap"]["minimum_free_bytes"] == 52000
    assert a["worst_stack"]["name"] == "BTx"
    assert a["worst_stack"]["stack"]["used_bytes"] == 460
    assert a["worst_stack"]["stack"]["percent"] == pytest.approx(44.921875)
    assert not a["tasks"][1]["present"]
    assert a["tasks"][2]["jitter"] is None
    assert a["cpu_load_percent"] is None
    assert a["estop"]["max_us"] is None
    assert result["STM_B"]["data"]["estop"] == {"max_us": 6, "samples": 3}
    assert health.snapshot(13)["STM_A"]["status"] == "stale"
    health.receive(A, b, 14)
    assert health.snapshot(14)["STM_A"]["status"] == "invalid"


@pytest.mark.parametrize("field", ["stack", "heap", "minimum", "missing", "counts", "wall"])
def test_malformed_runtime_is_invalid_without_clamping(field):
    health = RuntimeHealth()
    data = runtime_data()
    if field == "stack": data["tasks"][4]["stack_free_bytes"] = 1025
    if field == "heap": data["free_heap_bytes"] = 65537
    if field == "minimum": data["minimum_free_heap_bytes"] = 53000
    if field == "missing": del data["timing_available"]
    if field == "counts": data["tasks"][0]["completed"] = 1000
    if field == "wall": data["tasks"][0]["wall_us_average"] = 4000
    health.receive(A, data, 10)
    board = health.snapshot(10)["STM_A"]
    assert board["status"] == "invalid" and board["issues"]
    if field == "stack":
        assert board["data"]["worst_stack"] is None
        stack = board["data"]["tasks"][4]["stack"]
        assert stack["free_bytes"] == 1025 and stack["used_bytes"] is None


def test_timing_errors_and_periodic_miss_status():
    health = RuntimeHealth()
    data = runtime_data()
    data["timing_available"] = False
    health.receive(A, data, 10)
    assert health.snapshot(10)["STM_A"]["status"] == "timing_unavailable"
    assert health.snapshot(10)["STM_A"]["data"]["tasks"][0]["wall"] is None
    data = runtime_data(generation=11, uptime_ms=11000)
    data["tasks"][4]["deadline_misses"] = 2
    health.receive(A, data, 11)
    board = health.snapshot(11)["STM_A"]
    assert board["status"] == "check" and board["data"]["deadline_misses_delta"] == "2"
    data = deepcopy(data)
    data.update(generation=12, uptime_ms=12000)
    health.receive(A, data, 12)
    assert health.snapshot(12)["STM_A"]["status"] == "history"
    data = runtime_data(generation=13, uptime_ms=13000)
    data["instrumentation_errors"] = 1
    health.receive(A, data, 13)
    assert health.snapshot(13)["STM_A"]["status"] == "check"
    data = deepcopy(data)
    data.update(generation=14, uptime_ms=14000, instrumentation_errors=3)
    health.receive(A, data, 14)
    assert health.snapshot(14)["STM_A"]["data"]["instrumentation_errors_delta"] == 2
