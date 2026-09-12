from copy import deepcopy
import pytest
from vehicle_console.hil.runtime_health import RuntimeHealth
from runtime_samples import runtime_data, A, B

def test_deadline_reset_preserves_lifetime_and_warns_on_new_misses():
    health = RuntimeHealth()
    raw = runtime_data(2)
    raw["tasks"][7]["deadline_misses"] = 30
    health.receive(B, raw, 10)
    assert health.snapshot(10)["STM_B"]["status"] == "history"
    assert health.reset_deadline_history(10) == ["STM_B"]
    board = health.snapshot(10)["STM_B"]
    assert board["status"] == "ok"
    assert board["data"]["deadline_misses"] == "30"
    assert board["data"]["deadline_misses_since_reset"] == "0"
    assert raw["tasks"][7]["deadline_misses"] == 30
    newer = runtime_data(2, generation=11, uptime_ms=11000)
    newer["tasks"][7]["deadline_misses"] = 31
    health.receive(B, newer, 11)
    board = health.snapshot(11)["STM_B"]
    assert board["status"] == "check"
    assert board["data"]["deadline_misses_since_reset"] == "1"
    display = next(t for t in board["data"]["tasks"] if t["task_id"] == 7)
    assert display["deadline_misses"] == "31"
    assert display["deadline_misses_since_reset_delta"] == "1"
    health.reset_deadline_history(11)
    board = health.snapshot(11)["STM_B"]
    assert board["status"] == "ok"
    assert board["data"]["deadline_misses_since_reset_delta"] == "0"
    assert board["data"]["deadline_misses_delta"] == "1"


def test_stale_reset_does_not_acknowledge_unseen_misses():
    health = RuntimeHealth()
    raw = runtime_data(2)
    raw["tasks"][7]["deadline_misses"] = 30
    health.receive(B, raw, 10)
    health.reset_deadline_history(10)
    assert health.reset_deadline_history(14) == []
    assert health.snapshot(14)["STM_B"]["status"] == "stale"
    newer = runtime_data(2, generation=15, uptime_ms=15000)
    newer["tasks"][7]["deadline_misses"] = 32
    health.receive(B, newer, 15)
    assert health.snapshot(15)["STM_B"]["data"]["deadline_misses_since_reset"] == "2"


@pytest.mark.parametrize("reason", ["reboot", "wrap", "counter"])
def test_deadline_baseline_is_released_on_counter_discontinuity(reason):
    health = RuntimeHealth()
    raw = runtime_data(2, generation=2**32 - 1 if reason == "wrap" else 10)
    raw["tasks"][7]["deadline_misses"] = 30
    health.receive(B, raw, 10)
    health.reset_deadline_history(10)
    new = runtime_data(2, generation=1 if reason != "counter" else 11,
                       uptime_ms=1000 if reason == "reboot" else 11000)
    new["tasks"][7]["deadline_misses"] = 1 if reason != "wrap" else 31
    health.receive(B, new, 11)
    board = health.snapshot(11)["STM_B"]
    assert board["data"]["deadline_reset_uptime_ms"] is None
    assert board["data"]["deadline_misses_since_reset"] == str(new["tasks"][7]["deadline_misses"])


def test_deadline_reset_does_not_clear_other_diagnostic_conditions():
    health = RuntimeHealth()
    raw = runtime_data(2)
    raw["instrumentation_errors"] = 1
    raw["tasks"][7]["deadline_misses"] = 30
    health.receive(B, raw, 10)
    health.reset_deadline_history(10)
    assert health.snapshot(10)["STM_B"]["status"] == "check"
    raw = runtime_data(2, generation=11, uptime_ms=11000)
    raw.update(timing_available=False, estop_valid=False)
    health.receive(B, raw, 11)
    health.reset_deadline_history(11)
    assert health.snapshot(11)["STM_B"]["status"] == "timing_unavailable"
    raw["tasks"][7]["stack_free_bytes"] = 2049
    health.receive(B, raw, 12)
    assert health.reset_deadline_history(12) == []
    assert health.snapshot(12)["STM_B"]["status"] == "invalid"
