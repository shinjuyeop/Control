from pathlib import Path
from types import SimpleNamespace

import pytest

from vehicle_console.hil.scenario import (
    ScenarioExpectation,
    evaluate_expectation,
    hil_commands,
    load_scenario,
)
from vehicle_console.hil.hil_scenario_runner import HilScenarioRunner


SCENARIOS = Path(__file__).resolve().parents[2] / "config" / "scenarios"


@pytest.mark.parametrize("name", [
    "smoke_test.yaml", "estop_test.yaml", "ez_can_timeout.yaml", "keya_can_timeout.yaml"
])
def test_default_scenarios_parse(name):
    scenario = load_scenario(SCENARIOS / name)
    assert scenario.name
    assert scenario.steps
    assert scenario.expectations


def test_hil_step_mapping_builds_real_firmware_commands():
    commands = hil_commands(
        {"mode": "AUTO", "estop_dashboard": True, "steering_position": -10},
        {"ez_drop_tx": True},
    )
    assert commands == [
        "SET MODE AUTO",
        "SET ESTOP DASH ON",
        "SET STEER_POS -10",
        "FAULT EZ DROP_TX ON",
    ]


def test_safe_state_expands_to_complete_desired_state():
    commands = hil_commands({"safe_state": True}, {})
    assert "SET MODE MANUAL" in commands
    assert "SET EZ_CAN ON" in commands
    assert "SET KEYA_CAN ON" in commands
    assert "FAULT EZ OFF" in commands
    assert "FAULT KEYA OFF" in commands


def test_condition_evaluator_supports_equality_range_and_bitmask():
    message = {"valid": True, "speed": 2.5, "reasons": 5}
    assert evaluate_expectation(
        ScenarioExpectation("/hil/status", "valid", 0, 100, equals=True), message
    )[0]
    assert evaluate_expectation(
        ScenarioExpectation("/hil/status", "speed", 0, 100, minimum=2.0, maximum=3.0), message
    )[0]
    assert evaluate_expectation(
        ScenarioExpectation("/hil/status", "reasons", 0, 100, bitmask=4), message
    )[0]


def evaluate_window(*, sample_elapsed, evaluator_elapsed, sample_value=True):
    expectation = ScenarioExpectation(
        "/hil/status", "ready", after=1.0, within_ms=100, equals=True
    )
    runner = object.__new__(HilScenarioRunner)
    runner.start_monotonic = 100.0
    runner.definition = SimpleNamespace(expectations=[expectation])
    runner.expectation_state = ["pending"]
    runner.records = []
    runner.latest = {}
    if sample_elapsed is not None:
        runner.latest[expectation.topic] = (
            runner.start_monotonic + sample_elapsed,
            {"ready": sample_value},
        )
    runner._evaluate_expectations(evaluator_elapsed)
    return runner


def test_sample_inside_window_passes_when_evaluator_runs_late():
    runner = evaluate_window(sample_elapsed=1.05, evaluator_elapsed=1.12)
    assert runner.expectation_state == ["passed"]
    assert runner.records[0]["result"] == "PASS"
    assert runner.records[0]["latency_ms"] == pytest.approx(50.0)


def test_sample_after_deadline_fails_even_if_value_matches():
    runner = evaluate_window(sample_elapsed=1.105, evaluator_elapsed=1.12)
    assert runner.expectation_state == ["failed"]
    assert runner.records[0]["result"] == "FAIL"
    assert runner.records[0]["detail"] == (
        "matching sample arrived 5.000 ms after deadline"
    )


def test_sample_at_exact_deadline_passes():
    runner = evaluate_window(sample_elapsed=1.1, evaluator_elapsed=1.12)
    assert runner.expectation_state == ["passed"]


def test_no_sample_fails_at_deadline():
    runner = evaluate_window(sample_elapsed=None, evaluator_elapsed=1.1)
    assert runner.expectation_state == ["failed"]
    assert runner.records[0]["detail"] == "no sample received in window"


def test_sample_before_after_is_not_used():
    runner = evaluate_window(sample_elapsed=0.999, evaluator_elapsed=1.05)
    assert runner.expectation_state == ["pending"]
    runner._evaluate_expectations(1.1)
    assert runner.expectation_state == ["failed"]
    assert "before window" in runner.records[0]["detail"]


def test_non_matching_sample_after_deadline_reports_timing_and_value_class():
    runner = evaluate_window(
        sample_elapsed=1.105, evaluator_elapsed=1.12, sample_value=False
    )
    assert runner.expectation_state == ["failed"]
    assert runner.records[0]["detail"] == (
        "non-matching sample arrived 5.000 ms after deadline"
    )


def test_in_window_non_matching_sample_reports_expectation_failure():
    runner = evaluate_window(
        sample_elapsed=1.05, evaluator_elapsed=1.12, sample_value=False
    )
    assert runner.expectation_state == ["failed"]
    assert runner.records[0]["detail"].startswith(
        "in-window sample did not satisfy expectation:"
    )
