from pathlib import Path

import pytest

from kaiev26_hil.scenario import (
    ScenarioExpectation,
    evaluate_expectation,
    hil_commands,
    load_scenario,
)


SCENARIOS = Path(__file__).resolve().parents[1] / "config" / "scenarios"


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
