import math

import pytest
import rclpy
from rclpy.parameter import Parameter

from vehicle_test_scenarios.scenario_runner import (
    ScenarioConfig,
    VehicleTestScenarioRunner,
    kph_to_mps,
    scenario_setpoint,
)


def test_default_speed_converts_to_meters_per_second():
    assert kph_to_mps(3.0) == pytest.approx(0.8333333333)


def test_straight_uses_constant_speed_and_center():
    output = scenario_setpoint(ScenarioConfig(), 123.0)
    assert output.speed_kph == 3.0
    assert output.steering_deg == 0.0
    assert not output.brake_engage


@pytest.mark.parametrize(
    ("elapsed_sec", "expected_deg"),
    [(0.0, 0.0), (1.0, 10.0), (2.0, 0.0), (3.0, -10.0), (4.0, 0.0)],
)
def test_steering_sweep_is_smooth_periodic_sine(elapsed_sec, expected_deg):
    config = ScenarioConfig(scenario="steering_sweep")
    output = scenario_setpoint(config, elapsed_sec)
    assert output.speed_kph == 3.0
    assert output.steering_deg == pytest.approx(expected_deg, abs=1e-9)


@pytest.mark.parametrize(
    ("elapsed_sec", "expected_kph"),
    [(0.0, 0.0), (1.99, 0.0), (2.0, 3.0), (3.99, 3.0), (4.0, 0.0)],
)
def test_speed_step_alternates_low_and_high(elapsed_sec, expected_kph):
    config = ScenarioConfig(scenario="speed_step")
    assert scenario_setpoint(config, elapsed_sec).speed_kph == expected_kph


@pytest.mark.parametrize(
    ("elapsed_sec", "expected_kph"),
    [(0.0, 0.0), (2.5, 1.5), (5.0, 3.0), (7.5, 1.5), (10.0, 0.0)],
)
def test_speed_ramp_moves_linearly_up_and_down(elapsed_sec, expected_kph):
    config = ScenarioConfig(scenario="speed_ramp")
    output = scenario_setpoint(config, elapsed_sec)
    assert output.speed_kph == pytest.approx(expected_kph)


def test_brake_test_holds_brake_after_delay():
    config = ScenarioConfig(scenario="brake_test")
    before = scenario_setpoint(config, 2.99)
    after = scenario_setpoint(config, 3.0)
    assert before.speed_kph == 3.0
    assert not before.brake_engage
    assert after.speed_kph == 0.0
    assert after.brake_engage


def test_runtime_scenario_parameters_update_without_crashing():
    rclpy.init()
    node = VehicleTestScenarioRunner()
    try:
        results = node.set_parameters([
            Parameter("scenario", value="steering_sweep"),
            Parameter("cruise_speed_kph", value=4.0),
            Parameter("steering_amplitude_deg", value=5.0),
            Parameter("enabled", value=True),
        ])
        assert all(result.successful for result in results)
        assert node._config.scenario == "steering_sweep"
        assert node._config.cruise_speed_kph == 4.0
        assert node._config.steering_amplitude_deg == 5.0
        assert node._enabled
        assert node._started_at is not None
    finally:
        node.destroy_node()
        rclpy.shutdown()


@pytest.mark.parametrize(
    ("parameter", "value"),
    [
        ("scenario", "unknown"),
        ("cruise_speed_kph", -1.0),
        ("speed_high_kph", 10.1),
        ("speed_low_kph", 4.0),
        ("steering_center_deg", 20.0),
        ("steering_amplitude_deg", 28.1),
        ("steering_period_sec", 0.0),
        ("brake_delay_sec", math.nan),
    ],
)
def test_invalid_runtime_parameters_are_rejected(parameter, value):
    rclpy.init()
    node = VehicleTestScenarioRunner()
    try:
        result = node.set_parameters([Parameter(parameter, value=value)])[0]
        assert not result.successful
    finally:
        node.destroy_node()
        rclpy.shutdown()
