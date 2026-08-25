import math
import time

import pytest
import rclpy
from kaiev26_msgs.msg import DriveActuatorStatus
from rclpy.parameter import Parameter

from vehicle_test_scenarios.scenario_runner import (
    ScenarioConfig,
    VehicleTestScenarioRunner,
    kph_to_mps,
    scheduled_scenario_setpoint,
    scenario_setpoint,
    zero_speed_baseline_setpoint,
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
    ("elapsed_sec", "expected_deg"),
    [(0.0, -2.0), (5.0, -1.0), (10.0, -2.0), (15.0, -3.0), (20.0, -2.0)],
)
def test_steering_step_returns_to_center_between_directions(
    elapsed_sec, expected_deg
):
    config = ScenarioConfig(
        scenario="steering_step",
        cruise_speed_kph=3.6,
        steering_center_deg=-2.0,
        steering_amplitude_deg=1.0,
        steering_step_interval_sec=5.0,
    )
    output = scenario_setpoint(config, elapsed_sec)
    assert output.speed_kph == 3.6
    assert output.steering_deg == expected_deg


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


def test_finite_scenario_holds_initial_setpoint_after_repetitions():
    config = ScenarioConfig(
        scenario="speed_step",
        speed_low_kph=3.6,
        speed_high_kph=7.2,
        speed_step_interval_sec=5.0,
        scenario_repetitions=2,
    )
    active, active_complete = scheduled_scenario_setpoint(config, 15.0)
    finished, finished_complete = scheduled_scenario_setpoint(config, 20.0)
    assert active.speed_kph == 7.2
    assert not active_complete
    assert finished.speed_kph == 0.0
    assert finished.steering_deg == config.steering_center_deg
    assert not finished.brake_engage
    assert finished_complete


def test_initial_hold_precedes_sine_repetitions():
    config = ScenarioConfig(
        scenario="steering_sweep",
        steering_center_deg=-2.0,
        steering_amplitude_deg=1.0,
        steering_period_sec=8.0,
        initial_hold_sec=5.0,
        scenario_repetitions=1,
    )
    initial, initial_complete = scheduled_scenario_setpoint(config, 4.9)
    peak, peak_complete = scheduled_scenario_setpoint(config, 7.0)
    finished, finished_complete = scheduled_scenario_setpoint(config, 13.0)
    assert initial.steering_deg == -2.0
    assert not initial_complete
    assert peak.steering_deg == pytest.approx(-1.0)
    assert not peak_complete
    assert finished.steering_deg == -2.0
    assert finished.speed_kph == 0.0
    assert not finished.brake_engage
    assert finished_complete


def test_zero_speed_baseline_does_not_engage_brake():
    output = zero_speed_baseline_setpoint(
        ScenarioConfig(steering_center_deg=-2.0)
    )
    assert output.speed_kph == 0.0
    assert output.steering_deg == -2.0
    assert not output.brake_engage


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


def test_auto_mode_transition_starts_and_resets_scenario_timeline():
    rclpy.init(
        args=[
            "--ros-args",
            "-p",
            "enabled:=true",
            "-p",
            "wait_for_auto_mode:=true",
            "-p",
            "auto_start_delay_sec:=10.0",
            "-p",
            "scenario_label:=test_speed_step",
        ]
    )
    node = VehicleTestScenarioRunner()
    try:
        assert node._started_at is None

        status = DriveActuatorStatus()
        status.mode = DriveActuatorStatus.MODE_AUTO
        before_transition = time.monotonic()
        node._on_drive_status(status)
        assert node._started_at is not None
        assert node._started_at >= before_transition + 9.9

        commands = []
        scenario_statuses = []

        class CapturePublisher:
            def __init__(self, output):
                self._output = output

            def publish(self, message):
                self._output.append(message)

        node._publisher = CapturePublisher(commands)
        node._scenario_status_publisher = CapturePublisher(scenario_statuses)
        node._publish()
        assert commands[-1].speed_target_mps == 0.0
        assert commands[-1].steering_target_rad == 0.0
        assert not commands[-1].brake_engage
        assert scenario_statuses[-1].data == "test_speed_step"

        status.mode = DriveActuatorStatus.MODE_MANUAL
        node._on_drive_status(status)
        assert node._started_at is None
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
        ("steering_step_interval_sec", 0.0),
        ("steering_period_sec", 0.0),
        ("initial_hold_sec", -0.1),
        ("scenario_repetitions", -1),
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
