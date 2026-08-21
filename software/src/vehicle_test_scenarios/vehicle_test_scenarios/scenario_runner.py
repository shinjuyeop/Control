"""Parameterized Planning command scenarios for controlled vehicle tests."""

from __future__ import annotations

from dataclasses import asdict, dataclass
import math
import time

from kaiev26_msgs.msg import ActuatorCommand
import rclpy
from rcl_interfaces.msg import SetParametersResult
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node


SCENARIOS = {
    "straight",
    "steering_sweep",
    "speed_step",
    "speed_ramp",
    "brake_test",
}
MAX_TEST_SPEED_KPH = 10.0
MAX_STEERING_DEG = 28.0
STARTUP_ONLY_PARAMETERS = {"publish_rate_hz", "command_topic"}


@dataclass(frozen=True)
class ScenarioConfig:
    scenario: str = "straight"
    cruise_speed_kph: float = 3.0
    speed_low_kph: float = 0.0
    speed_high_kph: float = 3.0
    speed_step_interval_sec: float = 2.0
    speed_ramp_duration_sec: float = 5.0
    steering_center_deg: float = 0.0
    steering_amplitude_deg: float = 10.0
    steering_period_sec: float = 4.0
    brake_delay_sec: float = 3.0


@dataclass(frozen=True)
class ScenarioSetpoint:
    speed_kph: float
    steering_deg: float
    brake_engage: bool = False


def kph_to_mps(speed_kph: float) -> float:
    return speed_kph / 3.6


def scenario_setpoint(
    config: ScenarioConfig, elapsed_sec: float
) -> ScenarioSetpoint:
    elapsed_sec = max(0.0, elapsed_sec)
    if config.scenario == "straight":
        return ScenarioSetpoint(
            config.cruise_speed_kph, config.steering_center_deg
        )

    if config.scenario == "steering_sweep":
        phase = 2.0 * math.pi * elapsed_sec / config.steering_period_sec
        steering_deg = (
            config.steering_center_deg
            + config.steering_amplitude_deg * math.sin(phase)
        )
        return ScenarioSetpoint(config.cruise_speed_kph, steering_deg)

    if config.scenario == "speed_step":
        phase = int(elapsed_sec / config.speed_step_interval_sec) % 2
        speed_kph = (
            config.speed_low_kph if phase == 0 else config.speed_high_kph
        )
        return ScenarioSetpoint(speed_kph, config.steering_center_deg)

    if config.scenario == "speed_ramp":
        phase = (elapsed_sec / config.speed_ramp_duration_sec) % 2.0
        fraction = phase if phase <= 1.0 else 2.0 - phase
        speed_kph = config.speed_low_kph + fraction * (
            config.speed_high_kph - config.speed_low_kph
        )
        return ScenarioSetpoint(speed_kph, config.steering_center_deg)

    if config.scenario == "brake_test":
        if elapsed_sec < config.brake_delay_sec:
            return ScenarioSetpoint(
                config.cruise_speed_kph, config.steering_center_deg
            )
        return ScenarioSetpoint(0.0, config.steering_center_deg, True)

    raise ValueError(f"unknown scenario: {config.scenario}")


def validate_config(config: ScenarioConfig, publish_rate_hz: float) -> None:
    if config.scenario not in SCENARIOS:
        raise ValueError(
            f"scenario must be one of: {', '.join(sorted(SCENARIOS))}"
        )

    numeric_values = [
        value for name, value in asdict(config).items() if name != "scenario"
    ] + [publish_rate_hz]
    if not all(math.isfinite(float(value)) for value in numeric_values):
        raise ValueError("scenario parameters must be finite")

    for name in ("cruise_speed_kph", "speed_low_kph", "speed_high_kph"):
        value = float(getattr(config, name))
        if not 0.0 <= value <= MAX_TEST_SPEED_KPH:
            raise ValueError(f"{name} must be within 0..10 km/h")
    if config.speed_low_kph > config.speed_high_kph:
        raise ValueError("speed_low_kph must not exceed speed_high_kph")

    steering_extent = (
        abs(config.steering_center_deg) + config.steering_amplitude_deg
    )
    if steering_extent > MAX_STEERING_DEG:
        raise ValueError(
            "steering center plus amplitude must stay within +/-28 degrees"
        )
    if config.steering_amplitude_deg < 0.0:
        raise ValueError("steering_amplitude_deg must be non-negative")

    positive_parameters = {
        "speed_step_interval_sec": config.speed_step_interval_sec,
        "speed_ramp_duration_sec": config.speed_ramp_duration_sec,
        "steering_period_sec": config.steering_period_sec,
        "brake_delay_sec": config.brake_delay_sec,
    }
    for name, value in positive_parameters.items():
        if value <= 0.0:
            raise ValueError(f"{name} must be positive")
    if publish_rate_hz < 10.0:
        raise ValueError("publish_rate_hz must be at least 10 Hz")


class VehicleTestScenarioRunner(Node):
    def __init__(self) -> None:
        super().__init__("vehicle_test_scenario_runner")
        self.declare_parameter("enabled", False)
        for name, value in asdict(ScenarioConfig()).items():
            self.declare_parameter(name, value)
        self.declare_parameter("brake_override", False)
        self.declare_parameter("publish_rate_hz", 20.0)
        self.declare_parameter("command_topic", "/planning/command")

        self._enabled = bool(self.get_parameter("enabled").value)
        self._brake_override = bool(self.get_parameter("brake_override").value)
        self._config = self._read_config()
        publish_rate_hz = float(self.get_parameter("publish_rate_hz").value)
        validate_config(self._config, publish_rate_hz)

        command_topic = str(self.get_parameter("command_topic").value)
        self._publisher = self.create_publisher(
            ActuatorCommand, command_topic, 10
        )
        self._started_at = time.monotonic() if self._enabled else None
        self._parameter_callback = self.add_on_set_parameters_callback(
            self._on_parameters
        )
        self._timer = self.create_timer(1.0 / publish_rate_hz, self._publish)

        self.get_logger().warning(
            "REAL-VEHICLE TEST SCENARIO: default is 3 km/h straight. "
            f"Publishing on {command_topic} starts only after enabled=true; "
            "confirm there is no other Planning publisher."
        )

    def _read_config(self) -> ScenarioConfig:
        return ScenarioConfig(**{
            name: self.get_parameter(name).value
            for name in asdict(ScenarioConfig())
        })

    def _on_parameters(self, parameters):
        for parameter in parameters:
            if parameter.name in STARTUP_ONLY_PARAMETERS:
                return SetParametersResult(
                    successful=False,
                    reason=(
                        f"{parameter.name} is read at startup "
                        "(restart required)"
                    ),
                )

        proposed = asdict(self._config)
        for parameter in parameters:
            if parameter.name in proposed:
                proposed[parameter.name] = parameter.value
        proposed_config = ScenarioConfig(**proposed)
        try:
            validate_config(
                proposed_config,
                float(self.get_parameter("publish_rate_hz").value),
            )
        except ValueError as error:
            return SetParametersResult(successful=False, reason=str(error))

        reset_scenario = proposed_config != self._config
        self._config = proposed_config
        for parameter in parameters:
            if parameter.name == "enabled":
                self._enabled = bool(parameter.value)
                self._started_at = time.monotonic() if self._enabled else None
            elif parameter.name == "brake_override":
                self._brake_override = bool(parameter.value)
        if reset_scenario and self._enabled:
            self._started_at = time.monotonic()
        return SetParametersResult(successful=True)

    def _publish(self) -> None:
        if not self._enabled or self._started_at is None:
            return

        setpoint = scenario_setpoint(
            self._config, time.monotonic() - self._started_at
        )
        brake_engage = self._brake_override or setpoint.brake_engage

        message = ActuatorCommand()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = "base_link"
        message.speed_target_mps = (
            0.0 if brake_engage else kph_to_mps(setpoint.speed_kph)
        )
        message.steering_target_rad = (
            0.0 if brake_engage else math.radians(setpoint.steering_deg)
        )
        message.brake_engage = brake_engage
        self._publisher.publish(message)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = VehicleTestScenarioRunner()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
