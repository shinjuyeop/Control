"""Timer-driven runner for deterministic YAML HIL scenarios."""

from __future__ import annotations

from datetime import datetime, timezone
import json
from pathlib import Path
import time

from kaiev26_msgs.msg import (
    BrakeActuatorStatus,
    DriveActuatorStatus,
    SteeringActuatorStatus,
    VehicleState,
)
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.parameter import Parameter
from std_msgs.msg import String

from .scenario import evaluate_expectation, hil_commands, load_scenario
from .remote_parameters import RemoteParameterClient


class HilScenarioRunner(Node):
    def __init__(self, *, parameter_overrides=None, service_wait_sec=5.0) -> None:
        super().__init__("hil_scenario_runner", parameter_overrides=parameter_overrides)
        self.declare_parameter("scenario_file", "")
        self.declare_parameter("results_dir", "test_results")
        scenario_file = str(self.get_parameter("scenario_file").value)
        if not scenario_file:
            raise ValueError("scenario_file parameter is required")
        self.definition = load_scenario(scenario_file)
        self.results_dir = Path(str(self.get_parameter("results_dir").value))
        self.command_pub = self.create_publisher(String, "/hil/command", 20)
        self.create_subscription(String, "/hil/command_result", self._on_command_result, 20)
        self.command_client = RemoteParameterClient(self, "hil_command_generator")
        if not self.command_client.wait_for_service(timeout_sec=service_wait_sec):
            raise RuntimeError("hil_command_generator parameter service is unavailable")

        self.latest = {}
        self._subscribe(DriveActuatorStatus, "/drive/status")
        self._subscribe(BrakeActuatorStatus, "/brake/status")
        self._subscribe(SteeringActuatorStatus, "/steering/status")
        self._subscribe(VehicleState, "/vehicle/state")
        self.create_subscription(String, "/hil/status", self._on_hil_status, 10)

        self.start_monotonic = time.monotonic()
        self.start_wall = datetime.now(timezone.utc)
        self.step_index = 0
        self.expectation_state = ["pending"] * len(self.definition.expectations)
        self.records = []
        self.command_results = []
        self.infrastructure_failures = []
        self.complete = False
        self.result = None
        self.parameter_requests = []
        self.parameter_requested_at = {}
        self.create_timer(0.02, self._tick)
        self.get_logger().info(f"scenario started: {self.definition.name}")

    def _subscribe(self, message_type, topic):
        self.create_subscription(
            message_type, topic,
            lambda message, name=topic: self.latest.__setitem__(name, (time.monotonic(), message)),
            10,
        )

    def _on_hil_status(self, message: String) -> None:
        try:
            value = json.loads(message.data)
        except json.JSONDecodeError:
            return
        self.latest["/hil/status"] = (time.monotonic(), value)

    def _on_command_result(self, message: String) -> None:
        self.command_results.append({
            "elapsed_sec": time.monotonic() - self.start_monotonic,
            "response": message.data,
        })
        if message.data.startswith("ERR "):
            self.infrastructure_failures.append(message.data)

    def _publish_hil(self, command: str) -> None:
        message = String()
        message.data = command
        self.command_pub.publish(message)

    def _set_production_command(self, values: dict) -> None:
        parameters = []
        for name, value in values.items():
            parameters.append(Parameter(name=name, value=value))
        if parameters:
            future = self.command_client.set_parameters(parameters)
            if future is None:
                self.infrastructure_failures.append(
                    "hil_command_generator parameter service is unavailable"
                )
                return
            self.parameter_requests.append(future)
            self.parameter_requested_at[future] = time.monotonic()

            def completed(done):
                try:
                    response = done.result()
                    if not response.result.successful:
                        self.infrastructure_failures.append(
                            "production parameters rejected: " + response.result.reason
                        )
                except Exception as exc:
                    self.infrastructure_failures.append(
                        f"production parameter request failed: {exc}"
                    )

            future.add_done_callback(completed)

    def _execute_step(self, index, step, elapsed) -> None:
        commands = hil_commands(step.hil, step.fault)
        for command in commands:
            self._publish_hil(command)
        self._set_production_command(step.command)
        self.records.append({
            "kind": "step",
            "index": index,
            "scheduled_at_sec": step.at,
            "executed_at_sec": elapsed,
            "latency_ms": max(0.0, (elapsed - step.at) * 1000.0),
            "hil_commands": commands,
            "production_parameters": step.command,
            "result": "EXECUTED",
        })

    def _evaluate_expectations(self, elapsed) -> None:
        for index, expectation in enumerate(self.definition.expectations):
            if self.expectation_state[index] != "pending" or elapsed < expectation.after:
                continue
            deadline = expectation.after + expectation.within_ms / 1000.0
            sample = self.latest.get(expectation.topic)
            if sample is not None:
                received, message = sample
                window_start = self.start_monotonic + expectation.after
                window_deadline = window_start + expectation.within_ms / 1000.0
                if window_start <= received <= window_deadline:
                    sample_elapsed = received - self.start_monotonic
                    passed, detail = evaluate_expectation(expectation, message)
                    if passed:
                        self.expectation_state[index] = "passed"
                        self.records.append({
                            "kind": "expectation", "index": index, "topic": expectation.topic,
                            "field": expectation.field, "result": "PASS", "detail": detail,
                            "latency_ms": (sample_elapsed - expectation.after) * 1000.0,
                        })
                        continue
            if elapsed >= deadline:
                detail = "no sample received in window"
                if sample is not None:
                    received, message = sample
                    matches, observed_detail = evaluate_expectation(
                        expectation, message
                    )
                    if received < window_start:
                        early_ms = (window_start - received) * 1000.0
                        detail = (
                            f"sample arrived {early_ms:.3f} ms before window"
                        )
                    elif received > window_deadline:
                        late_ms = (received - window_deadline) * 1000.0
                        match_text = "matching" if matches else "non-matching"
                        detail = (
                            f"{match_text} sample arrived {late_ms:.3f} ms "
                            "after deadline"
                        )
                    else:
                        detail = (
                            "in-window sample did not satisfy expectation: "
                            + observed_detail
                        )
                self.expectation_state[index] = "failed"
                self.records.append({
                    "kind": "expectation", "index": index, "topic": expectation.topic,
                    "field": expectation.field, "result": "FAIL", "detail": detail,
                    "latency_ms": expectation.within_ms,
                })

    def _tick(self) -> None:
        if self.complete:
            return
        for request in self.parameter_requests:
            if not request.done() and time.monotonic() - self.parameter_requested_at[request] > 2.0:
                self.infrastructure_failures.append("production parameter response timed out")
                request.cancel()
        elapsed = time.monotonic() - self.start_monotonic
        while self.step_index < len(self.definition.steps):
            step = self.definition.steps[self.step_index]
            if elapsed < step.at:
                break
            self._execute_step(self.step_index, step, elapsed)
            self.step_index += 1
        self._evaluate_expectations(elapsed)
        if self.step_index == len(self.definition.steps) and all(
            state != "pending" for state in self.expectation_state
        ) and all(request.done() for request in self.parameter_requests):
            self._finish()

    def _finish(self, outcome=None) -> None:
        self.complete = True
        passed = (
            all(state == "passed" for state in self.expectation_state)
            and not self.infrastructure_failures
        )
        result = self.result = {
            "scenario": self.definition.name,
            "started_at": self.start_wall.isoformat(),
            "duration_sec": time.monotonic() - self.start_monotonic,
            "result": outcome or ("PASS" if passed else "FAIL"),
            "records": self.records,
            "command_results": self.command_results,
            "infrastructure_failures": self.infrastructure_failures,
        }
        self.results_dir.mkdir(parents=True, exist_ok=True)
        timestamp = self.start_wall.strftime("%Y%m%dT%H%M%SZ")
        output = self.results_dir / f"{self.definition.name}_{timestamp}.json"
        try:
            output.write_text(json.dumps(result, indent=2, sort_keys=True), encoding="utf-8")
        except OSError as exc:
            self.infrastructure_failures.append(f"result could not be saved: {exc}")
            result["result"] = "FAIL"
            passed = False
        log = self.get_logger().info if passed else self.get_logger().error
        log(f"scenario {result['result']}: {output}")


def main(args=None) -> None:
    rclpy.init(args=args)
    node = None
    try:
        node = HilScenarioRunner()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
