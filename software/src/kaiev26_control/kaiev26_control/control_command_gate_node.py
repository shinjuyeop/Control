#!/usr/bin/env python3
from __future__ import annotations

import math

from kaiev26_msgs.msg import ActuatorCommand, AEBState, Centerline, TargetSpeed, VehicleState
import rclpy
from rclpy.node import Node
from rclpy.time import Time as RosTime
from std_msgs.msg import String


INF_DISTANCE = 1.0e6


def clamp(value: float, lower: float, upper: float) -> float:
    return max(lower, min(upper, value))


def stamp_age_sec(now, stamp) -> float:
    if stamp.sec == 0 and stamp.nanosec == 0:
        return INF_DISTANCE
    age = now.nanoseconds - RosTime.from_msg(stamp).nanoseconds
    return max(0.0, age * 1.0e-9)


class ControlCommandGateNode(Node):
    def __init__(self) -> None:
        super().__init__("kaiev26_control_command_gate")
        self.declare_parameter("target_path_topic", "/planning/target_path")
        self.declare_parameter("target_speed_topic", "/planning/target_speed")
        self.declare_parameter("vehicle_state_topic", "/vehicle/state")
        self.declare_parameter("aeb_state_topic", "/safety/aeb_state")
        self.declare_parameter("command_topic", "/vehicle/actuator_command")
        self.declare_parameter("debug_topic", "/debug/control_gate")
        self.declare_parameter("control_rate_hz", 50.0)
        self.declare_parameter("target_stale_sec", 0.35)
        self.declare_parameter("wheelbase_m", 1.2991017929)
        self.declare_parameter("max_steering_rad", 0.4363)
        self.declare_parameter("steering_rate_limit_radps", 2.5)
        self.declare_parameter("lookahead_base_m", 2.5)
        self.declare_parameter("lookahead_gain_s", 0.7)
        self.declare_parameter("max_accel_mps2", 1.0)
        self.declare_parameter("max_decel_mps2", 2.0)
        self.declare_parameter("stop_speed_epsilon_mps", 0.05)
        self.declare_parameter("planned_stop_brake_distance_m", 2.0)

        self.target_stale_sec = float(self.get_parameter("target_stale_sec").value)
        self.wheelbase_m = float(self.get_parameter("wheelbase_m").value)
        self.max_steering_rad = float(self.get_parameter("max_steering_rad").value)
        self.steering_rate_limit_radps = float(self.get_parameter("steering_rate_limit_radps").value)
        self.lookahead_base_m = float(self.get_parameter("lookahead_base_m").value)
        self.lookahead_gain_s = float(self.get_parameter("lookahead_gain_s").value)
        self.max_accel_mps2 = float(self.get_parameter("max_accel_mps2").value)
        self.max_decel_mps2 = float(self.get_parameter("max_decel_mps2").value)
        self.stop_speed_epsilon_mps = float(self.get_parameter("stop_speed_epsilon_mps").value)
        self.planned_stop_brake_distance_m = max(
            0.0,
            float(self.get_parameter("planned_stop_brake_distance_m").value),
        )
        self.control_rate_hz = max(1.0, float(self.get_parameter("control_rate_hz").value))

        self.latest_target_path: Centerline | None = None
        self.latest_target_speed: TargetSpeed | None = None
        self.latest_vehicle_state: VehicleState | None = None
        self.latest_aeb_state: AEBState | None = None

        # TODO(Control safety contract): vehicle_state와 aeb_state는 현재 수신 시각과
        # 상위 source health를 검증하지 않고 마지막 값을 계속 사용한다. AEB의
        # active=false heartbeat가 끊기거나 vehicle_state_publisher가 단절된 actuator
        # status를 새 stamp로 재발행하면 통신 단절을 정상 상태로 오인할 수 있다.
        # 실차 적용 전에 제어팀이 freshness 기준, fault latch와 reset 절차, fault 시
        # steering과 brake 동작을 확정하고 이 입력 경계에 반영해야 한다.
        self.commanded_speed_mps = 0.0
        self.commanded_steering_rad = 0.0
        self.last_control_time = self.get_clock().now()

        self.command_pub = self.create_publisher(
            ActuatorCommand,
            str(self.get_parameter("command_topic").value),
            10,
        )
        self.debug_pub = self.create_publisher(
            String,
            str(self.get_parameter("debug_topic").value),
            10,
        )
        self.create_subscription(
            Centerline,
            str(self.get_parameter("target_path_topic").value),
            self.on_target_path,
            10,
        )
        self.create_subscription(
            TargetSpeed,
            str(self.get_parameter("target_speed_topic").value),
            self.on_target_speed,
            10,
        )
        self.create_subscription(
            VehicleState,
            str(self.get_parameter("vehicle_state_topic").value),
            self.on_vehicle_state,
            10,
        )
        self.create_subscription(
            AEBState,
            str(self.get_parameter("aeb_state_topic").value),
            self.on_aeb_state,
            10,
        )
        self.create_timer(1.0 / self.control_rate_hz, self.on_control_timer)

    def on_target_path(self, msg: Centerline) -> None:
        self.latest_target_path = msg

    def on_target_speed(self, msg: TargetSpeed) -> None:
        self.latest_target_speed = msg

    def on_vehicle_state(self, msg: VehicleState) -> None:
        self.latest_vehicle_state = msg

    def on_aeb_state(self, msg: AEBState) -> None:
        self.latest_aeb_state = msg

    @property
    def estop_active(self) -> bool:
        # E-stop 상태는 통합 차량 상태에서 읽는다 (별도 legacy 토픽 없음)
        return (
            self.latest_vehicle_state is not None
            and bool(self.latest_vehicle_state.estop_active)
        )

    def on_control_timer(self) -> None:
        now = self.get_clock().now()
        dt = max(1.0 / self.control_rate_hz, (now.nanoseconds - self.last_control_time.nanoseconds) * 1.0e-9)
        self.last_control_time = now

        command = ActuatorCommand()
        command.header.stamp = now.to_msg()
        command.gear = ActuatorCommand.GEAR_FORWARD

        reason = "normal"
        target_stale = self.targets_are_stale(now)
        if self.estop_active:
            reason = "estop"
            self.apply_stop_command(command, hold_steering=False)
        elif self.latest_aeb_state is not None and self.latest_aeb_state.active:
            reason = f"aeb:{self.latest_aeb_state.state}"
            self.apply_stop_command(command, hold_steering=True)
        elif target_stale:
            reason = "target stale"
            self.apply_stop_command(command, hold_steering=True)
        else:
            self.apply_nominal_command(command, dt)

        self.command_pub.publish(command)
        self.publish_debug(command, reason)

    def targets_are_stale(self, now) -> bool:
        if self.latest_target_path is None or self.latest_target_speed is None:
            return True
        path_age = stamp_age_sec(now, self.latest_target_path.header.stamp)
        speed_age = stamp_age_sec(now, self.latest_target_speed.header.stamp)
        if path_age > self.target_stale_sec or speed_age > self.target_stale_sec:
            return True
        if len(self.latest_target_path.points) < 2 or self.latest_target_path.confidence <= 0.0:
            return True
        return False

    def apply_stop_command(self, command: ActuatorCommand, hold_steering: bool) -> None:
        self.commanded_speed_mps = 0.0
        if hold_steering and self.latest_vehicle_state is not None:
            self.commanded_steering_rad = clamp(
                float(self.latest_vehicle_state.steering_rad),
                -self.max_steering_rad,
                self.max_steering_rad,
            )
        else:
            self.commanded_steering_rad = 0.0

        command.enable = True
        command.steering_target_rad = float(self.commanded_steering_rad)
        command.speed_target_mps = 0.0
        command.brake_engage = True
        command.gear = ActuatorCommand.GEAR_FORWARD

    def apply_nominal_command(self, command: ActuatorCommand, dt: float) -> None:
        assert self.latest_target_speed is not None
        target_speed = max(0.0, float(self.latest_target_speed.target_speed_mps))
        planned_stop = bool(self.latest_target_speed.need_stop)
        stop_target_distance = float(self.latest_target_speed.stop_target_distance)

        self.commanded_speed_mps = self.ramp_speed(self.commanded_speed_mps, target_speed, dt)
        self.commanded_steering_rad = self.compute_limited_steering(dt)
        actual_speed = (
            abs(float(self.latest_vehicle_state.speed_mps))
            if self.latest_vehicle_state is not None
            else abs(float(self.commanded_speed_mps))
        )
        hold_brake = (
            planned_stop
            and (
                (
                    target_speed <= self.stop_speed_epsilon_mps
                    and abs(float(self.commanded_speed_mps)) <= self.stop_speed_epsilon_mps
                    and actual_speed <= self.stop_speed_epsilon_mps
                )
                or stop_target_distance <= self.planned_stop_brake_distance_m
            )
        )

        command.enable = True
        command.steering_target_rad = float(self.commanded_steering_rad)
        command.speed_target_mps = 0.0 if hold_brake else float(self.commanded_speed_mps)
        command.brake_engage = hold_brake
        command.gear = ActuatorCommand.GEAR_FORWARD

    def ramp_speed(self, current: float, target: float, dt: float) -> float:
        if target >= current:
            return min(target, current + self.max_accel_mps2 * dt)
        return max(target, current - self.max_decel_mps2 * dt)

    def compute_limited_steering(self, dt: float) -> float:
        raw = self.compute_pure_pursuit_steering()
        max_delta = self.steering_rate_limit_radps * dt
        delta = clamp(raw - self.commanded_steering_rad, -max_delta, max_delta)
        return clamp(self.commanded_steering_rad + delta, -self.max_steering_rad, self.max_steering_rad)

    def compute_pure_pursuit_steering(self) -> float:
        if self.latest_target_path is None or not self.latest_target_path.points:
            return 0.0
        speed = self.latest_vehicle_state.speed_mps if self.latest_vehicle_state is not None else self.commanded_speed_mps
        lookahead = max(0.5, self.lookahead_base_m + max(0.0, float(speed)) * self.lookahead_gain_s)
        target = None
        for point in self.latest_target_path.points:
            if point.x >= lookahead:
                target = point
                break
        if target is None:
            forward = [point for point in self.latest_target_path.points if point.x > 0.1]
            target = forward[-1] if forward else None
        if target is None:
            return 0.0

        ld2 = max(0.25, target.x * target.x + target.y * target.y)
        curvature = 2.0 * target.y / ld2
        return math.atan(self.wheelbase_m * curvature)

    def publish_debug(self, command: ActuatorCommand, reason: str) -> None:
        msg = String()
        msg.data = (
            f"reason={reason} speed={command.speed_target_mps:.2f} "
            f"steer={command.steering_target_rad:.3f} brake={command.brake_engage}"
        )
        self.debug_pub.publish(msg)


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = ControlCommandGateNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
