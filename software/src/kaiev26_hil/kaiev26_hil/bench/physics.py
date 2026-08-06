"""물리/상태 계층.

통신부가 해석한 명령을 받아서
모터와 조향의 내부 상태를 시간에 따라 갱신한다.
"""

from __future__ import annotations

import math
import threading
from dataclasses import dataclass

from .can_io import MotorCommand, SteeringCommand


@dataclass(frozen=True)
class MotorSnapshot:
    """HMI와 통신부가 읽는 모터 상태 스냅샷."""

    handshake_done: bool
    target_rpm: float
    actual_rpm: float
    phase_limit_a: float
    bus_current_a: float
    phase_current_a: float
    bus_voltage_v: float
    ctrl_temp_c: float
    motor_temp_c: float
    ctrl_run: bool
    ctrl_speed_mode: bool
    error_group: int
    life: int


@dataclass(frozen=True)
class SteeringSnapshot:
    """HMI와 통신부가 읽는 조향 상태 스냅샷."""

    enabled: bool
    target_handle_deg: float
    handle_deg: float
    raw_target_handle_deg: float
    raw_handle_deg: float
    wheel_deg: float
    target_wheel_deg: float


class MotorPlant:
    """모터의 목표값, 실제값, 전류, 온도를 단순 모델로 계산한다."""

    RPM_OFFSET = 32000
    IA_OFFSET_A = -3200.0
    COMMAND_TIMEOUT_S = 2.0

    def __init__(
        self,
        *,
        gear_ratio: float = 4.6,
        wheel_radius_m: float = 0.235,
        vehicle_mass_kg: float = 220.0,
        rolling_resistance_coeff: float = 0.015,
        aero_drag_coeff: float = 0.35,
        road_grade_deg: float = 0.0,
        drivetrain_efficiency: float = 0.92,
        torque_per_amp_nm: float = 0.45,
        speed_gain_nm_per_rpm: float = 0.18,
    ) -> None:
        self.lock = threading.Lock()
        self.gear_ratio = gear_ratio
        self.wheel_radius_m = wheel_radius_m
        self.vehicle_mass_kg = vehicle_mass_kg
        self.rolling_resistance_coeff = rolling_resistance_coeff
        self.aero_drag_coeff = aero_drag_coeff
        self.road_grade_deg = road_grade_deg
        self.drivetrain_efficiency = drivetrain_efficiency
        self.torque_per_amp_nm = torque_per_amp_nm
        self.speed_gain_nm_per_rpm = speed_gain_nm_per_rpm
        self.reset()

    def reset(self) -> None:
        """초기 전원 인가 상태처럼 모터 상태를 되돌린다."""
        with self.lock:
            self.handshake_done = False
            self.target_rpm = 0.0
            self.actual_rpm = 0.0
            self.phase_limit_a = 0.0
            self.bus_current_a = 0.0
            self.phase_current_a = 0.0
            self.bus_voltage_v = 52.0
            self.ctrl_temp_c = 28.0
            self.motor_temp_c = 30.0
            self.ctrl_run = False
            self.ctrl_speed_mode = False
            self.life = 0
            self.error_group = 0
            self.last_cmd_time = 0.0
            self.vehicle_speed_mps = 0.0

    def apply_command(self, command: MotorCommand, now: float) -> None:
        """모터 command를 내부 상태에 반영한다."""
        with self.lock:
            if command.handshake_ack:
                # 0xAA ACK가 와야 실제 제어 명령을 받기 시작한다.
                self.handshake_done = True
                self.last_cmd_time = now
                return
            if not self.handshake_done:
                return
            self.phase_limit_a = command.phase_limit_a
            self.target_rpm = command.target_rpm
            self.ctrl_run = command.ctrl_run
            self.ctrl_speed_mode = command.ctrl_speed_mode
            self.life = command.life
            self.last_cmd_time = now

    def step(self, dt: float, now: float) -> None:
        """관성, 전류, 전압, 온도를 dt만큼 앞으로 진행한다."""
        with self.lock:
            if not self.handshake_done:
                return

            if now - self.last_cmd_time > self.COMMAND_TIMEOUT_S:
                # 제어 command가 끊기면 안전하게 목표 RPM을 놓는다.
                self.ctrl_run = False
                self.target_rpm = 0.0

            commanded = self.target_rpm if (self.ctrl_run and self.ctrl_speed_mode) else 0.0
            speed_error_rpm = commanded - self.actual_rpm

            max_torque_nm = abs(self.phase_limit_a) * self.torque_per_amp_nm
            requested_torque_nm = speed_error_rpm * self.speed_gain_nm_per_rpm
            drive_torque_nm = max(-max_torque_nm, min(max_torque_nm, requested_torque_nm))

            drive_force_n = 0.0
            if self.wheel_radius_m > 1e-6:
                drive_force_n = (
                    drive_torque_nm * self.gear_ratio * self.drivetrain_efficiency / self.wheel_radius_m
                )

            rolling_force_n = (
                self.vehicle_mass_kg * 9.81 * self.rolling_resistance_coeff * self._sign(self.vehicle_speed_mps)
            )
            aerodynamic_force_n = self.aero_drag_coeff * self.vehicle_speed_mps * abs(self.vehicle_speed_mps)
            grade_force_n = self.vehicle_mass_kg * 9.81 * math.sin(math.radians(self.road_grade_deg))
            net_force_n = drive_force_n - rolling_force_n - aerodynamic_force_n - grade_force_n
            acceleration_mps2 = net_force_n / max(self.vehicle_mass_kg, 1e-6)

            prev_speed = self.vehicle_speed_mps
            self.vehicle_speed_mps += acceleration_mps2 * dt
            if prev_speed > 0.0 > self.vehicle_speed_mps:
                self.vehicle_speed_mps = 0.0
            elif prev_speed < 0.0 < self.vehicle_speed_mps:
                self.vehicle_speed_mps = 0.0

            self.actual_rpm = self._vehicle_speed_to_motor_rpm(self.vehicle_speed_mps)
            if abs(commanded) < 1.0 and abs(self.actual_rpm) < 3.0:
                self.actual_rpm = 0.0
                self.vehicle_speed_mps = 0.0

            # 전류와 온도는 실제로 걸린 구동 토크 기준 1차 응답으로 근사한다.
            target_phase_a = 0.0
            if self.torque_per_amp_nm > 1e-6:
                target_phase_a = drive_torque_nm / self.torque_per_amp_nm
            self.phase_current_a += 0.25 * (target_phase_a - self.phase_current_a)
            self.bus_current_a += 0.22 * ((0.55 * self.phase_current_a) - self.bus_current_a)
            self.bus_voltage_v = 52.0 - min(3.5, abs(self.bus_current_a) * 0.006)

            self.ctrl_temp_c += ((0.020 * abs(self.bus_current_a)) - 0.18 * (self.ctrl_temp_c - 27.0)) * dt
            self.motor_temp_c += ((0.012 * abs(self.phase_current_a)) - 0.10 * (self.motor_temp_c - 28.0)) * dt

            self.error_group = 0
            if self.ctrl_temp_c >= 80.0:
                self.error_group |= 1 << 4
            if self.motor_temp_c >= 130.0:
                self.error_group |= 1 << 5

    def _vehicle_speed_to_motor_rpm(self, vehicle_speed_mps: float) -> float:
        wheel_rps = vehicle_speed_mps / max(self.wheel_radius_m, 1e-6) / (2.0 * math.pi)
        return wheel_rps * 60.0 * self.gear_ratio

    @staticmethod
    def _sign(value: float) -> float:
        if value > 1e-6:
            return 1.0
        if value < -1e-6:
            return -1.0
        return 0.0

    def snapshot(self) -> MotorSnapshot:
        """현재 모터 상태를 불변 스냅샷으로 복사한다."""
        with self.lock:
            return MotorSnapshot(
                handshake_done=self.handshake_done,
                target_rpm=self.target_rpm,
                actual_rpm=self.actual_rpm,
                phase_limit_a=self.phase_limit_a,
                bus_current_a=self.bus_current_a,
                phase_current_a=self.phase_current_a,
                bus_voltage_v=self.bus_voltage_v,
                ctrl_temp_c=self.ctrl_temp_c,
                motor_temp_c=self.motor_temp_c,
                ctrl_run=self.ctrl_run,
                ctrl_speed_mode=self.ctrl_speed_mode,
                error_group=self.error_group,
                life=self.life,
            )


class SteeringPlant:
    """조향 enable 상태와 목표 핸들각을 기반으로 조향 응답을 계산한다."""

    FEEDBACK_PERIOD_S = 0.02

    def __init__(
        self,
        *,
        max_handle_rate_deg_s: float = 160.0,
        backlash_deg: float = 0.8,
        static_friction_deg: float = 0.2,
        position_gain: float = 12.0,
        velocity_damping: float = 2.5,
    ) -> None:
        self.lock = threading.Lock()
        self.max_handle_rate_deg_s = max_handle_rate_deg_s
        self.backlash_deg = backlash_deg
        self.static_friction_deg = static_friction_deg
        self.position_gain = position_gain
        self.velocity_damping = velocity_damping
        self.reset()

    def reset(self) -> None:
        """조향 상태를 disable/0 deg로 초기화한다."""
        with self.lock:
            self.enabled = False
            self.target_handle_deg = 0.0
            self.handle_deg = 0.0
            self.handle_velocity_deg_s = 0.0

    def apply_command(self, command: SteeringCommand) -> None:
        """enable 명령 또는 목표 핸들각 명령을 반영한다."""
        with self.lock:
            if command.enable is not None:
                self.enabled = command.enable
                return
            if command.target_handle_deg is not None and self.enabled:
                # enable 이후에만 position command를 받는다.
                self.target_handle_deg = command.target_handle_deg

    def step(self, dt: float) -> None:
        """최대 조향 속도 제한을 둔 1차 응답으로 핸들각을 갱신한다."""
        with self.lock:
            target = self.target_handle_deg if self.enabled else 0.0
            error = target - self.handle_deg
            if abs(error) <= self.backlash_deg:
                effective_error = 0.0
            else:
                effective_error = math.copysign(abs(error) - self.backlash_deg, error)

            if abs(effective_error) <= self.static_friction_deg and abs(self.handle_velocity_deg_s) < self.static_friction_deg:
                desired_velocity = 0.0
            else:
                desired_velocity = self.position_gain * effective_error

            desired_velocity = max(
                -self.max_handle_rate_deg_s,
                min(self.max_handle_rate_deg_s, desired_velocity),
            )

            self.handle_velocity_deg_s += 0.35 * (desired_velocity - self.handle_velocity_deg_s)
            self.handle_velocity_deg_s *= max(0.0, 1.0 - self.velocity_damping * dt)
            self.handle_velocity_deg_s = max(
                -self.max_handle_rate_deg_s,
                min(self.max_handle_rate_deg_s, self.handle_velocity_deg_s),
            )
            self.handle_deg += self.handle_velocity_deg_s * dt

            if abs(self.handle_deg) < 0.05 and abs(target) < 0.05 and abs(self.handle_velocity_deg_s) < 0.05:
                self.handle_deg = 0.0
                self.handle_velocity_deg_s = 0.0

    def snapshot(self) -> SteeringSnapshot:
        """현재 조향 상태를 불변 스냅샷으로 복사한다."""
        with self.lock:
            raw_handle_deg = self.handle_deg
            raw_target_handle_deg = self.target_handle_deg
            # GUI에서는 handle과 wheel이 같은 부호 규약을 쓰도록 정규화해서 보여준다.
            handle_deg = self.raw_handle_to_display_deg(raw_handle_deg)
            target_handle_deg = self.raw_handle_to_display_deg(raw_target_handle_deg)
            return SteeringSnapshot(
                enabled=self.enabled,
                target_handle_deg=target_handle_deg,
                handle_deg=handle_deg,
                raw_target_handle_deg=raw_target_handle_deg,
                raw_handle_deg=raw_handle_deg,
                wheel_deg=self.handle_to_wheel_deg(raw_handle_deg),
                target_wheel_deg=self.handle_to_wheel_deg(raw_target_handle_deg),
            )

    @staticmethod
    def handle_to_wheel_deg(handle_deg: float) -> float:
        """핸들각을 기존 제어 코드가 쓰는 바퀴각 근사식으로 바꾼다."""
        x = -handle_deg
        magnitude = abs(x)
        if magnitude < 1e-6:
            return 0.0
        wheel_deg = (0.0002 * magnitude + 0.2159) * magnitude + 0.1612
        return math.copysign(wheel_deg, x)

    @staticmethod
    def raw_handle_to_display_deg(handle_deg: float) -> float:
        """raw handle 부호를 GUI 표시용 부호 규약으로 뒤집는다."""
        return -handle_deg
