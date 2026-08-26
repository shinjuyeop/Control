"""Runtime 조립 계층.

통신부와 물리부를 연결하고,
주기 루프에서 상태 frame 송신과 physics step을 함께 돌린다.
"""

from __future__ import annotations

import logging
import threading
import time
from dataclasses import dataclass
from typing import Callable, List, Tuple

from .can_io import (
    BusHealthSnapshot,
    CanFrame,
    SocketCANBusHub,
    encode_motor_handshake_frame,
    encode_motor_status_frames,
    encode_steer_feedback_frame,
    make_motor_cmd_frame,
    make_motor_handshake_ack,
    make_steer_enable_frame,
    make_steer_pos_frame_from_wheel_deg,
    parse_motor_command,
    parse_steering_command,
)
from .physics import MotorPlant, MotorSnapshot, SteeringPlant, SteeringSnapshot


LOGGER = logging.getLogger(__name__)


@dataclass(frozen=True)
class RuntimeDiagnosticsSnapshot:
    """runtime과 bus 계층의 누적 진단 정보."""

    listener_errors: int
    bus_health: BusHealthSnapshot | None


class VcanSimRuntime:
    """vcan simulator 전체를 하나로 묶는 runtime."""

    LOOP_PERIOD_S = 0.01
    MOTOR_HANDSHAKE_PERIOD_S = 0.50
    MOTOR_STATUS_PERIOD_S = 0.05
    STEER_FEEDBACK_PERIOD_S = 0.02

    def __init__(self, motor_iface: str, steer_iface: str) -> None:
        """버스, 물리 모델, 주기 thread를 모두 초기화한다."""
        self.motor_iface = motor_iface
        self.steer_iface = steer_iface
        self.hub = SocketCANBusHub([motor_iface, steer_iface])
        self.motor = MotorPlant()
        self.steer = SteeringPlant()
        self._running = True
        self._frame_callbacks: List[Callable[[CanFrame], None]] = []
        self._callback_lock = threading.Lock()
        self._listener_errors = 0
        self._last_listener_error_log_monotonic = 0.0
        self._last_motor_handshake_emit = 0.0
        self._last_motor_status_emit = 0.0
        self._last_steer_feedback_emit = 0.0

        # 두 버스 모두 같은 runtime이 받아서 각 plant로 라우팅한다.
        self.hub.subscribe(self.motor_iface, self._on_frame)
        self.hub.subscribe(self.steer_iface, self._on_frame)

        self._thread = threading.Thread(target=self._loop, daemon=True)
        self._thread.start()

    def close(self) -> None:
        """background loop와 CAN 소켓을 정리한다."""
        self._running = False
        self._thread.join(timeout=0.5)
        self.hub.close()

    def add_frame_listener(self, callback: Callable[[CanFrame], None]) -> None:
        """HMI처럼 raw frame 로그가 필요한 소비자를 등록한다."""
        with self._callback_lock:
            self._frame_callbacks.append(callback)

    def send_frame(self, frame: CanFrame) -> None:
        """임의 raw frame 송신용 공용 진입점."""
        self.hub.send(frame)

    def send_motor_ack(self) -> None:
        """모터 handshake ACK frame을 보낸다."""
        self.send_frame(make_motor_handshake_ack(self.motor_iface))

    def send_motor_rpm(self, rpm: int) -> None:
        """모터 목표 RPM command를 보낸다."""
        self.send_frame(make_motor_cmd_frame(self.motor_iface, rpm))

    def send_steer_enabled(self, enable: bool) -> None:
        """조향 enable/disable command를 보낸다."""
        self.send_frame(make_steer_enable_frame(self.steer_iface, enable))

    def send_steer_wheel_deg(self, wheel_deg: float) -> None:
        """바퀴각 기준 조향 목표 command를 보낸다."""
        self.send_frame(make_steer_pos_frame_from_wheel_deg(self.steer_iface, wheel_deg))

    def reset(self) -> None:
        """모터/조향 plant 상태를 함께 초기화한다."""
        self.motor.reset()
        self.steer.reset()

    def get_snapshots(self) -> Tuple[MotorSnapshot, SteeringSnapshot]:
        """HMI가 읽을 현재 상태 스냅샷을 돌려준다."""
        return self.motor.snapshot(), self.steer.snapshot()

    def get_diagnostics(self) -> RuntimeDiagnosticsSnapshot:
        """HMI와 테스트가 읽는 진단 스냅샷."""
        health_snapshot = None
        if hasattr(self.hub, "health_snapshot"):
            health_snapshot = self.hub.health_snapshot()
        return RuntimeDiagnosticsSnapshot(
            listener_errors=self._listener_errors,
            bus_health=health_snapshot,
        )

    def run_headless_demo(self, duration_s: float) -> None:
        """GUI 없이도 바로 동작 확인이 되게 기본 시나리오를 실행한다."""
        stop_time = time.monotonic() + duration_s
        time.sleep(0.6)
        self.send_motor_ack()
        self.send_motor_rpm(1200)
        self.send_steer_enabled(True)
        time.sleep(0.2)
        self.send_steer_wheel_deg(12.0)
        while time.monotonic() < stop_time:
            time.sleep(0.05)

    def _on_frame(self, frame: CanFrame) -> None:
        """수신 frame을 각 plant 명령으로 해석하고, 로그 구독자에게도 전달한다."""
        now = time.monotonic()
        motor_command = parse_motor_command(frame)
        if motor_command is not None:
            self.motor.apply_command(motor_command, now)

        steering_command = parse_steering_command(frame)
        if steering_command is not None:
            self.steer.apply_command(steering_command)

        # HMI 로그는 plant 처리와 별개로 같은 raw frame을 그대로 본다.
        with self._callback_lock:
            callbacks = list(self._frame_callbacks)
        for callback in callbacks:
            try:
                callback(frame)
            except Exception as exc:
                self._listener_errors += 1
                now_log = time.monotonic()
                if now_log - self._last_listener_error_log_monotonic >= 2.0:
                    LOGGER.warning(
                        "Runtime frame listener raised; listener_errors=%d: %s",
                        self._listener_errors,
                        exc,
                    )
                    self._last_listener_error_log_monotonic = now_log

    def _loop(self) -> None:
        """physics step과 상태 frame 송신을 주기적으로 돌리는 메인 loop."""
        last = time.monotonic()
        next_tick = last
        while self._running:
            now = time.monotonic()
            dt = min(0.05, max(0.001, now - last))
            last = now

            # 물리 상태를 먼저 진행시킨 뒤, 그 결과를 status/feedback으로 내보낸다.
            self.motor.step(dt, now)
            self.steer.step(dt)

            motor_snapshot, steer_snapshot = self.get_snapshots()
            if not motor_snapshot.handshake_done and now - self._last_motor_handshake_emit >= self.MOTOR_HANDSHAKE_PERIOD_S:
                self._last_motor_handshake_emit = now
                self.hub.send(encode_motor_handshake_frame(self.motor_iface))

            if motor_snapshot.handshake_done and now - self._last_motor_status_emit >= self.MOTOR_STATUS_PERIOD_S:
                self._last_motor_status_emit = now
                # 모터는 status1/status2 두 frame을 같이 보낸다.
                for frame in encode_motor_status_frames(
                    self.motor_iface,
                    motor_snapshot.bus_voltage_v,
                    motor_snapshot.bus_current_a,
                    motor_snapshot.phase_current_a,
                    motor_snapshot.actual_rpm,
                    motor_snapshot.ctrl_run,
                    motor_snapshot.ctrl_speed_mode,
                    motor_snapshot.ctrl_temp_c,
                    motor_snapshot.motor_temp_c,
                    motor_snapshot.error_group,
                ):
                    self.hub.send(frame)

            if now - self._last_steer_feedback_emit >= self.STEER_FEEDBACK_PERIOD_S:
                self._last_steer_feedback_emit = now
                # CAN 피드백은 기존 raw 부호 규약을 유지한다.
                self.hub.send(encode_steer_feedback_frame(self.steer_iface, steer_snapshot.raw_handle_deg))

            next_tick += self.LOOP_PERIOD_S
            sleep_s = next_tick - time.monotonic()
            if sleep_s > 0.0:
                time.sleep(sleep_s)
            else:
                next_tick = time.monotonic()
