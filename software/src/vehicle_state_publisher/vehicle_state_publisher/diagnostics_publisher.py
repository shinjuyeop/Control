"""kaiev26 status 세 개를 diagnostic_msgs로 요약해 /diagnostics로 낸다.

Console과 Foxglove의 공통 진단 화면이 `DiagnosticArray`를 읽을 수 있도록
canonical status를 변환한다.

판정 임계값은 docs/SAFETY.md의 timeout 표를 기준으로 한다. 실제 제어 timeout이
있는 항목은 같은 경계를 표시하고, brake command age는 관측 기준이다.
WARN은 각 기준의 WARN_RATIO 지점, ERROR는 기준에 둔다.
"""

from __future__ import annotations

import math

from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from kaiev26_msgs.msg import (
    BrakeActuatorStatus,
    DriveActuatorStatus,
    SteeringActuatorStatus,
)
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

OK = DiagnosticStatus.OK
WARN = DiagnosticStatus.WARN
ERROR = DiagnosticStatus.ERROR
STALE = DiagnosticStatus.STALE

# docs/SAFETY.md의 timeout 표. 값을 바꿀 때는 그 문서를 함께 갱신한다.
LIMIT_MS = {
    'drive_command': 500.0,      # AUTO traction. 초과 시 HALTED, 저장 속도 0
    'ezkontrol_part': 200.0,     # Part I/II. 초과 시 status valid false
    'steer_command': 200.0,      # ACTIVE 조향. 초과 시 KEYA Disable
    'keya_feedback': 500.0,      # KEYA CAN·position
    'steer_mode': 1500.0,        # 조향이 보는 /drive/status
    'brake_command': 500.0,      # Observation only; STM_B has no Host timeout action.
}
WARN_RATIO = 0.6

# status 자체가 끊긴 것과 status 안의 age가 늘어난 것은 다른 고장이다.
STATUS_STALE_SEC = 1.0

STEER_STATE = {
    0: 'INIT', 1: 'DISABLED', 2: 'WAIT_CMD', 3: 'ACTIVE', 4: 'ESTOP',
    5: 'BRAKE', 6: 'CAN_LOST', 7: 'FEEDBACK_LOST', 8: 'FAULT',
}
STEER_STATE_LEVEL = {
    3: OK, 0: WARN, 1: WARN, 2: WARN, 5: WARN,
    4: ERROR, 6: ERROR, 7: ERROR, 8: ERROR,
}
STEER_REASON = {
    0: 'NONE', 1: 'MODE_NOT_RECEIVED', 2: 'MODE_TIMEOUT', 3: 'MANUAL_MODE',
    4: 'INVALID_MODE', 5: 'ESTOP', 6: 'COMMAND_BRAKE',
    7: 'STEER_COMMAND_TIMEOUT', 8: 'CAN_TIMEOUT', 9: 'FEEDBACK_INVALID',
    10: 'MOTOR_FAULT',
}
STOP_REASON_BITS = {
    1: 'COMMAND_BRAKE', 4: 'ESTOP', 8: 'COMMAND_INVALID',
}


def age_level(age_ms: float, limit_ms: float) -> tuple[int, str]:
    """age 하나를 등급과 사람이 읽을 문구로 바꾼다."""
    if math.isinf(age_ms) or math.isnan(age_ms):
        return ERROR, f'미수신 (한계 {limit_ms:.0f} ms)'
    if age_ms >= limit_ms:
        return ERROR, f'{age_ms:.0f} ms ≥ {limit_ms:.0f} ms'
    if age_ms >= limit_ms * WARN_RATIO:
        return WARN, f'{age_ms:.0f} ms (한계 {limit_ms:.0f} ms에 근접)'
    return OK, f'{age_ms:.0f} ms'


class DiagnosticsPublisher(Node):
    def __init__(self) -> None:
        super().__init__('diagnostics_publisher')

        self.declare_parameter('publish_rate_hz', 1.0)
        rate_hz = max(0.2, float(self.get_parameter('publish_rate_hz').value))

        self._drive: DriveActuatorStatus | None = None
        self._steering: SteeringActuatorStatus | None = None
        self._brake_msg: BrakeActuatorStatus | None = None
        self._drive_sec: float | None = None
        self._steering_sec: float | None = None
        self._brake_sec: float | None = None

        self.create_subscription(
            DriveActuatorStatus, '/drive/status', self._on_drive, 10)
        self.create_subscription(
            SteeringActuatorStatus, '/steering/status', self._on_steering, 10)
        self.create_subscription(
            BrakeActuatorStatus, '/brake/status', self._on_brake, 10)

        self.pub = self.create_publisher(DiagnosticArray, '/diagnostics', 10)
        self.create_timer(1.0 / rate_hz, self._publish)

    def _now_sec(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def _on_drive(self, msg: DriveActuatorStatus) -> None:
        self._drive, self._drive_sec = msg, self._now_sec()

    def _on_steering(self, msg: SteeringActuatorStatus) -> None:
        self._steering, self._steering_sec = msg, self._now_sec()

    def _on_brake(self, msg: BrakeActuatorStatus) -> None:
        self._brake_msg, self._brake_sec = msg, self._now_sec()

    def _stale(self, name: str, topic: str, last_sec: float | None) -> DiagnosticStatus | None:
        """status 자체가 안 오는 경우. 이때 내부 필드는 판정하지 않는다."""
        if last_sec is None:
            return self._status(name, topic, STALE, f'{topic} 미수신', [])
        gap = self._now_sec() - last_sec
        if gap > STATUS_STALE_SEC:
            return self._status(
                name, topic, STALE, f'{topic} {gap:.1f} s 동안 끊김', [])
        return None

    @staticmethod
    def _status(name, hardware_id, level, message, values) -> DiagnosticStatus:
        s = DiagnosticStatus()
        s.name = name
        s.hardware_id = hardware_id
        s.level = level
        s.message = message
        s.values = [KeyValue(key=k, value=v) for k, v in values]
        return s

    @staticmethod
    def _worst(checks: list[tuple[str, int, str]]) -> tuple[int, str]:
        """가장 나쁜 항목을 요약 문구로 올린다. 접힌 상태에서 이 줄만 보인다."""
        level = max((c[1] for c in checks), default=OK)
        if level == OK:
            return OK, '정상'
        bad = [f'{k}: {v}' for k, lv, v in checks if lv == level]
        return level, ' / '.join(bad[:3])

    def _drive_link(self) -> DiagnosticStatus:
        stale = self._stale('구동 링크', '/drive/status', self._drive_sec)
        if stale:
            return stale
        m = self._drive
        checks = []
        lv, txt = age_level(m.command_age_ms, LIMIT_MS['drive_command'])
        checks.append(('Host command age', lv, txt))
        if m.command_timeout:
            checks.append(('command_timeout', ERROR, 'true — AUTO traction HALTED'))
        for label, age, valid in (
            ('EZkontrol Part I', m.part1_age_ms, m.part1_valid),
            ('EZkontrol Part II', m.part2_age_ms, m.part2_valid),
        ):
            lv, txt = age_level(age, LIMIT_MS['ezkontrol_part'])
            if not valid:
                lv, txt = ERROR, f'valid=false ({txt})'
            checks.append((f'{label} age', lv, txt))
        level, msg = self._worst(checks)
        return self._status('구동 링크', 'STM_A / EZkontrol', level, msg,
                            [(k, v) for k, _, v in checks])

    def _drive_state(self) -> DiagnosticStatus:
        stale = self._stale('구동 상태', '/drive/status', self._drive_sec)
        if stale:
            return stale
        m = self._drive
        checks = [
            ('mode', OK, 'AUTO' if m.mode else 'MANUAL'),
            ('gear', OK, 'REVERSE' if m.gear else 'FORWARD'),
            ('control mode', OK, 'speed' if m.controller_control_mode else 'torque'),
        ]
        # 지령과 피드백이 어긋나는 것이 이 그룹의 핵심 신호다.
        if not m.tractive_feedback_valid:
            checks.append(('tractive 피드백', WARN, 'Part II 미수신으로 판정 불가'))
        elif m.tractive_commanded != m.tractive_feedback:
            checks.append((
                'tractive 지령/피드백', ERROR,
                f'지령={m.tractive_commanded} 피드백={m.tractive_feedback} 불일치'))
        else:
            checks.append(('tractive', OK, 'RUN' if m.tractive_commanded else 'STOP'))
        if m.stop_request:
            checks.append(('stop_request', WARN, 'true'))
        if not m.fault_bits_valid:
            checks.append(('fault_bits', WARN, 'Part II 미수신으로 판정 불가'))
        elif m.fault_bits:
            checks.append(('fault_bits', WARN, f'0x{m.fault_bits:06X}'))
        level, msg = self._worst(checks)
        return self._status('구동 상태', 'STM_A / EZkontrol', level, msg,
                            [(k, v) for k, _, v in checks])

    def _steer_link(self) -> DiagnosticStatus:
        stale = self._stale('조향 링크', '/steering/status', self._steering_sec)
        if stale:
            return stale
        m = self._steering
        checks = []
        for label, age, key in (
            ('KEYA feedback age', m.feedback_age_ms, 'keya_feedback'),
            ('조향 command age', m.steer_command_age_ms, 'steer_command'),
            ('mode age', m.mode_age_ms, 'steer_mode'),
        ):
            lv, txt = age_level(age, LIMIT_MS[key])
            # 조향을 아직 쓰지 않는 구간에서는 command 미수신이 정상 운용이다.
            if key == 'steer_command' and math.isinf(age) and m.state != 3:
                lv, txt = WARN, '미수신 (조향 비활성 중)'
            checks.append((label, lv, txt))
        if not m.communication_ok:
            checks.append(('communication_ok', ERROR, 'false'))
        if m.can_timeout:
            checks.append(('can_timeout', ERROR, 'true'))
        if not m.feedback_valid:
            checks.append(('feedback_valid', ERROR, 'false'))
        level, msg = self._worst(checks)
        return self._status('조향 링크', 'KEYA / can0', level, msg,
                            [(k, v) for k, _, v in checks])

    def _steer_state(self) -> DiagnosticStatus:
        stale = self._stale('조향 상태', '/steering/status', self._steering_sec)
        if stale:
            return stale
        m = self._steering
        state_name = STEER_STATE.get(m.state, f'알 수 없음({m.state})')
        reason_name = STEER_REASON.get(
            m.disable_reason, f'알 수 없음({m.disable_reason})')
        state_level = STEER_STATE_LEVEL.get(m.state, ERROR)
        detail = state_name if m.state == 3 else f'{state_name} / {reason_name}'
        checks = [('state', state_level, detail)]
        if m.tracking_error_fault:
            checks.append(('tracking', ERROR, '추종 오차 fault'))
        elif m.tracking_error_warning:
            checks.append(('tracking', WARN, '추종 오차 경고'))
        checks.append(
            ('tracking error', OK, f'{m.tracking_error_rad:+.4f} rad'))
        level, msg = self._worst(checks)
        return self._status('조향 상태', 'KEYA', level, msg,
                            [(k, v) for k, _, v in checks])

    def _brake_group(self) -> DiagnosticStatus:
        stale = self._stale('브레이크', '/brake/status', self._brake_sec)
        if stale:
            return stale
        m = self._brake_msg
        checks = []
        lv, txt = age_level(m.command_age_ms, LIMIT_MS['brake_command'])
        checks.append(('command age', lv, txt))
        bits = m.stop_reason_bits
        if bits:
            names = [n for b, n in STOP_REASON_BITS.items() if bits & b]
            # 물리 E-stop은 지령 제동과 등급을 나눈다.
            lv = ERROR if bits & 4 else WARN
            checks.append(('stop_reason_bits', lv, f'{bits} ({" | ".join(names)})'))
        checks.append(
            ('brake_commanded', OK, 'true' if m.brake_commanded else 'false'))
        level, msg = self._worst(checks)
        return self._status('브레이크', 'STM_B', level, msg,
                            [(k, v) for k, _, v in checks])

    def _publish(self) -> None:
        array = DiagnosticArray()
        array.header.stamp = self.get_clock().now().to_msg()
        array.status = [
            self._drive_link(),
            self._drive_state(),
            self._steer_link(),
            self._steer_state(),
            self._brake_group(),
        ]
        self.pub.publish(array)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = DiagnosticsPublisher()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
