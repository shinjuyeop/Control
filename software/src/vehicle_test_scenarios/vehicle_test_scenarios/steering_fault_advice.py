"""Display-only KEYA/Host steering recovery advice for the field console.

Bit meanings/reset categories: KEYA KY170DJD01005-08G-V1 manual, section 5.2,
pages 19–20. A ROS node restart does not reset a latched fault in the driver.
"""

from dataclasses import dataclass


@dataclass(frozen=True)
class SteeringFaultAdvice:
    cause: str
    action: str


# Keep undervoltage first: this was the measured cause of the 2026-09-19 run stop.
# The manual's voltage thresholds are configurable; do not assert live settings.
_KEYA_FAULTS = (
    (4, '저전압',
     '정지·MANUAL에서 KEYA 전원/DC-DC·릴레이·커넥터·접지를 확인하고 전압 강하를 '
     '측정하세요. 표시 전압은 순간 강하를 놓칠 수 있습니다. 전원 정상화 후 '
     'Disable reset 절차와 새 fault 응답을 확인하세요.'),
    (1, '과전압',
     'KEYA 공급 전압·전원 설정을 확인하세요. 원인이 해소되면 드라이버 self-reset '
     '대상이므로 새 fault 응답으로 해제를 확인하세요.'),
    (2, '하드웨어 과전류',
     '정지·MANUAL에서 모터 상선 단락·배선·드라이버를 점검하세요. 원인 제거 후 '
     'KEYA 전원을 재시작하고 fault 해제를 확인하세요.'),
    (3, 'EEPROM 오류',
     'KEYA 설정 저장 상태를 확인하세요. 정지·MANUAL에서 KEYA 전원을 재시작하고 '
     '재발하면 설정 도구와 드라이버를 점검하세요.'),
    (5, 'Reserved 비트',
     '현재 Host는 이 비트도 고장으로 처리합니다. 원본 CAN fault word와 '
     '장치 firmware의 비트 정의를 확인하세요.'),
    (6, '드라이버 내부 과전류',
     '정지·MANUAL에서 조향 걸림·모터 부하·KEYA 전류 설정을 확인하세요. 원인 제거 후 '
     '드라이버 Disable reset 절차와 새 fault 응답을 확인하세요.'),
    (7, '제어모드 오류',
     'KEYA 제어방식 CAN 및 작업모드가 Host 설정과 같은지 확인하세요 '
     '(torque: 0020=2, position: 0020=3). 수정 후 정지·MANUAL에서 KEYA 전원을 '
     '재시작하세요.'),
    (8, '모터 결상',
     '정지·MANUAL에서 모터 상선·커넥터와 모터/드라이버 전압 사양을 확인하세요. '
     '원인 제거 후 KEYA 전원을 재시작하세요.'),
    (9, 'Stall 보호',
     '정지·MANUAL에서 기구 걸림·끝단 접촉·모터 부하를 확인하세요. 원인 제거 후 '
     '드라이버 Disable reset 절차로 복구하세요. 게인·전류 증가로 우회하지 마세요.'),
    (10, '드라이버 과열',
     '출력을 멈추고 냉각·방열·지속 부하를 확인하세요. 매뉴얼은 85°C 초과 보호와 '
     '75°C 회복을 명시합니다. 온도 회복 뒤 새 fault 응답을 확인하세요.'),
    (11, 'Hall 센서 오류',
     '정지·MANUAL에서 Hall 센서 전원·케이블·커넥터를 확인하세요. 원인 제거 후 '
     '드라이버 Disable reset 절차와 새 fault 응답을 확인하세요.'),
    (12, '내부 전류센서 오류',
     '정지·MANUAL에서 KEYA 전원을 재시작해 확인하고, 재발하면 드라이버 점검이 '
     '필요합니다.'),
    (13, 'Spare 비트',
     '현재 Host는 이 비트도 고장으로 처리합니다. 원본 CAN fault word와 '
     '장치 firmware의 비트 정의를 확인하세요.'),
    (14, '드라이버 CAN 신호 단절',
     'CAN 전원·배선·종단·인터페이스를 복구하세요. 매뉴얼은 Enable reset을 '
     '요구합니다. Host steering/safety 진단에서 bounded 자동복구 횟수와 '
     '결과를 확인하세요. HARD FAULT 승격 시 별도 드라이버 복구 점검이 필요합니다.'),
    (15, '드라이버 RS232 신호 단절',
     '현재 CAN 운용에서는 INFO이며 이 비트만으로 Host가 차단하지 않습니다. '
     'KEYA 제어방식 설정을 확인하세요.'),
)


def steering_fault_advice(status, *, fresh=True) -> tuple[SteeringFaultAdvice, ...]:
    """Return cause/action entries for a SteeringActuatorStatus, without I/O.

    ``fresh`` is the caller's existing ROS receipt-freshness decision. CAN loss
    also makes fault bits potentially cached. Bit 0 alone is normal Disable and
    produces no fault advice unless an independent Host/link fault is reported.
    """
    if status is None:
        return (SteeringFaultAdvice(
            '조향 상태 미수신',
            'steering_actuator_node 실행과 /steering/status 수신을 확인하세요. '
            '노드가 없으면 차량 launch 로그의 시작 오류를 확인하세요.'),)

    fault_bits = status.fault_bits & ~1
    can_lost = status.state == 6 or status.can_timeout or not status.communication_ok
    cached = not fresh or can_lost
    advice = []
    for bit, label, action in _KEYA_FAULTS:
        mask = 1 << bit
        if fault_bits & mask:
            suffix = ' · 마지막 수신값, 현재 고장 여부 미확인' if cached else ''
            advice.append(SteeringFaultAdvice(
                f'KEYA {label} (bit {bit}, 0x{mask:04X}){suffix}', action))

    if not fresh:
        advice.append(SteeringFaultAdvice(
            '조향 상태 수신 중단',
            '마지막 상태와 fault 비트는 캐시입니다. steering_actuator_node와 '
            '/steering/status 수신을 먼저 복구하고 새 상태로 고장 해제를 확인하세요.'))
    elif can_lost:
        advice.append(SteeringFaultAdvice(
            '조향 CAN 연결/송수신 실패',
            'KEYA 전원·can0 상태·CAN 배선/종단을 확인하세요. fault 비트는 캐시일 수 '
            '있습니다. Host가 backoff로 SocketCAN reopen을 시도합니다. can0 복구 후 '
            '새 fault·위치 응답과 steering/safety 진단을 확인한 뒤 '
            '유효한 새 명령이 필요합니다.'))
    elif status.state == 7 or status.feedback_timeout or not status.feedback_valid:
        advice.append(SteeringFaultAdvice(
            '조향 피드백 상실/제어주기 지연',
            'encoder 응답·feedback_age_ms와 /steering/torque_safety, 노드 로그의 '
            'control 지연을 확인하세요. 정상 피드백과 유효한 새 명령이면 Host는 '
            '복귀합니다. timeout을 늘리기 전에 수신/지연 원인을 해결하세요.'))

    # STATE_FAULT / REASON_MOTOR_FAULT with no actual KEYA bit can come from an
    # older Host binary. It must not be mislabeled as hardware stall or bit0 fault.
    if not fault_bits and (status.state == 8 or status.disable_reason == 10):
        advice.append(SteeringFaultAdvice(
            'Host FAULT 표시 · KEYA 실제 고장 비트 없음',
            'bit0은 정상 Disable입니다. /steering/torque_safety와 노드 로그, 실행 '
            '버전을 확인하세요. 하드웨어 고장으로 단정하거나 무조건 재시작하지 마세요.'))
    elif not fault_bits and status.state not in range(9):
        advice.append(SteeringFaultAdvice(
            f'알 수 없는 조향 상태 ({status.state})',
            '원본 /steering/status와 노드 로그, 메시지/실행 버전의 일치 여부를 '
            '확인하세요. 상태를 확인한 뒤 시험을 다시 시작하세요.'))
    return tuple(advice)
