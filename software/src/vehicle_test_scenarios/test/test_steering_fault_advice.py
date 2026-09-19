"""Operator advice for recorded KEYA faults and Host/link failure distinctions."""

from types import SimpleNamespace

import pytest

from vehicle_test_scenarios.steering_fault_advice import steering_fault_advice


def status(**changes):
    values = dict(fault_bits=0, state=3, disable_reason=0,
                  communication_ok=True, can_timeout=False,
                  feedback_valid=True, feedback_timeout=False)
    values.update(changes)
    return SimpleNamespace(**values)


@pytest.mark.parametrize('bits', (0, 1))
def test_normal_disable_does_not_create_hardware_fault_advice(bits):
    assert not steering_fault_advice(status(fault_bits=bits, state=1))


def test_recorded_undervoltage_is_decoded_before_other_hardware_bits():
    advice = steering_fault_advice(status(
        fault_bits=0x0013, state=8, disable_reason=10))
    assert len(advice) == 2
    assert '저전압' in advice[0].cause
    assert '과전압' in advice[1].cause
    assert '전압 강하' in advice[0].action
    assert 'Disable reset' in advice[0].action


def test_undervoltage_remains_visible_after_can_loss_with_cache_qualification():
    advice = steering_fault_advice(status(
        fault_bits=16, state=6, can_timeout=True, communication_ok=False,
        feedback_valid=False, feedback_timeout=True))
    assert len(advice) == 2
    assert '저전압' in advice[0].cause
    assert '마지막 수신값' in advice[0].cause
    assert 'CAN' in advice[1].cause
    assert '캐시' in advice[1].action


def test_state_age_prevents_presenting_cached_fault_as_current():
    advice = steering_fault_advice(status(fault_bits=0x0201), fresh=False)
    assert 'Stall' in advice[0].cause
    assert '현재 고장 여부 미확인' in advice[0].cause
    assert '수신 중단' in advice[1].cause


def test_driver_can_fault_and_host_can_timeout_have_distinct_recovery():
    hardware = steering_fault_advice(status(fault_bits=0x4000, state=8))
    host = steering_fault_advice(status(state=6, can_timeout=True))
    assert 'Enable reset' in hardware[0].action
    assert '별도 드라이버 복구' in hardware[0].action
    assert 'Enable reset' not in host[0].action
    assert 'can0' in host[0].action


def test_combined_fault_bits_preserve_both_hardware_recovery_paths():
    advice = steering_fault_advice(status(fault_bits=0x0004 | 0x0040, state=8))
    assert len(advice) == 2
    assert '하드웨어 과전류' in advice[0].cause
    assert '전원을 재시작' in advice[0].action
    assert '내부 과전류' in advice[1].cause
    assert 'Disable reset' in advice[1].action


def test_feedback_lost_state_is_explained_even_with_fresh_position_flags():
    advice = steering_fault_advice(status(state=7))
    assert len(advice) == 1
    assert '제어주기 지연' in advice[0].cause
    assert '/steering/torque_safety' in advice[0].action


def test_legacy_host_fault_does_not_turn_disable_bit_into_stall():
    advice = steering_fault_advice(status(fault_bits=1, state=8, disable_reason=10))
    assert len(advice) == 1
    assert '실제 고장 비트 없음' in advice[0].cause
    assert 'bit0은 정상 Disable' in advice[0].action
    assert '실행 버전' in advice[0].action


def test_reserved_fault_bits_are_not_silently_ignored():
    advice = steering_fault_advice(status(fault_bits=(1 << 5) | (1 << 13)))
    assert len(advice) == 2
    assert all('원본 CAN' in entry.action for entry in advice)
    assert not any('전원을 재시작' in entry.action for entry in advice)


def test_missing_status_and_unknown_state_provide_diagnostic_fallbacks():
    missing = steering_fault_advice(None)
    unknown = steering_fault_advice(status(state=255))
    assert '미수신' in missing[0].cause
    assert 'steering_actuator_node' in missing[0].action
    assert '알 수 없는 조향 상태' in unknown[0].cause
    assert '/steering/status' in unknown[0].action
