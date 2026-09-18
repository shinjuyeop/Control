"""Operator-selected physical test conditions, never inferred from ROS speed."""

CONDITIONS = {
    'driving': ('주행', '차량이 지면 위를 주행하는 시험'),
    'airborne': ('공중', '바퀴를 지면에서 띄운 상태'),
    'ground_stationary': ('접지 정지', '바퀴가 지면에 닿아 있고 차량은 정지한 상태'),
}
CONDITION_KEYS = tuple(CONDITIONS)


def condition_label(key):
    return CONDITIONS[key][0] if key in CONDITIONS else '선택 필요'


def condition_document(key):
    return {'key': key, 'label': condition_label(key), 'source': 'operator_selection',
            'changes_control': False}
