"""Operator-selected physical test conditions, never inferred from ROS speed."""

CONDITIONS = {
    'driving': ('주행', '차량이 지면 위를 주행하는 시험'),
    'airborne': ('공중', '바퀴를 지면에서 띄운 상태'),
}
CONDITION_KEYS = tuple(CONDITIONS)
TEST_CONDITIONS = {**CONDITIONS, 'stationary': ('접지 정지', '접지 정지 시나리오 시험 조건')}
TEST_CONDITION_KEYS = tuple(TEST_CONDITIONS)


def condition_label(key):
    return TEST_CONDITIONS[key][0] if key in TEST_CONDITIONS else '선택 필요'


def condition_document(key):
    return {'key': key, 'label': condition_label(key), 'source': 'operator_selection',
            'changes_control': False}
