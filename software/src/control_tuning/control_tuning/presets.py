"""Explicit, condition-specific snapshots of the editor's physical-unit values."""

from datetime import datetime, timezone
import fcntl

import yaml

from .conditions import CONDITIONS, CONDITION_KEYS, condition_label
from .schema import BY_KEY, STEERING, FIELDS, GOVERNOR, decode, encode, equivalent
from .longitudinal import LONGITUDINAL, PROTECTION
from .naming import canonical_values
from .storage import SettingsStore, fingerprint, read_document


def sections(values):
    result = {name: {} for name in ('steering', 'drive', 'governor', 'timeouts')}
    for field in FIELDS:
        section = ('drive' if field.node == LONGITUDINAL else
                   'governor' if field.node == GOVERNOR else
                   'timeouts' if 'timeout' in field.key else 'steering')
        result[section][field.key] = values[field.key]
    return result


class PresetStore:
    def __init__(self, settings_store, *, conditions=CONDITION_KEYS):
        self.settings = settings_store
        self.conditions = conditions
        self.directory = settings_store.path.with_suffix('.presets')
        self.legacy_path = settings_store.path.with_suffix('.presets.yaml')
        self.reload()

    def reload(self):
        entries, tokens = {}, {}
        legacy_token = fingerprint(self.legacy_path)
        legacy = None
        for condition in self.conditions:
            path = self.path_for(condition)
            token = fingerprint(path)
            if token is not None:
                entry = read_document(path)
                if entry.get('schema_version') not in (2, 3) or entry.get('condition') != condition:
                    raise ValueError(f'{path}: schema_version=2/3과 올바른 condition (v1은 명시적 변환 필요)이 필요합니다.')
            else:
                if legacy is None:
                    legacy = read_document(self.legacy_path) or {'schema_version': 1, 'presets': {}}
                    if legacy.get('schema_version') != 1 or not isinstance(legacy.get('presets'), dict):
                        raise ValueError('기존 프리셋 파일 형식을 확인하세요. schema_version=1이 필요합니다.')
                entry = legacy['presets'].get(condition)
                if entry is not None:
                    raise ValueError('schema v1 프리셋: 구동 값을 명시해 v3로 변환하세요.')
            if entry is not None and not isinstance(entry, dict):
                raise ValueError(f'{condition_label(condition)} 프리셋 형식이 올바르지 않습니다.')
            if fingerprint(path) != token or (token is None and fingerprint(self.legacy_path) != legacy_token):
                raise ValueError('프리셋 파일을 읽는 동안 변경되었습니다. 다시 불러오세요.')
            entries[condition] = entry
            tokens[condition] = (token, legacy_token if token is None else None)
        self.entries, self.tokens = entries, tokens

    def path_for(self, condition):
        if condition not in CONDITIONS:
            raise ValueError('주행 / 공중 중에서 프리셋을 선택하세요.')
        return self.directory / f'{condition}.yaml'

    def entry(self, condition):
        self.path_for(condition)
        return self.entries[condition]

    def check_unchanged(self, condition):
        token, legacy_token = self.tokens[condition]
        if (fingerprint(self.path_for(condition)) != token or
                (token is None and fingerprint(self.legacy_path) != legacy_token)):
            raise ValueError('프리셋 파일이 외부에서 변경되었습니다. F로 다시 읽으세요.')

    def normalize(self, values):
        if not isinstance(values, dict) or not set(BY_KEY).issubset(values):
            raise ValueError(f'프리셋에는 현재 편집 항목 {len(BY_KEY)}개의 값이 모두 필요합니다.')
        native = encode(values, self.settings.saved_native[STEERING])
        return decode({name: {**self.settings.base[name], **parameters}
                       for name, parameters in native.items()})

    def load(self, condition):
        entry = self.entry(condition)
        self.check_unchanged(condition)
        if entry is None:
            raise ValueError('저장된 프리셋이 없습니다. 먼저 프리셋을 만들어 주세요.')
        reference = self.settings.saved_native[STEERING]['keya_protection_current_a']
        reference_key = 'steering_current_reference_a' if entry['schema_version'] == 3 else 'current_reference_a'
        if ('current_reference_a' in entry and 'steering_current_reference_a' in entry and
                not equivalent(entry['current_reference_a'], entry['steering_current_reference_a'])):
            raise ValueError('이전 이름과 새 이름의 조향 전류 환산 기준이 서로 다릅니다.')
        if not equivalent(entry.get(reference_key), reference):
            raise ValueError('프리셋과 현재 설정의 전류 환산 기준이 다릅니다.')
        section_names = (('lateral', 'longitudinal', 'governor', 'timeouts')
                         if entry['schema_version'] == 2 else
                         ('steering', 'drive', 'governor', 'timeouts'))
        values = {}
        for section in section_names:
            block = entry.get(section, {})
            if not isinstance(block, dict):
                raise ValueError(f'{section}: 이름과 값의 mapping이 필요합니다.')
            for key, value in block.items():
                if key in values and values[key] != value:
                    raise ValueError(f'{key}: section 간 설정값이 서로 다릅니다.')
                values[key] = value
        values = canonical_values(values)
        # Presets saved before STM_A ABI v4 were tuned on the square-wave EZ. Their
        # coast-only floor becomes the square rolling minimum current; the other
        # v4 drive-protection and feedforward values take the firmware defaults.
        legacy_floor = values.pop('drive_coast_current_floor_a', None)
        if 'drive_motor_waveform' not in values:
            values['drive_motor_waveform'] = 'square'
            if legacy_floor is not None:
                values.setdefault('drive_min_current_square_a', legacy_floor)
        for key, value in canonical_values(PROTECTION).items():
            values.setdefault(key, value)
        # Existing v2/v3 presets predate the governor ramp; use the base setting.
        values.setdefault('governor_deceleration_slew_mps2',
                          self.settings.saved['governor_deceleration_slew_mps2'])
        values.setdefault('governor_acceleration_slew_mps2',
                          self.settings.saved['governor_acceleration_slew_mps2'])
        return self.normalize(values)

    def is_current(self, condition):
        """True when the saved file already names every current field."""
        entry = self.entry(condition)
        if not entry or entry.get('schema_version') != 3:
            return False
        saved = sections({f.key: None for f in FIELDS})
        return all(key in entry.get(section, {}) for section, keys in saved.items() for key in keys)

    def save(self, condition, values):
        self.entry(condition)
        normalized = self.normalize(values)
        path = self.path_for(condition)
        path.parent.mkdir(parents=True, exist_ok=True)
        with path.with_suffix('.yaml.lock').open('a') as lock:
            fcntl.flock(lock, fcntl.LOCK_EX)
            self.check_unchanged(condition)
            if self.settings.has_external_change():
                raise ValueError('저장 파일이 외부에서 변경되었습니다. F로 다시 읽으세요.')
            document = {
                'schema_version': 3,
                'condition': condition,
                'label': condition_label(condition),
                'saved_at': datetime.now(timezone.utc).isoformat(),
                'steering_current_reference_a': self.settings.saved_native[STEERING]['keya_protection_current_a'],
                **sections(normalized),
            }
            content = ('# tuning_preset으로 선택한 다음 구동 스택 실행에 적용됩니다. 저장은 현재 실행값을 바꾸지 않습니다.\n' +
                       yaml.safe_dump(document, sort_keys=False, allow_unicode=True)).encode('utf-8')
            if path.exists():
                SettingsStore._replace(path.with_suffix('.yaml.bak'), path.read_bytes())
            SettingsStore._replace(path, content)
            self.entries[condition], self.tokens[condition] = document, (fingerprint(path), None)
        return dict(normalized)


def load_launch_overrides(path, node, condition='driving', **base_paths):
    """Read the selected snapshot; never fall back to a different condition."""
    if condition not in CONDITIONS:
        raise ValueError(f'tuning_preset: 지원하지 않는 프리셋 {condition!r}')
    settings = SettingsStore(path, **base_paths)
    presets = PresetStore(settings, conditions=(condition,))
    if presets.entry(condition) is None:
        raise ValueError(f'프리셋이 없습니다: {presets.path_for(condition)}. '
                         'Control Tuning에서 해당 프리셋을 선택하고 S로 저장하세요.')
    values = presets.load(condition)
    parameters = encode(values, settings.saved_native[STEERING])[node]
    if node == GOVERNOR and values['drive_control_mode'] != 'speed':
        parameters['acceleration_slew_mps2'] = 0.0
    return parameters
