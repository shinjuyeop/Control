"""Explicit, condition-specific snapshots of the editor's physical-unit values."""

from datetime import datetime, timezone
import fcntl

import yaml

from .conditions import CONDITIONS, CONDITION_KEYS, condition_label
from .schema import BY_KEY, STEERING, decode, encode, equivalent
from .storage import SettingsStore, fingerprint, read_document


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
                if entry.get('schema_version') != 1 or entry.get('condition') != condition:
                    raise ValueError(f'{path}: schema_version=1과 올바른 condition이 필요합니다.')
            else:
                if legacy is None:
                    legacy = read_document(self.legacy_path) or {'schema_version': 1, 'presets': {}}
                    if legacy.get('schema_version') != 1 or not isinstance(legacy.get('presets'), dict):
                        raise ValueError('기존 프리셋 파일 형식을 확인하세요. schema_version=1이 필요합니다.')
                entry = legacy['presets'].get(condition)
            if entry is not None and not isinstance(entry, dict):
                raise ValueError(f'{condition_label(condition)} 프리셋 형식이 올바르지 않습니다.')
            if fingerprint(path) != token or (token is None and fingerprint(self.legacy_path) != legacy_token):
                raise ValueError('프리셋 파일을 읽는 동안 변경되었습니다. 다시 불러오세요.')
            entries[condition] = entry
            tokens[condition] = (token, legacy_token if token is None else None)
        self.entries, self.tokens = entries, tokens

    def path_for(self, condition):
        if condition not in CONDITIONS:
            raise ValueError('주행 / 공중 / 접지 정지 중에서 프리셋을 선택하세요.')
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
        if not isinstance(values, dict) or set(values) != set(BY_KEY):
            raise ValueError('프리셋에는 현재 편집 항목 20개의 값이 모두 필요합니다.')
        native = encode(values, self.settings.saved_native[STEERING])
        return decode({name: {**self.settings.base[name], **parameters}
                       for name, parameters in native.items()})

    def load(self, condition):
        entry = self.entry(condition)
        self.check_unchanged(condition)
        if entry is None:
            raise ValueError('저장된 프리셋이 없습니다. 먼저 프리셋을 만들어 주세요.')
        reference = self.settings.saved_native[STEERING]['keya_protection_current_a']
        if not equivalent(entry.get('current_reference_a'), reference):
            raise ValueError('프리셋과 현재 설정의 전류 환산 기준이 다릅니다.')
        return self.normalize(entry.get('values'))

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
                'schema_version': 1,
                'condition': condition,
                'label': condition_label(condition),
                'saved_at': datetime.now(timezone.utc).isoformat(),
                'current_reference_a': self.settings.saved_native[STEERING]['keya_protection_current_a'],
                'values': normalized,
            }
            content = ('# tuning_preset으로 선택한 다음 launch에 적용됩니다. 실행 중 적용은 TUI의 A를 사용합니다.\n' +
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
    return encode(values, settings.saved_native[STEERING])[node]
