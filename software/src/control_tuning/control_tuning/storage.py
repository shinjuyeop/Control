"""Shared launch overrides with atomic, conflict-checked persistence."""

import fcntl
import hashlib
import math
import os
from pathlib import Path
import tempfile

import yaml

from .longitudinal import LONGITUDINAL, DEFAULT
from .schema import STEERING, GOVERNOR, FIELDS, OWNED, decode, encode, equivalent


def default_settings_path():
    explicit = os.environ.get('KAI_CONTROL_TUNING_FILE')
    if explicit:
        return str(Path(explicit).expanduser().absolute())
    module = Path(__file__).resolve()
    # Source execution and colcon --symlink-install both resolve to this package.
    package = module.parents[1]
    if (package / 'package.xml').is_file():
        return str(package / 'config/control_tuning.yaml')
    # A regular colcon install copies Python files. Prefer the checkout in that
    # same workspace so saves stay under Git and launches see them without a build.
    for workspace in module.parents:
        package = workspace / 'src/Control/software/src/control_tuning'
        if (package / 'package.xml').is_file():
            return str(package / 'config/control_tuning.yaml')
    # A standalone installation without its checkout uses its packaged config.
    return str(config_directory('control_tuning') / 'control_tuning.yaml')


def read_document(path):
    path = Path(path).expanduser()
    if not path.exists():
        return {}
    document = yaml.safe_load(path.read_text(encoding='utf-8'))
    if document is None:
        return {}
    if not isinstance(document, dict):
        raise ValueError(f'{path}: YAML mapping이 필요합니다.')
    return document


def fingerprint(path):
    path = Path(path)
    return hashlib.sha256(path.read_bytes()).hexdigest() if path.exists() else None


def load_overrides(path, node):
    """Called by launches too. Never allow a tuning file to alter hardware routing."""
    document = read_document(path)
    block = document.get(node, {})
    if not isinstance(block, dict) or not isinstance(block.get('ros__parameters', {}), dict):
        raise ValueError(f'{path}: {node}.ros__parameters 형식 오류')
    values = block.get('ros__parameters', {})
    unknown = set(values) - OWNED[node]
    if unknown:
        raise ValueError(f'{path}: TUI 관리 대상이 아닌 설정: {", ".join(sorted(unknown))}')
    for f in FIELDS:
        if f.node != node or f.native not in values:
            continue
        value = values[f.native]
        if f.choices:
            if value not in f.choices:
                raise ValueError(f'{f.native}: 지원하지 않는 제어 방식')
        elif isinstance(value, bool) or not isinstance(value, (int, float)):
            raise ValueError(f'{f.native}: 숫자가 필요합니다.')
        elif not math.isfinite(value):
            raise ValueError(f'{f.native}: 유한한 숫자가 필요합니다.')
        elif (f.integer or f.native.endswith('_limit_raw')) and type(value) is not int:
            raise ValueError(f'{f.native}: 정수가 필요합니다.')
        elif not f.integer and f.native not in ('torque_peak_limit_raw', 'torque_sustained_limit_raw'):
            values[f.native] = float(value)
    return values


def config_directory(package):
    from ament_index_python.packages import get_package_share_directory, PackageNotFoundError
    try:
        return Path(get_package_share_directory(package)) / 'config'
    except PackageNotFoundError:
        for ancestor in Path(__file__).resolve().parents:
            candidate = ancestor / package / 'config'
            if candidate.is_dir():
                return candidate
        raise


class SettingsStore:
    def __init__(self, path=None, steering_config=None, governor_config=None, calibration=None):
        self.path = Path(path or default_settings_path()).expanduser().absolute()
        lateral = None if steering_config and calibration else config_directory('lateral_ctrl')
        self.base_paths = tuple(Path(p).expanduser().absolute() for p in (
            steering_config or lateral / 'steering_actuator.yaml',
            governor_config or config_directory('command_governor') / 'command_governor.yaml',
            calibration or lateral / 'steering_motor_calibration.yaml',
        ))
        self.base = {
            LONGITUDINAL: dict(DEFAULT),
            STEERING: read_document(self.base_paths[0])[STEERING]['ros__parameters'],
            GOVERNOR: read_document(self.base_paths[1])[GOVERNOR]['ros__parameters'],
        }
        self.base[STEERING].update(read_document(self.base_paths[2]))
        self.base_tokens = tuple(fingerprint(p) for p in self.base_paths)
        self.token = fingerprint(self.path)
        self.saved_native = {node: {**params, **load_overrides(self.path, node)}
                             for node, params in self.base.items()}
        self.saved = decode(self.saved_native)
        encode(self.saved, self.saved_native[STEERING])

    def has_external_change(self):
        return self.token != fingerprint(self.path) or self.base_tokens != tuple(fingerprint(p) for p in self.base_paths)

    def save(self, values):
        native = encode(values, self.saved_native[STEERING])
        self.path.parent.mkdir(parents=True, exist_ok=True)
        with self.path.with_suffix(self.path.suffix + '.lock').open('a') as lock:
            fcntl.flock(lock, fcntl.LOCK_EX)
            if self.has_external_change():
                raise ValueError('설정 파일이 외부에서 변경되었습니다. F로 다시 읽은 뒤 저장하십시오.')
            document = read_document(self.path)
            for node, params in native.items():
                changed = {key: value for key, value in params.items()
                           if not equivalent(value, self.base[node][key])}
                if changed:
                    document[node] = {'ros__parameters': changed}
                else:
                    document.pop(node, None)
            content = ('# Control Tuning 사용자 설정: 기본 YAML 위에 적용됩니다.\n'
                       '# 명령 전류 UI 입력은 기존 ROS 파라미터 단위로 저장합니다.\n'
                       + yaml.safe_dump(document, allow_unicode=True, sort_keys=False))
            if self.path.exists():
                self._replace(self.path.with_suffix(self.path.suffix + '.bak'), self.path.read_bytes())
            self._replace(self.path, content.encode('utf-8'))
            self.token = fingerprint(self.path)
            self.saved_native = {node: {**self.base[node], **params} for node, params in native.items()}
            self.saved = decode(self.saved_native)
        return dict(self.saved)

    @staticmethod
    def _replace(path, data):
        fd, temporary = tempfile.mkstemp(prefix='.' + path.name, dir=path.parent)
        try:
            with os.fdopen(fd, 'wb') as stream:
                stream.write(data)
                stream.flush()
                os.fsync(stream.fileno())
            os.replace(temporary, path)
            directory = os.open(path.parent, os.O_RDONLY | os.O_DIRECTORY)
            try:
                os.fsync(directory)
            finally:
                os.close(directory)
        finally:
            if os.path.exists(temporary):
                os.unlink(temporary)
