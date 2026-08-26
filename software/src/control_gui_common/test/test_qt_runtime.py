import os

import pytest

from control_gui_common import qt_runtime


def test_guard_prevents_qt_reexec(monkeypatch):
    monkeypatch.setenv(qt_runtime.REEXEC_GUARD, '1')
    monkeypatch.setattr(
        qt_runtime.os,
        'execv',
        lambda *_args: pytest.fail('guarded process must not re-exec'),
    )

    qt_runtime.ensure_bundled_qt_first()


def test_missing_bundled_qt_does_not_reexec(monkeypatch):
    monkeypatch.delenv(qt_runtime.REEXEC_GUARD, raising=False)
    monkeypatch.setattr(qt_runtime, 'bundled_qt_lib_dir', lambda: None)
    monkeypatch.setattr(
        qt_runtime.os,
        'execv',
        lambda *_args: pytest.fail('system PyQt5 must not re-exec'),
    )

    qt_runtime.ensure_bundled_qt_first()

    assert os.environ[qt_runtime.REEXEC_GUARD] == '1'


def test_bundled_qt_is_prepended_before_reexec(monkeypatch):
    bundled = '/opt/test/PyQt5/Qt5/lib'
    monkeypatch.delenv(qt_runtime.REEXEC_GUARD, raising=False)
    monkeypatch.setenv('LD_LIBRARY_PATH', f'/system/qt{os.pathsep}{bundled}')
    monkeypatch.setattr(qt_runtime, 'bundled_qt_lib_dir', lambda: bundled)

    invocation = {}

    def capture_execv(executable, arguments):
        invocation['executable'] = executable
        invocation['arguments'] = arguments
        raise RuntimeError('captured re-exec')

    monkeypatch.setattr(qt_runtime.os, 'execv', capture_execv)

    with pytest.raises(RuntimeError, match='captured re-exec'):
        qt_runtime.ensure_bundled_qt_first()

    assert os.environ['LD_LIBRARY_PATH'].split(os.pathsep) == [
        bundled,
        '/system/qt',
    ]
    assert invocation['executable'] == qt_runtime.sys.executable
    assert invocation['arguments'][0] == qt_runtime.sys.executable
