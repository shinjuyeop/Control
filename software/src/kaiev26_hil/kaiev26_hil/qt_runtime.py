"""Make the Qt bundled with PyQt5 win over a system Qt on ``LD_LIBRARY_PATH``.

PyQt5 installed from PyPI ships its own Qt under ``PyQt5/Qt5/lib``. The xcb
platform plugin is loaded from that same bundled tree, but the dynamic loader
searches ``LD_LIBRARY_PATH`` before a library's ``DT_RUNPATH``. A host that puts
a system Qt first therefore pairs the bundled plugin with system Qt libraries:

    Cannot load library .../PyQt5/Qt5/plugins/platforms/libqxcb.so:
      (/usr/lib/x86_64-linux-gnu/libQt5XcbQpa.so.5: undefined symbol: ...)

Qt then reports that no platform plugin could be initialized and no window
opens, on a local display as well as over SSH X11 forwarding.

``LD_LIBRARY_PATH`` is read by the loader at process start, so assigning it here
cannot help the running process. Re-exec once instead, with the bundled Qt
directory in front. A system PyQt5 has no bundled directory and is left alone.
"""

from __future__ import annotations

import importlib.util
import os
import sys


REEXEC_GUARD = 'KAIEV26_QT_LIBRARY_PATH_APPLIED'


def bundled_qt_lib_dir():
    """Return the Qt library directory shipped inside PyQt5, if there is one."""
    spec = importlib.util.find_spec('PyQt5')
    if spec is None or not spec.submodule_search_locations:
        return None
    candidate = os.path.join(
        spec.submodule_search_locations[0],
        'Qt5',
        'lib',
    )
    return candidate if os.path.isdir(candidate) else None


def ensure_bundled_qt_first():
    """Re-exec this process at most once so the bundled Qt is found first.

    Safe to call unconditionally: it returns without doing anything when the
    process already re-exec'd, when PyQt5 has no bundled Qt, or when the
    bundled directory is already the first entry.
    """
    if os.environ.get(REEXEC_GUARD):
        return
    os.environ[REEXEC_GUARD] = '1'

    lib_dir = bundled_qt_lib_dir()
    if lib_dir is None:
        return

    entries = [
        entry
        for entry in os.environ.get('LD_LIBRARY_PATH', '').split(os.pathsep)
        if entry
    ]
    if entries[:1] == [lib_dir]:
        return

    os.environ['LD_LIBRARY_PATH'] = os.pathsep.join(
        [lib_dir] + [entry for entry in entries if entry != lib_dir]
    )
    os.execv(sys.executable, [sys.executable, *sys.argv])
