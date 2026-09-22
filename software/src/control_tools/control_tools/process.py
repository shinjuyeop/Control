"""Owned process groups for recorders; never signal unrelated ROS nodes."""
import os
import signal
import subprocess
import threading
import time
from pathlib import Path

def group_members(pgid):
    result = []
    for path in Path('/proc').glob('[0-9]*/stat'):
        try:
            fields = path.read_text().rsplit(')', 1)[1].split()
            if int(fields[2]) == pgid and fields[0] != 'Z':
                result.append(int(path.parent.name))
        except (OSError, ValueError, IndexError):
            continue
    return result


class OwnedProcess:
    """Direct executables each own a session; no ROS CLI daemon is started."""

    def __init__(self, name, argv, emit):
        self.name, self.emit = name, emit
        self.last_output = ''
        self.process = subprocess.Popen(
            argv, stdin=subprocess.DEVNULL, stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT, text=True, encoding='utf-8', errors='replace',
            start_new_session=True, bufsize=1,
        )
        self.pgid = self.process.pid
        self.reader = threading.Thread(target=self._read, daemon=True)
        self.reader.start()

    def _read(self):
        for line in self.process.stdout:
            self.last_output = line.rstrip()
            self.emit(f'{self.name}: {self.last_output}')
        self.process.stdout.close()

    def stop(self):
        for sig, timeout in ((signal.SIGINT, 5), (signal.SIGTERM, 2), (signal.SIGKILL, 2)):
            if not group_members(self.pgid):
                break
            try:
                os.killpg(self.pgid, sig)
            except ProcessLookupError:
                break
            deadline = time.monotonic() + timeout
            while time.monotonic() < deadline:
                self.process.poll()
                if not group_members(self.pgid):
                    break
                time.sleep(0.05)
        self.process.wait(timeout=1)
        self.reader.join(timeout=1)
        remaining = group_members(self.pgid)
        if remaining:
            raise RuntimeError(f'{self.name}: remaining owned PIDs {remaining}')
        return self.process.returncode
