"""Terminal input and incremental Rich rendering; no ROS dependencies."""

from contextlib import contextmanager
import os
import select
import sys
import termios
import time
import tty
from rich.segment import Segment
from rich.table import Table

CLEAR_VIEWPORT = '\x1b[2J\x1b[H'
ENTER_TUI = '\x1b[?1049h\x1b[?25l\x1b[?1000h\x1b[?1006h'
LEAVE_TUI = '\x1b[?1006l\x1b[?1000l\x1b[?25h\x1b[?1049l'


def terminal_write(console, value):
    if not console.is_terminal:
        return
    console.file.write(value)
    console.file.flush()


@contextmanager
def tui_screen(console):
    """Hide the shell while the TUI owns an alternate terminal screen."""
    terminal_write(console, ENTER_TUI + CLEAR_VIEWPORT)
    try:
        yield
    finally:
        terminal_write(console, LEAVE_TUI)


class RenderedLines:
    def __init__(self, lines):
        self.lines = lines

    def __rich_console__(self, _console, _options):
        for index, line in enumerate(self.lines):
            yield from line
            if index + 1 < len(self.lines):
                yield Segment.line()


class ScreenRenderer:
    """Update changed rows in one write without exposing a cleared screen."""

    def __init__(self, console):
        self.console = console
        self.previous_rows = []
        self.previous_size = None

    def redraw(self, renderable, offset):
        console = self.console
        size = console.size
        lines = console.render_lines(
            renderable, console.options.update(height=None), pad=True,
        )
        visible_rows = max(1, size.height)
        max_offset = max(0, len(lines) - visible_rows)
        offset = max(0, min(offset, max_offset))
        visible = lines[offset:offset + visible_rows]
        with console.capture() as capture:
            console.print(
                RenderedLines(visible),
                end='', soft_wrap=True,
            )
        rows = capture.get().split('\n') if visible else []
        rows += [' ' * size.width] * (visible_rows - len(rows))
        previous = self.previous_rows if size == self.previous_size else []
        updates = [
            f'\x1b[{index + 1};1H{row}'
            for index, row in enumerate(rows)
            if index >= len(previous) or row != previous[index]
        ]
        if updates:
            terminal_write(console, ''.join(updates))
        self.previous_rows = rows
        self.previous_size = size
        return offset


def vertical_stack(*renderables):
    layout = Table.grid(expand=True)
    for renderable in renderables:
        layout.add_row(renderable)
    return layout


@contextmanager
def keyboard():
    original = termios.tcgetattr(sys.stdin.fileno())
    try:
        tty.setcbreak(sys.stdin.fileno())
        yield
    finally:
        try:
            termios.tcsetattr(sys.stdin.fileno(), termios.TCSADRAIN, original)
        except (termios.error, OSError):
            pass


def read_key(fd):
    raw = os.read(fd, 1)
    if not raw:
        return 'eof'
    if raw != b'\x1b':
        if raw in (b'\r', b'\n'):
            return 'enter'
        if raw in (b'\x7f', b'\x08'):
            return 'backspace'
        return raw.decode('ascii', errors='ignore')

    suffix = b''
    deadline = time.monotonic() + 0.02
    while len(suffix) < 32 and time.monotonic() < deadline:
        readable, _, _ = select.select([fd], [], [], max(0, deadline - time.monotonic()))
        if not readable:
            break
        suffix += os.read(fd, 1)
        if suffix.endswith((b'~', b'M', b'm')) or (
                len(suffix) == 2 and suffix.startswith(b'[')
                and suffix[-1:] in b'ABCD'):
            break
    if suffix.startswith(b'[<') and suffix.endswith((b'M', b'm')):
        try:
            button = int(suffix[2:-1].split(b';', 1)[0])
        except ValueError:
            return 'mouse'
        return {64: 'scroll_up', 65: 'scroll_down'}.get(button, 'mouse')
    return {
        b'[A': 'up', b'[B': 'down', b'[C': 'right', b'[D': 'left',
        b'[5~': 'page_up', b'[6~': 'page_down',
    }.get(suffix, 'escape')
