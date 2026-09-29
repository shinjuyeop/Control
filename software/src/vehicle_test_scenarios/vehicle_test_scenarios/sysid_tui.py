"""Rich field console for selecting, configuring, recording, and running tests."""

from contextlib import contextmanager
from dataclasses import replace
import fcntl
import os
from pathlib import Path
import select
import signal
import sys
import termios
import threading
import time
import tty

import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.parameter import Parameter
from rclpy.signals import SignalHandlerOptions
from rich.console import Console, Group
from rich import box
from rich.panel import Panel
from rich.segment import Segment
from rich.table import Table
from rich.text import Text

from .sysid_processes import DEFAULT_RECORDS_ROOT, ProcessManager
from .sysid_profiles import (
    SCENARIO_KEYS, SCENARIO_SPECS, default_run_settings, update_setting,
)
from .sysid_runtime import SysidNode
from control_tuning.conditions import CONDITION_KEYS, TEST_CONDITION_KEYS, condition_label


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


class MenuState:
    def __init__(self):
        self.settings_by_key = {
            key: default_run_settings(key) for key in SCENARIO_KEYS
        }
        self.hover_index = 0
        self.selected_key = SCENARIO_KEYS[0]
        self.focus = 'scenarios'
        self.field_index = 0
        self.edit_field = None
        self.edit_text = ''
        self.edit_choice = None
        self.record_mcap = True
        self.test_condition = 'unspecified'
        self.view_offset = 0

    @property
    def selection(self):
        return self.settings_by_key[self.selected_key]

    def run_settings(self):
        return replace(self.selection, record_mcap=self.record_mcap,
                       test_condition=self.test_condition)

    def adopt_launch_preset(self, preset):
        if self.test_condition == 'unspecified' and self.edit_field != 'test_condition':
            if preset in CONDITION_KEYS:
                self.test_condition = preset

    @property
    def selected_spec(self):
        return self.selection.spec

    @property
    def current_field(self):
        fields = self.selected_spec.fields
        return fields[min(self.field_index, len(fields) - 1)]

    def move(self, delta):
        if self.focus == 'scenarios':
            self.hover_index = (self.hover_index + delta) % len(SCENARIO_SPECS)
        else:
            count = len(self.selected_spec.fields) + 2  # MCAP and physical test condition.
            self.field_index = (self.field_index + delta) % count

    def select_hovered(self):
        self.selected_key = SCENARIO_SPECS[self.hover_index].key
        self.field_index = 0
        self.focus = 'parameters'

    def begin_edit(self):
        if self.field_index >= len(self.selected_spec.fields):
            self.edit_field = ('record_mcap' if self.field_index == len(self.selected_spec.fields)
                               else 'test_condition')
            value = getattr(self, self.edit_field)
            self.edit_choice = value if value in self.choice_options else self.choice_options[0]
            return
        field = self.current_field
        value = getattr(self.selection, field.name)
        self.edit_field = field.name
        self.edit_text = str(value)

    @property
    def choice_options(self):
        if self.edit_field == 'record_mcap':
            return (True, False)
        if self.edit_field == 'test_condition':
            return TEST_CONDITION_KEYS
        return ()

    def cancel_edit(self):
        self.edit_field = None
        self.edit_text = ''
        self.edit_choice = None

    def commit_edit(self):
        if self.choice_options:
            setattr(self, self.edit_field, self.edit_choice)
            self.cancel_edit()
            return
        field = self.current_field
        if not self.edit_text.strip():
            raise ValueError('값을 입력하세요')
        try:
            value = int(self.edit_text) if field.integer else float(self.edit_text)
        except ValueError as error:
            raise ValueError(f'{field.label}: 숫자를 입력하세요') from error
        updated = update_setting(self.selection, field.name, value)
        self.settings_by_key[self.selected_key] = updated
        self.cancel_edit()


def handle_key(menu, key, *, busy=False):
    """Apply one decoded key and return (action, operator message)."""
    if key.lower() == 'q':
        return 'quit', ''
    if key.lower() == 's' or key == ' ':
        menu.cancel_edit()
        return 'stop', ''
    if key in ('scroll_up', 'page_up'):
        menu.view_offset = max(
            0, menu.view_offset - (5 if key == 'page_up' else 1))
        return None, ''
    if key in ('scroll_down', 'page_down'):
        menu.view_offset += 5 if key == 'page_down' else 1
        return None, ''
    if key == 'mouse':
        return None, ''
    if busy:
        menu.cancel_edit()
        return None, '실행·저장이 끝난 뒤 설정을 바꿀 수 있습니다.'
    if menu.edit_field is not None:
        if key == 'enter':
            try:
                menu.commit_edit()
                return None, ''
            except ValueError as error:
                return None, str(error)
        if key == 'escape':
            menu.cancel_edit()
            return None, ''
        if menu.choice_options:
            if key in ('up', 'down'):
                options = menu.choice_options
                step = -1 if key == 'up' else 1
                menu.edit_choice = options[(options.index(menu.edit_choice) + step) % len(options)]
            return None, ''
        if key == 'backspace':
            menu.edit_text = menu.edit_text[:-1]
        elif len(key) == 1 and key in '0123456789.-':
            menu.edit_text += key
        return None, ''

    if key == 'left':
        menu.focus = 'scenarios'
    elif key == 'right':
        menu.focus = 'parameters'
    elif key == 'up':
        menu.move(-1)
    elif key == 'down':
        menu.move(1)
    elif key == 'enter':
        if menu.focus == 'scenarios':
            menu.select_hovered()
        else:
            menu.begin_edit()
    elif key.lower() == 'r':
        if menu.test_condition == 'unspecified':
            return None, '시험 조건에서 Enter → ↑↓ 선택 → Enter로 확정하세요.'
        return 'run', ''
    elif key == 'escape':
        menu.focus = 'scenarios'
    return None, ''


def readiness_rows(snapshot):
    rows = list(snapshot['gates'])
    shown_problems = {detail for _, go, detail in rows if not go}
    rows.extend(('실행 조건', False, issue)
                for issue in snapshot['issues'] if issue not in shown_problems)
    # Failures first, while keeping a stable order within each state.
    rows.sort(key=lambda row: row[1])
    return rows


def readiness_panel(snapshot):
    rows = readiness_rows(snapshot)
    failures = sum(not go for _, go, _ in rows)
    samples = {name: (icon, detail) for name, icon, detail in snapshot['rows']}
    table = Table(expand=True, box=box.SIMPLE_HEAD, padding=(0, 1))
    table.add_column('항목', ratio=2, overflow='fold')
    table.add_column('상태', width=7, no_wrap=True)
    table.add_column('상세', ratio=3, overflow='fold')
    for name, go, detail in rows:
        status, style = ('GO', 'green') if go else ('NO-GO', 'bold red')
        if name in samples:
            icon, sample_detail = samples[name]
            if go:
                detail = sample_detail
                if icon == '—' or name in snapshot['optional_topics']:
                    status, style = '선택', 'dim'
        for prefix in (name + ': ', name + ' '):
            if detail.startswith(prefix):
                detail = detail[len(prefix):]
                break
        table.add_row(Text(name), Text(status, style=style), Text(detail))
    return Panel(table, title=f'RUN NO-GO · {failures}개 조건' if failures else 'RUN GO',
                 border_style='red' if failures else 'green')


def steering_advice_panel(advice):
    table = Table.grid(expand=True, padding=(0, 1))
    table.add_column(ratio=1, style='bold yellow', overflow='fold')
    table.add_column(ratio=3, overflow='fold')
    for item in advice:
        table.add_row(Text(item.cause), Text(item.action))
    return Panel(table, title='조향 원인 · 조치 안내', border_style='yellow')


def scenario_menu(menu):
    table = Table.grid(expand=True)
    table.add_column()
    previous_category = previous_subgroup = None
    for index, spec in enumerate(SCENARIO_SPECS):
        if spec.category != previous_category:
            if previous_category is not None:
                table.add_row('')
            table.add_row(Text(spec.category, style='bold cyan'))
            previous_category = spec.category
            previous_subgroup = None
        if spec.subgroup and spec.subgroup != previous_subgroup:
            table.add_row(Text(f'  {spec.subgroup}', style='bold'))
            previous_subgroup = spec.subgroup
        hovered = index == menu.hover_index and menu.focus == 'scenarios'
        selected = spec.key == menu.selected_key
        prefix = '  ▶ ' if hovered else '    '
        marker = '● ' if selected else '○ '
        style = 'bold reverse' if hovered else ('bold green' if selected else '')
        table.add_row(Text(prefix + marker + spec.label, style=style,
                           no_wrap=True, overflow='ellipsis'))
    table.add_row(Text(f'{menu.hover_index + 1}/{len(SCENARIO_SPECS)} · ↑↓ 이동', style='dim'))
    return table


def parameter_menu(menu):
    table = Table.grid(expand=True, padding=(0, 1))
    table.add_column(ratio=2)
    table.add_column(justify='right', ratio=1, no_wrap=True, overflow='ellipsis')
    table.add_column(width=4, no_wrap=True)
    selection = menu.selection
    fields = menu.selected_spec.fields
    for index, field in enumerate(fields):
        focused = menu.focus == 'parameters' and index == menu.field_index
        editing = focused and menu.edit_field == field.name
        prefix = '▶ ' if focused else '  '
        value = menu.edit_text + '▌' if editing else f'{getattr(selection, field.name):g}'
        style = 'bold reverse' if focused else ''
        table.add_row(
            Text(prefix + field.label, style=style, no_wrap=True, overflow='ellipsis'),
            Text(value, style='bold cyan' if editing else style),
            Text(field.unit, style=style),
        )
    # Choice values use the full right-hand space, including the numeric unit
    # column, so 접지 정지 remains readable in a 71-column terminal.
    choices = Table.grid(expand=True, padding=(0, 1))
    choices.add_column(ratio=1, no_wrap=True)
    choices.add_column(justify='right', no_wrap=True, min_width=9)
    for offset, (name, label) in enumerate((('record_mcap', 'MCAP 녹화'),
                                          ('test_condition', '시험 조건'))):
        focused = menu.focus == 'parameters' and menu.field_index == len(fields) + offset
        editing = menu.edit_field == name
        value = menu.edit_choice if editing else getattr(menu, name)
        rendered = ('ON' if value else 'OFF') if name == 'record_mcap' else condition_label(value)
        style = 'bold cyan' if editing else 'bold reverse' if focused else ''
        choices.add_row(Text(('▶ ' if focused else '  ') + label, style=style),
                        Text(('↕ ' if editing else '') + rendered, style=style))
    duration = selection.duration
    timing = Table.grid(expand=True, padding=(0, 1))
    timing.add_column(ratio=1)
    timing.add_column(justify='right', no_wrap=True)
    timing.add_row(
        Text('예상 입력시간', style='dim'),
        Text('수동 종료' if duration is None else f'{duration:g} sec', style='dim'),
    )
    return Group(table, choices, timing)


def settings_panel(menu):
    columns = Table.grid(expand=True, padding=(0, 1))
    columns.add_column(ratio=1)
    columns.add_column(ratio=1)
    columns.add_row(
        Panel(scenario_menu(menu), title='시험 선택',
              border_style='cyan' if menu.focus == 'scenarios' else 'dim'),
        Panel(parameter_menu(menu), title=menu.selection.name,
              border_style='cyan' if menu.focus == 'parameters' else 'dim'),
    )
    return columns


def summary_panel(snapshot, message=''):
    target = snapshot['output'].steering_deg
    values = {
        'mode': snapshot['mode'],
        'speed': '—' if snapshot['speed'] is None else f"{snapshot['speed']:.1f} km/h",
        'target_speed': f"{snapshot['output'].speed_kph:.1f} km/h",
        'steering': '—' if snapshot['steering'] is None else f"{snapshot['steering']:.2f} deg",
        'target_steering': 'DISABLE 요청' if target is None else f'{target:.2f} deg',
        'rpm': '—' if snapshot['rpm'] is None else f"{snapshot['rpm']:.0f}",
        'current': '—' if snapshot['current'] is None else f"{snapshot['current']:.1f} A",
    }
    countdown = f" · {snapshot['countdown']}초" if snapshot['countdown'] is not None else ''
    table = Table.grid(expand=True, padding=(0, 1))
    table.add_column(width=11, style='cyan', no_wrap=True)
    table.add_column(ratio=1, no_wrap=True, overflow='ellipsis')
    table.add_column(width=11, style='cyan', no_wrap=True)
    table.add_column(ratio=1, no_wrap=True, overflow='ellipsis')
    table.add_row('상태', snapshot['label'] + countdown, '모드', values['mode'])
    table.add_row('차속', values['speed'], '목표 / RPM',
                  f"{values['target_speed']} / {values['rpm']}")
    table.add_row('조향 실제', values['steering'], '목표 / 전류',
                  f"{values['target_steering']} / {values['current']}")
    notice = Table.grid(expand=True, padding=(0, 1))
    notice.add_column(width=11, style='cyan', no_wrap=True)
    notice.add_column(ratio=1, overflow='fold')
    notice.add_row('안내', Text(message or '—', style='bold yellow' if message else 'dim'))
    return Panel(Group(table, notice), title='VEHICLE STATE · 시험 상태', border_style='cyan')


def footer_panel(snapshot, menu, records_root):
    record_enabled = (
        snapshot['settings'].record_mcap if snapshot['busy'] else menu.record_mcap
    )
    destination = (
        snapshot['run_dir'] or str(records_root)
        if record_enabled else '파일 저장 없이 실행'
    )
    table = Table.grid(expand=True, padding=(0, 1))
    table.add_column(width=8, style='cyan', no_wrap=True)
    table.add_column(ratio=1, no_wrap=True, overflow='ellipsis')
    table.add_row('스크롤', '휠/PgUp/PgDn 전체 화면 · ↑↓ 항목 · ←→ 영역')
    table.add_row('선택', '↑↓ 값 선택 · Enter 완료 · Esc 취소' if menu.choice_options else
                  'Enter 선택/수정 · Esc 취소')
    table.add_row('실행', 'R 실행 · S/Space 시나리오 종료 · Q TUI 종료')
    return Panel(
        table,
        title='키 안내',
        subtitle=Text(destination, no_wrap=True, overflow='ellipsis'),
    )


def dashboard(snapshot, menu, message='', height=40,
              records_root=DEFAULT_RECORDS_ROOT):
    warning = snapshot['run_warning']
    if snapshot['terminal_state'] and snapshot['finish_reason']:
        warning = snapshot['finish_reason']
    if not message:
        message = warning
    advice = snapshot.get('steering_advice', ())
    return vertical_stack(
        summary_panel(snapshot, message),
        *([steering_advice_panel(advice)] if advice else []),
        readiness_panel(snapshot),
        settings_panel(menu),
        footer_panel(snapshot, menu, records_root),
    )


def main(*, use_sim_time=False, records_root=DEFAULT_RECORDS_ROOT,
         require_ezkontrol_status=True):
    console = Console()
    if not sys.stdin.isatty():
        console.print('대화형 터미널에서 실행하세요.')
        return 2
    lock_path = Path('/tmp') / (
        f'auto_sysid_{os.getuid()}_{os.environ.get("ROS_DOMAIN_ID", "0")}.lock'
    )
    lock_fd = os.open(lock_path, os.O_CREAT | os.O_RDWR, 0o600)
    try:
        fcntl.flock(lock_fd, fcntl.LOCK_EX | fcntl.LOCK_NB)
    except BlockingIOError:
        os.close(lock_fd)
        console.print('같은 ROS domain에서 실차 시험 TUI가 이미 실행 중입니다.')
        return 2
    node = manager = executor = spin_thread = None
    exit_code = 0
    cleanup_errors = []
    old_term = signal.getsignal(signal.SIGTERM)
    old_hup = signal.getsignal(signal.SIGHUP)
    old_int = signal.getsignal(signal.SIGINT)
    quitting = threading.Event()
    signal.signal(signal.SIGTERM, lambda *_: quitting.set())
    signal.signal(signal.SIGHUP, lambda *_: quitting.set())
    try:
        rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
        node = SysidNode(require_ezkontrol_status=require_ezkontrol_status, parameter_overrides=[
            Parameter('use_sim_time', value=use_sim_time),
        ])
        if use_sim_time:
            node.clock = lambda: node.get_clock().now().nanoseconds / 1e9
        executor = SingleThreadedExecutor()
        executor.add_node(node)
        spin_thread = threading.Thread(target=executor.spin, name='sysid-ros')
        spin_thread.start()
        manager = ProcessManager(node, records_root=records_root)
        manager.start()
        menu = MenuState()
        renderer = ScreenRenderer(console)
        message = ''
        with keyboard(), tui_screen(console):
            while not quitting.is_set():
                snapshot = node.snapshot()
                if not node.busy:
                    menu.adopt_launch_preset(snapshot['launch_preset'])
                menu.view_offset = renderer.redraw(
                    dashboard(
                        snapshot, menu, message,
                        height=console.size.height, records_root=records_root,
                    ),
                    menu.view_offset,
                )
                readable, _, _ = select.select([sys.stdin], [], [], 0.2)
                if not readable:
                    continue
                key = read_key(sys.stdin.fileno())
                if key == 'eof':
                    break
                action, new_message = handle_key(menu, key, busy=node.busy)
                message = new_message
                if action == 'quit':
                    break
                if action == 'stop':
                    node.request_stop()
                elif action == 'run':
                    problems = node.issues()
                    if problems:
                        message = 'Run 불가: ' + '; '.join(problems)
                    else:
                        node.request_run(menu.run_settings())
                        message = ''
    except KeyboardInterrupt:
        pass
    except OSError as error:
        exit_code = 1
        if node:
            node.log(f'TUI 입출력 종료: {error}')
    finally:
        signal.signal(signal.SIGINT, signal.SIG_IGN)
        try:
            console.print('시험을 정리하고 종료합니다…')
        except OSError:
            pass
        if node and node.busy and rclpy.ok():
            node.request_stop()
            deadline = time.monotonic() + 30.0
            while node.busy and time.monotonic() < deadline and rclpy.ok():
                time.sleep(0.05)
            if node.busy:
                exit_code = 1
                node.log('시나리오 정리 시간 초과; 소유 프로세스 종료를 계속합니다.')
        if manager:
            cleanup_errors = manager.close()
            if cleanup_errors:
                exit_code = 1
        if executor:
            executor.shutdown()
        if spin_thread and spin_thread.ident is not None:
            spin_thread.join()
        if node:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        signal.signal(signal.SIGTERM, old_term)
        signal.signal(signal.SIGHUP, old_hup)
        signal.signal(signal.SIGINT, old_int)
        os.close(lock_fd)
    return exit_code
