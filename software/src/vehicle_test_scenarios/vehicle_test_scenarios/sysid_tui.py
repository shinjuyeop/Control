"""Rich console UI; ROS publication continues while the operator uses the menu."""

from contextlib import contextmanager
import fcntl
from itertools import zip_longest
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
from rich.console import Console
from rich.live import Live
from rich.panel import Panel
from rich.table import Table
from rich.text import Text

from .sysid_processes import DEFAULT_RECORDS_ROOT, ProcessManager
from .sysid_profiles import RunSettings, SCENARIO_NAMES
from .sysid_runtime import SysidNode


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
            # A disconnected PTY has no terminal attributes left to restore.
            pass


def gate_text(row, topic_rows):
    name, go, detail = row
    if name in topic_rows:
        icon, detail = topic_rows[name]
        if go and icon == '—':
            return Text(f'○ {detail} · 실행 가능', style='dim')
    elif detail.startswith(name + ': '):
        detail = detail[len(name) + 2:]
    elif detail.startswith(name + ' '):
        detail = detail[len(name) + 1:]
    return Text(f"{'🟢 GO' if go else '🔴 NO-GO'}  {detail}",
                style='green' if go else 'bold red')


def dashboard(snapshot, selection, speed_text=None, message='', height=40,
              records_root=DEFAULT_RECORDS_ROOT):
    actual = '—' if snapshot['speed'] is None else f"{snapshot['speed']:.1f}"
    rpm = '—' if snapshot['rpm'] is None else f"{snapshot['rpm']:.0f}"
    current = '—' if snapshot['current'] is None else f"{snapshot['current']:.0f}"
    steering = '—' if snapshot['steering'] is None else f"{snapshot['steering']:.2f}"
    steering_target = (
        '모터 DISABLE 요청' if snapshot['output'].steering_deg is None else
        f"목표 {snapshot['output'].steering_deg:.2f}° (앞바퀴 등가각)")
    countdown = f" · {snapshot['countdown']}초" if snapshot['countdown'] is not None else ''
    title = Text(
        f"🚗 AUTO SYSID   |   {snapshot['mode']}   |   {snapshot['label']}{countdown}\n"
        f"차속 {actual} km/h   목표 {snapshot['output'].speed_kph:.1f} km/h   "
        f"RPM {rpm}   상전류 {current} A\n"
        f"조향 실제 {steering}°   {steering_target}",
        style='bold',
    )
    display_gates = list(snapshot['gates'])
    shown_problems = {detail for _, go, detail in display_gates if not go}
    display_gates.extend(
        ('실행 조건', False, issue)
        for issue in snapshot['issues'] if issue not in shown_problems)
    topic_rows = {name: (icon, detail) for name, icon, detail in snapshot['rows']}
    gates = Table(expand=True, box=None)
    gates.add_column('토픽 / 실행 조건', ratio=3)
    gates.add_column('판정', ratio=2)
    for row in display_gates:
        gates.add_row(row[0], gate_text(row, topic_rows))
    all_go = all(go for _, go, _ in display_gates)
    drivers = Table(expand=True, box=None)
    drivers.add_column('드라이버')
    drivers.add_column('상태')
    for name, state in snapshot['drivers']:
        drivers.add_row(name, Text(state))
    columns = Table.grid(expand=True)
    columns.add_column(ratio=3)
    columns.add_column(ratio=2)
    columns.add_row(
        Panel(gates, title='RUN GO' if all_go else 'RUN NO-GO',
              border_style='green' if all_go else 'red'),
        Panel(drivers, title='외부 드라이버'),
    )
    choices = '\n'.join('   '.join(
        ('▶ ' if i == selection.scenario else '') + f'[{i}] {SCENARIO_NAMES[i]}'
        for i in row) for row in ((1, 2, 3, 4), (5, 6)))
    settings = Text(
        choices + '\n'
        + f'최고 목표속도 {selection.max_speed_kph:g} km/h  ·  '
        + selection.description() + '\n'
        + '1–6 선택   V 최고속도   R Run   S / Space Stop   Q 종료',
    )
    if speed_text is not None:
        settings.append('\n최고 목표속도 입력: ' + speed_text + '▌  (Enter 적용 / Esc 취소)', style='bold cyan')
    if message:
        settings.append('\n' + message, style='yellow')
    if snapshot['run_warning']:
        settings.append('\n주의: ' + snapshot['run_warning'], style='bold yellow')
    if snapshot['terminal_state'] in {'ABORTED', 'ERROR'} and snapshot['finish_reason']:
        settings.append('\n중단 원인: ' + snapshot['finish_reason'], style='bold red')
    recording = ('● REC  ' if snapshot['recording'] else '') + (snapshot['run_dir'] or str(records_root))
    if height < 36:
        pairs = Table.grid(expand=True)
        pairs.add_column(ratio=1)
        pairs.add_column(ratio=1)
        midpoint = (len(display_gates) + 1) // 2

        def compact(row):
            if row is None:
                return Text('')
            status = gate_text(row, topic_rows)
            return Text.assemble(f'{row[0]}  ', status)

        for left, right in zip_longest(
                display_gates[:midpoint], display_gates[midpoint:]):
            pairs.add_row(compact(left), compact(right))
        driver_summary = ' · '.join(
            name + (' ✓' if state == '감지됨' else ' 대기')
            for name, state in snapshot['drivers'])
        return vertical_stack(
            Panel(title, border_style='cyan'),
            Panel(settings, title='시험 선택'),
            Panel(pairs, title='RUN GO' if all_go else 'RUN NO-GO',
                  border_style='green' if all_go else 'red'),
            Text(driver_summary, overflow='ellipsis', no_wrap=True),
            Text(recording, overflow='ellipsis', no_wrap=True),
            Text('\n'.join(snapshot['logs'][-2:]), overflow='ellipsis', no_wrap=True),
        )
    return vertical_stack(
        Panel(title, border_style='cyan'),
        Panel(settings, title='시험 선택'),
        columns,
        Panel(Text(recording), title='MCAP'),
        Panel(Text('\n'.join(snapshot['logs'])), title='실행 로그'),
    )


def main(*, use_sim_time=False, records_root=DEFAULT_RECORDS_ROOT,
         require_ezkontrol_status=True):
    console = Console()
    if not sys.stdin.isatty():
        console.print('터미널에서 python3 auto_sysid.py로 실행하세요.')
        return 2
    lock_path = Path('/tmp') / f'auto_sysid_{os.getuid()}_{os.environ.get("ROS_DOMAIN_ID", "0")}.lock'
    lock_fd = os.open(lock_path, os.O_CREAT | os.O_RDWR, 0o600)
    try:
        fcntl.flock(lock_fd, fcntl.LOCK_EX | fcntl.LOCK_NB)
    except BlockingIOError:
        os.close(lock_fd)
        console.print('같은 ROS domain에서 AUTO SYSID가 이미 실행 중입니다.')
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
        # Keep ROS alive until the final stop command and recorder flush are complete.
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
        selection = RunSettings()
        speed_text = None
        message = ''
        with keyboard(), Live(console=console, screen=True, auto_refresh=False) as live:
            while not quitting.is_set():
                live.update(dashboard(node.snapshot(), selection, speed_text, message,
                                      height=console.size.height,
                                      records_root=records_root), refresh=True)
                readable, _, _ = select.select([sys.stdin], [], [], 0.2)
                if not readable:
                    continue
                raw = os.read(sys.stdin.fileno(), 1)
                if not raw:
                    break
                char = raw.decode('ascii', errors='ignore')
                if not char:
                    continue
                if char.lower() == 'q':
                    break
                if char.lower() == 's' or char == ' ':
                    node.request_stop()
                    continue
                if speed_text is not None:
                    if char in '\r\n':
                        try:
                            selection = RunSettings(selection.scenario, float(speed_text))
                            message = ''
                            speed_text = None
                        except ValueError as error:
                            message = str(error)
                    elif char == '\x1b':
                        speed_text = None
                    elif char in ('\x7f', '\b'):
                        speed_text = speed_text[:-1]
                    elif char in '0123456789.':
                        speed_text += char
                    continue
                if node.busy:
                    message = '실행·저장이 끝난 뒤 설정을 바꿀 수 있습니다.'
                    continue
                if char in '123456':
                    selection = RunSettings(int(char), selection.max_speed_kph)
                    message = ''
                elif char.lower() == 'v':
                    speed_text = ''
                elif char.lower() == 'r':
                    node.request_run(selection)
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
            console.print('기록을 저장하고 종료합니다…')
        except OSError:
            pass
        if node and node.output_active and rclpy.ok():
            node.request_stop()
            # Keep the existing governor/MCU command path alive while the stop reaches it.
            deadline = time.monotonic() + 1.0
            while time.monotonic() < deadline and rclpy.ok():
                time.sleep(0.05)
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
        try:
            if node and node.run_dir:
                console.print(Text('기록: ' + node.run_dir))
            for error in cleanup_errors:
                console.print(Text('정리 오류: ' + error, style='red'))
        except OSError:
            pass
    return exit_code
