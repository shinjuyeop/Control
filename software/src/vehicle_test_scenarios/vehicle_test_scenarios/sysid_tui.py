"""Rich console UI; ROS publication continues while the operator uses the menu."""

from contextlib import contextmanager
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
from rclpy.signals import SignalHandlerOptions
from rich.console import Console, Group
from rich.live import Live
from rich.panel import Panel
from rich.table import Table
from rich.text import Text

from .sysid_processes import ProcessManager
from .sysid_profiles import RunSettings
from .sysid_runtime import SysidNode


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


def dashboard(snapshot, selection, speed_text=None, message='', height=40):
    actual = '—' if snapshot['speed'] is None else f"{snapshot['speed']:.1f}"
    rpm = '—' if snapshot['rpm'] is None else f"{snapshot['rpm']:.0f}"
    current = '—' if snapshot['current'] is None else f"{snapshot['current']:.0f}"
    steering = '—' if snapshot['steering'] is None else f"{snapshot['steering']:.2f}"
    countdown = f" · {snapshot['countdown']}초" if snapshot['countdown'] is not None else ''
    title = Text(
        f"🚗 AUTO SYSID   |   {snapshot['mode']}   |   {snapshot['label']}{countdown}\n"
        f"차속 {actual} km/h   목표 {snapshot['output'].speed_kph:.1f} km/h   "
        f"RPM {rpm}   상전류 {current} A\n"
        f"조향 실제 {steering}°   목표 {snapshot['output'].steering_deg:.2f}° (앞바퀴 등가각)",
        style='bold',
    )
    sensors = Table(expand=True, box=None)
    sensors.add_column('필수 데이터', ratio=3)
    sensors.add_column('수신', ratio=2)
    for topic, icon, detail in snapshot['rows']:
        sensors.add_row(topic, Text(f'{icon} {detail}'))
    drivers = Table(expand=True, box=None)
    drivers.add_column('드라이버')
    drivers.add_column('상태')
    for name, state in snapshot['drivers']:
        drivers.add_row(name, Text(state))
    columns = Table.grid(expand=True)
    columns.add_column(ratio=3)
    columns.add_column(ratio=2)
    columns.add_row(Panel(sensors, title='실시간 수신'), Panel(drivers, title='자동 실행'))
    choices = '   '.join(
        ('▶ ' if i == selection.scenario else '') + f'[{i}] 시나리오 {i}' for i in range(1, 5))
    settings = Text(
        choices + '\n'
        + f'최고 목표속도 {selection.max_speed_kph:g} km/h (상한 15)  ·  '
        + selection.description() + '\n'
        + '1–4 선택   V 최고속도   R Run   S / Space Stop   Q 종료',
    )
    if speed_text is not None:
        settings.append('\n최고 목표속도 입력: ' + speed_text + '▌  (Enter 적용 / Esc 취소)', style='bold cyan')
    if message:
        settings.append('\n' + message, style='yellow')
    if snapshot['issues']:
        settings.append('\nRun 대기: ' + '; '.join(snapshot['issues'][:3]), style='yellow')
    recording = ('● REC  ' if snapshot['recording'] else '') + (snapshot['run_dir'] or '/home/kai/records')
    if height < 36:
        pairs = Table.grid(expand=True)
        pairs.add_column(ratio=1)
        pairs.add_column(ratio=1)
        for left, right in zip(snapshot['rows'][:4], snapshot['rows'][4:]):
            pairs.add_row(Text(f'{left[1]} {left[0]} · {left[2]}'),
                          Text(f'{right[1]} {right[0]} · {right[2]}'))
        driver_summary = ' · '.join(
            name + (' ✓' if state in ('실행 중', '기존 프로세스 사용') else ' 대기')
            for name, state in snapshot['drivers'])
        return Group(
            Panel(title, border_style='cyan'),
            Panel(settings, title='시험 선택'),
            Panel(pairs, title='실시간 수신'),
            Text(driver_summary, overflow='ellipsis', no_wrap=True),
            Text(recording, overflow='ellipsis', no_wrap=True),
            Text('\n'.join(snapshot['logs'][-2:]), overflow='ellipsis', no_wrap=True),
        )
    return Group(
        Panel(title, border_style='cyan'),
        Panel(settings, title='시험 선택'),
        columns,
        Panel(Text(recording), title='MCAP'),
        Panel(Text('\n'.join(snapshot['logs'])), title='실행 로그'),
    )


def main():
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
        node = SysidNode()
        executor = SingleThreadedExecutor()
        executor.add_node(node)
        spin_thread = threading.Thread(target=executor.spin, name='sysid-ros')
        spin_thread.start()
        manager = ProcessManager(node)
        manager.start()
        selection = RunSettings()
        speed_text = None
        message = ''
        with keyboard(), Live(console=console, screen=True, auto_refresh=False) as live:
            while not quitting.is_set():
                live.update(dashboard(node.snapshot(), selection, speed_text, message,
                                      height=console.size.height), refresh=True)
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
                if char in '1234':
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
            console.print('기록과 직접 시작한 드라이버를 정리합니다…')
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
