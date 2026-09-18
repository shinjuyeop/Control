"""Terminal output regressions for incremental viewport rendering."""

from io import StringIO

from rich.console import Console, Group
from rich.text import Text

from vehicle_test_scenarios.sysid_tui import ScreenRenderer


class TerminalCapture(StringIO):
    def __init__(self):
        super().__init__()
        self.writes = []

    def write(self, value):
        if value:
            self.writes.append(value)
        return super().write(value)


def test_redraw_batches_changed_rows_and_skips_identical_frames():
    output = TerminalCapture()
    console = Console(file=output, force_terminal=True, width=20, height=3)
    renderer = ScreenRenderer(console)
    content = Text('첫 번째\n두 번째\n세 번째')
    renderer.redraw(content, 0)
    assert len(output.writes) == 1
    assert '\x1b[2J' not in output.writes[0]
    assert '\n' not in output.writes[0]  # No scrolling at the bottom terminal row.
    renderer.redraw(content, 0)
    assert len(output.writes) == 1

    renderer.redraw(Text('첫 번째\n변경\n세 번째'), 0)
    assert len(output.writes) == 2
    assert output.writes[-1] == '\x1b[2;1H변경' + ' ' * 16


def test_redraw_erases_removed_rows_including_an_empty_frame():
    output = TerminalCapture()
    renderer = ScreenRenderer(Console(file=output, force_terminal=True, width=10, height=3))
    renderer.redraw(Text('first\nsecond\nthird'), 0)
    renderer.redraw(Text('first'), 0)
    assert output.writes[-1] == '\x1b[2;1H' + ' ' * 10 + '\x1b[3;1H' + ' ' * 10
    renderer.redraw(Group(), 0)
    assert output.writes[-1] == '\x1b[1;1H' + ' ' * 10


def test_scroll_and_resize_repaint_the_correct_viewport():
    output = TerminalCapture()
    console = Console(file=output, force_terminal=True, width=10, height=2)
    renderer = ScreenRenderer(console)
    content = Text('first\nsecond\nthird')
    assert renderer.redraw(content, 99) == 1
    assert output.writes[-1] == '\x1b[1;1Hsecond    \x1b[2;1Hthird     '
    console.width = 12
    console.height = 4
    assert renderer.redraw(content, 1) == 0
    assert len(output.writes) == 2
    assert output.writes[-1] == (
        '\x1b[1;1Hfirst       \x1b[2;1Hsecond      '
        '\x1b[3;1Hthird       \x1b[4;1H            '
    )


def test_style_changes_are_redrawn_even_when_text_is_identical():
    output = TerminalCapture()
    console = Console(
        file=output, force_terminal=True, color_system='standard', no_color=False,
        width=10, height=1,
    )
    renderer = ScreenRenderer(console)
    renderer.redraw(Text('GO', style='green'), 0)
    renderer.redraw(Text('GO', style='red'), 0)
    assert len(output.writes) == 2
    assert '\x1b[31m' in output.writes[-1]
    assert '\x1b[2J' not in output.writes[-1]


def test_choice_labels_remain_readable_in_narrow_two_column_settings():
    from vehicle_test_scenarios.sysid_tui import MenuState, handle_key, settings_panel
    for width in (71, 80, 110):
        menu = MenuState()
        menu.selected_key = 'steering_step'
        menu.focus = 'parameters'
        menu.field_index = len(menu.selected_spec.fields) + 1
        menu.test_condition = 'ground_stationary'
        for editing in (False, True):
            if editing:
                handle_key(menu, 'enter')
            output = StringIO()
            Console(file=output, width=width, force_terminal=False).print(settings_panel(menu))
            rendered = output.getvalue()
            assert 'MCAP 녹화' in rendered
            assert '시험 조건' in rendered
            assert '접지 정지' in rendered
            assert 'ON M' not in rendered
            assert '정지 C' not in rendered
            assert ('↕' in rendered) == editing


def test_notices_are_in_vehicle_state_and_help_has_no_duplicate_status():
    from types import SimpleNamespace
    from vehicle_test_scenarios.sysid_tui import MenuState, dashboard, footer_panel, handle_key
    menu = MenuState()
    snapshot = dict(output=SimpleNamespace(steering_deg=0., speed_kph=0.), mode='MANUAL',
                    speed=0., steering=0., rpm=0., current=0., countdown=None, label='정지',
                    gates=[], rows=[], issues=[], optional_topics=set(),
                    busy=False, recording=False, run_dir='', settings=menu.run_settings(),
                    run_warning='', terminal_state='ABORTED', finish_reason='사용자 Stop')
    output = StringIO()
    Console(file=output, width=80, force_terminal=False).print(dashboard(snapshot, menu))
    rendered = output.getvalue()
    assert 'VEHICLE STATE · 시험 상태' in rendered
    assert 'VEHICLE TEST' not in rendered
    assert rendered.index('사용자 Stop') < rendered.index('RUN GO')
    assert rendered.count('사용자 Stop') == 1
    output = StringIO()
    console = Console(file=output, width=80, force_terminal=False)
    console.print(footer_panel(snapshot, menu, '/tmp/records'))
    rendered = output.getvalue()
    assert '키 안내' in rendered
    assert all(label not in rendered for label in ('시험 조건', 'MCAP', '사용자 Stop', 'C ', 'M '))
    menu.focus = 'parameters'
    menu.field_index = len(menu.selected_spec.fields)
    handle_key(menu, 'enter')
    output.seek(0)
    output.truncate(0)
    console.print(footer_panel(snapshot, menu, '/tmp/records'))
    assert '↑↓ 값 선택 · Enter 완료 · Esc 취소' in output.getvalue()
