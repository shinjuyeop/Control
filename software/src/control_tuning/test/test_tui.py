from io import StringIO

import pyte
import pytest
from rich.console import Console
from rich.cells import cell_len

from control_tuning.terminal import ScreenRenderer, tui_screen, ENTER_TUI, LEAVE_TUI
from control_tuning.tui import Editor, layout


@pytest.mark.parametrize('width,height', [(68, 24), (80, 24), (110, 32), (140, 42)])
def test_preset_selector_is_first_and_cannot_be_skipped(store, width, height):
    editor = Editor(store)
    output = StringIO()
    console = Console(file=output, force_terminal=True, width=width, height=height)
    renderer = ScreenRenderer(console)
    screen = pyte.Screen(width, height)
    stream = pyte.Stream(screen)
    for key in ['escape', 'right', 'enter']:
        editor.key(key)
        renderer.redraw(layout(editor, None, width, height), 0)
        stream.feed(output.getvalue())
        output.seek(0)
        output.truncate(0)
        text = '\n'.join(screen.display)
        assert '공중' in text and '접지 정지' in text and '주행' in text
        if key == 'enter':
            assert text.index('프리셋  ') < text.index('1 조향 응답')
        else:
            assert all(label not in text for label in ('1 조향 응답', '2 출력', '3 운용', '4 수신'))
        assert '선택 안 함' not in text
        assert '가져올 값 확인' not in text
        assert '프리셋 수정 20' not in text
        assert editor.modal is None
        assert not editor.presets.directory.exists()
        assert not store.path.exists()
        renderer.redraw(layout(editor, None, width, height), 0)
        assert not output.getvalue()
    assert editor.preset_condition == 'airborne'
    assert 'S 프리셋 저장' in text
    assert 'P 게인' in text and 'A/deg' in text and 'A/(deg/s)' in text


def test_edit_save_and_discard(store, editor):
    for key in ['enter', '4', '0', 'enter']:
        editor.key(key)
    assert editor.draft['p_gain_a_per_deg'] == 40
    assert not store.path.exists()
    editor.key('s')
    assert editor.modal is None
    assert editor.presets.load('airborne')['p_gain_a_per_deg'] == 40
    assert not editor.preset_dirty
    assert not store.path.exists()
    editor.key('b')
    assert editor.draft['p_gain_a_per_deg'] == 32
    editor.key('q')
    assert not editor.done
    editor.key('escape')
    editor.key('x')
    editor.key('q')
    assert editor.done


def test_startup_mode_and_invalid_edits(store, editor):
    editor.key('3')
    editor.key('enter')
    assert editor.draft['actuation_mode'] == 'position'
    editor.key('a')
    assert editor.error and '오프라인' in editor.message
    editor.key('1')
    editor.commit(0)
    editor.key('s')
    assert editor.error and editor.modal is None
    editor.key('x')
    editor.key('s')
    assert editor.presets.load('airborne')['actuation_mode'] == 'position'
    assert store.saved['actuation_mode'] == 'torque'


@pytest.mark.parametrize('width,height', [(68, 24), (80, 24), (110, 32), (140, 42)])
def test_terminal_navigation_scroll_and_incremental_updates(store, editor, width, height):
    output = StringIO()
    console = Console(file=output, force_terminal=True, width=width, height=height, color_system='truecolor')
    renderer = ScreenRenderer(console)
    screen = pyte.Screen(width, height)
    stream = pyte.Stream(screen)
    for group in '1234':
        editor.key(group)
        for _ in editor.fields:
            renderer.redraw(layout(editor, None, width, height), 0)
            stream.feed(output.getvalue())
            output.seek(0)
            output.truncate(0)
            text = '\n'.join(screen.display)
            assert 'CONTROL TUNING' in text
            assert 'A 현재 적용' in text and 'Q 종료' in text
            assert 'S 프리셋 저장' in text
            assert '오프라인' in text
            assert editor.field.label in text  # full label remains in the help panel
            assert editor.field.native in text  # ROS name must be visible without opening help
            assert '›' in text
            renderer.redraw(layout(editor, None, width, height), 0)
            assert output.getvalue() == ''
            editor.key('down')
    editor.key('1')
    editor.key('enter')
    renderer.redraw(layout(editor, None, width, height), 0)
    stream.feed(output.getvalue())
    assert '새 값:' in '\n'.join(screen.display)
    assert '\x1b[2J' not in output.getvalue()


def test_modal_review_scroll_and_terminal_cleanup(store, editor):
    for f in editor.fields:
        editor.draft[f.key] += f.step
    # Apply retains the review list; save is now a direct per-preset write.
    editor.modal = dict(action='apply', rows=[(f.label, store.saved[f.key], editor.draft[f.key])
                                            for f in editor.fields])
    output = StringIO()
    console = Console(file=output, force_terminal=True, width=80, height=24)
    with pytest.raises(RuntimeError):
        with tui_screen(console):
            raise RuntimeError('test cleanup')
    assert ENTER_TUI in output.getvalue()
    assert output.getvalue().endswith(LEAVE_TUI)
    editor.key('down')
    assert editor.modal_offset == 1
    editor.key('escape')
    assert editor.modal is None
    assert editor.preset_dirty


def test_80_column_screen_has_aligned_units_and_details_next_to_table(store, editor):
    output = StringIO()
    console = Console(file=output, width=80, height=35, force_terminal=False)
    console.print(layout(editor, None, 80, 35))
    lines = output.getvalue().splitlines()
    header = next(line for line in lines if '항목' in line and '단위' in line)
    unit_column = cell_len(header[:header.index('단위')])
    for f in editor.fields:
        row = next(line for line in lines if f.label in line and f.unit in line and '프리셋 저장값' not in line)
        assert cell_len(row[:row.index(f.unit)]) == unit_column
    assert next(i for i, line in enumerate(lines) if 'ROS:' in line) < 20
    assert '프리셋 수정 20' not in output.getvalue()


@pytest.mark.parametrize('width,height', [(68, 24), (80, 35), (140, 32)])
def test_detailed_help_is_optional_and_preserves_editing_context(store, editor, width, height):
    output = StringIO()
    console = Console(file=output, width=width, height=height, force_terminal=True)
    renderer = ScreenRenderer(console)
    screen = pyte.Screen(width, height)
    stream = pyte.Stream(screen)
    for help_visible in (False, True, False):
        editor.help = help_visible
        renderer.redraw(layout(editor, None, width, height), 0)
        stream.feed(output.getvalue())
        output.seek(0)
        output.truncate(0)
        text = '\n'.join(screen.display)
        assert editor.field.native in text
        assert ('높일수록' in text) == help_visible
        assert 'Q 종료' in text
        assert len(screen.display) == height
