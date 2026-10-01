from io import StringIO

import pyte
import pytest
from rich.console import Console
from rich.cells import cell_len

from control_tuning.terminal import ScreenRenderer, tui_screen, ENTER_TUI, LEAVE_TUI
from control_tuning.tui import Editor, layout
from control_tuning.schema import GROUPS


def test_drive_mode_filters_editable_fields_and_preserves_hidden_values(editor):
    editor.key('3')
    editor.commit('speed')
    assert [f.key for f in editor.fields] == [
        'drive_control_mode', 'drive_motor_waveform', 'governor_deceleration_slew_mps2',
        'governor_acceleration_slew_mps2']
    editor.key('down')
    editor.key('down')
    editor.commit(.3)
    editor.key('down')
    editor.commit(1.2)
    # MANUAL uses the rolling floor in every calibration; only the active
    # waveform's minimum current is editable.
    editor.key('5')
    assert [f.key for f in editor.fields] == [
        'drive_min_current_sine_a', 'drive_start_step', 'drive_hold_on_speed_mps',
        'drive_hold_off_speed_mps']
    editor.key('4')
    assert {f.key for f in editor.fields} == {'drive_current_limit_a', 'drive_motor_speed_limit_rpm'}
    original_pi = editor.draft['drive_speed_kp_a_per_mps']
    editor.commit(10)
    editor.key('s')
    assert not editor.error
    saved = editor.presets.load('airborne')
    assert saved['governor_deceleration_slew_mps2'] == .3
    assert saved['governor_acceleration_slew_mps2'] == 1.2
    assert saved['drive_speed_kp_a_per_mps'] == original_pi
    editor.key('3')
    editor.commit('torque')
    assert 'drive_speed_kp_a_per_mps' in {f.key for f in editor.fields}
    assert 'drive_coast_release_error_mps' not in {f.key for f in editor.fields}
    assert 'governor_acceleration_slew_mps2' not in {f.key for f in editor.fields}
    assert 'drive_ff_current_gain_sine_mps2_per_a' in {f.key for f in editor.fields}
    assert 'drive_ff_current_gain_square_mps2_per_a' not in {f.key for f in editor.fields}
    editor.commit('external_current')
    assert [f.key for f in editor.fields] == ['drive_control_mode', 'drive_motor_waveform']
    editor.key('5')
    assert {'drive_reverse_speed_mps', 'drive_reverse_debounce_frames',
            'drive_collapse_hold_frames'} <= {f.key for f in editor.fields}


def test_waveform_switch_shows_its_own_values_and_keeps_the_other(editor):
    editor.key('3')
    editor.commit('torque')
    editor.key('down')
    assert editor.field.key == 'drive_motor_waveform'
    editor.key('enter')  # sine -> square
    assert editor.draft['drive_motor_waveform'] == 'square'
    assert 'drive_ff_current_gain_square_mps2_per_a' in {f.key for f in editor.fields}
    editor.key('5')
    assert editor.field.key == 'drive_min_current_square_a'
    assert editor.draft['drive_min_current_square_a'] == 12.
    editor.commit(10)
    editor.key('s')
    assert not editor.error
    saved = editor.presets.load('airborne')
    assert saved['drive_motor_waveform'] == 'square'
    assert saved['drive_min_current_square_a'] == 10. and saved['drive_min_current_sine_a'] == 0.
    # 0 frames is a valid "observe only" collapse reaction.
    editor.selected = [f.key for f in editor.fields].index('drive_collapse_hold_frames')
    editor.commit(0)
    editor.key('s')
    assert not editor.error


@pytest.mark.parametrize('width,height', [(80, 24), (110, 32), (140, 42)])
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
        assert '공중' in text and '주행' in text and '접지 정지' not in text
        if key == 'enter':
            assert text.index('프리셋  ') < text.index('1 조향응답')
        else:
            assert all(label not in text for label in tuple(f'{i + 1} {label}' for i, label in enumerate(GROUPS)))
        assert '선택 안 함' not in text
        assert '가져올 값 확인' not in text
        assert '프리셋 수정 20' not in text
        assert editor.modal is None
        assert not editor.presets.directory.exists()
        assert not store.path.exists()
        renderer.redraw(layout(editor, None, width, height), 0)
        assert not output.getvalue()
    assert editor.preset_condition == 'airborne'
    assert 'S 저장' in text
    assert '조향 P 게인' in text and 'A/deg' in text and 'A/(deg/s)' in text


def test_edit_save_and_discard(store, editor):
    for key in ['enter', '4', '0', 'enter']:
        editor.key(key)
    assert editor.draft['steering_kp_a_per_deg'] == 40
    assert not store.path.exists()
    editor.key('s')
    assert editor.modal is None
    assert editor.presets.load('airborne')['steering_kp_a_per_deg'] == 40
    assert not editor.preset_dirty
    assert not store.path.exists()
    editor.key('b')
    assert editor.draft['steering_kp_a_per_deg'] == 32
    editor.key('q')
    assert not editor.done
    editor.key('escape')
    editor.key('x')
    editor.key('q')
    assert editor.done


def test_startup_mode_and_invalid_edits(store, editor):
    editor.key('1')
    editor.key('enter')
    assert editor.draft['steering_control_mode'] == 'position'
    editor.key('a')
    assert editor.modal is None and not editor.error
    editor.key('1')
    editor.key('down')
    editor.commit(0)
    editor.key('s')
    assert editor.error and editor.modal is None
    editor.key('x')
    editor.key('s')
    assert editor.presets.load('airborne')['steering_control_mode'] == 'position'
    assert store.saved['steering_control_mode'] == 'torque'


@pytest.mark.parametrize('width,height', [(80, 24), (110, 32), (140, 42)])
def test_terminal_navigation_scroll_and_incremental_updates(store, editor, width, height):
    output = StringIO()
    console = Console(file=output, force_terminal=True, width=width, height=height, color_system='truecolor')
    renderer = ScreenRenderer(console)
    screen = pyte.Screen(width, height)
    stream = pyte.Stream(screen)
    editor.key('1')
    for group in range(len(GROUPS)):
        if group:
            editor.key('right')
        assert editor.group == group
        for _ in editor.fields:
            renderer.redraw(layout(editor, None, width, height), 0)
            stream.feed(output.getvalue())
            output.seek(0)
            output.truncate(0)
            text = '\n'.join(screen.display)
            assert 'CONTROL TUNING' in text
            tabs = next(line for line in screen.display if '1 조향응답' in line)
            assert all(f'{i + 1} {label}' in tabs for i, label in enumerate(GROUPS))
            assert f'[{group + 1} {GROUPS[group]}]' in tabs
            keys = next(line for line in screen.display if '←→ 분류' in line)
            assert all(hint in keys for hint in ('↑↓ 항목', 'Enter 수정', 'S 저장', 'P 프리셋', '? 도움말', 'Q 종료'))
            assert 'A 현재 적용' not in text
            assert 'S 저장' in text
            assert '오프라인' in text
            assert editor.field.label in text  # full label remains in the help panel
            assert editor.field.native in text  # ROS name must be visible without opening help
            assert '›' in text
            renderer.redraw(layout(editor, None, width, height), 0)
            assert output.getvalue() == ''
            editor.key('down')
    editor.key('right')
    assert editor.group == 0
    editor.key('left')
    assert editor.group == len(GROUPS) - 1
    editor.key('1')
    editor.key('down')
    editor.key('enter')
    renderer.redraw(layout(editor, None, width, height), 0)
    stream.feed(output.getvalue())
    assert '새 값:' in '\n'.join(screen.display)
    assert '\x1b[2J' not in output.getvalue()


def test_discard_confirmation_and_terminal_cleanup(store, editor):
    editor.commit(40)
    editor.key('q')
    assert editor.modal['action'] == 'quit'
    output = StringIO()
    console = Console(file=output, force_terminal=True, width=80, height=24)
    with pytest.raises(RuntimeError):
        with tui_screen(console):
            raise RuntimeError('test cleanup')
    assert ENTER_TUI in output.getvalue()
    assert output.getvalue().endswith(LEAVE_TUI)
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
        if not f.unit:
            continue
        row = next(line for line in lines if f.label in line and f.unit in line and '프리셋 저장값' not in line)
        assert cell_len(row[:row.index(f.unit)]) == unit_column
    assert next(i for i, line in enumerate(lines) if 'ROS:' in line) < 20
    assert '프리셋 수정 20' not in output.getvalue()


@pytest.mark.parametrize('width,height', [(80, 24), (80, 35), (140, 32)])
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
