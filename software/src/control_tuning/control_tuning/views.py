"""Compact terminal views with stable columns and contextual detail."""

from rich import box
from rich.console import Group
from rich.layout import Layout
from rich.padding import Padding
from rich.rule import Rule
from rich.table import Table
from rich.text import Text

from .conditions import CONDITION_KEYS, condition_label
from .schema import FIELDS, GROUPS, STEERING, encode, equivalent, format_value, range_hint


NUMERIC_COLUMNS = {'편집값', '현재값', '저장값', '이전', '변경 후'}


def _table(*columns):
    table = Table(box=box.SIMPLE_HEAD, expand=True, padding=(0, 1), pad_edge=False,
                  show_edge=False, border_style='bright_black', header_style='dim')
    for label, width in columns:
        table.add_column(label, width=width, ratio=1 if width is None else None,
                         no_wrap=True, overflow='ellipsis',
                         justify='right' if label in NUMERIC_COLUMNS else 'left')
    return table


def _conversion(editor):
    f = editor.field
    if f.key not in ('p_gain_a_per_deg', 'd_gain_a_per_deg_s',
                     'peak_current_command_a', 'continuous_current_command_a'):
        return ''
    try:
        raw = encode(editor.draft, editor.store.saved_native[STEERING])[STEERING][f.native]
        unit = {'p_gain_a_per_deg': 'deg', 'd_gain_a_per_deg_s': 'raw/(rad/s)'}.get(f.key, 'raw')
        converted = f'ROS 변환: {format_value(raw)} {unit}'
        if f.key == 'p_gain_a_per_deg':
            converted += f' · 오차 0.1° → P {editor.draft[f.key] * .1:.3g} A'
        elif f.key == 'd_gain_a_per_deg_s':
            converted += f' · 60°/s → D {editor.draft[f.key] * 60:.3g} A'
        return converted
    except ValueError:
        return 'ROS 변환: 입력값을 확인하세요.'


def _detail(editor):
    f = editor.field
    hint = range_hint(f, editor.draft, editor.store.saved_native[STEERING])
    lines = [
        Text(f'{f.label}  · ' + ('A로 적용' if f.live else 'S 저장 후 재시작'), style='bold'),
        Text('ROS: ' + f.native, style='dim', no_wrap=True),
        Text('허용 범위: ' + hint + (' ' + f.unit if f.unit else ''), style='dim', no_wrap=True),
    ]
    if editor.editing is not None:
        lines.append(Text('새 값: ' + (editor.editing or '…') + '▏', style='bold cyan'))
    else:
        lines.append(Text(f'프리셋 저장값: {format_value(editor.preset_baseline[f.key])} {f.unit}', style='dim'))
    if editor.help:
        lines.append(Text(f.help))
        conversion = _conversion(editor)
        if conversion:
            lines.append(Text(conversion, style='dim'))
    return Group(*lines)


def _parameters(editor, live, width, visible):
    columns = [('항목', None)]
    if width >= 132:
        columns.append(('ROS 파라미터', 32))
    columns += [('단위', 9), ('편집값', 10), ('현재값', 10)]
    if width >= 104:
        columns.append(('저장값', 10))
    table = _table(*columns)
    offset = max(0, editor.selected - visible + 1)
    for index, f in enumerate(editor.fields[offset:offset + visible], offset):
        selected = index == editor.selected
        changed = not equivalent(editor.draft[f.key], editor.preset_baseline[f.key])
        row = [Text(('› ' if selected else '  ') + f.label + (' *' if changed else ''),
                    style='bold cyan' if selected else 'white')]
        if width >= 132:
            row.append(Text(f.native, style='dim'))
        row += [Text(f.unit, style='dim'),
                Text(format_value(editor.draft[f.key]), style='bold cyan' if changed else 'bold'),
                Text(format_value(live.get(f.key)), style='dim')]
        if width >= 104:
            row.append(Text(format_value(editor.preset_baseline[f.key]), style='dim'))
        table.add_row(*row, style='on #18232b' if selected else '')
    return table


def _review(editor, visible):
    modal = editor.modal
    action = modal['action']
    if action in ('quit', 'reload', 'preset_switch'):
        operation = {'quit': '종료', 'reload': '파일 다시 읽기',
                     'preset_switch': f'{condition_label(modal.get("condition"))} 프리셋으로 변경'}[action]
        return Text(f'{operation}\n\n저장하지 않은 수정값을 버립니다.', style='yellow')
    title = '현재 조향에 적용'
    table = _table(('항목', None), ('이전', 12), ('변경 후', 12))
    count = max(1, visible - 2)
    offset = min(editor.modal_offset, max(0, len(modal['rows']) - count))
    for label, before, after in modal['rows'][offset:offset + count]:
        table.add_row(label, format_value(before), Text(format_value(after), style='bold cyan'))
    more = f'  {offset + 1}–{min(offset + count, len(modal["rows"]))}/{len(modal["rows"])}' if len(modal['rows']) > count else ''
    return Group(Text(title + more, style='bold'), Text(''), table)


def _keys(editor, choosing):
    if editor.modal:
        return 'Enter 확인   Esc 취소' + ('   ↑↓ 목록 이동' if editor.modal.get('rows') else '')
    if choosing:
        return '←→ 프리셋   Enter 선택   Q 종료' + ('   Esc 돌아가기' if editor.preset_condition else '')
    if editor.editing is not None:
        return 'Enter 입력 완료   Esc 취소   Backspace 지우기'
    if editor.help:
        return ('L 현재값   B 기본값   X 저장값   R 조회   F 파일 다시 읽기\n'
                '? 도움말 닫기   Q 종료')
    return ('1–4 분류   ↑↓ 항목   Enter 수정   P 프리셋   ? 도움말   Q 종료\n'
            'S 프리셋 저장   A 현재 적용')


def layout(editor, runtime, width, height):
    if width < 68 or height < 24:
        message = 'Control Tuning\n터미널을 68열 × 24행 이상으로 넓혀 주세요.\n'
        message += ('Enter 종료 · Esc 취소' if editor.modal and editor.modal['action'] == 'quit' else 'Q 종료')
        return Text(message, style='yellow')
    choosing = editor.selecting_preset or editor.preset_condition is None
    detail_size = 10 if editor.help and not choosing and not editor.modal else 5
    tabs_size = 0 if choosing else 2
    available_body = height - detail_size - 8 - tabs_size
    body_size = min(available_body, 9) if not editor.modal else available_body
    view = Layout()
    view.split_column(Layout(name='header', size=2), Layout(name='preset', size=1),
                      Layout(name='tabs', size=2, visible=not choosing), Layout(name='body', size=body_size),
                      Layout(name='detail', size=detail_size),
                      Layout(name='spacer', visible=available_body > body_size),
                      Layout(name='status', size=2),
                      Layout(name='keys', size=3))
    visible = max(1, body_size - 2)
    live = editor.live_values(runtime)
    connected = runtime and runtime.fresh_values(STEERING)
    connection = ('조향 연결됨' if connected else '조향 연결 대기') if runtime else '오프라인'
    pending = sum(not f.live and f.key in live and not equivalent(live[f.key], editor.preset_baseline[f.key])
                  for f in FIELDS) if editor.preset_baseline and not choosing else 0
    header = Table.grid(expand=True)
    header.add_column(ratio=1)
    header.add_column(justify='right')
    state = connection + (f' · 재시작 항목 차이 {pending}' if pending else '')
    header.add_row(Text('CONTROL TUNING', style='bold'), Text(state, style='dim'))
    view['header'].update(Group(Padding(header, (0, 1)), Rule(style='bright_black')))
    preset_row = Table.grid(expand=True)
    preset_row.add_column(ratio=1)
    preset_row.add_column(justify='right')
    presets = Text('프리셋  ')
    for index, condition in enumerate(CONDITION_KEYS):
        active = index == editor.preset_index if choosing else condition == editor.preset_condition
        presets.append(('[' if active else ' ') + condition_label(condition) + (']' if active else ' ') + '  ',
                       style='bold cyan' if active else 'dim')
    state = '' if choosing else ('수정됨 *' if editor.preset_dirty else
                                 '새 프리셋' if editor.presets.entry(editor.preset_condition) is None else '')
    preset_row.add_row(presets, Text(state, style='dim'))
    view['preset'].update(Padding(preset_row, (0, 1)))
    tabs = Text()
    for index, label in enumerate(GROUPS):
        tabs.append(f' {index + 1} {label} ',
                    style='bold black on cyan' if not choosing and not editor.modal and index == editor.group else 'dim')
        if index < len(GROUPS) - 1:
            tabs.append(' ')
    view['tabs'].update(Padding(tabs, (0, 1)))
    if editor.modal:
        body = _review(editor, visible)
        notes = {'apply': '조향 튜닝만 적용합니다. 운용·수신 시간은 저장 후 해당 프리셋으로 재시작하세요.',
                 'preset_switch': '보관하려면 Esc로 돌아가 S로 저장하세요.',
                 'quit': '보관하려면 Esc로 돌아가 S로 저장하세요.',
                 'reload': '보관하려면 Esc로 돌아가 S로 저장하세요.'}
        detail = Text(notes[editor.modal['action']], style='dim')
    elif choosing:
        body = Text('프리셋을 선택하세요.', style='dim')
        detail = Text('')
    else:
        body = _parameters(editor, live, width, visible)
        detail = _detail(editor)
    view['body'].update(Padding(body, (0, 1)))
    view['detail'].update(Text('') if choosing and not editor.modal else
                          Group(Rule(style='bright_black'), Padding(detail, (0, 1))))
    view['spacer'].update(Text(''))
    message = Text(editor.message, style='red' if editor.error else 'dim')
    if editor.help and not editor.error and not editor.modal and not choosing:
        message = Text('설정: ' + str(editor.presets.path_for(editor.preset_condition)), style='dim')
    view['status'].update(Padding(message, (0, 1)))
    view['keys'].update(Group(Rule(style='bright_black'), Padding(Text(_keys(editor, choosing)), (0, 1))))
    return view
