"""Interactive, categorized control settings editor."""

import argparse
import math
import select
import signal
import sys
import time
import yaml

from rich.console import Console
from rich.text import Text

from .longitudinal import LONGITUDINAL
from .schema import (FIELDS, GROUPS, STEERING, GOVERNOR, decode, decode_node,
                     equivalent)
from .storage import SettingsStore
from .conditions import CONDITION_KEYS, condition_label
from .presets import PresetStore
from .terminal import ScreenRenderer, tui_screen, keyboard, read_key
from .views import layout


class Editor:
    def __init__(self, store):
        self.store = store
        self.draft = dict(store.saved)
        self.presets = PresetStore(store)
        self.preset_condition = None
        self.preset_baseline = None
        self.selecting_preset = True
        self.preset_index = 0
        self.group = 0
        self.selected = 0
        self.editing = None
        self.modal = None
        self.message = ''
        self.error = False
        self.help = False
        self.done = False

    @property
    def fields(self):
        return [f for f in FIELDS if f.group == GROUPS[self.group]]

    @property
    def field(self):
        return self.fields[self.selected]

    @property
    def preset_dirty(self):
        if self.preset_condition is None:
            return []
        return [f for f in FIELDS if not equivalent(self.draft[f.key], self.preset_baseline[f.key])]

    def require_preset(self):
        if self.preset_condition is None:
            raise ValueError('먼저 프리셋을 선택하세요. ←→ 선택 · Enter 확인')

    def tell(self, message, error=False):
        self.message, self.error = message, error

    def commit(self, value):
        self.require_preset()
        f = self.field
        if not f.choices:
            value = int(value) if f.integer else float(value)
        candidate = {**self.draft, f.key: value}
        # Single-field edits may temporarily violate coupled limits; validate the
        # complete batch at save so either edit order is possible.
        if not f.choices and (not math.isfinite(value) or value < 0):
            raise ValueError('0 이상의 유한한 숫자를 입력하십시오.')
        self.draft = candidate
        self.editing = None
        self.tell('수정됨')

    def live_values(self, runtime):
        values = {}
        if runtime:
            for name in (STEERING, GOVERNOR, LONGITUDINAL):
                native = runtime.fresh_values(name)
                if native:
                    try:
                        values.update(decode_node(name, native))
                    except (KeyError, ValueError, ZeroDivisionError):
                        pass
        return values

    def review(self, action):
        if self.preset_dirty:
            self.modal = dict(action=action)
        elif action == 'quit':
            self.done = True
        else:
            self.reload()

    def reload(self):
        store = SettingsStore(self.store.path, *self.store.base_paths)
        presets = PresetStore(store)
        self.store, self.presets = store, presets
        self.preset_condition = self.preset_baseline = None
        self.draft = dict(self.store.saved)
        self.selecting_preset, self.preset_index = True, 0
        self.help = False
        self.tell('파일을 다시 읽었습니다.')

    def open_presets(self):
        self.preset_index = CONDITION_KEYS.index(self.preset_condition) if self.preset_condition else 0
        self.selecting_preset = True
        self.help = False
        self.tell('')

    def use_preset(self, condition, values):
        self.draft = dict(values)
        self.preset_condition = condition
        self.preset_baseline = dict(values)
        self.selecting_preset = False
        self.help = False
        self.group, self.selected = 0, 0
        self.tell('')

    def preset_values(self, condition):
        self.presets.check_unchanged(condition)
        created = self.presets.entry(condition) is None
        if created:
            if self.store.has_external_change():
                raise ValueError('시작 설정이 변경되었습니다. F로 다시 읽으세요.')
            values = self.presets.normalize(self.store.saved)
        else:
            values = self.presets.load(condition)
        return values, created

    def choose_preset(self):
        condition = CONDITION_KEYS[self.preset_index]
        if condition == self.preset_condition:
            self.selecting_preset = False
            self.tell('')
            return
        values, created = self.preset_values(condition)
        if self.preset_dirty:
            self.modal = dict(action='preset_switch', condition=condition,
                              normalized=values, created=created)
        else:
            self.use_preset(condition, values)

    def save_preset(self):
        self.require_preset()
        condition = self.preset_condition
        self.presets.check_unchanged(condition)
        if not self.preset_dirty and self.presets.path_for(condition).exists():
            self.tell('프리셋에 저장할 변경이 없습니다.')
            return
        values = self.presets.save(condition, self.draft)
        self.draft, self.preset_baseline = dict(values), dict(values)
        self.tell(f'{condition_label(condition)} 프리셋 저장 완료 · 다음 구동 스택 실행에 반영')

    def confirm(self):
        modal, self.modal = self.modal, None
        action = modal['action']
        if action == 'reload':
            self.reload()
        elif action == 'preset_switch':
            values, created = self.preset_values(modal['condition'])
            if created != modal['created'] or any(not equivalent(values[k], modal['normalized'][k]) for k in values):
                raise ValueError('프리셋이 변경되었습니다. 다시 선택하세요.')
            self.use_preset(modal['condition'], values)
        else:
            self.done = True

    def key(self, key, runtime=None):
        try:
            if self.modal:
                if key == 'escape':
                    self.modal = None
                elif key == 'enter':
                    self.confirm()
                return
            if self.selecting_preset or self.preset_condition is None:
                if key in ('right', 'down', '\t'):
                    self.preset_index = (self.preset_index + 1) % len(CONDITION_KEYS)
                elif key in ('left', 'up'):
                    self.preset_index = (self.preset_index - 1) % len(CONDITION_KEYS)
                elif key == 'enter':
                    self.choose_preset()
                elif key == 'escape' and self.preset_condition is not None:
                    self.selecting_preset = False
                    self.tell('')
                elif key.lower() in ('q', 'f'):
                    self.review('quit' if key.lower() == 'q' else 'reload')
                else:
                    self.tell('←→로 프리셋을 고른 뒤 Enter를 누르세요.')
                return
            if self.editing is not None:
                if key == 'escape':
                    self.editing = None
                elif key == 'enter':
                    self.commit(self.editing)
                elif key == 'backspace':
                    self.editing = self.editing[:-1]
                elif len(key) == 1 and key in '0123456789.eE+-':
                    self.editing += key
                return
            lower = key.lower()
            if key in ('right', '\t'):
                self.group, self.selected = (self.group + 1) % len(GROUPS), 0
            elif key == 'left':
                self.group, self.selected = (self.group - 1) % len(GROUPS), 0
            elif key in ('up', 'scroll_up'):
                if key == 'up' and self.selected == 0:
                    self.open_presets()
                else:
                    self.selected = (self.selected - 1) % len(self.fields)
            elif key in ('down', 'scroll_down'):
                self.selected = (self.selected + 1) % len(self.fields)
            elif key in tuple(str(i + 1) for i in range(len(GROUPS))):
                self.group, self.selected = int(key) - 1, 0
            elif key == 'enter':
                if self.field.choices:
                    choices = self.field.choices
                    self.commit(choices[(choices.index(self.draft[self.field.key]) + 1) % len(choices)])
                else:
                    self.editing = ''
            elif key in ('+', '=', '-') and not self.field.choices:
                self.commit(self.draft[self.field.key] + self.field.step * (-1 if key == '-' else 1))
            elif lower == 'x':
                self.draft[self.field.key] = self.preset_baseline[self.field.key]
                self.tell('선택 항목을 저장값으로 되돌렸습니다.')
            elif lower == 'b':
                self.draft[self.field.key] = decode(self.store.base)[self.field.key]
                self.tell('선택 항목의 기본 YAML 값을 가져왔습니다. 저장 후 다음 실행에 반영됩니다.')
            elif lower == 'l':
                values = self.live_values(runtime)
                if self.field.key not in values:
                    raise ValueError('선택 항목의 현재 적용값을 읽지 못했습니다.')
                self.draft[self.field.key] = values[self.field.key]
                self.tell('선택 항목의 적용값을 편집란으로 가져왔습니다.')
            elif lower == 'r' and runtime:
                runtime.tick(force=True)
                self.tell('실행 노드의 값을 다시 읽습니다.')
            elif lower == 'p':
                self.open_presets()
            elif lower == 's':
                self.save_preset()
            elif lower in ('q', 'f'):
                self.review({'q': 'quit', 'f': 'reload'}[lower])
            elif key == '?':
                self.help = not self.help
        except (ValueError, KeyError, OSError, OverflowError, yaml.YAMLError) as error:
            self.tell(str(error), True)


def main(argv=None):
    parser = argparse.ArgumentParser(description='차량 공통 제어 설정 · 명령 전류 단위 편집')
    parser.add_argument('--config', help='저장 파일 경로 (launch의 control_tuning_file과 동일하게 지정)')
    parser.add_argument('--offline', action='store_true', help='ROS 연결 없이 편집·저장')
    parser.add_argument('--preview', action='store_true', help='첫 화면 출력 후 종료; ROS와 파일 변경 없음')
    parser.add_argument('--steering-config', help='기본 조향 YAML')
    parser.add_argument('--governor-config', help='기본 거버너 YAML')
    parser.add_argument('--calibration', help='조향 보정 YAML (읽기 전용)')
    args = parser.parse_args(argv)
    console = Console(highlight=False)
    try:
        store = SettingsStore(args.config, args.steering_config, args.governor_config, args.calibration)
        editor = Editor(store)
    except Exception as error:
        console.print(Text(f'설정 읽기 실패: {error}', style='red'))
        return 2
    if args.preview:
        preview = Console(width=console.width, height=max(console.height, 28), highlight=False)
        preview.print(layout(editor, None, preview.width, preview.height))
        return 0
    if not sys.stdin.isatty() or not console.is_terminal:
        console.print('대화형 터미널에서 실행하십시오. 화면 확인은 --preview를 사용하십시오.')
        return 2
    runtime = None
    ros = None
    old_term = signal.getsignal(signal.SIGTERM)
    try:
        if not args.offline:
            import rclpy
            from rclpy.signals import SignalHandlerOptions
            from .runtime import TuningNode
            rclpy.init(args=[], signal_handler_options=SignalHandlerOptions.NO)
            ros = rclpy
            runtime = TuningNode()
        signal.signal(signal.SIGTERM, lambda *_: setattr(editor, 'done', True))
        renderer = ScreenRenderer(console)
        with keyboard(), tui_screen(console):
            next_frame = 0.0
            while not editor.done:
                if runtime:
                    ros.spin_once(runtime, timeout_sec=0.0)
                    runtime.tick()
                if select.select([sys.stdin], [], [], 0.02)[0]:
                    key = read_key(sys.stdin.fileno())
                    if key == 'eof':
                        break
                    editor.key(key, runtime)
                    next_frame = 0.0
                if time.monotonic() >= next_frame:
                    renderer.redraw(layout(editor, runtime, console.width, console.height), 0)
                    next_frame = time.monotonic() + 0.15
    except KeyboardInterrupt:
        pass
    except Exception as error:
        console.print(Text(f'실행 오류: {error}', style='red'))
        return 1
    finally:
        signal.signal(signal.SIGTERM, old_term)
        if runtime:
            runtime.destroy_node()
        if ros and ros.ok():
            ros.shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())
