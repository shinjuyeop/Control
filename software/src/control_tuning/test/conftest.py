from pathlib import Path
import sys

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from control_tuning.storage import SettingsStore


@pytest.fixture
def store(tmp_path):
    packages = Path(__file__).resolve().parents[2]
    paths = [packages / 'lateral_ctrl/config/steering_actuator.yaml',
             packages / 'command_governor/config/command_governor.yaml',
             packages / 'lateral_ctrl/config/steering_motor_calibration.yaml']
    copies = []
    for path in paths:
        copy = tmp_path / path.name
        copy.write_text(path.read_text())
        copies.append(copy)
    return SettingsStore(tmp_path / 'user/tuning.yaml', *copies)


@pytest.fixture
def editor(store):
    from control_tuning.tui import Editor
    result = Editor(store)
    result.key('right')
    result.key('enter')  # Select airborne and enter its parameter editor directly.
    result.key('s')
    result.selected = next(i for i, f in enumerate(result.fields) if f.key == 'p_gain_a_per_deg')
    return result
