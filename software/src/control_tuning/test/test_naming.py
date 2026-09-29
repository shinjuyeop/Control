"""Persisted names and units remain usable across the steering/drive rename."""
from pathlib import Path
import json

import pytest
import yaml

from control_tuning.naming import canonical_capture
from control_tuning.presets import PresetStore, load_launch_overrides
from control_tuning.schema import STEERING, GOVERNOR
from control_tuning.longitudinal import LONGITUDINAL
from control_tuning.tui import Editor


def legacy_preset(store):
    presets = PresetStore(store)
    path = presets.path_for('driving')
    path.parent.mkdir(parents=True)
    path.write_bytes((Path(__file__).parent/'fixtures/driving_v2.yaml').read_bytes())
    return path


def test_v2_read_and_save_migration_preserves_controller_values(store):
    path = legacy_preset(store)
    original = path.read_bytes()
    editor = Editor(store)
    editor.key('enter')
    assert editor.draft['steering_kp_a_per_deg'] == 2.4
    assert editor.draft['steering_torque_feedback_timeout_ms'] == 100
    assert editor.draft['drive_current_limit_a'] == 480
    assert editor.draft['drive_coast_release_error_mps'] == .12
    assert editor.draft['governor_command_timeout_ms'] == 200
    assert path.read_bytes() == original  # loading never overwrites a user's preset
    kwargs = dict(steering_config=store.base_paths[0], governor_config=store.base_paths[1],
                  calibration=store.base_paths[2])
    before = {node:load_launch_overrides(store.path,node,**kwargs)
              for node in (STEERING,GOVERNOR,LONGITUDINAL)}
    assert before[STEERING]['torque_full_scale_error_deg'] == pytest.approx(16/2.4)
    assert before[STEERING]['torque_feedback_timeout_ms'] == 100
    assert before[LONGITUDINAL]['control_mode'] == 'torque'
    assert before[LONGITUDINAL]['max_drive_current_a'] == 480
    assert not editor.preset_dirty
    editor.key('s')
    saved = yaml.safe_load(path.read_text())
    assert saved['schema_version'] == 3
    assert saved['steering']['steering_kp_a_per_deg'] == 2.4
    assert saved['drive']['drive_current_limit_a'] == 480
    assert path.with_suffix('.yaml.bak').read_bytes() == original
    after = {node:load_launch_overrides(store.path,node,**kwargs) for node in before}
    assert after == before  # rename cannot change gains, current authority or timeouts


def test_conflicting_old_and_new_names_are_not_silently_selected(store):
    path = legacy_preset(store)
    document = yaml.safe_load(path.read_text())
    document['longitudinal']['drive_current_limit_a'] = 8
    path.write_text(yaml.safe_dump(document))
    with pytest.raises(ValueError, match='서로 다릅니다'):
        PresetStore(store).load('driving')


def test_v3_missing_value_does_not_fall_back_to_firmware_defaults(store):
    presets = PresetStore(store)
    presets.save('driving',store.saved)
    path = presets.path_for('driving')
    document = yaml.safe_load(path.read_text())
    del document['drive']['drive_current_limit_a']
    path.write_text(yaml.safe_dump(document))
    with pytest.raises(ValueError):
        PresetStore(store).load('driving')


def test_old_mcap_snapshot_names_and_units_normalize_without_touching_raw_ros():
    old = {'schema_version':1,'nodes':{'/steering_actuator_node':{
        'ros__parameters':{'torque_feedback_timeout_ms':100,'torque_kd':1723.},
        'display_values':{'p_gain_a_per_deg':2.4,'torque_feedback_timeout_ms':100},
        'display_units':{'p_gain_a_per_deg':'A/deg'},
    }}}
    encoded = json.dumps(old)
    new = canonical_capture(json.loads(encoded))
    entry = new['nodes']['/steering_actuator_node']
    assert new['schema_version'] == 2
    assert entry['display_values']['steering_kp_a_per_deg'] == 2.4
    assert entry['display_values']['steering_torque_feedback_timeout_ms'] == 100
    assert entry['display_units']['steering_kp_a_per_deg'] == 'A/deg'
    assert entry['ros__parameters'] == old['nodes']['/steering_actuator_node']['ros__parameters']
    assert canonical_capture(new) == new
