import yaml
import pytest

from control_tuning.conditions import CONDITION_KEYS
from control_tuning.presets import PresetStore
from control_tuning.schema import STEERING
from control_tuning.tui import Editor


def test_presets_save_in_independent_files_with_individual_backups(store):
    presets = PresetStore(store)
    assert all(presets.entry(c) is None for c in CONDITION_KEYS)
    for i, condition in enumerate(CONDITION_KEYS):
        presets.save(condition, {**store.saved, 'p_gain_a_per_deg': 32 + i * 4,
                                 'peak_current_command_a': 12.0007})
        document = yaml.safe_load(presets.path_for(condition).read_text())
        assert document['condition'] == condition
        assert document['values']['p_gain_a_per_deg'] == 32 + i * 4
    assert not store.path.exists()  # Saving one preset must not change the shared base.
    assert not presets.legacy_path.exists()
    reread = PresetStore(store)
    for i, condition in enumerate(CONDITION_KEYS):
        loaded = reread.load(condition)
        assert loaded['p_gain_a_per_deg'] == 32 + i * 4
        assert loaded['peak_current_command_a'] == 12
    before = {c: presets.path_for(c).read_bytes() for c in CONDITION_KEYS}
    presets.save('airborne', store.saved)
    assert presets.path_for('airborne').with_suffix('.yaml.bak').read_bytes() == before['airborne']
    for c in CONDITION_KEYS:
        if c == 'airborne':
            continue
        assert presets.path_for(c).read_bytes() == before[c]
        assert not presets.path_for(c).with_suffix('.yaml.bak').exists()


def test_preset_conflict_blocks_same_condition_but_allows_other_conditions(store):
    first, second = PresetStore(store), PresetStore(store)
    first.save('airborne', store.saved)
    original = first.path_for('airborne').read_bytes()
    with pytest.raises(ValueError, match='외부'):
        second.save('airborne', {**store.saved, 'p_gain_a_per_deg': 40})
    assert first.path_for('airborne').read_bytes() == original
    with pytest.raises(ValueError, match='변경'):
        second.load('airborne')
    second.save('driving', {**store.saved, 'p_gain_a_per_deg': 40})
    reread = PresetStore(store)
    assert reread.load('airborne')['p_gain_a_per_deg'] == 32
    assert reread.load('driving')['p_gain_a_per_deg'] == 40


def test_preset_load_checks_reference_completeness_and_filename(store):
    presets = PresetStore(store)
    presets.save('airborne', store.saved)
    path = presets.path_for('airborne')
    document = yaml.safe_load(path.read_text())
    document['current_reference_a'] = 10
    path.write_text(yaml.safe_dump(document))
    presets.reload()
    with pytest.raises(ValueError, match='환산 기준'):
        presets.load('airborne')
    document['current_reference_a'] = 20
    document['values'].pop('p_gain_a_per_deg')
    path.write_text(yaml.safe_dump(document))
    presets.reload()
    with pytest.raises(ValueError, match='25개'):
        presets.load('airborne')
    document['condition'] = 'driving'
    path.write_text(yaml.safe_dump(document))
    with pytest.raises(ValueError, match='condition'):
        presets.reload()


def test_legacy_presets_remain_readable_and_move_individually_on_save(store):
    presets = PresetStore(store)
    legacy = {'schema_version': 1, 'presets': {
        c: {'current_reference_a': 20, 'values': {**store.saved, 'p_gain_a_per_deg': 32 + i * 4}}
        for i, c in enumerate(CONDITION_KEYS)}}
    presets.legacy_path.parent.mkdir(parents=True)
    presets.legacy_path.write_text(yaml.safe_dump(legacy))
    original = presets.legacy_path.read_bytes()
    presets.reload()
    assert presets.load('driving')['p_gain_a_per_deg'] == 32
    assert not presets.directory.exists()  # Reading never migrates or writes.
    presets.save('airborne', {**presets.load('airborne'), 'p_gain_a_per_deg': 44})
    assert presets.legacy_path.read_bytes() == original
    reread = PresetStore(store)
    assert reread.load('airborne')['p_gain_a_per_deg'] == 44
    assert reread.load('driving')['p_gain_a_per_deg'] == 32
    assert not reread.path_for('driving').exists()
    legacy['presets']['driving']['values']['p_gain_a_per_deg'] = 50
    presets.legacy_path.write_text(yaml.safe_dump(legacy))
    with pytest.raises(ValueError, match='외부'):
        reread.load('driving')
    assert reread.load('airborne')['p_gain_a_per_deg'] == 44


def test_selection_is_required_and_empty_slot_opens_editor_immediately(store):
    editor = Editor(store)
    for key in ['escape', '1', '+', 's', 'a', 'w', 'b', 'l', 'x']:
        editor.key(key)
        assert editor.preset_condition is None
        assert editor.modal is None
        assert editor.editing is None
        assert editor.draft == store.saved
        assert not store.path.exists()
        assert not editor.presets.directory.exists()
    editor.key('right')
    editor.key('enter')
    assert editor.preset_condition == 'airborne'
    assert editor.modal is None
    assert not editor.selecting_preset
    assert editor.draft == store.saved
    assert not editor.preset_dirty
    assert not editor.presets.directory.exists()
    editor.key('s')
    assert editor.modal is None
    assert editor.presets.path_for('airborne').exists()
    assert not store.path.exists()
    assert not editor.preset_dirty


def test_switching_presets_reviews_unsaved_edits_and_cancels_without_loss(store, editor):
    editor.presets.save('driving', {**store.saved, 'p_gain_a_per_deg': 48})
    editor.commit(40)
    editor.key('p')
    editor.key('left')
    editor.key('enter')
    assert editor.modal['action'] == 'preset_switch'
    assert not editor.modal.get('rows')  # Only a concise discard confirmation.
    assert editor.draft['p_gain_a_per_deg'] == 40
    editor.key('escape')
    editor.key('escape')
    assert editor.preset_condition == 'airborne'
    assert not editor.selecting_preset
    assert editor.draft['p_gain_a_per_deg'] == 40
    editor.key('p')
    editor.key('left')
    editor.key('enter')
    editor.key('enter')
    assert editor.preset_condition == 'driving'
    assert editor.draft['p_gain_a_per_deg'] == 48
    assert not editor.preset_dirty
    assert not store.path.exists()
    assert store.saved_native[STEERING]['torque_full_scale_error_deg'] == .5


def test_empty_condition_starts_from_startup_not_previous_condition(store, editor):
    editor.commit(40)
    editor.key('p')
    editor.key('right')
    editor.key('enter')
    assert editor.modal['action'] == 'preset_switch'
    editor.key('enter')
    assert editor.preset_condition == 'ground_stationary'
    assert editor.draft == store.saved
    assert editor.preset_baseline == store.saved
    assert not editor.preset_dirty


def test_save_targets_only_selected_preset_and_clears_unsaved_edits(store, editor):
    editor.presets.save('driving', store.saved)
    driving_before = editor.presets.path_for('driving').read_bytes()
    editor.commit(40)
    editor.key('s')
    assert editor.modal is None
    assert editor.presets.load('airborne')['p_gain_a_per_deg'] == 40
    assert store.saved['p_gain_a_per_deg'] == 32
    assert not store.path.exists()
    assert editor.presets.path_for('driving').read_bytes() == driving_before
    editor.key('q')
    assert editor.done


def test_external_change_before_save_is_not_overwritten(store, editor):
    editor.commit(40)
    other = PresetStore(store)
    other.save('airborne', {**store.saved, 'p_gain_a_per_deg': 44})
    editor.key('s')
    assert editor.error
    assert other.load('airborne')['p_gain_a_per_deg'] == 44
    assert editor.draft['p_gain_a_per_deg'] == 40
    editor.key('f')
    editor.key('enter')
    assert editor.selecting_preset and editor.preset_condition is None
    editor.key('right')
    editor.key('enter')
    assert editor.draft['p_gain_a_per_deg'] == 44


def test_invalid_values_do_not_write_a_preset(store, editor):
    before = editor.presets.path_for('airborne').read_bytes()
    editor.commit(0)
    editor.key('s')
    assert editor.error
    assert editor.presets.path_for('airborne').read_bytes() == before


def test_unedited_new_preset_does_not_prompt_or_create_files_on_exit(store):
    editor = Editor(store)
    editor.key('enter')
    assert not editor.preset_dirty
    editor.key('q')
    assert editor.done and editor.modal is None
    assert not store.path.exists()
    assert not editor.presets.directory.exists()


def test_selecting_active_preset_preserves_draft_without_confirmation(store, editor):
    editor.commit(40)
    editor.key('p')
    editor.key('enter')
    assert editor.modal is None and not editor.selecting_preset
    assert editor.draft['p_gain_a_per_deg'] == 40
    assert editor.preset_dirty


def test_external_change_during_preset_switch_preserves_current_draft(store, editor):
    editor.presets.save('driving', {**store.saved, 'p_gain_a_per_deg': 48})
    editor.commit(40)
    editor.key('p')
    editor.key('left')
    editor.key('enter')
    assert editor.modal['action'] == 'preset_switch'
    other = PresetStore(store)
    other.save('driving', {**store.saved, 'p_gain_a_per_deg': 50})
    editor.key('enter')
    assert editor.error
    assert editor.preset_condition == 'airborne'
    assert editor.draft['p_gain_a_per_deg'] == 40
