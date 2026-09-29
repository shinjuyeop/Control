import math
from pathlib import Path

import pytest
import yaml

from control_tuning.schema import FIELDS, STEERING, GOVERNOR, decode, encode, equivalent
from control_tuning.storage import SettingsStore, default_settings_path, load_overrides


def write_yaml(path, document):
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(yaml.safe_dump(document))


def test_default_unit_conversion_and_round_trip(store):
    values = store.saved
    assert values['steering_kp_a_per_deg'] == pytest.approx(32)
    assert values['steering_kd_a_per_deg_s'] == pytest.approx(1500 * math.pi / 180 * .0016)
    assert values['steering_peak_current_limit_a'] == 16
    assert values['steering_sustained_current_limit_a'] == 8
    native = encode(values, store.saved_native[STEERING])
    for node, params in native.items():
        for key, value in params.items():
            assert equivalent(value, store.base[node][key]), key
            assert type(value) is type(store.base[node][key]), key


def test_peak_change_preserves_p_and_quantizes_current(store):
    values = {**store.saved, 'steering_peak_current_limit_a': 12.0007}
    native = encode(values, store.saved_native[STEERING])
    assert native[STEERING]['torque_peak_limit_raw'] == 7500
    assert native[STEERING]['torque_full_scale_error_deg'] == .375
    decoded = decode({n: {**store.base[n], **p} for n, p in native.items()})
    assert decoded['steering_kp_a_per_deg'] == 32
    assert decoded['steering_peak_current_limit_a'] == 12


@pytest.mark.parametrize('key,value', [
    ('steering_kp_a_per_deg', 0), ('steering_kp_a_per_deg', 320),
    ('steering_kd_a_per_deg_s', -1), ('steering_kd_a_per_deg_s', float('nan')),
    ('steering_peak_current_limit_a', 16.1), ('steering_peak_current_limit_a', .0001),
    ('steering_sustained_current_limit_a', 17), ('steering_peak_duration_ms', 19),
    ('steering_peak_duration_ms', 5001), ('steering_peak_duration_ms', 100.1),
    ('steering_peak_recovery_ms', 4999), ('steering_rate_filter_hz', 25.1),
    ('steering_settle_error_deg', .5), ('steering_settle_rate_deg_s', 0),
    ('steering_angle_margin_deg', 30),
    ('steering_min_vehicle_speed_kph', -1), ('steering_control_mode', 'bad'),
    ('steering_torque_feedback_timeout_ms', 39), ('steering_torque_feedback_timeout_ms', 201),
    ('steering_vehicle_mode_timeout_ms', 2**32), ('governor_command_timeout_ms', 0),
])
def test_invalid_settings_rejected_before_save(store, key, value):
    with pytest.raises(ValueError):
        store.save({**store.saved, key: value})
    assert not store.path.exists()


def test_save_differences_reload_backup_and_restore(store):
    initial = dict(store.saved)
    saved = store.save({**initial, 'steering_kp_a_per_deg': 40,
                        'governor_command_timeout_ms': 250, 'steering_min_vehicle_speed_kph': 1})
    document = yaml.safe_load(store.path.read_text())
    assert document[STEERING]['ros__parameters'] == {
        'torque_full_scale_error_deg': .4, 'minimum_steering_speed_kph': 1.0}
    assert document[GOVERNOR]['ros__parameters'] == {'input_timeout_ms': 250.0}
    assert SettingsStore(store.path, *store.base_paths).saved == saved
    content = store.path.read_bytes()
    store.save(initial)
    assert yaml.safe_load(store.path.read_text()) == {}
    assert store.path.with_suffix('.yaml.bak').read_bytes() == content
    assert load_overrides(store.path, STEERING) == {}


@pytest.mark.parametrize('target', ['profile', 'base', 'other_editor'])
def test_conflicts_never_overwrite(store, target):
    if target == 'profile':
        write_yaml(store.path, {})
    elif target == 'base':
        store.base_paths[0].write_text(store.base_paths[0].read_text() + '\n# external edit\n')
    else:
        other = SettingsStore(store.path, *store.base_paths)
        other.save({**other.saved, 'steering_kp_a_per_deg': 40})
    original = store.path.read_bytes() if store.path.exists() else None
    with pytest.raises(ValueError, match='외부'):
        store.save({**store.saved, 'steering_kp_a_per_deg': 35})
    assert (store.path.read_bytes() if store.path.exists() else None) == original


def test_failed_atomic_replace_preserves_file(store, monkeypatch):
    store.save({**store.saved, 'steering_kp_a_per_deg': 40})
    original = store.path.read_bytes()
    import os
    replace = os.replace

    def fail_main(source, destination):
        if destination == store.path:
            raise OSError('simulated disk error')
        return replace(source, destination)

    monkeypatch.setattr(os, 'replace', fail_main)
    with pytest.raises(OSError):
        store.save({**store.saved, 'steering_kp_a_per_deg': 50})
    assert store.path.read_bytes() == original
    assert not list(store.path.parent.glob('.tuning.yaml*'))


@pytest.mark.parametrize('parameters', [
    {'can_interface': 'other'}, {'torque_kd': True}, {'torque_kd': float('inf')},
    {'torque_peak_limit_raw': 100.5}, {'steering_vehicle_mode_timeout_ms': 100.0}, {'steering_control_mode': 'unknown'},
])
def test_launch_rejects_unmanaged_or_bad_types(store, parameters):
    write_yaml(store.path, {STEERING: {'ros__parameters': parameters}})
    with pytest.raises(ValueError):
        load_overrides(store.path, STEERING)


def test_paths_and_missing_profile(tmp_path, monkeypatch):
    monkeypatch.delenv('KAI_CONTROL_TUNING_FILE', raising=False)
    monkeypatch.setenv('XDG_CONFIG_HOME', str(tmp_path))
    monkeypatch.chdir(tmp_path)
    expected = Path(__file__).resolve().parents[1] / 'config/control_tuning.yaml'
    assert default_settings_path() == str(expected)
    monkeypatch.setenv('KAI_CONTROL_TUNING_FILE', str(tmp_path / 'custom.yaml'))
    assert default_settings_path() == str(tmp_path / 'custom.yaml')
    assert load_overrides(default_settings_path(), STEERING) == {}


@pytest.mark.parametrize('symlink', [False, True])
def test_installed_module_uses_source_checkout_config(tmp_path, monkeypatch, symlink):
    from control_tuning import storage
    monkeypatch.delenv('KAI_CONTROL_TUNING_FILE', raising=False)
    package = tmp_path / 'workspace/src/Control/software/src/control_tuning'
    package.mkdir(parents=True)
    (package / 'package.xml').write_text('<package/>')
    module = package / 'control_tuning/storage.py'
    module.parent.mkdir()
    module.touch()
    installed = tmp_path / 'workspace/install/control_tuning/lib/python3.10/site-packages/control_tuning/storage.py'
    installed.parent.mkdir(parents=True)
    if symlink:
        installed.symlink_to(module)
    else:
        installed.touch()
    monkeypatch.setattr(storage, '__file__', str(installed))
    monkeypatch.chdir(tmp_path)
    assert Path(default_settings_path()) == package / 'config/control_tuning.yaml'


def test_standalone_install_uses_packaged_config(tmp_path, monkeypatch):
    from control_tuning import storage
    monkeypatch.delenv('KAI_CONTROL_TUNING_FILE', raising=False)
    monkeypatch.setattr(storage, '__file__', str(tmp_path / 'standalone/control_tuning/storage.py'))
    share_config = tmp_path / 'standalone/share/control_tuning/config'

    def config(package):
        assert package == 'control_tuning'
        return share_config

    monkeypatch.setattr(storage, 'config_directory', config)
    assert Path(default_settings_path()) == share_config / 'control_tuning.yaml'


def test_every_field_round_trips_and_is_unique(store):
    assert len({f.key for f in FIELDS}) == len(FIELDS)
    for f in FIELDS:
        assert f.key in store.saved
