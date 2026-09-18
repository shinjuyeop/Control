"""Use real launch files and C++ nodes, with CAN/output disabled in an isolated domain."""

from contextlib import contextmanager
import importlib.util
import os
from pathlib import Path
import shutil
import signal
import subprocess
import time

import pytest
import rclpy
from ament_index_python.packages import get_package_prefix, PackageNotFoundError
from kaiev26_msgs.msg import DriveActuatorStatus
from launch import LaunchContext
from launch_ros.utilities import evaluate_parameters, normalize_parameters
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node

from control_tuning.runtime import TuningNode
from control_tuning.schema import STEERING, GOVERNOR
from control_tuning.storage import SettingsStore
from control_tuning.presets import PresetStore, load_launch_overrides
from control_tuning.conditions import CONDITION_KEYS


@pytest.mark.parametrize('package,filename,helper', [
    ('lateral_ctrl', 'steering_actuator.launch.py', '_make_actuator'),
    ('command_governor', 'command_governor.launch.py', '_make_governor'),
    ('control_launch', 'vehicle.launch.py', '_governor'),
])
def test_launch_defaults_read_the_same_source_file_as_tui(store, tmp_path, monkeypatch,
                                                        package, filename, helper):
    from control_tuning import storage
    from launch.actions import DeclareLaunchArgument
    monkeypatch.delenv('KAI_CONTROL_TUNING_FILE', raising=False)
    source_package = tmp_path / 'checkout/control_tuning'
    source_package.mkdir(parents=True)
    (source_package / 'package.xml').write_text('<package/>')
    monkeypatch.setattr(storage, '__file__', str(source_package / 'control_tuning/storage.py'))
    settings = SettingsStore(None, *store.base_paths)
    presets = PresetStore(settings)
    presets.save('driving', {**settings.saved, 'p_gain_a_per_deg': 40., 'governor_input_timeout_ms': 250.})
    presets.save('airborne', {**settings.saved, 'p_gain_a_per_deg': 80., 'governor_input_timeout_ms': 100.})
    source = Path(__file__).resolve().parents[2] / package / 'launch' / filename
    spec = importlib.util.spec_from_file_location('test_default_launch_module', source)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    # Generate launch declarations without requiring the optional vehicle URDF package.
    get_share = module.get_package_share_directory
    monkeypatch.setattr(module, 'get_package_share_directory', lambda name:
                        str(tmp_path) if name == 'kaiev26_description' else get_share(name))
    description = module.generate_launch_description()
    argument = next(item for item in description.entities
                    if isinstance(item, DeclareLaunchArgument) and item.name == 'control_tuning_file')
    context = LaunchContext()
    argument.execute(context)
    next(item for item in description.entities
         if isinstance(item, DeclareLaunchArgument) and item.name == 'tuning_preset').execute(context)
    assert context.launch_configurations['tuning_preset'] == 'driving'
    assert Path(context.launch_configurations['control_tuning_file']) == settings.path
    context.launch_configurations.update(params_file=str(store.base_paths[0]), actuation_mode='',
                                        steering_enabled='', can_interface='unused', hardware_can_enabled='false',
                                        actuator_output_enabled='false', test_input_injection_enabled='false',
                                        use_sim_time='false')
    monkeypatch.setattr(module, 'Node', lambda **kwargs: kwargs)
    node = getattr(module, helper)(context, {} if package == 'lateral_ctrl' else str(store.base_paths[1]))[0]
    layers = evaluate_parameters(context, normalize_parameters(node['parameters']))
    if package == 'lateral_ctrl':
        assert layers[1]['torque_full_scale_error_deg'] == .4
    else:
        assert layers[1]['input_timeout_ms'] == 250.


@pytest.mark.parametrize('package,filename,helper', [
    ('lateral_ctrl', 'steering_actuator.launch.py', '_make_actuator'),
    ('command_governor', 'command_governor.launch.py', '_make_governor'),
    ('control_launch', 'vehicle.launch.py', '_governor'),
])
@pytest.mark.parametrize('condition', CONDITION_KEYS)
def test_launch_helpers_layer_profile_before_explicit_overrides(store, monkeypatch, package, filename, helper, condition):
    presets = PresetStore(store)
    presets.save(condition, {**store.saved, 'p_gain_a_per_deg': 40., 'governor_input_timeout_ms': 250.})
    source = Path(__file__).resolve().parents[2] / package / 'launch' / filename
    spec = importlib.util.spec_from_file_location('test_launch_module', source)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    monkeypatch.setattr(module, 'Node', lambda **kwargs: kwargs)
    context = LaunchContext()
    context.launch_configurations.update(control_tuning_file=str(store.path), tuning_preset=condition,
                                        params_file=str(store.base_paths[0]), actuation_mode='position',
                                        steering_enabled='', can_interface='unused', hardware_can_enabled='false',
                                        actuator_output_enabled='false', test_input_injection_enabled='false',
                                        use_sim_time='false')
    argument = {} if package == 'lateral_ctrl' else str(store.base_paths[1])
    node = getattr(module, helper)(context, argument)[0]
    layers = evaluate_parameters(context, normalize_parameters(node['parameters']))
    if package == 'lateral_ctrl':
        assert layers[1]['torque_full_scale_error_deg'] == .4
        assert layers[-1]['actuation_mode'] == 'position'
        assert layers[-1]['hardware_can_enabled'] is False
        assert layers[-1]['actuator_output_enabled'] is False
    else:
        assert layers[1]['input_timeout_ms'] == 250.
        if package == 'control_launch':
            assert node['respawn'] is True


def test_actual_nodes_save_apply_restart_and_cli_priority(store, monkeypatch, tmp_path):
    if os.environ.get('CONTROL_TUNING_LAUNCH_TESTS') != '1':
        pytest.skip('Opt in with CONTROL_TUNING_LAUNCH_TESTS=1 after building lateral_ctrl and command_governor')
    for package in ('lateral_ctrl', 'command_governor', 'control_tuning'):
        try:
            get_package_prefix(package)
        except PackageNotFoundError:
            pytest.fail(f'Build {package} before running the launch integration test')
    monkeypatch.setenv('ROS_DOMAIN_ID', '219')
    monkeypatch.setenv('ROS_LOCALHOST_ONLY', '1')
    preset = {**store.saved, 'p_gain_a_per_deg': 40., 'peak_current_command_a': 12.,
              'minimum_vehicle_speed_kph': 1., 'governor_input_timeout_ms': 250.}
    presets = PresetStore(store)
    presets.save('driving', preset)
    presets.save('airborne', {**preset, 'p_gain_a_per_deg': 24., 'governor_input_timeout_ms': 300.})
    processes = []

    @contextmanager
    def launch(package, filename, *arguments):
        log = (tmp_path / f'{package}-{len(processes)}.log').open('w+')
        process = subprocess.Popen([shutil.which('ros2'), 'launch', package, filename,
                                    f'control_tuning_file:={store.path}', *arguments],
                                   env=os.environ.copy(), stdout=log, stderr=log, start_new_session=True)
        processes.append((process, log))
        try:
            yield process
        finally:
            if process.poll() is None:
                os.killpg(process.pid, signal.SIGINT)
                try:
                    process.wait(timeout=8)
                except subprocess.TimeoutExpired:
                    os.killpg(process.pid, signal.SIGKILL)
                    process.wait(timeout=3)
            log.close()

    rclpy.init(args=[])
    client = TuningNode()
    status_source = Node('tuning_status_fixture', enable_rosout=False)
    drive_pub = status_source.create_publisher(DriveActuatorStatus, '/drive/status', 10)
    executor = SingleThreadedExecutor()
    executor.add_node(client)
    executor.add_node(status_source)
    drive = DriveActuatorStatus(mode=DriveActuatorStatus.MODE_MANUAL, speed_mps=0., part1_valid=True)

    def pump(until):
        deadline = time.monotonic() + 12
        while time.monotonic() < deadline:
            drive_pub.publish(drive)
            executor.spin_once(timeout_sec=.02)
            client.tick()
            if until():
                return
        logs = '\n'.join(p.read_text() for p in tmp_path.glob('*.log'))
        pytest.fail(f'Actual-node condition timed out: {client.errors}, {client.result}\n{logs}')

    try:
        with launch('command_governor', 'command_governor.launch.py'):
            with launch('lateral_ctrl', 'steering_actuator.launch.py',
                        'hardware_can_enabled:=false', 'actuator_output_enabled:=false'):
                pump(lambda: client.fresh_values(STEERING) and client.fresh_values(GOVERNOR)
                     and not client.apply_reason(store.saved_native[STEERING]))
                assert client.values[STEERING]['torque_full_scale_error_deg'] == .3
                assert client.values[STEERING]['minimum_steering_speed_kph'] == 1.
                assert client.values[GOVERNOR]['input_timeout_ms'] == 250.
                client.begin_apply({**preset, 'p_gain_a_per_deg': 48.},
                                   store.saved_native[STEERING], dict(client.values[STEERING]))
                pump(lambda: client.result is not None)
                assert client.result[0], client.result
                assert client.values[STEERING]['torque_full_scale_error_deg'] == .25
                assert presets.load('driving')['p_gain_a_per_deg'] == 40.
            client.received.pop(STEERING, None)
            with launch('lateral_ctrl', 'steering_actuator.launch.py',
                        'hardware_can_enabled:=false', 'actuator_output_enabled:=false',
                        'actuation_mode:=position'):
                pump(lambda: client.fresh_values(STEERING) and client.values[STEERING]['actuation_mode'] == 'position')
                assert client.values[STEERING]['torque_full_scale_error_deg'] == .3
                assert 'Position' in client.apply_reason(store.saved_native[STEERING])
        client.received.clear()
        with launch('command_governor', 'command_governor.launch.py', 'tuning_preset:=airborne'):
            with launch('lateral_ctrl', 'steering_actuator.launch.py',
                        'hardware_can_enabled:=false', 'actuator_output_enabled:=false',
                        'tuning_preset:=airborne'):
                pump(lambda: client.fresh_values(STEERING) and client.fresh_values(GOVERNOR)
                     and client.values[STEERING]['torque_full_scale_error_deg'] == .5
                     and client.values[GOVERNOR]['input_timeout_ms'] == 300.)
    finally:
        executor.shutdown()
        client.destroy_node()
        status_source.destroy_node()
        rclpy.shutdown()


@pytest.mark.parametrize('node', [STEERING, GOVERNOR])
def test_selected_preset_is_required_and_other_conditions_are_ignored(store, node):
    presets = PresetStore(store)
    presets.save('driving', store.saved)
    kwargs = dict(steering_config=store.base_paths[0], governor_config=store.base_paths[1],
                  calibration=store.base_paths[2])
    with pytest.raises(ValueError, match='프리셋이 없습니다'):
        load_launch_overrides(store.path, node, 'airborne', **kwargs)
    with pytest.raises(ValueError, match='지원하지 않는'):
        load_launch_overrides(store.path, node, 'unknown', **kwargs)
    presets.path_for('airborne').write_text('malformed: [')
    assert load_launch_overrides(store.path, node, **kwargs)
    path = presets.path_for('driving')
    path.write_text(path.read_text().replace('current_reference_a: 20.0', 'current_reference_a: 10.0'))
    with pytest.raises(ValueError, match='환산 기준'):
        load_launch_overrides(store.path, node, **kwargs)
