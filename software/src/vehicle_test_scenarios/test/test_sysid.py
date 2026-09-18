"""Field input, mode transition, and real ROS-to-MCAP tests."""

import time
from dataclasses import replace
import os
import json
import yaml
from pathlib import Path
from unittest.mock import Mock

import pytest
import rclpy
from ament_index_python.packages import get_package_prefix, get_package_share_directory
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import DurabilityPolicy, QoSProfile
from geometry_msgs.msg import TransformStamped
from kaiev26_msgs.msg import ActuatorCommand

from vehicle_test_scenarios.sysid_profiles import (
    CENTER_DEG, SCENARIO_KEYS, RunSettings, baseline, default_run_settings,
    input_setpoint,
)
from vehicle_test_scenarios.sysid_runtime import SysidNode, TOPICS, FOLLOW_TOPICS
from vehicle_test_scenarios import sysid_processes
from vehicle_test_scenarios.sysid_tui import (
    MenuState, handle_key, read_key,
)


def test_catalog_groups_parameterized_inputs_and_routes_without_square():
    assert SCENARIO_KEYS == (
        'straight', 'speed_step', 'speed_ramp', 'brake_test',
        'steering_step', 'steering_sine', 'steering_chirp',
        'route_quali', 'route_final',
    )
    assert 'steering_square' not in SCENARIO_KEYS
    assert default_run_settings('speed_step').spec.category == '입력 응답 시험'
    assert default_run_settings('route_quali').spec.category == '경로 추종 시험'


@pytest.mark.parametrize('scenario', SCENARIO_KEYS)
def test_catalog_defaults_are_valid_and_document_only_visible_parameters(scenario):
    settings = default_run_settings(scenario)
    document = settings.document()
    assert set(document['parameters']) == {
        field.name for field in settings.spec.fields
    }
    assert settings.cruise_speed_kph == 0.0
    assert settings.speed_low_kph == settings.speed_high_kph == 0.0
    assert settings.steering_center_deg == 0.0
    if not settings.follows_route:
        for elapsed in (0.0, 3.0, 10.0, 100.0):
            assert input_setpoint(settings, elapsed)[0].speed_kph == 0.0


def test_arrow_menu_selects_scenario_and_edits_its_visible_parameter():
    menu = MenuState()
    assert menu.selected_key == 'straight'
    handle_key(menu, 'down')
    handle_key(menu, 'enter')
    assert menu.selected_key == 'speed_step'
    assert menu.focus == 'parameters'
    menu.settings_by_key['speed_step'] = replace(menu.selection, speed_high_kph=3.0)
    handle_key(menu, 'enter')
    for _ in range(len(menu.edit_text)):
        handle_key(menu, 'backspace')
    for character in '2.5':
        handle_key(menu, character)
    action, message = handle_key(menu, 'enter')
    assert action is None and message == ''
    assert menu.selection.speed_low_kph == 2.5


def test_busy_menu_keeps_stop_and_quit_but_locks_configuration():
    menu = MenuState()
    handle_key(menu, 'down', busy=True)
    assert menu.hover_index == 0
    assert handle_key(menu, ' ', busy=True)[0] == 'stop'
    assert handle_key(menu, 'q', busy=True)[0] == 'quit'


def edit_choice(menu, offset, *keys):
    menu.focus = 'parameters'
    menu.field_index = len(menu.selected_spec.fields) + offset
    for key in ('enter', *keys):
        handle_key(menu, key)


def test_recording_choice_is_global_and_fixed_for_the_active_run():
    menu = MenuState()
    edit_choice(menu, 0, 'down')
    assert menu.record_mcap  # Candidate changes only until Enter.
    assert menu.edit_choice is False
    assert handle_key(menu, 'r')[0] is None
    handle_key(menu, 'enter')
    assert not menu.run_settings().record_mcap
    handle_key(menu, 'left')
    handle_key(menu, 'down')
    handle_key(menu, 'enter')
    assert not menu.run_settings().record_mcap
    active = menu.run_settings()
    edit_choice(menu, 0, 'up', 'escape')
    assert not menu.record_mcap
    edit_choice(menu, 0, 'up', 'enter')
    assert menu.record_mcap
    assert not active.record_mcap
    handle_key(menu, 'enter', busy=True)
    assert menu.edit_field is None


def test_test_condition_must_be_selected_and_never_changes_scenario_commands():
    menu = MenuState()
    assert handle_key(menu, 'r')[0] is None
    before = menu.run_settings().to_scenario_config()
    edit_choice(menu, 1)
    assert menu.edit_choice == 'driving'
    assert menu.test_condition == 'unspecified'
    assert handle_key(menu, 'r')[0] is None
    handle_key(menu, 'escape')
    assert menu.test_condition == 'unspecified'
    edit_choice(menu, 1, 'down', 'enter')
    assert menu.test_condition == 'airborne'
    assert handle_key(menu, 'r')[0] == 'run'
    active = menu.run_settings()
    handle_key(menu, 'enter', busy=True)
    assert menu.edit_field is None and menu.test_condition == 'airborne'
    edit_choice(menu, 1, 'down', 'enter')
    assert menu.test_condition == 'ground_stationary'
    assert active.test_condition == 'airborne'
    assert menu.run_settings().to_scenario_config() == before
    assert active.document()['test_condition'] == {
        'key': 'airborne', 'label': '공중', 'source': 'operator_selection', 'changes_control': False}
    edit_choice(menu, 1, 'down', 'enter')
    assert menu.test_condition == 'driving'
    edit_choice(menu, 1, 'up', 'enter')
    assert menu.test_condition == 'ground_stationary'
    with pytest.raises(ValueError):
        replace(active, test_condition='inferred_from_speed')


@pytest.mark.parametrize('key,action', [('s', 'stop'), (' ', 'stop'), ('q', 'quit')])
def test_stop_and_quit_remain_available_while_selecting_choices(key, action):
    menu = MenuState()
    edit_choice(menu, 1, 'down')
    assert handle_key(menu, key, busy=True)[0] == action
    assert menu.test_condition == 'unspecified'


def test_removed_shortcuts_do_not_change_recording_or_condition():
    menu = MenuState()
    handle_key(menu, 'm')
    handle_key(menu, 'c')
    assert menu.record_mcap and menu.test_condition == 'unspecified'


def test_whole_tui_scrolls_independently_from_scenario_selection():
    menu = MenuState()
    handle_key(menu, 'scroll_down')
    handle_key(menu, 'page_down')
    assert menu.view_offset == 6
    assert menu.hover_index == 0
    handle_key(menu, 'd')
    assert menu.view_offset == 6
    handle_key(menu, 'down')
    assert menu.view_offset == 6
    assert menu.hover_index == 1


@pytest.mark.parametrize(('sequence', 'key'), [
    (b'\x1b[<64;10;5M', 'scroll_up'),
    (b'\x1b[<65;10;5M', 'scroll_down'),
    (b'\x1b[5~', 'page_up'),
    (b'\x1b[6~', 'page_down'),
])
def test_terminal_wheel_and_page_keys_control_status_scroll(sequence, key):
    read_fd, write_fd = os.pipe()
    try:
        os.write(write_fd, sequence)
        assert read_key(read_fd) == key
    finally:
        os.close(read_fd)
        os.close(write_fd)


def test_parameterized_speed_step_and_ramp_use_runner_math():
    step = replace(default_run_settings('speed_step'), speed_low_kph=1.0, speed_high_kph=3.0)
    assert [input_setpoint(step, t)[0].speed_kph for t in (0, 1.99, 2, 3.99)] == [
        1, 1, 3, 3,
    ]
    ramp = replace(default_run_settings('speed_ramp'), speed_high_kph=3.0)
    assert [input_setpoint(ramp, t)[0].speed_kph for t in (0, 2.5, 5, 7.5, 10)] == [
        0, 1.5, 3, 1.5, 0,
    ]


@pytest.mark.parametrize('speed', [-1, float('nan'), float('inf')])
def test_invalid_route_speed_is_rejected(speed):
    with pytest.raises(ValueError):
        replace(default_run_settings('route_quali'), cruise_speed_kph=speed)


def test_chirp_crossings_get_closer_and_finite_run_ends_at_center():
    settings = replace(
        default_run_settings('steering_chirp'), initial_hold_sec=0.0,
    )
    crossings = []
    previous = 0
    for k in range(1, 10000):
        t = k / 1000
        value = input_setpoint(settings, t)[0].steering_deg - CENTER_DEG
        if previous < 0 <= value:
            crossings.append(t)
        previous = value
    assert len(crossings) >= 4
    periods = [b - a for a, b in zip(crossings, crossings[1:])]
    assert all(a > b for a, b in zip(periods, periods[1:]))
    output, complete = input_setpoint(settings, settings.duration)
    assert complete
    assert output == baseline(settings=settings)


def telemetry():
    result = {topic: kind() for topic, kind in TOPICS.items()}
    drive = result['/drive/status']
    drive.part1_valid = drive.part2_valid = True
    drive.fault_bits_valid = True
    steering = result['/steering/status']
    steering.communication_ok = steering.feedback_valid = True
    fix = result['/gnss/fix']
    fix.status.status = 2
    fix.latitude, fix.longitude = 37.24, 126.77
    for child in ('gps', 'ebimu_imu_link'):
        transform = TransformStamped()
        transform.header.frame_id = 'base_link'
        transform.child_frame_id = child
        transform.transform.rotation.w = 1.0
        result['/tf_static'].transforms.append(transform)
    return result


@pytest.fixture
def controlled():
    rclpy.init()
    clock = [100.0]
    node = SysidNode(clock=lambda: clock[0])
    node.steering_control.ready = Mock(return_value=True)
    data = telemetry()
    node.graph_errors = []

    def tick(dt=0, auto=False, speed=0):
        clock[0] += dt
        data['/drive/status'].mode = int(auto)
        data['/drive/status'].speed_mps = speed / 3.6
        for topic, message in data.items():
            node._receive(topic, message)
        node._receive('/vehicle/command', ActuatorCommand())
        node._tick()

    tick()
    yield node, tick, clock
    node.destroy_node()
    rclpy.shutdown()


def settings_with_speed(scenario, speed=10.0):
    settings = default_run_settings(scenario)
    if scenario in ('speed_step', 'speed_ramp'):
        return replace(settings, speed_high_kph=speed)
    return replace(settings, cruise_speed_kph=speed)


def begin(node, tick, scenario='speed_step'):
    node.request_run(settings_with_speed(scenario))
    tick()
    node.commands.put(('record_ready', None))
    tick(auto=True)
    assert node.state == 'COUNTDOWN'


def test_run_waits_for_auto_then_five_seconds_and_manual_cancels(controlled):
    node, tick, _ = controlled
    node.request_run(replace(default_run_settings(), cruise_speed_kph=3.0))
    tick()
    node.commands.put(('record_ready', None))
    tick()
    assert node.state == 'WAIT_AUTO'
    assert node.output.speed_kph == 0
    tick(auto=True)
    tick(4.99, auto=True)
    assert node.output.speed_kph == 0
    tick(0.01, auto=True)
    assert node.state == 'RUNNING'
    assert node.output.speed_kph > 0
    tick(auto=False)
    assert node.state == 'TAIL'
    assert node.output == baseline(settings=node.settings)
    tick(2.1, auto=True)
    assert node.state == 'CLOSING'
    assert node.output.speed_kph == 0


def test_duplicate_run_does_not_restart_recording_or_change_active_settings(controlled):
    node, tick, _ = controlled
    begin(node, tick)
    node.request_run(settings_with_speed('steering_chirp', 15))
    tick(auto=True)
    assert node.settings == settings_with_speed('speed_step', 10)
    assert node.process_actions.qsize() == 1


@pytest.mark.parametrize('scenario', ['route_quali', 'route_final'])
def test_route_start_waits_for_fresh_post_enable_command_and_stale_input_stops(
        controlled, scenario):
    node, tick, clock = controlled
    begin(node, tick, scenario=scenario)
    tick(4.9, auto=True)
    assert node.output.speed_kph == 0
    clock[0] += 0.1
    data = {topic: kind() for topic, kind in FOLLOW_TOPICS.items()}
    data['/sysid/route_context'].route_projection_valid = True
    data['/sysid/route_context'].distance_to_finish = 100.0
    data['/sysid/route_command'].speed_target_mps = 1.0
    data['/sysid/route_command'].steering_target_rad = 0.1
    data['/sysid/route_command'].brake_engage = True
    for topic, message in data.items():
        node._receive(topic, message)
    tick(auto=True)
    assert node.state == 'ROUTE_ARMING'
    assert node.output == baseline(settings=node.settings)

    clock[0] += 0.01
    data['/sysid/route_command'].brake_engage = False
    node._receive('/sysid/route_command', data['/sysid/route_command'])
    tick(auto=True)
    assert node.state == 'RUNNING'
    assert node.output.speed_kph == pytest.approx(3.6)
    tick(0.21, auto=True)
    assert node.output.brake_engage
    assert node.terminal_state == 'ERROR'


def test_route_without_recording_starts_and_closes_workers(controlled, tmp_path, monkeypatch):
    from vehicle_test_scenarios import sysid_routes
    node, tick, _ = controlled
    route = Mock()
    factory = Mock(return_value=route)
    monkeypatch.setattr(sysid_routes, 'RouteProcesses', factory)
    settings = replace(default_run_settings('route_quali'), record_mcap=False)
    manager = sysid_processes.ProcessManager(node, records_root=tmp_path / 'unused')
    try:
        node.request_run(settings)
        tick()
        action, value = node.process_actions.get_nowait()
        manager._start_recording(value)
        tick()
        assert node.state == 'WAIT_AUTO'
        assert not node.record_open
        assert manager.recorder is None
        factory.assert_called_once_with(node, settings)
        node.request_stop()
        tick()
        assert node.output.brake_engage
        assert node.busy
        action, value = node.process_actions.get_nowait()
        manager._finish_recording(value)
        tick()
        route.close.assert_called_once()
        assert node.state == 'ABORTED'
        assert not node.session_open
        assert not (tmp_path / 'unused').exists()
    finally:
        manager._close_route()


@pytest.mark.parametrize('scenario', ['steering_step', 'steering_chirp'])
def test_cruise_timeout_warns_and_runs_lateral_input_to_completion(controlled, scenario):
    node, tick, _ = controlled
    begin(node, tick, scenario=scenario)
    tick(5, auto=True, speed=2)
    assert node.state == 'CRUISE'
    tick(15, auto=True, speed=2)
    assert node.state == 'RUNNING'
    assert '정속 판정 실패' in node.run_warning
    assert node.output.speed_kph == 10
    excitation_time = (
        node.settings.initial_hold_sec
        + (node.settings.steering_step_interval_sec
           if scenario == 'steering_step' else 1.0)
    )
    tick(excitation_time, auto=True, speed=2)
    assert node.output.steering_deg != CENTER_DEG
    tick(node.settings.duration - excitation_time, auto=True, speed=2)
    assert node.state == 'TAIL'
    assert node.output == baseline(settings=node.settings)
    assert node.terminal_state == 'COMPLETE'
    assert '정속 판정 실패' in node.snapshot()['run_warning']


def test_cruise_starts_steering_after_one_second_near_target(controlled):
    node, tick, _ = controlled
    begin(node, tick, scenario='steering_step')
    tick(5, auto=True, speed=10)
    tick(1, auto=True, speed=10)
    assert node.state == 'RUNNING'
    tick(5, auto=True, speed=10)
    assert node.output.steering_deg == CENTER_DEG + 1


def test_stop_and_recording_failure_request_brake(controlled):
    node, tick, _ = controlled
    begin(node, tick)
    node.commands.put(('record_failed', 'disk full'))
    tick(auto=True)
    assert node.state == 'CLOSING'
    assert node.output.brake_engage
    node.commands.put(('record_closed', '/record'))
    tick(auto=True)
    assert node.state == 'ERROR'
    assert not node.record_open


def test_route_failure_survives_recorder_cleanup_and_stop(controlled):
    node, tick, _ = controlled
    begin(node, tick)
    node.commands.put(('route_failed', 'route worker import failed'))
    tick(auto=True)
    reason = node.finish_reason
    node.commands.put(('record_failed', 'missing route topics'))
    node.request_stop()
    tick(auto=True)
    node.commands.put(('record_closed', '/record'))
    tick(auto=True)
    assert node.finish_reason == reason
    assert node.terminal_state == 'ERROR'
    assert node.output.brake_engage
    assert node.record_error
    node.request_run(default_run_settings())
    tick()
    assert node.finish_reason == ''


def test_motor_setting_wait_blocks_input_but_not_stop(controlled):
    node, tick, _ = controlled
    node.steering_control.ready.return_value = False
    node.command_publisher = Mock()
    node.request_run(settings_with_speed('speed_step', 10))
    tick(auto=True)
    node.command_publisher.publish.assert_not_called()
    node.steering_control.ready.assert_called_with(False)
    node.request_stop()
    tick(auto=True)
    assert node.command_publisher.publish.call_args.args[0].brake_engage


def test_lost_telemetry_stops_active_input(controlled):
    node, tick, clock = controlled
    begin(node, tick)
    tick(5, auto=True)
    clock[0] += 1.1
    node._tick()
    assert node.output.brake_engage
    assert node.terminal_state == 'ERROR'


def test_ezkontrol_invalid_status_stops_only_after_500_ms_continuously(controlled):
    node, tick, _ = controlled
    begin(node, tick)
    tick(5, auto=True)
    drive = node.received['/drive/status'][1]

    drive.part2_valid = False
    tick(auto=True)
    tick(0.49, auto=True)
    assert node.state == 'RUNNING'
    assert not node.output.brake_engage

    drive.part2_valid = True
    tick(auto=True)
    drive.part2_valid = False
    tick(auto=True)
    tick(0.49, auto=True)
    assert node.state == 'RUNNING'
    tick(0.02, auto=True)
    assert node.terminal_state == 'ERROR'
    assert node.output.brake_engage
    assert 'EZkontrol 상태 미수신 500 ms 지속' in node.finish_reason


@pytest.mark.parametrize('missing_topics', [
    ('/imu/data_raw',),
    ('/gnss/fix', '/gnss/fix_velocity', '/tf_static'),
])
def test_missing_or_lost_optional_sensors_do_not_block_or_stop_run(
        controlled, missing_topics):
    node, tick, clock = controlled
    for topic in missing_topics:
        node.received.pop(topic)
        node.sensor_errors.pop(topic)
    node.scan_graph = lambda: None
    assert not node.issues()
    begin(node, tick)
    tick(5, auto=True)
    clock[0] += 1.1
    for topic, (_, message) in list(node.received.items()):
        if topic not in missing_topics:
            node.received[topic] = (clock[0], message)
    node._tick()
    assert node.state == 'RUNNING'
    assert not node.output.brake_engage


def test_gnss_inputs_remain_required_for_route_following(controlled):
    node, _, _ = controlled
    for topic in ('/gnss/fix', '/gnss/fix_velocity', '/tf_static'):
        node.received.pop(topic)
        node.sensor_errors.pop(topic)
    issues = node.issues(route_input=True)
    assert any('/gnss/fix 미수신' == issue for issue in issues)
    assert any('/gnss/fix_velocity 미수신' == issue for issue in issues)
    assert any('/tf_static 미수신' == issue for issue in issues)


@pytest.mark.parametrize('extra_planning', [True, False])
def test_unrelated_command_publisher_prevents_run_and_command_output(controlled, extra_planning):
    node, tick, _ = controlled
    other = Node('other_planner')
    if extra_planning:
        other.create_publisher(ActuatorCommand, '/planning/command', 10)
    other.create_publisher(ActuatorCommand, '/vehicle/command', 10)
    messages = []
    observer = Node('command_observer')
    observer.create_subscription(ActuatorCommand, '/planning/command', messages.append, 10)
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    executor.add_node(other)
    executor.add_node(observer)
    try:
        deadline = time.monotonic() + 2
        while len(node.get_publishers_info_by_topic('/planning/command')) < 1 + int(extra_planning):
            assert time.monotonic() < deadline
            executor.spin_once(timeout_sec=0.02)
        node.scan_graph()
        node.request_run(default_run_settings())
        tick()
        deadline = time.monotonic() + 0.2
        while time.monotonic() < deadline:
            executor.spin_once(timeout_sec=0.02)
        assert node.state == 'IDLE'
        assert node.process_actions.empty()
        assert not messages
    finally:
        executor.shutdown()
        observer.destroy_node()
        other.destroy_node()


@pytest.mark.parametrize('record_mcap', [True, False])
def test_real_governor_and_mcap_record_mode_and_input(tmp_path, record_mcap):
    # Existing ROS transports are used; the test supplies telemetry instead of hardware.
    rclpy.init()
    node = SysidNode()
    source = Node('steering_actuator_node')
    source.declare_parameter('steering_enabled', True)
    lateral_config = Path(get_package_share_directory('lateral_ctrl')) / 'config'
    steering_defaults = yaml.safe_load((lateral_config / 'steering_actuator.yaml').read_text())['steering_actuator_node']['ros__parameters']
    steering_defaults.update(yaml.safe_load((lateral_config / 'steering_motor_calibration.yaml').read_text()))
    for key, value in steering_defaults.items():
        source.declare_parameter(key, value)
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    executor.add_node(source)
    messages = telemetry()
    publishers = {}
    optional_sensor_topics = {
        '/gnss/fix', '/gnss/fix_velocity', '/imu/data_raw', '/tf_static',
    }
    for topic, kind in TOPICS.items():
        if topic in optional_sensor_topics:
            continue
        qos = (QoSProfile(depth=10, durability=DurabilityPolicy.TRANSIENT_LOCAL)
               if topic == '/tf_static' else 10)
        publishers[topic] = source.create_publisher(kind, topic, qos)
    from std_msgs.msg import Bool, Float32, Int32, String
    from vehicle_test_scenarios.sysid_capture import TORQUE_TOPICS
    for topic in TORQUE_TOPICS:
        if topic.endswith(('/actuation_mode', '/torque_safety')):
            kind, message = String, String(data='torque' if topic.endswith('/actuation_mode') else 'none')
        elif topic.endswith(('/torque_command_raw', '/torque_limit_raw')):
            kind, message = Int32, Int32(data=200)
        elif topic.endswith(('/torque_saturated', '/torque_peak_limited', '/torque_settled', '/torque_rate_ready')):
            kind, message = Bool, Bool(data=False)
        else:
            kind, message = Float32, Float32(data=1.0)
        publishers[topic] = source.create_publisher(kind, topic, 10)
        messages[topic] = message
    received = []
    source.create_subscription(ActuatorCommand, '/vehicle/command',
                               lambda m: received.append(m), 10)
    manager = sysid_processes.ProcessManager(node, records_root=tmp_path)
    last_publish = [0.0]

    def spin_until(condition, seconds=15):
        deadline = time.monotonic() + seconds
        while time.monotonic() < deadline:
            now = time.monotonic()
            if now - last_publish[0] >= 0.05:
                for topic, message in messages.items():
                    if topic in publishers:
                        publishers[topic].publish(message)
                last_publish[0] = now
            executor.spin_once(timeout_sec=0.03)
            if condition():
                return
        raise AssertionError((node.state, node.issues(), list(node.logs)[-8:]))

    governor = None
    try:
        governor = sysid_processes.OwnedProcess('test governor', [
            str(Path(get_package_prefix('command_governor')) /
                'lib/command_governor/command_governor_node'),
            '--ros-args', '--params-file',
            str(Path(get_package_share_directory('command_governor')) /
                'config/command_governor.yaml'),
        ], node.log)
        manager.start()
        spin_until(lambda: not node.issues())
        assert not received
        node.request_run(replace(
            default_run_settings('speed_ramp'), speed_high_kph=5.0,
            speed_ramp_duration_sec=1.0, scenario_repetitions=1,
            record_mcap=record_mcap,
            test_condition='driving',
        ))
        spin_until(lambda: node.state == 'WAIT_AUTO')
        # Change a gain after the start snapshot so the bag must retain the delta.
        source.set_parameters([Parameter('torque_kd', value=1800.0)])
        assert not source.get_parameter('steering_enabled').value
        assert all(m.speed_target_mps == 0 for m in received)
        messages['/drive/status'].mode = 1
        spin_until(lambda: node.state == 'COUNTDOWN')
        spin_until(lambda: node.state == 'RUNNING')
        spin_until(lambda: node.state in ('TAIL', 'CLOSING', 'COMPLETE'), seconds=16)
        assert any(m.speed_target_mps > 0.5 for m in received)
        assert all(m.speed_target_mps <= 5 / 3.6 + 1e-6 for m in received)
        spin_until(lambda: node.state in ('COMPLETE', 'ERROR'))
        assert node.state == 'COMPLETE', list(node.logs)
        assert node.output == baseline(settings=node.settings)
        if record_mcap:
            import rosbag2_py
            meta = rosbag2_py.Info().read_metadata(str(Path(node.run_dir) / 'rosbag'), 'mcap')
            counts = {entry.topic_metadata.name: entry.message_count for entry in meta.topics_with_message_count}
            assert counts['/planning/command'] > 0
            assert counts['/vehicle/command'] > 0
            assert counts['/drive/status'] > 0
            assert (Path(node.run_dir) / 'scenario.yaml').is_file()
            document = yaml.safe_load((Path(node.run_dir) / 'scenario.yaml').read_text())
            capture = yaml.safe_load((Path(node.run_dir) / 'control_parameters.yaml').read_text())
            assert document['test_condition']['key'] == 'driving'
            assert document['control_parameters']['status'] == 'complete'
            assert capture['nodes']['/steering_actuator_node']['ros__parameters']['torque_kd'] == 1500.0
            assert capture['nodes']['/steering_actuator_node']['ros__parameters']['steering_enabled'] is False
            assert capture['nodes']['/command_governor']['ros__parameters']['input_timeout_ms'] == 200.0
            assert counts['/sysid/control_parameters'] == 1
            assert counts['/parameter_events'] > 0
            assert all(counts.get(topic, 0) > 0 for topic in TORQUE_TOPICS)
            from rcl_interfaces.msg import ParameterEvent
            from rclpy.serialization import deserialize_message
            from std_msgs.msg import String
            reader = rosbag2_py.SequentialReader()
            reader.open(rosbag2_py.StorageOptions(uri=str(Path(node.run_dir) / 'rosbag'), storage_id='mcap'),
                        rosbag2_py.ConverterOptions('', ''))
            event_found = snapshot_found = False
            while reader.has_next():
                topic, payload, _ = reader.read_next()
                if topic == '/parameter_events':
                    event = deserialize_message(payload, ParameterEvent)
                    event_found |= any(p.name == 'torque_kd' and p.value.double_value == 1800.0 for p in event.changed_parameters)
                elif topic == '/sysid/control_parameters':
                    snapshot_found = json.loads(deserialize_message(payload, String).data) == capture
            assert event_found and snapshot_found
        else:
            assert node.run_dir == ''
            assert not list(tmp_path.iterdir())
            assert manager.recorder is None
        node.request_stop()
        spin_until(lambda: received[-1].brake_engage)
        manager.records_root = Path('/dev/null')
        node.request_run(replace(default_run_settings(), cruise_speed_kph=5.0))
        spin_until(lambda: node.state == 'ERROR')
        assert node.output.brake_engage
        assert not node.record_open
        assert manager.recorder is None
    finally:
        node.request_stop()
        executor.spin_once(timeout_sec=0.1)
        manager.close()
        if governor is not None:
            governor.stop()
        executor.shutdown()
        source.destroy_node()
        node.destroy_node()
        rclpy.shutdown()
        if governor is not None:
            assert not sysid_processes.group_members(governor.pgid)
