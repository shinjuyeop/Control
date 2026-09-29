import json
from pathlib import Path
import signal
import subprocess
import sys
import time

import pytest
import yaml

from control_tools.topics import recording_topics
from control_tools.recording import storage_options, recorded_counts


def test_camera_lidar_and_other_sensor_selections_are_preserved():
    before = json.loads((Path(__file__).parent / 'fixtures/recording_topics_before.json').read_text())
    prefixes = ('/perception/', '/ouster/', '/lidar', '/imu/', '/left/', '/right/', '/gnss/', '/ebimu/')
    for profile, original in before.items():
        actual = recording_topics(profile)
        assert {t for t in original if t.startswith(prefixes)} == {t for t in actual if t.startswith(prefixes)}
        assert len(actual) == len(set(actual))
        assert '/sysid/scenario' in actual and '/sysid/scinario' not in actual
        assert {'/stm_a/runtime_stats', '/stm_b/runtime_stats', '/rosout', '/lv_battery/state',
                '/drive/longitudinal_status', '/sysid/control_parameters'} <= set(actual)


def longitudinal_data(p=1., revision=1):
    from control_tuning.longitudinal import config_wire, AIRBORNE, VERSION
    return [float(VERSION), float(revision), 1., 1., 1., *config_wire(AIRBORNE),
            .1, p, 2., 0., p + 2., p + 2., 0., 0., 0., 1., 0.]


def test_analysis_preserves_pi_samples_with_unchanged_revision(tmp_path):
    import rosbag2_py
    from rclpy.serialization import serialize_message
    from std_msgs.msg import Float32MultiArray
    from kaiev26_msgs.msg import DriveActuatorStatus, ActuatorCommand
    from control_tools.longitudinal_analysis import read_bag
    writer = rosbag2_py.SequentialWriter()
    writer.open(storage_options(tmp_path / 'bag'), rosbag2_py.ConverterOptions('', ''))
    for topic, typename, message in (
        ('/drive/status', 'kaiev26_msgs/msg/DriveActuatorStatus', DriveActuatorStatus()),
        ('/vehicle/command', 'kaiev26_msgs/msg/ActuatorCommand', ActuatorCommand()),
        ('/drive/longitudinal_status', 'std_msgs/msg/Float32MultiArray', Float32MultiArray(data=longitudinal_data())),
    ):
        writer.create_topic(rosbag2_py.TopicMetadata(name=topic, type=typename, serialization_format='cdr'))
        writer.write(topic, serialize_message(message), 1_000_000_000)
    writer.write('/drive/longitudinal_status', serialize_message(Float32MultiArray(data=longitudinal_data(7.))), 1_200_000_000)
    del writer
    _, provenance = read_bag(str(tmp_path / 'bag'))
    assert len(provenance['calibration']) == 1
    assert [s['p_current_a'] for s in provenance['pi_samples']] == [1., 7.]
    assert [s['t'] for s in provenance['pi_samples']] == [1., 1.2]


@pytest.mark.parametrize('provide_longitudinal', [True, False])
def test_independent_recording_snapshot_shutdown_and_missing_feedback(tmp_path, monkeypatch, provide_longitudinal):
    import rclpy
    from rclpy.node import Node
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.qos import qos_profile_sensor_data
    from std_msgs.msg import Float32MultiArray
    from control_tuning.schema import STEERING, GOVERNOR
    monkeypatch.setenv('ROS_DOMAIN_ID', '226')
    monkeypatch.setenv('ROS_LOCALHOST_ONLY', '1')
    rclpy.init(args=[])
    steering, governor, stm = Node(STEERING), Node(GOVERNOR), Node('stm_a_node')
    config = Path(__file__).resolve().parents[2] / 'lateral_ctrl/config'
    values = yaml.safe_load((config / 'steering_actuator.yaml').read_text())[STEERING]['ros__parameters']
    values.update(yaml.safe_load((config / 'steering_motor_calibration.yaml').read_text()))
    values.update(steering_enabled=True, torque_direction_sign=-1)
    for key, value in values.items():
        steering.declare_parameter(key, value)
    for key, value in dict(input_topic='/planning/command', output_topic='/vehicle/command', input_timeout_ms=234.).items():
        governor.declare_parameter(key, value)
    publisher = stm.create_publisher(Float32MultiArray, '/drive/longitudinal_status', qos_profile_sensor_data) if provide_longitudinal else None
    executor = SingleThreadedExecutor()
    for node in (steering, governor, stm):
        executor.add_node(node)
    output = (tmp_path / 'process.log').open('w')
    child = subprocess.Popen([sys.executable, '-u', '-c',
        'from control_tools.record_mcap import main; raise SystemExit(main())',
        '--output-base-dir', str(tmp_path / 'records')], stdout=output, stderr=subprocess.STDOUT,
        start_new_session=True)
    try:
        deadline = time.monotonic() + 24
        run = None
        while time.monotonic() < deadline and child.poll() is None:
            executor.spin_once(timeout_sec=.03)
            if publisher is not None:
                publisher.publish(Float32MultiArray(data=longitudinal_data()))
            manifests = list((tmp_path / 'records').glob('*/recording.yaml'))
            if manifests:
                run = manifests[0].parent
                if yaml.safe_load(manifests[0].read_text())['status'] == 'recording':
                    assert provide_longitudinal
                    # Let the reliable snapshot reach the worker before requesting stop.
                    for _ in range(15):
                        publisher.publish(Float32MultiArray(data=longitudinal_data()))
                        executor.spin_once(timeout_sec=.03)
                    child.send_signal(signal.SIGINT)
                    break
        assert run is not None
        code = child.wait(timeout=12)
        document = yaml.safe_load((run / 'recording.yaml').read_text())
        if provide_longitudinal:
            assert code == 0, (tmp_path / 'process.log').read_text()
            assert document['status'] == 'complete'
            capture = yaml.safe_load((run / 'control_parameters.yaml').read_text())
            assert capture['nodes']['/' + STEERING]['ros__parameters']['torque_direction_sign'] == -1
            assert capture['longitudinal']['readback']['config']['control_mode'] == 'torque'
            counts = recorded_counts(run / 'rosbag')
            assert counts['/sysid/control_parameters'] == 1
            assert counts['/drive/longitudinal_status'] > 0
        else:
            assert code == 1
            assert document['status'] == 'incomplete'
            assert document['parameter_capture'] == 'pending'
    finally:
        if child.poll() is None:
            child.send_signal(signal.SIGINT)
            child.wait(timeout=12)
        output.close()
        executor.shutdown()
        for node in (steering, governor, stm):
            node.destroy_node()
        rclpy.shutdown()


def test_independent_recording_without_controllers_closes_worker_on_sigterm(tmp_path, monkeypatch):
    from control_tools.process import group_members
    monkeypatch.setenv('ROS_DOMAIN_ID', '229')
    monkeypatch.setenv('ROS_LOCALHOST_ONLY', '1')
    with (tmp_path / 'process.log').open('w') as output:
        child = subprocess.Popen([sys.executable, '-u', '-c',
            'from control_tools.record_mcap import main; raise SystemExit(main())',
            '--output-base-dir', str(tmp_path / 'records'), '--no-capture-parameters'],
            stdout=output, stderr=subprocess.STDOUT, start_new_session=True)
        try:
            deadline = time.monotonic() + 20
            run = None
            while child.poll() is None and time.monotonic() < deadline:
                manifests = list((tmp_path / 'records').glob('*/recording.yaml'))
                if manifests and yaml.safe_load(manifests[0].read_text())['status'] == 'recording':
                    run = manifests[0].parent
                    break
                time.sleep(.05)
            assert run is not None, (tmp_path / 'process.log').read_text()
            workers = [int(pid) for pid in Path(f'/proc/{child.pid}/task/{child.pid}/children').read_text().split()]
            assert workers
            child.terminate()
            assert child.wait(timeout=12) == 0, (tmp_path / 'process.log').read_text()
            document = yaml.safe_load((run / 'recording.yaml').read_text())
            assert document['status'] == 'complete'
            assert document['parameter_capture'] == 'disabled'
            assert (run / 'rosbag/metadata.yaml').exists()
            assert not (run / 'control_parameters.yaml').exists()
            assert not any(group_members(pid) for pid in workers)
        finally:
            if child.poll() is None:
                child.send_signal(signal.SIGINT)
                child.wait(timeout=12)
