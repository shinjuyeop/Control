from kaiev26_hil.models import HilDesiredState, HilStatus, ProtocolResponse
from kaiev26_hil.stm_test_bridge import BridgeWorker


def make_status(*, protocol=2, plant=False, speed=0.0, rpm=0.0):
    return HilStatus(
        mode="MANUAL", gear="FWD", pedal=0.0,
        estop_dash=False, estop_side=False, estop_rf=False,
        speed=speed, rpm=rpm, voltage=0.0, current=0.0, motor_temp=0.0,
        brake_period_us=0, brake_us=0, brake="UNKNOWN", pwm_lost=True,
        steer=0.0, steer_target=0.0,
        ez_rx=0, ez_tx=0, keya_rx=0, keya_tx=0,
        ez_can=True, keya_can=True, ez_ok=True, keya_ok=True,
        ez_fault=False, keya_fault=False,
        plant=plant, ez_target_rpm=0.0, protocol=protocol,
    )


class FakeClient:
    def __init__(self, status):
        self.status = status
        self.commands = []

    def request(self, command):
        self.commands.append(command)
        if command == "STATUS":
            return ProtocolResponse("status", "STATUS ...", status=self.status)
        return ProtocolResponse("ok", f"OK {command}", ok_command=command)


def make_worker(events):
    return BridgeWorker(
        serial_port="fake", baud_rate=115200, status_poll_hz=5.0,
        command_timeout_sec=0.1, retry_count=0,
        reconnect_interval_sec=0.1,
        event_callback=lambda kind, payload: events.append((kind, payload)),
    )


def test_replay_omits_plant_commands_for_protocol_1_firmware():
    events = []
    worker = make_worker(events)
    worker._client = FakeClient(make_status(protocol=1))
    worker._protocol_version = 1

    worker._apply_desired_after_connect()

    assert not any(
        command.startswith("SET PLANT ") for command in worker._client.commands
    )


def test_plant_off_captures_current_output_for_reconnect_replay():
    events = []
    worker = make_worker(events)
    worker._client = FakeClient(
        make_status(protocol=2, plant=True, speed=16.25, rpm=842.0)
    )
    worker._protocol_version = 2

    worker._handle_command("SET PLANT OFF")

    desired = worker.desired_snapshot()
    assert worker._client.commands == ["STATUS", "SET PLANT OFF"]
    assert desired.plant_enabled is False
    assert desired.motion_source == "RPM"
    assert desired.rpm == 842.0
    assert desired.speed == 16.25


def test_protocol_1_plant_command_is_rejected_without_serial_disconnect():
    events = []
    worker = make_worker(events)
    worker._client = FakeClient(make_status(protocol=1))
    worker._protocol_version = 1

    worker._handle_command("SET PLANT ON")

    assert worker._client.commands == []
    assert events[-1] == (
        "result", "ERR plant_not_supported_by_STM_TEST_protocol_1"
    )


def test_safe_state_replacement_discards_commands_queued_while_disconnected():
    events = []
    worker = make_worker(events)
    worker.enqueue("SET PEDAL 80")

    worker.replace_desired(HilDesiredState(), enqueue=True)

    assert worker.commands.empty()
    assert worker.desired_snapshot().pedal == 0.0
