from __future__ import annotations

import threading

from kaiev26_hil.vcan_sim import app
from kaiev26_hil.vcan_sim.can_io import CanFrame, ID_MOTOR_CMD, SocketCANBusHub


class DummyHub:
    def __init__(self, ifaces):
        self.ifaces = list(ifaces)
        self.subscriptions = {}

    def subscribe(self, iface, callback):
        self.subscriptions.setdefault(iface, []).append(callback)

    def send(self, frame):
        return None

    def health_snapshot(self):
        return None

    def close(self):
        return None


def test_socketcan_hub_tracks_callback_errors(caplog):
    hub = object.__new__(SocketCANBusHub)
    hub._lock = threading.Lock()
    hub._subs = {"vcan0": [lambda _frame: (_ for _ in ()).throw(RuntimeError("boom"))]}

    frame = CanFrame("vcan0", ID_MOTOR_CMD, b"\xAA" * 8)

    with caplog.at_level("WARNING", logger="can_io"):
        hub._dispatch(frame)

    snapshot = hub.health_snapshot()
    assert snapshot.dispatched_frames == 1
    assert snapshot.callback_errors == 1
    assert "callback raised" in caplog.text


def test_runtime_tracks_frame_listener_errors(monkeypatch):
    monkeypatch.setattr(app, "SocketCANBusHub", DummyHub)
    runtime = app.VcanSimRuntime("vcan0", "vcan1")

    try:
        runtime.add_frame_listener(lambda _frame: (_ for _ in ()).throw(RuntimeError("listener boom")))
        runtime._on_frame(CanFrame("vcan0", ID_MOTOR_CMD, b"\xAA" * 8))
        diagnostics = runtime.get_diagnostics()
        assert diagnostics.listener_errors == 1
    finally:
        runtime.close()
