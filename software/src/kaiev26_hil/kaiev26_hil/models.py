"""Data models shared by the serial bridge, GUI, and scenario runner."""

from __future__ import annotations

from dataclasses import asdict, dataclass
import json
from typing import Optional


@dataclass(frozen=True)
class HilStatus:
    mode: str
    gear: str
    pedal: float
    estop_dash: bool
    estop_side: bool
    estop_rf: bool
    speed: float
    rpm: float
    voltage: float
    current: float
    motor_temp: float
    brake_period_us: int
    brake_us: int
    brake: str
    pwm_lost: bool
    steer: float
    steer_target: float
    ez_rx: int
    ez_tx: int
    keya_rx: int
    keya_tx: int
    ez_can: bool
    keya_can: bool
    ez_ok: bool
    keya_ok: bool
    ez_fault: bool
    keya_fault: bool
    plant: bool
    ez_target_rpm: float
    ez_running: bool = False
    ez_speed_control: bool = False
    ez_command_rpm: float = 0.0
    ez_command_current: float = 0.0
    ez_rx_age_ms: int = 4294967295
    keya_enabled: bool = False
    keya_rx_age_ms: int = 4294967295
    protocol: int = 1
    received_monotonic: float = 0.0

    def to_json(self) -> str:
        return json.dumps(asdict(self), sort_keys=True)


@dataclass
class HilDesiredState:
    mode: str = "MANUAL"
    gear: str = "FORWARD"
    pedal: float = 0.0
    estop_dashboard: bool = False
    estop_side: bool = False
    estop_rf: bool = False
    speed: float = 0.0
    rpm: float = 0.0
    motion_source: str = "RPM"
    voltage: float = 0.0
    current: float = 0.0
    motor_temp: float = 0.0
    steering_position: float = 0.0
    ez_can_enabled: bool = True
    keya_can_enabled: bool = True
    # Closed-loop traction is the default the bench runs in: the bridge replays
    # this on connect, so STM_TEST leaves its plant-off power-on state without
    # the operator having to select it. Safe State returns here too. The KEYA
    # steering emulator needs no equivalent -- it follows its CAN2 position
    # command whenever the motor is enabled, with no open-loop mode to pick.
    plant_enabled: bool = True
    ez_drop_tx: bool = False
    keya_drop_tx: bool = False
    ez_fault: bool = False
    keya_fault: bool = False

    def to_json(self) -> str:
        return json.dumps(asdict(self), sort_keys=True)


@dataclass(frozen=True)
class ProtocolResponse:
    kind: str
    raw: str
    ok_command: Optional[str] = None
    error_reason: Optional[str] = None
    status: Optional[HilStatus] = None
