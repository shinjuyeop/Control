"""제어 소유 절반의 시험 — 안전 override와 입력 단절 시의 처신.

경로추종·속도계획 절반은 판단 저장소의 `kaiev26_motion_control` 시험이 갖는다.
"""
from kaiev26_actuator_command_bridge.actuator_command_bridge_node import (
    ActuatorCommandBridgeNode,
)
from kaiev26_msgs.msg import ActuatorCommand, AEBState, VehicleState


class FakeClockTime:
    """`rclpy.time.Time` 대신 쓰는 최소 대역. nanoseconds와 to_msg만 필요하다."""

    def __init__(self, nanoseconds: int) -> None:
        self.nanoseconds = nanoseconds

    def to_msg(self):
        stamp = ActuatorCommand().header.stamp
        stamp.sec = self.nanoseconds // 1_000_000_000
        stamp.nanosec = self.nanoseconds % 1_000_000_000
        return stamp


def make_bridge(
    motion: ActuatorCommand | None,
    estop: bool = False,
    aeb: bool = False,
    steering_rad: float = 0.0,
) -> ActuatorCommandBridgeNode:
    bridge = ActuatorCommandBridgeNode.__new__(ActuatorCommandBridgeNode)
    bridge.motion_stale_sec = 0.35
    bridge.max_steering_rad = 0.4363
    bridge.latest_motion = motion
    bridge.latest_vehicle_state = VehicleState()
    bridge.latest_vehicle_state.estop_active = estop
    bridge.latest_vehicle_state.steering_rad = steering_rad
    bridge.latest_aeb_state = AEBState()
    bridge.latest_aeb_state.active = aeb
    bridge.publish_debug = lambda command, reason: None
    return bridge


def make_motion(stamp_ns: int, speed: float = 3.0, steering: float = 0.2) -> ActuatorCommand:
    motion = ActuatorCommand()
    motion.header.stamp.sec = stamp_ns // 1_000_000_000
    motion.header.stamp.nanosec = stamp_ns % 1_000_000_000
    motion.enable = True
    motion.speed_target_mps = speed
    motion.steering_target_rad = steering
    motion.brake_engage = False
    motion.gear = ActuatorCommand.GEAR_FORWARD
    return motion


NOW_NS = 10 * 1_000_000_000


def test_fresh_motion_passes_through_unchanged() -> None:
    bridge = make_bridge(make_motion(NOW_NS))

    command, reason = bridge.compose_command(FakeClockTime(NOW_NS))

    assert reason == "normal"
    assert command.enable is True
    assert command.speed_target_mps == 3.0
    assert abs(command.steering_target_rad - 0.2) < 1e-6
    assert command.brake_engage is False


def test_estop_overrides_motion_and_centers_steering() -> None:
    bridge = make_bridge(make_motion(NOW_NS), estop=True, steering_rad=0.3)

    command, reason = bridge.compose_command(FakeClockTime(NOW_NS))

    assert reason == "estop"
    assert command.speed_target_mps == 0.0
    assert command.brake_engage is True
    assert command.steering_target_rad == 0.0


def test_aeb_overrides_motion_and_holds_measured_steering() -> None:
    bridge = make_bridge(make_motion(NOW_NS), aeb=True, steering_rad=0.3)

    command, reason = bridge.compose_command(FakeClockTime(NOW_NS))

    assert reason.startswith("aeb")
    assert command.speed_target_mps == 0.0
    assert command.brake_engage is True
    assert abs(command.steering_target_rad - 0.3) < 1e-6


def test_estop_wins_over_aeb() -> None:
    bridge = make_bridge(make_motion(NOW_NS), estop=True, aeb=True, steering_rad=0.3)

    _command, reason = bridge.compose_command(FakeClockTime(NOW_NS))

    assert reason == "estop"


def test_missing_motion_is_stale() -> None:
    bridge = make_bridge(None)

    assert bridge.motion_is_stale(FakeClockTime(NOW_NS)) is True


def test_old_motion_is_stale_and_is_not_relaundered() -> None:
    """오래된 운동 명령에 새 stamp를 붙여 내보내지 않는다.

    분할이 만든 유일한 새 판정이다. 발행을 멈추는 것은 분할 전 게이트 노드가
    죽었을 때와 같은 결과이며, 새 정지 게이트가 아니다.
    """
    stale_ns = NOW_NS - int(0.5 * 1_000_000_000)
    bridge = make_bridge(make_motion(stale_ns))

    assert bridge.motion_is_stale(FakeClockTime(NOW_NS)) is True


def test_motion_within_the_window_is_fresh() -> None:
    fresh_ns = NOW_NS - int(0.3 * 1_000_000_000)
    bridge = make_bridge(make_motion(fresh_ns))

    assert bridge.motion_is_stale(FakeClockTime(NOW_NS)) is False
