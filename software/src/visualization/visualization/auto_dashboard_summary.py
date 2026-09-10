"""Aggregate AUTO display evidence and publish a compact STM_B summary."""

from __future__ import annotations

import math
import time

from kaiev26_msgs.msg import (
    BehaviorDecision,
    BrakeActuatorStatus,
    DriveActuatorStatus,
    RouteContext,
    SteeringActuatorStatus,
    TargetSpeed,
)
from nav_msgs.msg import Odometry
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CompressedImage, PointCloud2
from std_msgs.msg import Int32, UInt8MultiArray
from um980_msgs.msg import Status as GnssStatus

from .auto_dashboard_protocol import (
    ACTIVITY_AUTO_ACTIVE,
    ACTIVITY_AVOID_OBSTACLE,
    ACTIVITY_EMERGENCY,
    ACTIVITY_FINISH,
    ACTIVITY_FOLLOW_ROUTE,
    ACTIVITY_INTERSECTION,
    ACTIVITY_REJOIN_ROUTE,
    ACTIVITY_TRAFFIC_CONTROL,
    ACTIVITY_WAIT,
    AutoDashboardSummary,
    BRAKE_CMD_LOST,
    BRAKE_OFF,
    BRAKE_ON,
    BRAKE_STATUS_LOST,
    BRAKE_WAIT,
    DRIVE_CMD_LOST,
    DRIVE_FAULT,
    DRIVE_HOLD,
    DRIVE_NO_FEEDBACK,
    DRIVE_RUN,
    DRIVE_RUN_MISMATCH,
    DRIVE_STATUS_LOST,
    DRIVE_WAIT,
    DRIVE_WAIT_CMD,
    FEEDBACK_LIVE,
    FEEDBACK_LOST,
    FEEDBACK_UNKNOWN,
    GPS_FIX,
    GPS_FIXED,
    GPS_FLOAT,
    GPS_NO_FIX,
    GPS_WAIT,
    INVALID_HEADING_CENTIDEG,
    INVALID_I16,
    INVALID_I32,
    OUTPUT_OFF,
    OUTPUT_ON,
    OUTPUT_UNKNOWN,
    pack_summary,
    STEER_ACTIVE,
    STEER_BRAKE,
    STEER_CAN_LOST,
    STEER_CMD_LOST,
    STEER_DISABLED,
    STEER_ESTOP,
    STEER_MOTOR_FAULT,
    STEER_POS_LOST,
    STEER_STATUS_LOST,
    STEER_TRACK_FAULT,
    STEER_TRACK_WARN,
    STEER_WAIT,
    STEER_WAIT_CMD,
    STREAM_LOST,
    STREAM_RX,
    STREAM_WAIT,
)


SENSOR_QOS = QoSProfile(
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.VOLATILE,
)


class TimedValue:

    def __init__(self) -> None:
        self.value = None
        self.received_at = None

    def update(self, value) -> None:
        self.value = value
        self.received_at = time.monotonic()

    def fresh(self, timeout: float, now: float) -> bool:
        return self.received_at is not None and now - self.received_at <= timeout


def heading_centideg(yaw_rad: float) -> int:
    # ROS ENU yaw: east=0 and CCW positive. Cockpit: north=0 and CW positive.
    return round(((90.0 - math.degrees(yaw_rad)) % 360.0) * 100.0) % 36000


def yaw_from_odometry(message: Odometry) -> float | None:
    q = message.pose.pose.orientation
    values = (float(q.x), float(q.y), float(q.z), float(q.w))
    if not all(math.isfinite(value) for value in values):
        return None
    norm = math.sqrt(sum(value * value for value in values))
    if norm <= 1.0e-9:
        return None
    x, y, z, w = (value / norm for value in values)
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


class AutoDashboardSummaryNode(Node):

    def __init__(self) -> None:
        super().__init__('auto_dashboard_summary')
        self.declare_parameter('publish_hz', 10.0)
        self.declare_parameter('actuator_stale_sec', 0.30)
        self.declare_parameter('sensor_stale_sec', 0.50)
        self.declare_parameter('gnss_stale_sec', 1.50)
        self.declare_parameter('planning_stale_sec', 0.60)

        self.actuator_stale = float(self.get_parameter('actuator_stale_sec').value)
        self.sensor_stale = float(self.get_parameter('sensor_stale_sec').value)
        self.gnss_stale = float(self.get_parameter('gnss_stale_sec').value)
        self.planning_stale = float(self.get_parameter('planning_stale_sec').value)
        publish_hz = float(self.get_parameter('publish_hz').value)

        self.gnss = TimedValue()
        self.lidar = TimedValue()
        self.camera_left = TimedValue()
        self.camera_right = TimedValue()
        self.drive = TimedValue()
        self.steer = TimedValue()
        self.brake = TimedValue()
        self.odometry = TimedValue()
        self.route = TimedValue()
        self.target_speed = TimedValue()
        self.behavior = TimedValue()
        self.target_motor_count = TimedValue()

        self.create_subscription(GnssStatus, '/gnss/status', self.gnss.update, 10)
        self.create_subscription(
            PointCloud2, '/ouster/points', self.lidar.update, SENSOR_QOS
        )
        self.create_subscription(
            CompressedImage,
            '/perception/camera/left/source/image_raw/compressed',
            self.camera_left.update,
            SENSOR_QOS,
        )
        self.create_subscription(
            CompressedImage,
            '/perception/camera/right/source/image_raw/compressed',
            self.camera_right.update,
            SENSOR_QOS,
        )
        self.create_subscription(
            DriveActuatorStatus, '/drive/status', self.drive.update, 10
        )
        self.create_subscription(
            SteeringActuatorStatus, '/steering/status', self.steer.update, 10
        )
        self.create_subscription(
            BrakeActuatorStatus, '/brake/status', self.brake.update, 10
        )
        self.create_subscription(
            Odometry, '/localization/odometry', self.odometry.update, 10
        )
        self.create_subscription(
            RouteContext, '/planning/route_context', self.route.update, 10
        )
        self.create_subscription(
            TargetSpeed, '/planning/target_speed', self.target_speed.update, 10
        )
        self.create_subscription(
            BehaviorDecision, '/planning/behavior_decision', self.behavior.update, 10
        )
        self.create_subscription(
            Int32,
            '/steering/target_motor_position_count',
            self.target_motor_count.update,
            10,
        )

        self.publisher = self.create_publisher(
            UInt8MultiArray, '/dashboard/auto_status', 10
        )
        self.create_timer(1.0 / publish_hz, self.publish_summary)

    @staticmethod
    def stream_state(value: TimedValue, now: float, timeout: float) -> int:
        if value.received_at is None:
            return STREAM_WAIT
        return STREAM_RX if value.fresh(timeout, now) else STREAM_LOST

    def gps_state(self, now: float) -> int:
        if self.gnss.received_at is None or not self.gnss.fresh(self.gnss_stale, now):
            return GPS_WAIT
        mode = str(self.gnss.value.fix_mode).upper()
        if mode == 'RTK_FIXED':
            return GPS_FIXED
        if mode == 'RTK_FLOAT':
            return GPS_FLOAT
        if mode in ('DGNSS', 'GNSS'):
            return GPS_FIX
        if mode == 'NO_GNSS':
            return GPS_NO_FIX
        return GPS_WAIT

    def drive_state(self, now: float) -> int:
        if self.drive.received_at is None:
            return DRIVE_WAIT
        if not self.drive.fresh(self.actuator_stale, now):
            return DRIVE_STATUS_LOST
        status = self.drive.value
        if status.fault_bits_valid and status.fault_bits:
            return DRIVE_FAULT
        if status.command_timeout:
            return DRIVE_CMD_LOST
        if not status.command_received:
            return DRIVE_WAIT_CMD
        if not (
            status.part1_valid
            and status.part2_valid
            and status.tractive_feedback_valid
            and status.fault_bits_valid
        ):
            return DRIVE_NO_FEEDBACK
        if status.tractive_commanded != status.tractive_feedback:
            return DRIVE_RUN_MISMATCH
        if status.stop_request or not status.tractive_commanded:
            return DRIVE_HOLD
        return DRIVE_RUN

    def steer_state(self, now: float) -> int:
        if self.steer.received_at is None:
            return STEER_WAIT
        if not self.steer.fresh(self.actuator_stale, now):
            return STEER_STATUS_LOST
        status = self.steer.value
        if status.state == SteeringActuatorStatus.STATE_FAULT or status.fault_bits & ~1:
            return STEER_MOTOR_FAULT
        if not status.communication_ok:
            return STEER_CAN_LOST
        if not status.feedback_valid:
            return STEER_POS_LOST
        if status.mode_timeout or status.steer_command_timeout:
            return STEER_CMD_LOST
        if status.tracking_error_fault:
            return STEER_TRACK_FAULT
        if status.tracking_error_warning:
            return STEER_TRACK_WARN
        if status.state == SteeringActuatorStatus.STATE_ACTIVE and status.enabled_commanded:
            return STEER_ACTIVE
        if status.state == SteeringActuatorStatus.STATE_BRAKE:
            return STEER_BRAKE
        if status.state == SteeringActuatorStatus.STATE_ESTOP:
            return STEER_ESTOP
        if status.state == SteeringActuatorStatus.STATE_WAIT_CMD:
            return STEER_WAIT_CMD
        return STEER_DISABLED

    def brake_state(self, now: float) -> int:
        if self.brake.received_at is None:
            return BRAKE_WAIT
        if not self.brake.fresh(self.actuator_stale, now):
            return BRAKE_STATUS_LOST
        status = self.brake.value
        if status.stop_reason_bits & BrakeActuatorStatus.STOP_REASON_COMMAND_INVALID:
            return BRAKE_CMD_LOST
        return BRAKE_ON if status.brake_commanded else BRAKE_OFF

    def activity_state(self, now: float) -> int:
        if self.behavior.received_at is None or not self.behavior.fresh(self.planning_stale, now):
            return ACTIVITY_WAIT
        behavior = str(self.behavior.value.active_behavior).upper()
        return {
            'LANE_FOLLOW': ACTIVITY_FOLLOW_ROUTE,
            'GLOBAL_ROUTE_FOLLOW': ACTIVITY_FOLLOW_ROUTE,
            'ROUTE_REJOIN': ACTIVITY_REJOIN_ROUTE,
            'TRAFFIC_LIGHT': ACTIVITY_TRAFFIC_CONTROL,
            'OBSTACLE': ACTIVITY_AVOID_OBSTACLE,
            'INTERSECTION': ACTIVITY_INTERSECTION,
            'FINISH': ACTIVITY_FINISH,
            'EMERGENCY': ACTIVITY_EMERGENCY,
        }.get(behavior, ACTIVITY_AUTO_ACTIVE)

    def motion_values(self, now: float) -> tuple[int, int, int]:
        heading = INVALID_HEADING_CENTIDEG
        target_heading = INVALID_HEADING_CENTIDEG
        target_speed = INVALID_I16
        yaw = None
        if self.odometry.fresh(self.sensor_stale, now):
            yaw = yaw_from_odometry(self.odometry.value)
            if yaw is not None:
                heading = heading_centideg(yaw)
        if (
            yaw is not None
            and self.route.fresh(self.planning_stale, now)
            and self.route.value.route_projection_valid
            and math.isfinite(float(self.route.value.heading_error))
        ):
            target_heading = heading_centideg(yaw + float(self.route.value.heading_error))
        if self.target_speed.fresh(self.planning_stale, now):
            speed_mps = float(self.target_speed.value.target_speed_mps)
            if math.isfinite(speed_mps):
                target_speed = max(-32767, min(32767, round(speed_mps * 360.0)))
        return heading, target_heading, target_speed

    def steering_values(self, now: float) -> tuple[int, int, int, int, int]:
        actual = INVALID_I32
        target = INVALID_I32
        current = INVALID_I16
        output = OUTPUT_UNKNOWN
        feedback = FEEDBACK_UNKNOWN
        if self.steer.fresh(self.actuator_stale, now):
            status = self.steer.value
            output = OUTPUT_ON if status.enabled_commanded else OUTPUT_OFF
            feedback = (
                FEEDBACK_LIVE
                if status.communication_ok and status.feedback_valid
                else FEEDBACK_LOST
            )
            if status.feedback_valid:
                actual = int(status.motor_position_count)
            if status.communication_ok and math.isfinite(float(status.motor_current_a)):
                current = max(-32767, min(32767, round(float(status.motor_current_a) * 10.0)))
            if (
                status.steer_command_received
                and not status.steer_command_timeout
                and self.target_motor_count.fresh(self.actuator_stale, now)
            ):
                target = int(self.target_motor_count.value.data)
        return actual, target, current, output, feedback

    def publish_summary(self) -> None:
        now = time.monotonic()
        heading, target_heading, target_speed = self.motion_values(now)
        actual, target, current, output, feedback = self.steering_values(now)
        summary = AutoDashboardSummary(
            gps=self.gps_state(now),
            lidar=self.stream_state(self.lidar, now, self.sensor_stale),
            camera_left=self.stream_state(self.camera_left, now, self.sensor_stale),
            camera_right=self.stream_state(self.camera_right, now, self.sensor_stale),
            drive=self.drive_state(now),
            steer=self.steer_state(now),
            brake=self.brake_state(now),
            activity=self.activity_state(now),
            output=output,
            feedback=feedback,
            heading_centideg=heading,
            target_heading_centideg=target_heading,
            target_speed_centi_kmh=target_speed,
            actual_motor_count=actual,
            steering_current_deciamp=current,
            target_motor_count=target,
        )
        message = UInt8MultiArray()
        message.data = pack_summary(summary)
        self.publisher.publish(message)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = AutoDashboardSummaryNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
