"""Shared ROS reception and RuntimeStats interpretation; no command paths."""

import math
import time
from collections import defaultdict, deque

from diagnostic_msgs.msg import DiagnosticArray
from kaiev26_msgs.msg import (ActuatorCommand, BrakeActuatorStatus, DriveActuatorStatus, RuntimeStats, SteeringActuatorStatus)
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rosidl_runtime_py.convert import message_to_ordereddict

from .runtime_config import BOARDS, RUNTIME_STALE_SECONDS
from .runtime_health import RuntimeHealth

STALE_SECONDS = 1.0
RATE_WINDOW_SECONDS = 5.0
VEHICLE_TOPICS = (
    (ActuatorCommand, "/vehicle/command"),
    (DriveActuatorStatus, "/drive/status"),
    (SteeringActuatorStatus, "/steering/status"),
    (BrakeActuatorStatus, "/brake/status"),
    *((RuntimeStats, topic) for topic in BOARDS),
    (DiagnosticArray, "/diagnostics"),
)


def json_message(message):
    # ROS explicitly uses +inf for an unavailable age. JSON sends it as null.
    def convert(value):
        if isinstance(value, dict):
            return {key: convert(item) for key, item in value.items()}
        if isinstance(value, (tuple, list)):
            return [convert(item) for item in value]
        if isinstance(value, float) and not math.isfinite(value):
            return None
        return value

    result = convert(message_to_ordereddict(message))
    if isinstance(message, DiagnosticArray):
        # Humble represents DiagnosticStatus.byte as a one-character string in
        # rosidl dictionaries. The browser's severity is the numeric ROS enum.
        for status in result["status"]:
            if isinstance(status["level"], str):
                status["level"] = ord(status["level"])
    return result


class ObserverNode(Node):
    def __init__(self, name, *, stale_seconds=None, runtime_health=None, **kwargs):
        super().__init__(name, **kwargs)
        self.samples = {}
        self.diagnostics = {}
        self.runtime_health = runtime_health if runtime_health is not None else RuntimeHealth()
        self.received_times = defaultdict(deque)
        self.received_count = defaultdict(int)
        self.stale_seconds = stale_seconds or {}
        for msg_type, topic in VEHICLE_TOPICS:
            self.subscribe(msg_type, topic)

    def subscribe(self, msg_type, topic):
        self.create_subscription(
            msg_type, topic,
            lambda message: self.receive(topic, json_message(message)),
            qos_profile_sensor_data,
        )

    def receive(self, topic, data):
        now = time.monotonic()
        self.samples[topic] = (now, data)
        if topic == "/diagnostics":
            for status in data["status"]:
                # Identity and freshness belong to each status, never to its publisher's array.
                self.diagnostics[(status["hardware_id"], status["name"])] = (now, status)
        if topic in BOARDS:
            self.runtime_health.receive(topic, data, now)
        arrivals = self.received_times[topic]
        arrivals.append(now)
        while arrivals and arrivals[0] < now - RATE_WINDOW_SECONDS:
            arrivals.popleft()
        self.received_count[topic] += 1

    def topic_snapshot(self, now):
        topics = {}
        for name, (received, data) in self.samples.items():
            arrivals = self.received_times[name]
            while arrivals and arrivals[0] < now - RATE_WINDOW_SECONDS:
                arrivals.popleft()
            span = arrivals[-1] - arrivals[0] if len(arrivals) > 1 else 0.0
            period = span / (len(arrivals) - 1) if span > 0 else None
            stale_after = self.stale_seconds.get(name, RUNTIME_STALE_SECONDS if name in BOARDS else STALE_SECONDS)
            fresh = now - received < stale_after
            if name == "/diagnostics":
                data = {"status": [
                    {**status, "age_ms": round((now - stamp) * 1000),
                     "fresh": now - stamp < stale_after}
                    for stamp, status in self.diagnostics.values()
                ]}
            topics[name] = dict(
                data=data,
                age_ms=round((now - received) * 1000),
                fresh=fresh,
                stale_after_ms=stale_after * 1000,
                rx_count=self.received_count[name],
                rx_hz=(1.0 / period if period else None) if fresh else 0.0,
                period_ms=period * 1000 if period else None,
            )
        return topics
