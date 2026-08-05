"""PyQt5 HIL operator GUI. Serial ownership remains in stm_test_bridge."""

from __future__ import annotations

import json
import signal
import sys
import threading

from diagnostic_msgs.msg import DiagnosticArray
import rclpy
from rclpy.executors import ExternalShutdownException, SingleThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool, String
from std_srvs.srv import Trigger

try:
    from PyQt5 import QtCore, QtWidgets
except ImportError as gui_import_error:
    QtCore = None
    QtWidgets = None
    GUI_IMPORT_ERROR = gui_import_error
else:
    GUI_IMPORT_ERROR = None

from .protocol import (
    build_fault,
    build_set_can,
    build_set_estop,
    build_set_gear,
    build_set_mode,
    build_set_numeric,
)
from .remote_parameters import RemoteParameterClient


class HilGuiRosNode(Node):
    def __init__(self) -> None:
        super().__init__("hil_gui")
        self._lock = threading.Lock()
        self.connected = False
        self.status = {}
        self.desired = {}
        self.serial_port = "--"
        self.last_result = ""
        latched_qos = QoSProfile(depth=1)
        latched_qos.reliability = ReliabilityPolicy.RELIABLE
        latched_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.command_pub = self.create_publisher(String, "/hil/command", 20)
        self.create_subscription(Bool, "/hil/connected", self._connected, latched_qos)
        self.create_subscription(String, "/hil/status", self._status, 10)
        self.create_subscription(String, "/hil/desired_state", self._desired, latched_qos)
        self.create_subscription(String, "/hil/command_result", self._result, 20)
        self.create_subscription(DiagnosticArray, "/diagnostics", self._diagnostics, 10)
        self.safe_client = self.create_client(Trigger, "/hil/reset_safe_state")
        self.refresh_client = self.create_client(Trigger, "/hil/refresh_status")
        self.generator_client = RemoteParameterClient(self, "hil_command_generator")

    def _connected(self, message):
        with self._lock:
            self.connected = bool(message.data)

    def _status(self, message):
        try:
            status = json.loads(message.data)
        except json.JSONDecodeError:
            return
        with self._lock:
            self.status = status

    def _desired(self, message):
        try:
            desired = json.loads(message.data)
        except json.JSONDecodeError:
            return
        with self._lock:
            self.desired = desired

    def _diagnostics(self, message):
        for status in message.status:
            if status.name != "STM_TEST serial bridge":
                continue
            values = {item.key: item.value for item in status.values}
            with self._lock:
                self.serial_port = values.get("serial_port", status.hardware_id or "--")

    def _result(self, message):
        with self._lock:
            self.last_result = message.data

    def snapshot(self):
        with self._lock:
            return (
                self.connected,
                dict(self.status),
                dict(self.desired),
                self.serial_port,
                self.last_result,
            )

    def send(self, command: str):
        message = String()
        message.data = command
        self.command_pub.publish(message)

    def safe_state(self):
        if self.safe_client.service_is_ready():
            self.safe_client.call_async(Trigger.Request())

    def refresh(self):
        if self.refresh_client.service_is_ready():
            self.refresh_client.call_async(Trigger.Request())

    def production_command(self, enabled, speed, steering, brake):
        self.generator_client.set_parameters([
            Parameter("enabled", value=bool(enabled)),
            Parameter("speed_target_mps", value=float(speed)),
            Parameter("steering_target_rad", value=float(steering)),
            Parameter("brake_engage", value=bool(brake)),
        ])


if QtWidgets is not None:
    class HilWindow(QtWidgets.QMainWindow):
        def __init__(self, ros_node):
            super().__init__()
            self.node = ros_node
            self.setWindowTitle("KAI EV26 HIL Operator")
            self.resize(1040, 760)
            root = QtWidgets.QWidget()
            self.setCentralWidget(root)
            layout = QtWidgets.QVBoxLayout(root)

            top = QtWidgets.QHBoxLayout()
            self.connection = QtWidgets.QLabel("DISCONNECTED")
            self.connection.setStyleSheet("font-weight:bold;color:#d1242f")
            self.serial_port = QtWidgets.QLabel("Serial: --")
            self.age = QtWidgets.QLabel("STATUS age: --")
            self.ez_health = QtWidgets.QLabel("EZ: --")
            self.keya_health = QtWidgets.QLabel("KEYA: --")
            for widget in (self.connection, self.serial_port, self.age, self.ez_health, self.keya_health):
                top.addWidget(widget)
            top.addStretch()
            self.safe_button = QtWidgets.QPushButton("SAFE STATE")
            self.safe_button.setStyleSheet("font-weight:bold;background:#2da44e;color:white;padding:8px")
            self.safe_button.clicked.connect(self.node.safe_state)
            top.addWidget(self.safe_button)
            layout.addLayout(top)

            columns = QtWidgets.QHBoxLayout()
            layout.addLayout(columns, 1)
            left = QtWidgets.QVBoxLayout(); right = QtWidgets.QVBoxLayout()
            columns.addLayout(left); columns.addLayout(right)
            self.control_groups = []

            vehicle = QtWidgets.QGroupBox("STM_TEST Environment Inputs")
            grid = QtWidgets.QGridLayout(vehicle)
            self.mode = QtWidgets.QComboBox(); self.mode.addItems(["MANUAL", "AUTO"])
            self.gear = QtWidgets.QComboBox(); self.gear.addItems(["FORWARD", "REVERSE"])
            self._row_apply(grid, 0, "Mode", self.mode, lambda: build_set_mode(self.mode.currentText()))
            self._row_apply(grid, 1, "Gear", self.gear, lambda: build_set_gear(self.gear.currentText()))
            self.numeric = {}
            specs = [
                ("PEDAL", 0, 100, 1), ("SPEED", -200, 200, 2), ("RPM", -32000, 32000, 0),
                ("VOLTAGE", 0, 300, 1), ("CURRENT", -3200, 3200, 1),
                ("MOTOR_TEMP", -40, 210, 1), ("STEER_POS", -1800, 1800, 2),
            ]
            for row, (name, low, high, decimals) in enumerate(specs, start=2):
                box = QtWidgets.QDoubleSpinBox(); box.setRange(low, high); box.setDecimals(decimals)
                self.numeric[name] = box
                editor = box
                if name == "PEDAL":
                    editor = QtWidgets.QWidget()
                    pedal_layout = QtWidgets.QHBoxLayout(editor)
                    pedal_layout.setContentsMargins(0, 0, 0, 0)
                    self.pedal_slider = QtWidgets.QSlider(QtCore.Qt.Horizontal)
                    self.pedal_slider.setRange(0, 100)
                    self.pedal_slider.valueChanged.connect(box.setValue)
                    box.valueChanged.connect(lambda value: self.pedal_slider.setValue(round(value)))
                    pedal_layout.addWidget(self.pedal_slider, 1)
                    pedal_layout.addWidget(box)
                self._row_apply(grid, row, name, editor, lambda n=name, b=box: build_set_numeric(n, b.value()))
            left.addWidget(vehicle); self.control_groups.append(vehicle)

            estop = QtWidgets.QGroupBox("E-STOP")
            estop_layout = QtWidgets.QHBoxLayout(estop)
            self.estop_buttons = {}
            for location in ("DASH", "SIDE", "RF"):
                button = self._toggle(location, lambda checked, loc=location: build_set_estop(loc, checked), danger=True)
                self.estop_buttons[location] = button; estop_layout.addWidget(button)
            left.addWidget(estop); self.control_groups.append(estop)

            fault = QtWidgets.QGroupBox("Fault Injection / CAN")
            fault_grid = QtWidgets.QGridLayout(fault)
            self.fault_buttons = {}
            definitions = [
                ("EZ TX DROP", lambda value: build_fault("EZ", "DROP_TX", value), True),
                ("KEYA TX DROP", lambda value: build_fault("KEYA", "DROP_TX", value), True),
                ("EZ FAULT", lambda value: build_fault("EZ", None, value), True),
                ("KEYA FAULT", lambda value: build_fault("KEYA", None, value), True),
                ("EZ CAN", lambda value: build_set_can("EZ", value), False),
                ("KEYA CAN", lambda value: build_set_can("KEYA", value), False),
            ]
            for index, (label, builder, danger) in enumerate(definitions):
                button = self._toggle(label, builder, danger=danger)
                if not danger:
                    button.blockSignals(True); button.setChecked(True)
                    button.setText(label + ": ON"); button.blockSignals(False)
                self.fault_buttons[label] = button
                fault_grid.addWidget(button, index // 2, index % 2)
            left.addWidget(fault); self.control_groups.append(fault)

            feedback = QtWidgets.QGroupBox("STM_TEST Feedback")
            feedback_layout = QtWidgets.QFormLayout(feedback)
            keys = ["mode", "gear", "pedal", "brake_period_us", "brake_us", "brake", "pwm_lost",
                    "steer", "steer_target", "ez_rx", "ez_tx", "keya_rx", "keya_tx"]
            self.feedback = {}
            for key in keys:
                label = QtWidgets.QLabel("--"); self.feedback[key] = label
                feedback_layout.addRow(key, label)
            right.addWidget(feedback)

            production = QtWidgets.QGroupBox("Production Command Generator (/vehicle/command)")
            prod = QtWidgets.QFormLayout(production)
            self.prod_enabled = QtWidgets.QCheckBox("Enabled")
            self.prod_speed = QtWidgets.QDoubleSpinBox(); self.prod_speed.setRange(-50, 50); self.prod_speed.setDecimals(2)
            self.prod_steer = QtWidgets.QDoubleSpinBox(); self.prod_steer.setRange(-1.0, 1.0); self.prod_steer.setDecimals(3)
            self.prod_brake = QtWidgets.QCheckBox("Brake engage")
            apply_prod = QtWidgets.QPushButton("Apply production command")
            apply_prod.clicked.connect(lambda: self.node.production_command(
                self.prod_enabled.isChecked(), self.prod_speed.value(),
                self.prod_steer.value(), self.prod_brake.isChecked()))
            prod.addRow(self.prod_enabled); prod.addRow("speed_target_mps", self.prod_speed)
            prod.addRow("steering_target_rad", self.prod_steer); prod.addRow(self.prod_brake); prod.addRow(apply_prod)
            right.addWidget(production)

            quick = QtWidgets.QGroupBox("Quick Actions")
            quick_layout = QtWidgets.QGridLayout(quick)
            actions = [
                ("AUTO + Forward", lambda: [self.node.send(build_set_mode("AUTO")), self.node.send(build_set_gear("FORWARD"))]),
                ("Pedal 10%", lambda: self.node.send(build_set_numeric("PEDAL", 10))),
                ("Pedal 30%", lambda: self.node.send(build_set_numeric("PEDAL", 30))),
                ("Dashboard E-stop", lambda: self.node.send(build_set_estop("DASH", True))),
                ("Clear All Faults", self._clear_faults),
                ("Refresh STATUS", self.node.refresh),
            ]
            for index, (label, callback) in enumerate(actions):
                button = QtWidgets.QPushButton(label); button.clicked.connect(callback)
                quick_layout.addWidget(button, index // 2, index % 2)
            right.addWidget(quick); self.control_groups.append(quick)
            self.result = QtWidgets.QLabel("Command result: --")
            layout.addWidget(self.result)
            self.timer = QtCore.QTimer(self); self.timer.timeout.connect(self._refresh); self.timer.start(100)

        def _row_apply(self, layout, row, label, editor, builder):
            button = QtWidgets.QPushButton("Apply")
            button.clicked.connect(lambda: self.node.send(builder()))
            layout.addWidget(QtWidgets.QLabel(label), row, 0); layout.addWidget(editor, row, 1); layout.addWidget(button, row, 2)

        def _toggle(self, label, builder, *, danger):
            button = QtWidgets.QPushButton(label + ": OFF")
            button.setCheckable(True)
            button.setProperty("hil_label", label)
            button.setProperty("hil_danger", danger)
            def changed(value):
                self._render_toggle(button, value)
                self.node.send(builder(value))
            button.toggled.connect(changed)
            return button

        @staticmethod
        def _render_toggle(button, value):
            label = str(button.property("hil_label"))
            danger = bool(button.property("hil_danger"))
            button.setText(label + (": ON" if value else ": OFF"))
            color = "#d1242f" if danger and value else ("#2da44e" if value else "")
            button.setStyleSheet(
                f"font-weight:bold;background:{color};color:white" if color else ""
            )

        def _clear_faults(self):
            for command in (build_fault("EZ", "DROP_TX", False), build_fault("KEYA", "DROP_TX", False),
                            build_fault("EZ", None, False), build_fault("KEYA", None, False)):
                self.node.send(command)

        def _refresh(self):
            connected, status, desired, serial_port, result = self.node.snapshot()
            self.connection.setText("CONNECTED" if connected else "DISCONNECTED")
            self.connection.setStyleSheet("font-weight:bold;color:" + ("#2da44e" if connected else "#d1242f"))
            self.serial_port.setText("Serial: " + serial_port)
            for group in self.control_groups: group.setEnabled(connected)
            self.result.setText("Command result: " + (result or "--"))
            if status:
                age = max(0.0, __import__("time").monotonic() - float(status.get("received_monotonic", 0.0)))
                self.age.setText(f"STATUS age: {age:.2f}s")
                self.ez_health.setText("EZ: " + ("OK" if status.get("ez_ok") else "FAULT"))
                self.keya_health.setText("KEYA: " + ("OK" if status.get("keya_ok") else "FAULT"))
                for key, label in self.feedback.items(): label.setText(str(status.get(key, "--")))
            toggle_values = {
                self.estop_buttons["DASH"]: status.get("estop_dash"),
                self.estop_buttons["SIDE"]: status.get("estop_side"),
                self.estop_buttons["RF"]: status.get("estop_rf"),
                self.fault_buttons["EZ TX DROP"]: desired.get("ez_drop_tx"),
                self.fault_buttons["KEYA TX DROP"]: desired.get("keya_drop_tx"),
                self.fault_buttons["EZ FAULT"]: status.get("ez_fault"),
                self.fault_buttons["KEYA FAULT"]: status.get("keya_fault"),
                self.fault_buttons["EZ CAN"]: status.get("ez_can"),
                self.fault_buttons["KEYA CAN"]: status.get("keya_can"),
            }
            for button, value in toggle_values.items():
                if value is None or button.isChecked() == bool(value):
                    continue
                button.blockSignals(True)
                button.setChecked(bool(value))
                button.blockSignals(False)
                self._render_toggle(button, bool(value))


def main(args=None) -> None:
    if GUI_IMPORT_ERROR is not None:
        raise RuntimeError("PyQt5 is required for hil_gui") from GUI_IMPORT_ERROR
    rclpy.init(args=args)
    node = HilGuiRosNode()
    executor = SingleThreadedExecutor(); executor.add_node(node)

    def spin_ros():
        try:
            executor.spin()
        except ExternalShutdownException:
            pass

    thread = threading.Thread(target=spin_ros, daemon=True); thread.start()
    application = QtWidgets.QApplication(sys.argv)
    window = HilWindow(node); window.show()
    shutdown_timer = QtCore.QTimer()
    shutdown_timer.timeout.connect(lambda: None if rclpy.ok() else application.quit())
    shutdown_timer.start(100)
    signal.signal(signal.SIGINT, lambda _signum, _frame: application.quit())
    try:
        application.exec_()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown(timeout_sec=1.0)
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()
        thread.join(timeout=1.0)
