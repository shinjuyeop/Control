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

from .qt_runtime import ensure_bundled_qt_first

ensure_bundled_qt_first()

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
    build_set_plant,
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
            self.resize(1360, 860)
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
            refresh_button = QtWidgets.QPushButton("Refresh STATUS")
            refresh_button.clicked.connect(self.node.refresh)
            top.addWidget(refresh_button)
            layout.addLayout(top)

            self.control_groups = []
            self.tabs = QtWidgets.QTabWidget()
            layout.addWidget(self.tabs, 1)
            self._build_signal_flow_tab()
            self._build_production_tab()
            self._build_fault_tab()

            self.result = QtWidgets.QLabel("Command result: --")
            self.result.setTextInteractionFlags(QtCore.Qt.TextSelectableByMouse)
            layout.addWidget(self.result)
            self.timer = QtCore.QTimer(self)
            self.timer.timeout.connect(self._refresh)
            self.timer.start(100)

        @staticmethod
        def _description(text):
            label = QtWidgets.QLabel(text)
            label.setWordWrap(True)
            label.setStyleSheet("color:#57606a;padding:3px")
            return label

        def _build_signal_flow_tab(self):
            page = QtWidgets.QWidget()
            page_layout = QtWidgets.QVBoxLayout(page)
            direction = QtWidgets.QLabel(
                "Signal flow:  STM_TEST injection  →  vehicle controllers  →  "
                "observed commands  →  STM_TEST emulated feedback"
            )
            direction.setStyleSheet(
                "font-weight:bold;background:#ddf4ff;color:#0969da;padding:8px"
            )
            page_layout.addWidget(direction)

            splitter = QtWidgets.QSplitter(QtCore.Qt.Horizontal)
            page_layout.addWidget(splitter, 1)
            splitter.addWidget(self._build_injection_panel())
            splitter.addWidget(self._build_observation_panel())
            splitter.addWidget(self._build_feedback_panel())
            splitter.setSizes([360, 430, 550])
            self.tabs.addTab(page, "Signal Flow")

        def _build_injection_panel(self):
            group = QtWidgets.QGroupBox("1. STM_TEST → Controllers | Injected inputs")
            layout = QtWidgets.QVBoxLayout(group)
            layout.addWidget(self._description(
                "Physical outputs generated by STM_TEST: switches/GPIO and the "
                "accelerator-pedal DAC presented to STM_A/STM_B."
            ))
            grid = QtWidgets.QGridLayout()
            layout.addLayout(grid)
            self.mode = QtWidgets.QComboBox()
            self.mode.addItems(["MANUAL", "AUTO"])
            self.gear = QtWidgets.QComboBox()
            self.gear.addItems(["FORWARD", "REVERSE"])
            self._row_apply(grid, 0, "Mode [GPIO]", self.mode,
                            lambda: build_set_mode(self.mode.currentText()))
            self._row_apply(grid, 1, "Gear [GPIO]", self.gear,
                            lambda: build_set_gear(self.gear.currentText()))

            pedal_editor = QtWidgets.QWidget()
            pedal_layout = QtWidgets.QHBoxLayout(pedal_editor)
            pedal_layout.setContentsMargins(0, 0, 0, 0)
            self.pedal_slider = QtWidgets.QSlider(QtCore.Qt.Horizontal)
            self.pedal_slider.setRange(0, 100)
            self.pedal = QtWidgets.QDoubleSpinBox()
            self.pedal.setRange(0, 100)
            self.pedal.setDecimals(1)
            self.pedal.setSuffix(" %")
            self.pedal_slider.valueChanged.connect(self.pedal.setValue)
            self.pedal.valueChanged.connect(
                lambda value: self.pedal_slider.setValue(round(value)))
            pedal_layout.addWidget(self.pedal_slider, 1)
            pedal_layout.addWidget(self.pedal)
            self._row_apply(grid, 2, "Pedal [DAC]", pedal_editor,
                            lambda: build_set_numeric("PEDAL", self.pedal.value()))

            estop = QtWidgets.QGroupBox("E-stop switch outputs [GPIO]")
            estop_layout = QtWidgets.QHBoxLayout(estop)
            self.estop_buttons = {}
            for location in ("DASH", "SIDE", "RF"):
                button = self._toggle(
                    location,
                    lambda checked, loc=location: build_set_estop(loc, checked),
                    danger=True,
                )
                self.estop_buttons[location] = button
                estop_layout.addWidget(button)
            layout.addWidget(estop)

            presets = QtWidgets.QGroupBox("Quick injection")
            preset_layout = QtWidgets.QGridLayout(presets)
            actions = [
                ("AUTO + Forward", lambda: [
                    self.node.send(build_set_mode("AUTO")),
                    self.node.send(build_set_gear("FORWARD")),
                ]),
                ("Pedal 10%", lambda: self.node.send(build_set_numeric("PEDAL", 10))),
                ("Pedal 30%", lambda: self.node.send(build_set_numeric("PEDAL", 30))),
                ("Dashboard E-stop", lambda: self.node.send(build_set_estop("DASH", True))),
            ]
            for index, (label, callback) in enumerate(actions):
                button = QtWidgets.QPushButton(label)
                button.clicked.connect(callback)
                preset_layout.addWidget(button, index // 2, index % 2)
            layout.addWidget(presets)
            layout.addStretch()
            self.control_groups.append(group)
            return group

        def _build_observation_panel(self):
            group = QtWidgets.QGroupBox("2. Controllers → STM_TEST | Observed commands")
            layout = QtWidgets.QVBoxLayout(group)
            layout.addWidget(self._description(
                "Read-only signals received by STM_TEST. Interface OK only means "
                "the CAN peripheral started; ACTIVE requires recent RX frames."
            ))
            self.observed = {}

            ez = QtWidgets.QGroupBox("STM_A traction / EZKontrol command [CAN1]")
            ez_form = QtWidgets.QFormLayout(ez)
            self.ez_link = QtWidgets.QLabel("--")
            ez_form.addRow("Link", self.ez_link)
            for key, title in (
                ("ez_rx", "Valid RX frames"),
                ("ez_rx_age_ms", "Last RX age"),
                ("ez_running", "RUN command"),
                ("ez_control_mode", "Control mode"),
                ("ez_command_rpm", "RPM field"),
                ("ez_command_current", "Phase-current command"),
                ("ez_target_rpm", "Plant target RPM"),
            ):
                label = QtWidgets.QLabel("--")
                self.observed[key] = label
                ez_form.addRow(title, label)
            layout.addWidget(ez)

            brake = QtWidgets.QGroupBox("STM_B brake command [PWM input]")
            brake_form = QtWidgets.QFormLayout(brake)
            for key, title in (
                ("brake_period_us", "Period"),
                ("brake_us", "Pulse width"),
                ("brake", "Decoded state"),
                ("pwm_lost", "Signal lost"),
            ):
                label = QtWidgets.QLabel("--")
                self.observed[key] = label
                brake_form.addRow(title, label)
            layout.addWidget(brake)

            keya = QtWidgets.QGroupBox("Steering controller / KEYA command [CAN2]")
            keya_form = QtWidgets.QFormLayout(keya)
            self.keya_link = QtWidgets.QLabel("--")
            keya_form.addRow("Link", self.keya_link)
            for key, title in (
                ("keya_rx", "Valid RX frames"),
                ("keya_rx_age_ms", "Last RX age"),
                ("keya_enabled", "Motor-enable command"),
                ("steer_target", "Commanded shaft angle"),
            ):
                label = QtWidgets.QLabel("--")
                self.observed[key] = label
                keya_form.addRow(title, label)
            self.keya_note = self._description(
                "Transceiver absent is valid: NO RX; manual feedback remains usable."
            )
            self.keya_note.setMinimumHeight(38)
            keya_form.addRow(self.keya_note)
            layout.addWidget(keya)
            layout.addStretch()
            return group

        def _build_feedback_panel(self):
            group = QtWidgets.QGroupBox("3. STM_TEST → Controllers | Emulated feedback")
            layout = QtWidgets.QVBoxLayout(group)
            layout.addWidget(self._description(
                "Values STM_TEST returns as the simulated vehicle/motor. Closed-loop "
                "uses the EZ command observed in panel 2; Direct mode uses your override."
            ))

            response = QtWidgets.QGroupBox("Traction response mode")
            response_form = QtWidgets.QFormLayout(response)
            self.response_mode = QtWidgets.QComboBox()
            self.response_mode.addItems([
                "Closed-loop simulation (EZ command → actual RPM)",
                "Direct feedback override",
            ])
            self.response_mode.activated[int].connect(self._response_mode_changed)
            response_form.addRow("Mode", self.response_mode)
            self.motion_source = QtWidgets.QComboBox()
            self.motion_source.addItems(["SPEED", "RPM"])
            self.motion_source.currentTextChanged.connect(self._motion_source_changed)
            self.motion_override = QtWidgets.QDoubleSpinBox()
            self.motion_override.setDecimals(2)
            self.motion_override.setRange(-200.0, 200.0)
            self.motion_override.setSuffix(" km/h")
            self.apply_motion = QtWidgets.QPushButton("Apply direct override")
            self.apply_motion.clicked.connect(lambda: self.node.send(
                build_set_numeric(self.motion_source.currentText(), self.motion_override.value())
            ))
            response_form.addRow("Direct source", self.motion_source)
            response_form.addRow("Direct value", self.motion_override)
            response_form.addRow(self.apply_motion)
            self.actual_rpm = QtWidgets.QLabel("--")
            self.actual_speed = QtWidgets.QLabel("--")
            response_form.addRow("Actual motor RPM [CAN feedback]", self.actual_rpm)
            response_form.addRow("Actual vehicle speed", self.actual_speed)
            layout.addWidget(response)

            motor = QtWidgets.QGroupBox("EZ motor feedback payload [CAN1]")
            motor_grid = QtWidgets.QGridLayout(motor)
            self.feedback_numeric = {}
            specs = [
                ("VOLTAGE", "DC voltage", 0, 300, 1, " V"),
                ("CURRENT", "Motor current", -3200, 3200, 1, " A"),
                ("MOTOR_TEMP", "Motor temperature", -40, 210, 1, " °C"),
            ]
            for row, (field, title, low, high, decimals, suffix) in enumerate(specs):
                box = QtWidgets.QDoubleSpinBox()
                box.setRange(low, high)
                box.setDecimals(decimals)
                box.setSuffix(suffix)
                self.feedback_numeric[field] = box
                self._row_apply(
                    motor_grid, row, title, box,
                    lambda name=field, editor=box: build_set_numeric(name, editor.value()),
                )
            layout.addWidget(motor)

            steering = QtWidgets.QGroupBox("KEYA steering feedback [CAN2]")
            steering_form = QtWidgets.QFormLayout(steering)
            self.steer_actual = QtWidgets.QLabel("--")
            self.steer_target = QtWidgets.QLabel("--")
            steering_form.addRow("Simulated shaft angle", self.steer_actual)
            steering_form.addRow("Received target angle", self.steer_target)
            self.steer_override = QtWidgets.QDoubleSpinBox()
            self.steer_override.setRange(-1800, 1800)
            self.steer_override.setDecimals(2)
            self.steer_override.setSuffix(" deg")
            steer_editor = QtWidgets.QWidget()
            steer_layout = QtWidgets.QHBoxLayout(steer_editor)
            steer_layout.setContentsMargins(0, 0, 0, 0)
            steer_layout.addWidget(self.steer_override, 1)
            apply_steer = QtWidgets.QPushButton("Set feedback position")
            apply_steer.clicked.connect(lambda: self.node.send(
                build_set_numeric("STEER_POS", self.steer_override.value())))
            steer_layout.addWidget(apply_steer)
            steering_form.addRow("Manual initial/feedback position", steer_editor)
            steering_form.addRow(self._description(
                "KEYA simulation is always command-following when CAN2 receives an "
                "enable and position command; it does not use the EZ Plant switch."
            ))
            layout.addWidget(steering)
            layout.addStretch()
            self.control_groups.append(group)
            return group

        def _build_production_tab(self):
            page = QtWidgets.QWidget()
            layout = QtWidgets.QVBoxLayout(page)
            warning = QtWidgets.QLabel(
                "End-to-end test source: /vehicle/command → STM_A/lateral controller → "
                "physical outputs → STM_TEST. These are not direct STM_TEST injections."
            )
            warning.setWordWrap(True)
            warning.setStyleSheet(
                "font-weight:bold;background:#fff8c5;color:#9a6700;padding:10px"
            )
            layout.addWidget(warning)
            production = QtWidgets.QGroupBox("Production Command Generator (/vehicle/command)")
            prod = QtWidgets.QFormLayout(production)
            self.prod_enabled = QtWidgets.QCheckBox("Publish commands")
            self.prod_speed = QtWidgets.QDoubleSpinBox()
            self.prod_speed.setRange(-50, 50)
            self.prod_speed.setDecimals(2)
            self.prod_speed.setSuffix(" m/s")
            self.prod_steer = QtWidgets.QDoubleSpinBox()
            self.prod_steer.setRange(-1.0, 1.0)
            self.prod_steer.setDecimals(3)
            self.prod_steer.setSuffix(" rad")
            self.prod_brake = QtWidgets.QCheckBox("Brake engage")
            apply_prod = QtWidgets.QPushButton("Apply production command")
            apply_prod.clicked.connect(lambda: self.node.production_command(
                self.prod_enabled.isChecked(), self.prod_speed.value(),
                self.prod_steer.value(), self.prod_brake.isChecked()))
            prod.addRow(self.prod_enabled)
            prod.addRow("Speed target", self.prod_speed)
            prod.addRow("Steering target", self.prod_steer)
            prod.addRow(self.prod_brake)
            prod.addRow(apply_prod)
            layout.addWidget(production)
            layout.addWidget(self._description(
                "Traction verification requires STM_A and the EZ CAN1 transceiver. "
                "Steering verification additionally requires the lateral controller "
                "and KEYA CAN2 transceiver; until then its NO RX state is expected."
            ))
            layout.addStretch()
            self.control_groups.append(production)
            self.tabs.addTab(page, "End-to-End Command Source")

        def _build_fault_tab(self):
            page = QtWidgets.QWidget()
            layout = QtWidgets.QVBoxLayout(page)
            layout.addWidget(self._description(
                "Advanced controls for CAN interface shutdown, transmitted-frame loss, "
                "and controller fault-bit injection."
            ))
            fault = QtWidgets.QGroupBox("Fault injection / interface control")
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
                    button.blockSignals(True)
                    button.setChecked(True)
                    button.blockSignals(False)
                    self._render_toggle(button, True)
                self.fault_buttons[label] = button
                fault_grid.addWidget(button, index // 2, index % 2)
            clear = QtWidgets.QPushButton("Clear all injected faults")
            clear.clicked.connect(self._clear_faults)
            fault_grid.addWidget(clear, 3, 0, 1, 2)
            layout.addWidget(fault)
            layout.addStretch()
            self.control_groups.append(fault)
            self.tabs.addTab(page, "Faults & Interfaces")

        def _row_apply(self, layout, row, label, editor, builder):
            button = QtWidgets.QPushButton("Apply")
            button.clicked.connect(lambda: self.node.send(builder()))
            layout.addWidget(QtWidgets.QLabel(label), row, 0); layout.addWidget(editor, row, 1); layout.addWidget(button, row, 2)

        def _response_mode_changed(self, index):
            closed_loop = index == 0
            self.node.send(build_set_plant(closed_loop))
            self._set_direct_controls(not closed_loop)

        def _set_direct_controls(self, enabled):
            self.motion_source.setEnabled(enabled)
            self.motion_override.setEnabled(enabled)
            self.apply_motion.setEnabled(enabled)

        def _motion_source_changed(self, source):
            value = self.motion_override.value()
            if source == "RPM":
                self.motion_override.setRange(-32000.0, 32000.0)
                self.motion_override.setDecimals(0)
                self.motion_override.setSuffix(" rpm")
            else:
                self.motion_override.setRange(-200.0, 200.0)
                self.motion_override.setDecimals(2)
                self.motion_override.setSuffix(" km/h")
            self.motion_override.setValue(value)

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

        @staticmethod
        def _set_badge(label, text, color):
            label.setText(text)
            label.setStyleSheet(
                f"font-weight:bold;color:{color};border:1px solid {color};"
                "padding:2px 5px;border-radius:3px"
            )

        def _render_can_link(self, label, status, prefix):
            enabled = bool(status.get(f"{prefix}_can"))
            interface_ok = bool(status.get(f"{prefix}_ok"))
            rx_count = int(status.get(f"{prefix}_rx", 0))
            protocol = int(status.get("protocol", 1))
            rx_age = int(status.get(f"{prefix}_rx_age_ms", 4294967295))
            if not enabled:
                self._set_badge(label, "DISABLED", "#57606a")
            elif not interface_ok:
                self._set_badge(label, "INTERFACE FAULT", "#cf222e")
            elif rx_count == 0:
                self._set_badge(label, "NO RX / NOT CONNECTED", "#9a6700")
            elif protocol < 2 or rx_age >= 4294967295:
                self._set_badge(label, f"RX SEEN ({rx_count})", "#0969da")
            elif rx_age > 1000:
                self._set_badge(label, f"STALE RX ({rx_age} ms)", "#cf222e")
            else:
                self._set_badge(label, f"ACTIVE ({rx_age} ms)", "#1a7f37")

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
                self._render_can_link(self.ez_link, status, "ez")
                self._render_can_link(self.keya_link, status, "keya")
                self.ez_health.setText("EZ RX: " + self.ez_link.text())
                self.keya_health.setText("KEYA RX: " + self.keya_link.text())
                display = dict(status)
                display["ez_control_mode"] = (
                    "SPEED" if status.get("ez_speed_control") else "TORQUE"
                )
                display["ez_running"] = "RUN" if status.get("ez_running") else "STOP"
                display["keya_enabled"] = "ENABLED" if status.get("keya_enabled") else "DISABLED"
                for key in ("ez_rx_age_ms", "keya_rx_age_ms"):
                    age_value = int(status.get(key, 4294967295))
                    display[key] = "never / unavailable" if age_value >= 4294967295 else f"{age_value} ms"
                display["ez_command_rpm"] = f"{float(status.get('ez_command_rpm', 0.0)):.0f} rpm"
                display["ez_command_current"] = f"{float(status.get('ez_command_current', 0.0)):.1f} A"
                display["ez_target_rpm"] = f"{float(status.get('ez_target_rpm', 0.0)):.0f} rpm"
                display["brake_period_us"] = f"{int(status.get('brake_period_us', 0))} µs"
                display["brake_us"] = f"{int(status.get('brake_us', 0))} µs"
                display["pwm_lost"] = "LOST" if status.get("pwm_lost") else "RECEIVING"
                display["steer_target"] = f"{float(status.get('steer_target', 0.0)):.2f} deg"
                for key, label in self.observed.items():
                    label.setText(str(display.get(key, "--")))
                self.actual_rpm.setText(f"{float(status.get('rpm', 0.0)):.0f} rpm")
                self.actual_speed.setText(f"{float(status.get('speed', 0.0)):.2f} km/h")
                self.steer_actual.setText(f"{float(status.get('steer', 0.0)):.2f} deg")
                self.steer_target.setText(f"{float(status.get('steer_target', 0.0)):.2f} deg")
                mode_index = 0 if status.get("plant") else 1
                if self.response_mode.currentIndex() != mode_index:
                    self.response_mode.blockSignals(True)
                    self.response_mode.setCurrentIndex(mode_index)
                    self.response_mode.blockSignals(False)
                self._set_direct_controls(mode_index == 1 and connected)
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
    ensure_bundled_qt_first()
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
