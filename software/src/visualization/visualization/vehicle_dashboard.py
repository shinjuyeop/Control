"""Read-only Qt dashboard for the canonical vehicle ROS 2 interface."""

from collections import deque
import sys
import threading
import time

from kaiev26_msgs.msg import (
    ActuatorCommand,
    BrakeActuatorStatus,
    DriveActuatorStatus,
    SteeringActuatorStatus,
)
import rclpy
from rclpy.executors import ExternalShutdownException, SingleThreadedExecutor
from rclpy.node import Node

from visualization.ros_state_cache import (
    bool_text,
    BRAKE,
    brake_reason_text,
    COMMAND,
    DRIVE,
    drive_gear_text,
    drive_mode_text,
    format_if_valid,
    NA,
    NO_DATA,
    OK,
    physical_estop_active,
    rad_to_deg,
    RosStateCache,
    STALE,
    STEERING,
    steering_disable_reason_text,
    steering_state_text,
    valid_bool_text,
)

try:
    from PyQt5 import QtCore, QtGui, QtWidgets
    import pyqtgraph as pg
except ImportError as gui_import_error:
    QtCore = None
    QtGui = None
    QtWidgets = None
    pg = None
    GUI_IMPORT_ERROR = gui_import_error
else:
    GUI_IMPORT_ERROR = None


class VehicleStatusSubscriber(Node):
    """Receive vehicle topics and write only to the thread-safe cache."""

    def __init__(self, cache):
        super().__init__('vehicle_dashboard')
        self.cache = cache

        self.declare_parameter('command_stale_ms', 300.0)
        self.declare_parameter('drive_status_stale_ms', 300.0)
        self.declare_parameter('brake_status_stale_ms', 300.0)
        self.declare_parameter('steering_status_stale_ms', 500.0)
        self.declare_parameter('history_sec', 10.0)
        self.declare_parameter('ui_update_hz', 10.0)

        self.stale_ms = {
            COMMAND: float(self.get_parameter('command_stale_ms').value),
            DRIVE: float(self.get_parameter('drive_status_stale_ms').value),
            BRAKE: float(
                self.get_parameter('brake_status_stale_ms').value
            ),
            STEERING: float(
                self.get_parameter('steering_status_stale_ms').value
            ),
        }
        self.history_sec = max(
            1.0,
            float(self.get_parameter('history_sec').value),
        )
        self.ui_update_hz = max(
            1.0,
            float(self.get_parameter('ui_update_hz').value),
        )

        self.create_subscription(
            ActuatorCommand,
            '/vehicle/command',
            lambda msg: self.cache.update(COMMAND, msg),
            10,
        )
        self.create_subscription(
            DriveActuatorStatus,
            '/drive/status',
            lambda msg: self.cache.update(DRIVE, msg),
            10,
        )
        self.create_subscription(
            BrakeActuatorStatus,
            '/brake/status',
            lambda msg: self.cache.update(BRAKE, msg),
            10,
        )
        self.create_subscription(
            SteeringActuatorStatus,
            '/steering/status',
            lambda msg: self.cache.update(STEERING, msg),
            10,
        )


if QtWidgets is not None and pg is not None:

    class VehicleDashboard(QtWidgets.QMainWindow):
        """Developer-oriented, read-only status and trend display."""

        STATUS_COLORS = {
            NO_DATA: '#8b949e',
            OK: '#3fb950',
            STALE: '#d29922',
        }

        def __init__(self, node, cache):
            super().__init__()
            self.node = node
            self.cache = cache
            self.labels = {}
            self.start_monotonic = time.monotonic()
            self.speed_history = deque()
            self.steering_history = deque()

            self.setWindowTitle('KAI Vehicle Debug Dashboard (Read Only)')
            self.resize(1500, 920)
            self._build_ui()

            interval_ms = max(20, int(1000.0 / self.node.ui_update_hz))
            self.timer = QtCore.QTimer(self)
            self.timer.timeout.connect(self.refresh)
            self.timer.start(interval_ms)
            self.refresh()

        def _build_ui(self):
            central = QtWidgets.QWidget()
            outer = QtWidgets.QVBoxLayout(central)

            title = QtWidgets.QLabel(
                'Vehicle Debug Dashboard  ·  READ ONLY'
            )
            title_font = QtGui.QFont()
            title_font.setPointSize(16)
            title_font.setBold(True)
            title.setFont(title_font)
            title.setStyleSheet('color: #58a6ff; padding: 6px;')
            outer.addWidget(title)

            splitter = QtWidgets.QSplitter(QtCore.Qt.Horizontal)
            outer.addWidget(splitter, 1)

            scroll = QtWidgets.QScrollArea()
            scroll.setWidgetResizable(True)
            details = QtWidgets.QWidget()
            detail_layout = QtWidgets.QVBoxLayout(details)
            detail_layout.addWidget(
                self._make_group(
                    'System',
                    [
                        ('MODE', 'mode'),
                        ('GEAR', 'gear'),
                        ('/vehicle/command', 'command_rx'),
                        ('/drive/status', 'drive_rx'),
                        ('/brake/status', 'brake_rx'),
                        ('/steering/status', 'steering_rx'),
                        ('Brake commanded', 'brake_commanded'),
                        ('Brake actual feedback', 'brake_actual'),
                        ('Physical E-stop', 'physical_estop'),
                        ('Brake reasons', 'brake_reasons'),
                        ('Steering state', 'system_steering_state'),
                        (
                            'Steering disable reason',
                            'system_steering_reason',
                        ),
                    ],
                )
            )
            detail_layout.addWidget(
                self._make_group(
                    'Longitudinal',
                    [
                        ('Target speed', 'target_speed'),
                        ('Actual speed', 'actual_speed'),
                        ('Speed error', 'speed_error'),
                        ('Motor RPM', 'motor_rpm'),
                        ('Tractive commanded', 'tractive_commanded'),
                        ('Tractive feedback', 'tractive_feedback'),
                        ('Bus voltage', 'drive_bus_voltage'),
                        ('Bus current', 'drive_bus_current'),
                        ('Phase current', 'phase_current'),
                        (
                            'Controller temperature',
                            'controller_temperature',
                        ),
                        ('Motor temperature', 'motor_temperature'),
                        ('Command received', 'command_received'),
                        ('Command timeout', 'command_timeout'),
                        ('Command age', 'drive_command_age'),
                        ('Part I valid', 'part1_valid'),
                        ('Part I age', 'part1_age'),
                        ('Part II valid', 'part2_valid'),
                        ('Part II age', 'part2_age'),
                        ('Motor fault bits', 'motor_fault_bits'),
                    ],
                )
            )
            detail_layout.addWidget(
                self._make_group(
                    'Lateral',
                    [
                        ('Target angle', 'target_steering'),
                        ('Commanded angle', 'commanded_steering'),
                        ('Corrected angle', 'corrected_steering'),
                        ('Actual angle', 'actual_steering'),
                        ('Tracking error', 'tracking_error'),
                        ('Steering rate', 'steering_rate'),
                        ('State', 'steering_state'),
                        ('Disable reason', 'steering_reason'),
                        ('Enabled commanded', 'enabled_commanded'),
                        ('Enabled feedback', 'enabled_feedback'),
                        (
                            'Enabled feedback valid',
                            'enabled_feedback_valid',
                        ),
                        ('CAN timeout', 'can_timeout'),
                        ('Mode received', 'mode_received'),
                        ('Mode timeout', 'mode_timeout'),
                        (
                            'Steer command received',
                            'steer_command_received',
                        ),
                        (
                            'Steer command timeout',
                            'steer_command_timeout',
                        ),
                        ('Feedback timeout', 'feedback_timeout'),
                        ('Communication OK', 'communication_ok'),
                        (
                            'Pending command valid',
                            'pending_command_valid',
                        ),
                        (
                            'Movement target valid',
                            'movement_target_valid',
                        ),
                        ('Feedback valid', 'feedback_valid'),
                        ('Feedback age', 'feedback_age'),
                        ('Mode age', 'mode_age'),
                        ('Steer command age', 'steer_command_age'),
                        ('Motor position/count', 'motor_position'),
                        ('Motor angle', 'motor_angle'),
                        ('Motor speed', 'steering_motor_speed'),
                        ('Motor current', 'steering_motor_current'),
                        ('Bus voltage', 'steering_bus_voltage'),
                        ('Driver temperature', 'driver_temperature'),
                        ('Steering fault bits', 'steering_fault_bits'),
                    ],
                )
            )
            detail_layout.addStretch(1)
            scroll.setWidget(details)
            splitter.addWidget(scroll)

            plot_panel = QtWidgets.QWidget()
            plot_layout = QtWidgets.QVBoxLayout(plot_panel)
            self.speed_plot = pg.PlotWidget(title='Speed — recent history')
            self.speed_plot.setLabel('left', 'Speed', units='m/s')
            self.speed_plot.setLabel('bottom', 'Time', units='s')
            self.speed_plot.addLegend()
            self.speed_target_curve = self.speed_plot.plot(
                pen=pg.mkPen('#58a6ff', width=2),
                name='target',
            )
            self.speed_actual_curve = self.speed_plot.plot(
                pen=pg.mkPen('#3fb950', width=2),
                name='actual',
            )
            plot_layout.addWidget(self.speed_plot)

            self.steering_plot = pg.PlotWidget(
                title='Steering — recent history'
            )
            self.steering_plot.setLabel('left', 'Wheel angle', units='deg')
            self.steering_plot.setLabel('bottom', 'Time', units='s')
            self.steering_plot.addLegend()
            self.steering_target_curve = self.steering_plot.plot(
                pen=pg.mkPen('#58a6ff', width=2),
                name='target',
            )
            self.steering_actual_curve = self.steering_plot.plot(
                pen=pg.mkPen('#f0883e', width=2),
                name='actual',
            )
            plot_layout.addWidget(self.steering_plot)
            splitter.addWidget(plot_panel)
            splitter.setSizes([650, 850])

            self.setCentralWidget(central)
            self.setStyleSheet(
                'QMainWindow, QWidget { background: #0d1117; color: #c9d1d9; }'
                'QGroupBox { border: 1px solid #30363d; border-radius: 5px;'
                ' margin-top: 9px; font-weight: bold; }'
                'QGroupBox::title { subcontrol-origin: margin; left: 10px;'
                ' padding: 0 4px; color: #58a6ff; }'
                'QScrollArea { border: none; }'
            )

        def _make_group(self, title, rows):
            group = QtWidgets.QGroupBox(title)
            grid = QtWidgets.QGridLayout(group)
            for row, (caption, key) in enumerate(rows):
                name_label = QtWidgets.QLabel(caption)
                value_label = QtWidgets.QLabel(NO_DATA)
                value_label.setTextInteractionFlags(
                    QtCore.Qt.TextSelectableByMouse
                )
                value_label.setAlignment(
                    QtCore.Qt.AlignRight | QtCore.Qt.AlignVCenter
                )
                grid.addWidget(name_label, row, 0)
                grid.addWidget(value_label, row, 1)
                self.labels[key] = value_label
            grid.setColumnStretch(1, 1)
            return group

        def _set(self, key, text, state=None):
            label = self.labels[key]
            label.setText(str(text))
            color = self.STATUS_COLORS.get(state, '#c9d1d9')
            label.setStyleSheet(f'color: {color};')

        @staticmethod
        def _receive_text(topic_snapshot):
            if topic_snapshot.state == NO_DATA:
                return NO_DATA
            return (
                f'{topic_snapshot.state} '
                f'({topic_snapshot.age_ms:.0f} ms local)'
            )

        def _refresh_system(self, snapshot):
            drive = snapshot[DRIVE].message
            brake = snapshot[BRAKE].message
            steering = snapshot[STEERING].message

            self._set(
                'mode',
                drive_mode_text(drive.mode) if drive else NO_DATA,
                snapshot[DRIVE].state,
            )
            self._set(
                'gear',
                drive_gear_text(drive.gear) if drive else NO_DATA,
                snapshot[DRIVE].state,
            )
            for key, label_key in (
                (COMMAND, 'command_rx'),
                (DRIVE, 'drive_rx'),
                (BRAKE, 'brake_rx'),
                (STEERING, 'steering_rx'),
            ):
                self._set(
                    label_key,
                    self._receive_text(snapshot[key]),
                    snapshot[key].state,
                )

            self._set(
                'brake_commanded',
                bool_text(brake.brake_commanded) if brake else NO_DATA,
            )
            self._set(
                'brake_actual',
                (
                    'SUPPORTED'
                    if brake and brake.actual_valid
                    else (NA if brake else NO_DATA)
                ),
            )
            estop = physical_estop_active(brake)
            self._set(
                'physical_estop',
                bool_text(estop) if estop is not None else NO_DATA,
            )
            self._set(
                'brake_reasons',
                brake_reason_text(brake.stop_reason_bits)
                if brake
                else NO_DATA,
            )
            self._set(
                'system_steering_state',
                steering_state_text(steering.state)
                if steering
                else NO_DATA,
            )
            self._set(
                'system_steering_reason',
                steering_disable_reason_text(steering.disable_reason)
                if steering
                else NO_DATA,
            )

        def _refresh_longitudinal(self, snapshot):
            command = snapshot[COMMAND].message
            drive = snapshot[DRIVE].message
            target = command.speed_target_mps if command else None
            actual = drive.speed_mps if drive else None

            self._set(
                'target_speed',
                f'{target:.2f} m/s' if target is not None else NO_DATA,
            )
            self._set(
                'actual_speed',
                f'{actual:.2f} m/s' if actual is not None else NO_DATA,
            )
            self._set(
                'speed_error',
                f'{target - actual:.2f} m/s'
                if target is not None and actual is not None
                else NO_DATA,
            )
            if not drive:
                for key in (
                    'motor_rpm',
                    'tractive_commanded',
                    'tractive_feedback',
                    'drive_bus_voltage',
                    'drive_bus_current',
                    'phase_current',
                    'controller_temperature',
                    'motor_temperature',
                    'command_received',
                    'command_timeout',
                    'drive_command_age',
                    'part1_valid',
                    'part1_age',
                    'part2_valid',
                    'part2_age',
                    'motor_fault_bits',
                ):
                    self._set(key, NO_DATA)
                return

            self._set('motor_rpm', f'{drive.motor_rpm:.1f} rpm')
            self._set(
                'tractive_commanded',
                bool_text(drive.tractive_commanded),
            )
            self._set(
                'tractive_feedback',
                valid_bool_text(
                    drive.tractive_feedback_valid,
                    drive.tractive_feedback,
                ),
            )
            self._set(
                'drive_bus_voltage',
                format_if_valid(
                    drive.part1_valid,
                    drive.bus_voltage_v,
                    '.2f',
                    ' V',
                ),
            )
            self._set(
                'drive_bus_current',
                format_if_valid(
                    drive.part1_valid,
                    drive.bus_current_a,
                    '.2f',
                    ' A',
                ),
            )
            self._set(
                'phase_current',
                format_if_valid(
                    drive.part1_valid,
                    drive.phase_current_a,
                    '.2f',
                    ' A',
                ),
            )
            self._set(
                'controller_temperature',
                format_if_valid(
                    drive.part2_valid,
                    drive.controller_temperature_c,
                    '.1f',
                    ' °C',
                ),
            )
            self._set(
                'motor_temperature',
                format_if_valid(
                    drive.part2_valid,
                    drive.motor_temperature_c,
                    '.1f',
                    ' °C',
                ),
            )
            self._set('command_received', bool_text(drive.command_received))
            self._set('command_timeout', bool_text(drive.command_timeout))
            self._set(
                'drive_command_age',
                f'{drive.command_age_ms:.0f} ms',
            )
            self._set('part1_valid', bool_text(drive.part1_valid))
            self._set(
                'part1_age',
                f'{drive.part1_age_ms:.0f} ms'
                if drive.part1_valid
                else NA,
            )
            self._set('part2_valid', bool_text(drive.part2_valid))
            self._set(
                'part2_age',
                f'{drive.part2_age_ms:.0f} ms'
                if drive.part2_valid
                else NA,
            )
            self._set(
                'motor_fault_bits',
                f'0x{int(drive.fault_bits):08X}'
                if drive.fault_bits_valid
                else NA,
            )

        def _refresh_lateral(self, snapshot):
            command = snapshot[COMMAND].message
            steering = snapshot[STEERING].message
            self._set(
                'target_steering',
                (
                    f'{rad_to_deg(command.steering_target_rad):.2f}°'
                    if command
                    else NO_DATA
                ),
                snapshot[COMMAND].state,
            )
            if not steering:
                for key in (
                    'commanded_steering',
                    'corrected_steering',
                    'actual_steering',
                    'tracking_error',
                    'steering_rate',
                    'steering_state',
                    'steering_reason',
                    'enabled_commanded',
                    'enabled_feedback',
                    'enabled_feedback_valid',
                    'can_timeout',
                    'mode_received',
                    'mode_timeout',
                    'steer_command_received',
                    'steer_command_timeout',
                    'feedback_timeout',
                    'communication_ok',
                    'pending_command_valid',
                    'movement_target_valid',
                    'feedback_valid',
                    'feedback_age',
                    'mode_age',
                    'steer_command_age',
                    'motor_position',
                    'motor_angle',
                    'steering_motor_speed',
                    'steering_motor_current',
                    'steering_bus_voltage',
                    'driver_temperature',
                    'steering_fault_bits',
                ):
                    self._set(key, NO_DATA)
                return

            angle_fields = (
                (
                    'commanded_steering',
                    steering.commanded_wheel_angle_rad,
                    True,
                ),
                (
                    'corrected_steering',
                    steering.corrected_wheel_angle_rad,
                    True,
                ),
                (
                    'actual_steering',
                    steering.actual_wheel_angle_rad,
                    steering.feedback_valid,
                ),
                (
                    'tracking_error',
                    steering.tracking_error_rad,
                    steering.feedback_valid,
                ),
            )
            for key, value, valid in angle_fields:
                self._set(
                    key,
                    (
                        f'{rad_to_deg(value):.2f}°'
                        if valid
                        else NA
                    ),
                )
            self._set(
                'steering_rate',
                (
                    f'{rad_to_deg(steering.steering_rate_radps):.2f} °/s'
                    if steering.feedback_valid
                    else NA
                ),
            )
            self._set(
                'steering_state',
                steering_state_text(steering.state),
            )
            self._set(
                'steering_reason',
                steering_disable_reason_text(steering.disable_reason),
            )
            self._set(
                'enabled_commanded',
                bool_text(steering.enabled_commanded),
            )
            self._set(
                'enabled_feedback',
                valid_bool_text(
                    steering.enabled_feedback_valid,
                    steering.enabled_feedback,
                ),
            )
            self._set(
                'enabled_feedback_valid',
                bool_text(steering.enabled_feedback_valid),
            )
            self._set('can_timeout', bool_text(steering.can_timeout))
            self._set('mode_received', bool_text(steering.mode_received))
            self._set('mode_timeout', bool_text(steering.mode_timeout))
            self._set(
                'steer_command_received',
                bool_text(steering.steer_command_received),
            )
            self._set(
                'steer_command_timeout',
                bool_text(steering.steer_command_timeout),
            )
            self._set(
                'feedback_timeout',
                bool_text(steering.feedback_timeout),
            )
            self._set(
                'communication_ok',
                bool_text(steering.communication_ok),
            )
            self._set(
                'pending_command_valid',
                bool_text(steering.pending_command_valid),
            )
            self._set(
                'movement_target_valid',
                bool_text(steering.movement_target_valid),
            )
            self._set('feedback_valid', bool_text(steering.feedback_valid))
            self._set(
                'feedback_age',
                f'{steering.feedback_age_ms:.0f} ms'
                if steering.feedback_valid
                else NA,
            )
            self._set('mode_age', f'{steering.mode_age_ms:.0f} ms')
            self._set(
                'steer_command_age',
                f'{steering.steer_command_age_ms:.0f} ms',
            )
            self._set(
                'motor_position',
                (
                    f'{steering.motor_position_count} count'
                    if steering.feedback_valid
                    else NA
                ),
            )
            self._set(
                'motor_angle',
                (
                    f'{steering.motor_angle_deg:.2f}°'
                    if steering.feedback_valid
                    else NA
                ),
            )
            self._set(
                'steering_motor_speed',
                (
                    f'{steering.motor_speed_rpm:.1f} rpm'
                    if steering.feedback_valid
                    else NA
                ),
            )
            self._set(
                'steering_motor_current',
                (
                    f'{steering.motor_current_a:.2f} A'
                    if steering.feedback_valid
                    else NA
                ),
            )
            self._set(
                'steering_bus_voltage',
                f'{steering.bus_voltage_v:.2f} V',
            )
            self._set(
                'driver_temperature',
                f'{steering.driver_temperature_c:.1f} °C',
            )
            self._set(
                'steering_fault_bits',
                f'0x{int(steering.fault_bits):04X}',
            )

        def _refresh_plots(self, snapshot, now):
            relative_time = now - self.start_monotonic
            command = snapshot[COMMAND].message
            drive = snapshot[DRIVE].message
            steering = snapshot[STEERING].message

            if (
                command
                and drive
                and snapshot[COMMAND].state == OK
                and snapshot[DRIVE].state == OK
            ):
                self.speed_history.append(
                    (
                        relative_time,
                        float(command.speed_target_mps),
                        float(drive.speed_mps),
                    )
                )
            if (
                command
                and steering
                and snapshot[COMMAND].state == OK
                and snapshot[STEERING].state == OK
                and steering.feedback_valid
            ):
                self.steering_history.append(
                    (
                        relative_time,
                        rad_to_deg(command.steering_target_rad),
                        rad_to_deg(steering.actual_wheel_angle_rad),
                    )
                )

            cutoff = relative_time - self.node.history_sec
            for history in (self.speed_history, self.steering_history):
                while history and history[0][0] < cutoff:
                    history.popleft()

            if self.speed_history:
                x, target, actual = zip(*self.speed_history)
                self.speed_target_curve.setData(x, target)
                self.speed_actual_curve.setData(x, actual)
                self.speed_plot.setXRange(
                    max(0.0, relative_time - self.node.history_sec),
                    max(self.node.history_sec, relative_time),
                    padding=0.0,
                )
            if self.steering_history:
                x, target, actual = zip(*self.steering_history)
                self.steering_target_curve.setData(x, target)
                self.steering_actual_curve.setData(x, actual)
                self.steering_plot.setXRange(
                    max(0.0, relative_time - self.node.history_sec),
                    max(self.node.history_sec, relative_time),
                    padding=0.0,
                )

        def refresh(self):
            if not rclpy.ok():
                self.close()
                return
            now = time.monotonic()
            snapshot = self.cache.snapshot(self.node.stale_ms, now=now)
            self._refresh_system(snapshot)
            self._refresh_longitudinal(snapshot)
            self._refresh_lateral(snapshot)
            self._refresh_plots(snapshot, now)

        def closeEvent(self, event):
            self.timer.stop()
            event.accept()

else:

    class VehicleDashboard:
        """Dependency error placeholder used only when Qt cannot import."""

        def __init__(self, *_args, **_kwargs):
            raise RuntimeError(
                'vehicle_dashboard requires PyQt5 and pyqtgraph'
            ) from GUI_IMPORT_ERROR


def _spin_executor(executor):
    try:
        executor.spin()
    except (ExternalShutdownException, KeyboardInterrupt):
        pass


def main(args=None):
    if GUI_IMPORT_ERROR is not None:
        raise RuntimeError(
            'vehicle_dashboard requires python3-pyqt5 and '
            'python3-pyqtgraph; see README.md'
        ) from GUI_IMPORT_ERROR

    rclpy.init(args=args)
    cache = RosStateCache()
    node = VehicleStatusSubscriber(cache)
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    ros_thread = threading.Thread(
        target=_spin_executor,
        args=(executor,),
        name='vehicle-dashboard-ros',
        daemon=True,
    )
    ros_thread.start()

    # ROS arguments belong to rclpy and must not be parsed as Qt options.
    app = QtWidgets.QApplication([sys.argv[0]])
    app.setApplicationName('KAI Vehicle Debug Dashboard')
    dashboard = VehicleDashboard(node, cache)
    dashboard.show()

    try:
        exit_code = app.exec_()
    finally:
        executor.shutdown(timeout_sec=1.0)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        ros_thread.join(timeout=2.0)

    return exit_code


if __name__ == '__main__':
    sys.exit(main())
