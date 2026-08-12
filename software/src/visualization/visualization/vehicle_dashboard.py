"""Read-only Qt dashboard for the canonical vehicle ROS 2 interface.

Everything is on one page, laid out along the signal flow: what the vehicle was
told to do, what the drive axis did, what the steering axis did. There are no
plots and no scrolling -- this is a debugging read-out, and a value you have to
scroll to find is a value you did not see.

Two devices carry most of the work:

* **Active conditions.** The strip under the header repeats the reason fields
  the messages already carry -- ``stop_reason_bits``, ``disable_reason``, the
  timeout flags -- naming the field each one came from. It reports; it does not
  diagnose. Per this package's contract the GUI never infers a state that the
  message did not state.
* **Flag chips.** Booleans are chips that stay dim while nominal and light up
  only when they are not, so a wall of twenty flags reads at a glance. Each
  chip declares which way is nominal, because ``command_received`` is bad when
  false while ``can_timeout`` is bad when true, and some -- brake
  ``actual_valid``, steering ``enabled_feedback_valid`` -- are contractually
  false on this hardware and must never be shown as faults.
"""

import sys
import threading

from kaiev26_hil.qt_runtime import ensure_bundled_qt_first
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
)

ensure_bundled_qt_first()

try:
    from PyQt5 import QtCore, QtGui, QtWidgets
except ImportError as gui_import_error:
    QtCore = None
    QtGui = None
    QtWidgets = None
    GUI_IMPORT_ERROR = gui_import_error
else:
    GUI_IMPORT_ERROR = None


#: Which way round a flag is nominal. ``INFO`` never alarms: it is for flags
#: this hardware holds at a fixed value by contract, where colouring the normal
#: case as a fault would train the operator to ignore the colour.
BAD_TRUE = 'bad_true'
BAD_FALSE = 'bad_false'
WARN_TRUE = 'warn_true'
INFO = 'info'

#: The vehicle contract carries speed in m/s everywhere. The operator reads
#: km/h, so every speed is shown in both rather than converted in the head.
MPS_TO_KMH = 3.6


class VehicleStatusSubscriber(Node):
    """Receive vehicle topics and write only to the thread-safe cache."""

    def __init__(self, cache):
        super().__init__('vehicle_dashboard')
        self.cache = cache

        self.declare_parameter('command_stale_ms', 300.0)
        self.declare_parameter('drive_status_stale_ms', 300.0)
        self.declare_parameter('brake_status_stale_ms', 300.0)
        self.declare_parameter('steering_status_stale_ms', 500.0)
        # One tick repaints only the values that changed -- there are no plots
        # to redraw -- so this is cheap even on a forwarded X11 display. 4 Hz
        # is the remote default; raise it to 10 on a local screen.
        self.declare_parameter('ui_update_hz', 4.0)

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


if QtWidgets is not None:

    class VehicleDashboard(QtWidgets.QMainWindow):
        """Developer-oriented, read-only status display on a single page."""

        FG = '#f0f6fc'
        #: Captions and not-yet-populated values. DIM grey was unreadable on
        #: the dark background, so only the flag chips still use DIM -- there
        #: the dim state carries meaning rather than merely being decoration.
        CAPTION = '#e6edf3'
        DIM = '#6e7681'
        ACCENT = '#58a6ff'
        GOOD = '#3fb950'
        WARN = '#d29922'
        BAD = '#f85149'

        #: One place for type scale, so the window can be resized as a unit.
        BASE_PT = 12
        CAPTION_PT = 11
        HEADLINE_PT = 20
        HEADLINE_SMALL_PT = 15
        CHIP_PT = 10

        STATUS_COLORS = {NO_DATA: CAPTION, OK: GOOD, STALE: WARN}

        DRIVE_FLAGS = (
            ('command_received', 'cmd received', BAD_FALSE),
            ('command_timeout', 'cmd timeout', BAD_TRUE),
            ('stop_request', 'stop request', BAD_TRUE),
            ('tractive_commanded', 'tractive cmd', INFO),
            ('tractive_feedback', 'tractive fb', INFO),
            ('tractive_feedback_valid', 'tractive fb valid', BAD_FALSE),
            ('part1_valid', 'Part I valid', BAD_FALSE),
            ('part2_valid', 'Part II valid', BAD_FALSE),
            ('fault_bits_valid', 'fault bits valid', INFO),
        )

        STEERING_FLAGS = (
            ('communication_ok', 'comm OK', BAD_FALSE),
            ('can_timeout', 'CAN timeout', BAD_TRUE),
            ('feedback_valid', 'feedback valid', BAD_FALSE),
            ('feedback_timeout', 'feedback timeout', BAD_TRUE),
            ('mode_received', 'mode received', BAD_FALSE),
            ('mode_timeout', 'mode timeout', BAD_TRUE),
            ('steer_command_received', 'steer cmd rx', BAD_FALSE),
            ('steer_command_timeout', 'steer cmd timeout', BAD_TRUE),
            ('pending_command_valid', 'pending cmd valid', INFO),
            ('movement_target_valid', 'move target valid', INFO),
            ('estop_active', 'E-stop', BAD_TRUE),
            ('command_brake_active', 'command brake', BAD_TRUE),
            ('enabled_commanded', 'enable cmd', INFO),
            ('enabled_feedback', 'enable fb', INFO),
            # Contractually false on the real KEYA: no trustworthy enable query.
            ('enabled_feedback_valid', 'enable fb valid', INFO),
            ('enable_mismatch', 'enable mismatch', BAD_TRUE),
            ('tracking_control_enabled', 'tracking ctrl', INFO),
            ('tracking_error_warning', 'tracking warn', WARN_TRUE),
            ('tracking_error_fault', 'tracking fault', BAD_TRUE),
        )

        def __init__(self, node, cache):
            super().__init__()
            self.node = node
            self.cache = cache
            self.values = {}
            self.chips = {}
            self._styles = {}

            self.setWindowTitle('KAI Vehicle Debug Dashboard (Read Only)')
            self._build_ui()
            # Wide rather than tall: the three columns are the layout, and the
            # content ends well short of the bottom on any sane height.
            self.resize(1900, 900)
            self.setMinimumSize(1280, 780)

            interval_ms = max(20, int(1000.0 / self.node.ui_update_hz))
            self.timer = QtCore.QTimer(self)
            self.timer.timeout.connect(self.refresh)
            self.timer.start(interval_ms)
            # Shutdown is a separate concern from rendering: keeping it out of
            # refresh leaves refresh a pure function of the cache, which is what
            # makes it testable without a live ROS context.
            self.shutdown_timer = QtCore.QTimer(self)
            self.shutdown_timer.timeout.connect(self._close_if_ros_stopped)
            self.shutdown_timer.start(200)
            self.refresh()

        # ---------------------------------------------------------------- UI

        def _build_ui(self):
            central = QtWidgets.QWidget()
            outer = QtWidgets.QVBoxLayout(central)
            outer.setContentsMargins(10, 8, 10, 8)
            outer.setSpacing(8)
            outer.addLayout(self._build_header())
            outer.addWidget(self._build_conditions())

            columns = QtWidgets.QHBoxLayout()
            columns.setSpacing(8)
            columns.addWidget(self._build_system_column(), 3)
            columns.addWidget(self._build_longitudinal_column(), 4)
            columns.addWidget(self._build_lateral_column(), 5)
            outer.addLayout(columns, 1)

            self.setCentralWidget(central)
            self.setStyleSheet(
                f'QMainWindow, QWidget {{ background:#0d1117; color:{self.FG};'
                f' font-size:{self.BASE_PT}pt; }}'
                # margin-top has to clear the bold title drawn in it, or the
                # top of the caption is cut off by the frame.
                'QGroupBox { border:1px solid #30363d; border-radius:6px;'
                ' margin-top:18px; font-weight:bold; }'
                'QGroupBox::title { subcontrol-origin:margin; left:9px;'
                f' padding:0 4px; color:{self.ACCENT}; }}'
            )

        def _build_header(self):
            header = QtWidgets.QHBoxLayout()
            title = QtWidgets.QLabel('Vehicle Debug Dashboard')
            title.setStyleSheet(
                f'color:{self.ACCENT};font-size:{self.HEADLINE_PT}pt;'
                'font-weight:bold;'
            )
            header.addWidget(title)
            read_only = QtWidgets.QLabel('READ ONLY')
            read_only.setStyleSheet(
                f'color:{self.CAPTION};border:1px solid {self.CAPTION};'
                'border-radius:3px;padding:1px 6px;font-weight:bold;'
            )
            header.addWidget(read_only)
            header.addStretch(1)
            # One badge per subscribed topic: the first thing to check when a
            # column reads NO DATA is whether the topic is arriving at all.
            for key, caption in (
                (COMMAND, '/vehicle/command'),
                (DRIVE, '/drive/status'),
                (BRAKE, '/brake/status'),
                (STEERING, '/steering/status'),
            ):
                badge = QtWidgets.QLabel(self._link_text(caption, NO_DATA))
                badge.setTextInteractionFlags(QtCore.Qt.TextSelectableByMouse)
                self.values[f'link_{key}'] = badge
                header.addWidget(badge)
            return header

        @staticmethod
        def _link_text(caption, state, age_ms=None):
            """Fixed-width badge text.

            These badges repaint several times a second. Letting the state word
            and the age digits size themselves made every badge to the right of
            a changing one jump sideways, so both fields are padded to their
            widest form: the row keeps its geometry and only the glyphs change.
            """
            if age_ms is None:
                age = '--'
            else:
                age = f'{min(float(age_ms), 99999.0):.0f}'
            return f'{caption}  {state:<7} {age:>5} ms'

        def _build_conditions(self):
            box = QtWidgets.QGroupBox(
                'Active conditions  ·  reported by the messages, not inferred'
            )
            layout = QtWidgets.QHBoxLayout(box)
            layout.setContentsMargins(9, 4, 9, 6)
            self.conditions = QtWidgets.QLabel(NO_DATA)
            self.conditions.setWordWrap(True)
            self.conditions.setTextInteractionFlags(
                QtCore.Qt.TextSelectableByMouse
            )
            layout.addWidget(self.conditions)
            return box

        def _build_system_column(self):
            # This column mixes three topics -- the command, the drive message's
            # physical switch fields and the brake message -- so it is named for
            # what it shows rather than for one source topic.
            # No "&" in the caption: Qt reads it as a mnemonic and renders the
            # title as "Command _physical switches".
            group = QtWidgets.QGroupBox('1. Command  ·  physical switches')
            layout = QtWidgets.QVBoxLayout(group)
            layout.addLayout(self._headline_row((
                ('MODE', 'mode'),
                ('GEAR', 'gear'),
            )))
            grid = self._grid(layout)
            rows = (
                ('Pedal [driver]', 'pedal', False),
                ('Pedal ADC raw', 'pedal_adc', False),
                ('', None, False),
                ('Target speed', 'cmd_speed', False),
                ('Target steering', 'cmd_steer', False),
                ('Brake engage [cmd]', 'cmd_brake', False),
                ('', None, False),
                ('Physical E-stop', 'physical_estop', False),
                ('Brake commanded', 'brake_commanded', False),
                ('Brake command age', 'brake_age', False),
                ('Brake actual feedback', 'brake_actual', False),
                # Three reason bits can be asserted at once, so this one gets
                # its own line rather than sharing with the caption.
                ('Stop reasons', 'brake_reasons', True),
            )
            index = 0
            for caption, key, wide in rows:
                self._row(grid, index, caption, key, wide=wide)
                index += 2 if wide else 1
            layout.addStretch(1)
            return group

        def _build_longitudinal_column(self):
            group = QtWidgets.QGroupBox('2. Longitudinal  ·  /drive/status')
            layout = QtWidgets.QVBoxLayout(group)
            layout.addLayout(self._headline_row((
                ('TARGET  [km/h]', 'speed_target', 'speed_target_mps'),
                ('ACTUAL  [km/h]', 'speed_actual', 'speed_actual_mps'),
                ('ERROR  [km/h]', 'speed_error', 'speed_error_mps'),
            )))
            grid = self._grid(layout)
            rows = (
                ('Motor RPM', 'motor_rpm'),
                ('Controller control mode', 'control_mode'),
                ('Commanded motor RPM', 'commanded_rpm'),
                ('Commanded phase current', 'commanded_current'),
                ('', None),
                ('Bus voltage [Part I]', 'bus_voltage'),
                ('Bus current [Part I]', 'bus_current'),
                ('Phase current [Part I]', 'phase_current'),
                ('Controller temp [Part II]', 'controller_temp'),
                ('Motor temp [Part II]', 'motor_temp'),
                ('', None),
                ('Host command age', 'drive_cmd_age'),
                ('Part I age', 'part1_age'),
                ('Part II age', 'part2_age'),
                ('Motor fault bits', 'drive_faults'),
            )
            for index, (caption, key) in enumerate(rows):
                self._row(grid, index, caption, key)
            layout.addWidget(self._flag_panel('drive', self.DRIVE_FLAGS, 3))
            layout.addStretch(1)
            return group

        def _build_lateral_column(self):
            group = QtWidgets.QGroupBox('3. Lateral  ·  /steering/status')
            layout = QtWidgets.QVBoxLayout(group)
            layout.addLayout(self._headline_row((
                ('STATE', 'steering_state'),
                ('DISABLE REASON', 'steering_reason'),
            )))
            # The four angles are one pipeline, so they read as a chain rather
            # than as four unrelated rows: a limit or a PID correction shows up
            # as a step between two neighbouring cells.
            layout.addLayout(self._headline_row((
                ('target', 'angle_target'),
                ('→ commanded', 'angle_commanded'),
                ('→ corrected', 'angle_corrected'),
                ('actual', 'angle_actual'),
                ('error', 'angle_error'),
            ), small=True))
            grid = self._grid(layout)
            rows = (
                ('Motor position', 'motor_position'),
                ('Motor angle [shaft]', 'motor_angle'),
                ('Motor speed', 'motor_speed'),
                ('Motor current', 'motor_current'),
                ('Bus voltage', 'steer_voltage'),
                ('Driver temperature', 'driver_temp'),
                ('', None),
                ('Feedback age', 'feedback_age'),
                ('Mode age', 'mode_age'),
                ('Steer command age', 'steer_cmd_age'),
                ('Drive mode seen by steering', 'steer_drive_mode'),
                ('KEYA fault bits', 'steer_faults'),
            )
            for index, (caption, key) in enumerate(rows):
                self._row(grid, index, caption, key)
            layout.addWidget(self._flag_panel('steering', self.STEERING_FLAGS, 4))
            layout.addStretch(1)
            return group

        # ------------------------------------------------------------ pieces

        @staticmethod
        def _grid(parent_layout):
            grid = QtWidgets.QGridLayout()
            grid.setHorizontalSpacing(10)
            grid.setVerticalSpacing(2)
            grid.setColumnStretch(1, 1)
            parent_layout.addLayout(grid)
            return grid

        def _row(self, grid, row, caption, key, *, wide=False):
            """One caption/value row. ``wide`` gives the value its own line.

            A value whose length is unbounded -- a decoded bit list -- cannot
            share a line with its caption: it overruns the caption and clips.
            """
            if key is None:  # spacer keeps related rows visually grouped
                spacer = QtWidgets.QLabel(' ')
                spacer.setFixedHeight(6)
                grid.addWidget(spacer, row, 0, 1, 2)
                return
            name = QtWidgets.QLabel(caption)
            name.setStyleSheet(f'color:{self.CAPTION};')
            value = QtWidgets.QLabel(NO_DATA)
            value.setTextInteractionFlags(QtCore.Qt.TextSelectableByMouse)
            value.setStyleSheet(f'color:{self.FG};font-family:monospace;')
            if wide:
                value.setWordWrap(True)
                value.setAlignment(QtCore.Qt.AlignLeft | QtCore.Qt.AlignVCenter)
                grid.addWidget(name, row, 0, 1, 2)
                grid.addWidget(value, row + 1, 0, 1, 2)
            else:
                value.setAlignment(
                    QtCore.Qt.AlignRight | QtCore.Qt.AlignVCenter
                )
                grid.addWidget(name, row, 0)
                grid.addWidget(value, row, 1)
            self.values[key] = value

        def _headline_style(self, color, small):
            size = self.HEADLINE_SMALL_PT if small else self.HEADLINE_PT
            return (
                f'color:{color};font-family:monospace;font-weight:bold;'
                f'font-size:{size}pt;'
            )

        def _sub_style(self, color):
            return (
                f'color:{color};font-family:monospace;'
                f'font-size:{self.CAPTION_PT}pt;'
            )

        def _headline_row(self, specs, *, small=False):
            """A row of large read-at-a-glance values under a caption.

            A spec may carry a third element, the key of a secondary line drawn
            under the value. It exists so a quantity can be shown in both of
            its units without either one leaving the headline.
            """
            row = QtWidgets.QHBoxLayout()
            row.setSpacing(12)
            for spec in specs:
                caption, key = spec[0], spec[1]
                sub_key = spec[2] if len(spec) > 2 else None
                cell = QtWidgets.QVBoxLayout()
                cell.setSpacing(0)
                label = QtWidgets.QLabel(caption)
                label.setStyleSheet(
                    f'color:{self.CAPTION};font-size:{self.CAPTION_PT}pt;'
                )
                value = QtWidgets.QLabel(NO_DATA)
                value.setTextInteractionFlags(
                    QtCore.Qt.TextSelectableByMouse
                )
                value.setStyleSheet(self._headline_style(self.FG, small))
                cell.addWidget(label)
                cell.addWidget(value)
                self.values[key] = value
                if sub_key is not None:
                    sub = QtWidgets.QLabel('--')
                    sub.setTextInteractionFlags(
                        QtCore.Qt.TextSelectableByMouse
                    )
                    sub.setStyleSheet(self._sub_style(self.CAPTION))
                    cell.addWidget(sub)
                    self.values[sub_key] = sub
                row.addLayout(cell)
            row.addStretch(1)
            return row

        def _flag_panel(self, prefix, specs, columns):
            box = QtWidgets.QGroupBox('Flags')
            grid = QtWidgets.QGridLayout(box)
            grid.setHorizontalSpacing(4)
            grid.setVerticalSpacing(3)
            for index, (field, caption, polarity) in enumerate(specs):
                chip = QtWidgets.QLabel(caption)
                chip.setAlignment(QtCore.Qt.AlignCenter)
                grid.addWidget(chip, index // columns, index % columns)
                self.chips[f'{prefix}.{field}'] = (chip, caption, polarity)
            return box

        # ----------------------------------------------------------- styling

        def _style(self, widget, style):
            """Apply a stylesheet only when it changed.

            setStyleSheet re-polishes and repaints unconditionally, and this
            window rewrites every widget on every tick. On a forwarded display
            the skipped repaints are the whole cost of the refresh.
            """
            if self._styles.get(widget) == style:
                return
            self._styles[widget] = style
            widget.setStyleSheet(style)

        def _set(self, key, text, color=None):
            value = self.values[key]
            value.setText(str(text))
            self._style(
                value,
                f'color:{color or self.FG};font-family:monospace;',
            )

        def _set_headline(self, key, text, color=None, *, small=False):
            value = self.values[key]
            value.setText(str(text))
            self._style(value, self._headline_style(color or self.FG, small))

        def _set_sub(self, key, text, color=None):
            """Secondary line under a headline -- the same value, other unit."""
            value = self.values[key]
            value.setText(str(text))
            self._style(value, self._sub_style(color or self.CAPTION))

        def _set_chip(self, prefix, field, state):
            """Render one flag. ``state`` is True/False, or None for no data."""
            chip, caption, polarity = self.chips[f'{prefix}.{field}']
            chip.setText(caption)
            if state is None:
                color, filled = self.DIM, False
            elif polarity == INFO:
                color, filled = (self.FG if state else self.DIM), False
            elif polarity == BAD_TRUE:
                color, filled = (self.BAD if state else self.DIM), bool(state)
            elif polarity == WARN_TRUE:
                color, filled = (self.WARN if state else self.DIM), bool(state)
            else:  # BAD_FALSE
                color, filled = (self.GOOD if state else self.BAD), not state
            if filled:
                style = (
                    f'background:{color};color:#0d1117;font-weight:bold;'
                    'border-radius:3px;padding:2px 4px;'
                    f'font-size:{self.CHIP_PT}pt;'
                )
            else:
                style = (
                    f'color:{color};border:1px solid {color};'
                    'border-radius:3px;padding:1px 3px;'
                    f'font-size:{self.CHIP_PT}pt;'
                )
            self._style(chip, style)

        def _blank_chips(self, prefix, specs):
            for field, _caption, _polarity in specs:
                self._set_chip(prefix, field, None)

        # ---------------------------------------------------------- refresh

        def _close_if_ros_stopped(self):
            if not rclpy.ok():
                self.close()

        def refresh(self):
            snapshot = self.cache.snapshot(self.node.stale_ms)
            self._refresh_links(snapshot)
            self._refresh_conditions(snapshot)
            self._refresh_system(snapshot)
            self._refresh_longitudinal(snapshot)
            self._refresh_lateral(snapshot)

        def _refresh_links(self, snapshot):
            for key, caption in (
                (COMMAND, '/vehicle/command'),
                (DRIVE, '/drive/status'),
                (BRAKE, '/brake/status'),
                (STEERING, '/steering/status'),
            ):
                topic = snapshot[key]
                color = self.STATUS_COLORS.get(topic.state, self.FG)
                if topic.state == NO_DATA:
                    text = self._link_text(caption, NO_DATA)
                else:
                    text = self._link_text(caption, topic.state, topic.age_ms)
                badge = self.values[f'link_{key}']
                badge.setText(text)
                self._style(
                    badge,
                    f'color:{color};border:1px solid {color};'
                    'border-radius:3px;padding:1px 6px;font-family:monospace;',
                )

        def _refresh_conditions(self, snapshot):
            """Repeat the reason fields the messages carry, naming each source."""
            drive = snapshot[DRIVE].message
            brake = snapshot[BRAKE].message
            steering = snapshot[STEERING].message
            conditions = []

            missing = [
                caption
                for key, caption in (
                    (COMMAND, '/vehicle/command'),
                    (DRIVE, '/drive/status'),
                    (BRAKE, '/brake/status'),
                    (STEERING, '/steering/status'),
                )
                if snapshot[key].state == NO_DATA
            ]
            if missing:
                conditions.append('NO DATA: ' + ', '.join(missing))

            if brake is not None and int(brake.stop_reason_bits):
                conditions.append(
                    f'{brake_reason_text(brake.stop_reason_bits)} '
                    '(brake.stop_reason_bits)'
                )
            if drive is not None:
                if drive.command_timeout:
                    conditions.append('COMMAND_TIMEOUT (drive.command_timeout)')
                if drive.stop_request:
                    conditions.append('STOP_REQUEST (drive.stop_request)')
                if drive.fault_bits_valid and int(drive.fault_bits):
                    conditions.append(
                        f'FAULT 0x{int(drive.fault_bits):06X} (drive.fault_bits)'
                    )
            if steering is not None:
                if steering.state != SteeringActuatorStatus.STATE_ACTIVE:
                    conditions.append(
                        f'{steering_state_text(steering.state)} (steering.state)'
                    )
                if steering.disable_reason != SteeringActuatorStatus.REASON_NONE:
                    conditions.append(
                        f'{steering_disable_reason_text(steering.disable_reason)} '
                        '(steering.disable_reason)'
                    )
                # bit0 is the KEYA enable state, not a fault.
                steering_faults = int(steering.fault_bits) & ~1
                if steering_faults:
                    conditions.append(
                        f'FAULT 0x{steering_faults:04X} (steering.fault_bits)'
                    )

            if conditions:
                self.conditions.setText('   ·   '.join(conditions))
                self._style(
                    self.conditions,
                    f'color:{self.BAD};font-weight:bold;font-family:monospace;',
                )
            else:
                self.conditions.setText('none asserted')
                self._style(
                    self.conditions,
                    f'color:{self.GOOD};font-family:monospace;',
                )

        def _refresh_system(self, snapshot):
            command = snapshot[COMMAND].message
            drive = snapshot[DRIVE].message
            brake = snapshot[BRAKE].message

            self._set_headline(
                'mode',
                drive_mode_text(drive.mode) if drive else NO_DATA,
                self.ACCENT if drive else self.FG,
            )
            self._set_headline(
                'gear',
                drive_gear_text(drive.gear) if drive else NO_DATA,
                self.ACCENT if drive else self.FG,
            )
            self._set(
                'pedal',
                f'{int(drive.pedal_percent)} %' if drive else NO_DATA,
            )
            self._set(
                'pedal_adc',
                f'{int(drive.pedal_adc_raw)}' if drive else NO_DATA,
            )

            self._set(
                'cmd_speed',
                f'{command.speed_target_mps:.2f} m/s  ·  '
                f'{command.speed_target_mps * MPS_TO_KMH:.2f} km/h'
                if command else NO_DATA,
            )
            self._set(
                'cmd_steer',
                f'{rad_to_deg(command.steering_target_rad):.2f}°'
                if command else NO_DATA,
            )
            self._set(
                'cmd_brake',
                bool_text(command.brake_engage) if command else NO_DATA,
                self.BAD if command and command.brake_engage else None,
            )

            estop = physical_estop_active(brake)
            self._set(
                'physical_estop',
                bool_text(estop) if estop is not None else NO_DATA,
                self.BAD if estop else None,
            )
            self._set(
                'brake_commanded',
                bool_text(brake.brake_commanded) if brake else NO_DATA,
                self.BAD if brake and brake.brake_commanded else None,
            )
            self._set(
                'brake_reasons',
                brake_reason_text(brake.stop_reason_bits) if brake else NO_DATA,
                self.BAD if brake and int(brake.stop_reason_bits) else None,
            )
            self._set(
                'brake_age',
                f'{brake.command_age_ms:.0f} ms' if brake else NO_DATA,
            )
            # Permanently unsupported on this hardware, so it is stated as a
            # capability rather than coloured like a fault.
            self._set(
                'brake_actual',
                ('SUPPORTED' if brake.actual_valid else 'NOT WIRED')
                if brake else NO_DATA,
            )

        def _refresh_longitudinal(self, snapshot):
            command = snapshot[COMMAND].message
            drive = snapshot[DRIVE].message
            target = command.speed_target_mps if command else None
            actual = drive.speed_mps if drive else None

            # The headline carries km/h because that is the unit the operator
            # reads; the contract's own m/s stays underneath so the number in
            # the message is never hidden behind a conversion.
            error = (
                target - actual
                if target is not None and actual is not None else None
            )
            self._set_headline(
                'speed_target',
                f'{target * MPS_TO_KMH:.2f}' if target is not None else '--',
                self.ACCENT if target is not None else self.FG,
            )
            self._set_sub(
                'speed_target_mps',
                f'{target:.2f} m/s' if target is not None else '--',
            )
            self._set_headline(
                'speed_actual',
                f'{actual * MPS_TO_KMH:.2f}' if actual is not None else '--',
                self.GOOD if actual is not None else self.FG,
            )
            self._set_sub(
                'speed_actual_mps',
                f'{actual:.2f} m/s' if actual is not None else '--',
            )
            self._set_headline(
                'speed_error',
                f'{error * MPS_TO_KMH:+.2f}' if error is not None else '--',
                self.FG,
            )
            self._set_sub(
                'speed_error_mps',
                f'{error:+.2f} m/s' if error is not None else '--',
            )

            if drive is None:
                for key in (
                    'motor_rpm', 'control_mode', 'commanded_rpm',
                    'commanded_current', 'bus_voltage', 'bus_current',
                    'phase_current', 'controller_temp', 'motor_temp',
                    'drive_cmd_age', 'part1_age', 'part2_age', 'drive_faults',
                ):
                    self._set(key, NO_DATA, self.FG)
                self._blank_chips('drive', self.DRIVE_FLAGS)
                return

            self._set('motor_rpm', f'{drive.motor_rpm:.1f} rpm')
            self._set(
                'control_mode',
                'SPEED' if int(drive.controller_control_mode) == 1 else 'TORQUE',
            )
            self._set('commanded_rpm', f'{drive.commanded_motor_rpm:.0f} rpm')
            self._set(
                'commanded_current',
                f'{drive.commanded_phase_current_a:.1f} A',
            )
            self._set(
                'bus_voltage',
                format_if_valid(drive.part1_valid, drive.bus_voltage_v, '.2f', ' V'),
            )
            self._set(
                'bus_current',
                format_if_valid(drive.part1_valid, drive.bus_current_a, '.2f', ' A'),
            )
            self._set(
                'phase_current',
                format_if_valid(drive.part1_valid, drive.phase_current_a, '.2f', ' A'),
            )
            self._set(
                'controller_temp',
                format_if_valid(
                    drive.part2_valid, drive.controller_temperature_c, '.1f', ' °C'
                ),
            )
            self._set(
                'motor_temp',
                format_if_valid(
                    drive.part2_valid, drive.motor_temperature_c, '.1f', ' °C'
                ),
            )
            self._set(
                'drive_cmd_age',
                f'{drive.command_age_ms:.0f} ms',
                self.BAD if drive.command_timeout else None,
            )
            self._set(
                'part1_age',
                f'{drive.part1_age_ms:.0f} ms' if drive.part1_valid else NA,
            )
            self._set(
                'part2_age',
                f'{drive.part2_age_ms:.0f} ms' if drive.part2_valid else NA,
            )
            faults = int(drive.fault_bits)
            self._set(
                'drive_faults',
                f'0x{faults:06X}' if drive.fault_bits_valid else NA,
                self.BAD if drive.fault_bits_valid and faults else None,
            )
            for field, _caption, _polarity in self.DRIVE_FLAGS:
                self._set_chip('drive', field, bool(getattr(drive, field)))

        def _refresh_lateral(self, snapshot):
            command = snapshot[COMMAND].message
            steering = snapshot[STEERING].message

            if steering is None:
                self._set_headline('steering_state', NO_DATA, self.FG)
                self._set_headline('steering_reason', NO_DATA, self.FG)
                for key in (
                    'angle_target', 'angle_commanded', 'angle_corrected',
                    'angle_actual', 'angle_error',
                ):
                    self._set_headline(key, '--', self.FG, small=True)
                for key in (
                    'motor_position', 'motor_angle', 'motor_speed',
                    'motor_current', 'steer_voltage', 'driver_temp',
                    'feedback_age', 'mode_age', 'steer_cmd_age',
                    'steer_drive_mode', 'steer_faults',
                ):
                    self._set(key, NO_DATA, self.FG)
                self._blank_chips('steering', self.STEERING_FLAGS)
                # The commanded angle still comes from /vehicle/command.
                self._set_headline(
                    'angle_target',
                    f'{rad_to_deg(command.steering_target_rad):.2f}°'
                    if command else '--',
                    self.ACCENT if command else self.FG,
                    small=True,
                )
                return

            active = steering.state == SteeringActuatorStatus.STATE_ACTIVE
            self._set_headline(
                'steering_state',
                steering_state_text(steering.state),
                self.GOOD if active else self.BAD,
            )
            reason_none = (
                steering.disable_reason == SteeringActuatorStatus.REASON_NONE
            )
            self._set_headline(
                'steering_reason',
                steering_disable_reason_text(steering.disable_reason),
                self.FG if reason_none else self.BAD,
            )

            self._set_headline(
                'angle_target',
                f'{rad_to_deg(steering.target_wheel_angle_rad):.2f}°',
                self.ACCENT,
                small=True,
            )
            self._set_headline(
                'angle_commanded',
                f'{rad_to_deg(steering.commanded_wheel_angle_rad):.2f}°',
                small=True,
            )
            self._set_headline(
                'angle_corrected',
                f'{rad_to_deg(steering.corrected_wheel_angle_rad):.2f}°',
                small=True,
            )
            self._set_headline(
                'angle_actual',
                f'{rad_to_deg(steering.actual_wheel_angle_rad):.2f}°'
                if steering.feedback_valid else NA,
                self.GOOD if steering.feedback_valid else self.FG,
                small=True,
            )
            self._set_headline(
                'angle_error',
                f'{rad_to_deg(steering.tracking_error_rad):+.2f}°'
                if steering.feedback_valid else NA,
                self.WARN if steering.tracking_error_warning else None,
                small=True,
            )

            valid = steering.feedback_valid
            self._set(
                'motor_position',
                f'{steering.motor_position_count} count' if valid else NA,
            )
            self._set(
                'motor_angle',
                f'{steering.motor_angle_deg:.2f}°' if valid else NA,
            )
            self._set(
                'motor_speed',
                f'{steering.motor_speed_rpm:.1f} rpm' if valid else NA,
            )
            self._set(
                'motor_current',
                f'{steering.motor_current_a:.2f} A' if valid else NA,
            )
            self._set('steer_voltage', f'{steering.bus_voltage_v:.2f} V')
            self._set('driver_temp', f'{steering.driver_temperature_c:.1f} °C')
            self._set(
                'feedback_age',
                f'{steering.feedback_age_ms:.0f} ms' if valid else NA,
                self.BAD if steering.feedback_timeout else None,
            )
            self._set(
                'mode_age',
                f'{steering.mode_age_ms:.0f} ms',
                self.BAD if steering.mode_timeout else None,
            )
            self._set(
                'steer_cmd_age',
                f'{steering.steer_command_age_ms:.0f} ms',
                self.BAD if steering.steer_command_timeout else None,
            )
            self._set(
                'steer_drive_mode',
                drive_mode_text(steering.drive_mode),
            )
            # bit0 carries the enable state, so it is split out rather than
            # reported as a fault.
            raw = int(steering.fault_bits)
            enable_bit = raw & 1
            self._set(
                'steer_faults',
                f'0x{raw:04X}  (bit0 enable={enable_bit})',
                self.BAD if raw & ~1 else None,
            )
            for field, _caption, _polarity in self.STEERING_FLAGS:
                self._set_chip('steering', field, bool(getattr(steering, field)))

        def closeEvent(self, event):
            self.timer.stop()
            self.shutdown_timer.stop()
            event.accept()

else:

    class VehicleDashboard:
        """Dependency error placeholder used only when Qt cannot import."""

        def __init__(self, *_args, **_kwargs):
            raise RuntimeError(
                'vehicle_dashboard requires PyQt5'
            ) from GUI_IMPORT_ERROR


def _spin_executor(executor):
    try:
        executor.spin()
    except (ExternalShutdownException, KeyboardInterrupt):
        pass


def main(args=None):
    ensure_bundled_qt_first()
    if GUI_IMPORT_ERROR is not None:
        raise RuntimeError(
            'vehicle_dashboard requires python3-pyqt5; see README.md'
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
