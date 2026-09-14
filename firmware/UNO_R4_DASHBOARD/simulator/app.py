"""Tk GUI and snapshot CLI for the dashboard preview."""

from __future__ import annotations

import argparse
from collections import deque
from dataclasses import replace
from pathlib import Path
import time

from PIL import Image

from .model import *
from .model import _speed_centi_mps
from .backend import NativeDashboard, schedule_samples


UI_BG = "#F2F5F4"
UI_MUTED = "#66757D"


class PreviewApp:
    def __init__(
        self, *, scenario: str, telemetry_hz: int,
        timing_profile: Path | None = None,
    ) -> None:
        import tkinter as tk
        from tkinter import filedialog, ttk
        from PIL import ImageTk

        self.tk = tk
        self.ttk = ttk
        self.filedialog = filedialog
        self.ImageTk = ImageTk
        self.root = tk.Tk()
        self.root.title("KAI 4-inch Dashboard Preview")
        self.root.configure(bg=UI_BG)

        self.native = NativeDashboard(timing_profile)
        self.scenario_var = tk.StringVar(value=scenario)
        self.auto_case_var = tk.StringVar(value="")
        self.telemetry_var = tk.IntVar(value=telemetry_hz)
        self.paused_var = tk.BooleanVar(value=False)
        self.manual_var = tk.BooleanVar(value=False)
        self.mode_switch_var = tk.BooleanVar(value=False)
        self.manual_auto_telemetry: AutoTelemetry | None = None
        self.timeline_elapsed = 0.0
        self.wall_origin_ns = time.monotonic_ns()
        self.virtual_origin_us = self.native.now_us
        self.next_display_ns = self.wall_origin_ns
        self.display_period_ns = round(1_000_000_000 / 60)
        self.last_present_ns: int | None = None
        self.display_hz = 0.0
        self.next_sample_us: int | None = None
        self.sample = SCENARIO_BUILDERS[scenario](0.0)
        self.manual_changes = deque()
        self.manual_change_pending = False
        self.native_manual_enabled = False
        self.native_manual_sample = self.sample
        self.current_image: Image.Image | None = None
        self.photo = None
        self.startup_active = True
        self.native_alive = 0
        self.native_replay_requested = False

        self._make_manual_variables()
        self._build_ui()
        self.root.protocol("WM_DELETE_WINDOW", self._close)
        self._scenario_changed()
        self._register_manual_traces()
        self.native_manual_enabled = self.manual_var.get()
        self.native_manual_sample = self._manual_sample()
        self.root.after(0, self._tick)

    def _make_manual_variables(self) -> None:
        tk = self.tk
        self.pedal_var = tk.IntVar(value=24)
        self.speed_var = tk.DoubleVar(value=25.0)
        self.voltage_var = tk.DoubleVar(value=55.0)
        self.bus_current_var = tk.DoubleVar(value=62.0)
        self.phase_current_var = tk.DoubleVar(value=146.0)
        self.controller_temp_var = tk.DoubleVar(value=41.0)
        self.motor_temp_var = tk.DoubleVar(value=47.0)
        self.auto_var = tk.BooleanVar(value=False)
        self.reverse_var = tk.BooleanVar(value=False)
        self.vehicle_valid_var = tk.BooleanVar(value=True)
        self.detail_valid_var = tk.BooleanVar(value=True)
        self.part1_valid_var = tk.BooleanVar(value=True)
        self.part2_valid_var = tk.BooleanVar(value=True)
        self.estop_var = tk.BooleanVar(value=False)
        self.brake_var = tk.BooleanVar(value=False)
        self.tractive_var = tk.BooleanVar(value=True)
        self.command_received_var = tk.BooleanVar(value=False)
        self.command_timeout_var = tk.BooleanVar(value=False)
        self.drive_stop_var = tk.BooleanVar(value=False)
        self.can_lost_var = tk.BooleanVar(value=False)
        self.gnss_var = tk.StringVar(value=GNSS_STATE_NAMES[GNSS_RTK_FIXED])

    def _build_ui(self) -> None:
        tk = self.tk
        ttk = self.ttk
        outer = ttk.Frame(self.root, padding=12)
        outer.grid(row=0, column=0, sticky="nsew")
        self.root.columnconfigure(0, weight=1)
        self.root.rowconfigure(0, weight=1)

        preview_frame = ttk.Frame(outer)
        preview_frame.grid(row=0, column=0, sticky="n")
        self.preview_label = ttk.Label(preview_frame)
        self.preview_label.grid(row=0, column=0)
        self.preview_status = ttk.Label(
            preview_frame,
            text="",
            font=("TkDefaultFont", 10),
        )
        self.preview_status.grid(row=1, column=0, sticky="w", pady=(8, 0))

        switch_bar = ttk.Frame(preview_frame)
        switch_bar.grid(row=2, column=0, sticky="ew", pady=(12, 0))
        ttk.Label(switch_bar, text="Mode switch").pack(side="left", padx=(0, 16))
        switch = ttk.Frame(switch_bar)
        switch.pack(side="left")
        ttk.Radiobutton(
            switch, text="↑  AUTO", variable=self.mode_switch_var, value=True,
            command=lambda: self._set_mode(True),
        ).pack(anchor="w", pady=2)
        ttk.Radiobutton(
            switch, text="↓  MANUAL", variable=self.mode_switch_var, value=False,
            command=lambda: self._set_mode(False),
        ).pack(anchor="w", pady=2)
        ttk.Label(
            switch_bar, text="native max-speed wipe · reverse at any point\nAUTO tab: mock ROS health inputs",
            foreground=UI_MUTED,
        ).pack(side="left", padx=22)

        notebook = ttk.Notebook(outer, width=330, height=610)
        notebook.grid(row=0, column=1, sticky="ns", padx=(14, 0))
        self._build_scenario_tab(notebook)
        self._build_auto_tab(notebook)
        self._build_values_tab(notebook)
        self._build_states_tab(notebook)
        self._build_faults_tab(notebook)

    def _build_scenario_tab(self, notebook) -> None:
        ttk = self.ttk
        tab = ttk.Frame(notebook, padding=12)
        notebook.add(tab, text="Scenario")

        ttk.Label(tab, text="Automatic scenario").pack(anchor="w")
        scenario_box = ttk.Combobox(
            tab,
            textvariable=self.scenario_var,
            values=SCENARIOS,
            state="readonly",
        )
        scenario_box.pack(fill="x", pady=(4, 8))
        scenario_box.bind("<<ComboboxSelected>>", lambda _event: self._scenario_changed())

        ttk.Label(tab, text="Telemetry sample rate").pack(anchor="w")
        for rate in (10, 20):
            ttk.Radiobutton(
                tab,
                text=f"{rate} Hz",
                value=rate,
                variable=self.telemetry_var,
            ).pack(anchor="w", pady=2)

        ttk.Checkbutton(tab, text="Pause timeline", variable=self.paused_var).pack(
            anchor="w", pady=(10, 4)
        )
        ttk.Checkbutton(
            tab, text="Use manual controls", variable=self.manual_var
        ).pack(anchor="w", pady=4)
        ttk.Button(tab, text="Copy current to manual", command=self._copy_to_manual).pack(
            fill="x", pady=(10, 4)
        )
        ttk.Button(tab, text="Restart scenario", command=self._scenario_changed).pack(
            fill="x", pady=4
        )
        ttk.Button(tab, text="Replay startup", command=self._replay_startup).pack(
            fill="x", pady=4
        )
        ttk.Button(tab, text="Save 480x320 PNG", command=self._save_png).pack(
            fill="x", pady=(20, 4)
        )

        ttk.Label(
            tab,
            text=(
                "20 Hz is the intended preview target.\n"
                "10 Hz shows the current drive-detail cadence."
            ),
            foreground=UI_MUTED,
            justify="left",
        ).pack(anchor="w", pady=(18, 0))

    def _build_auto_tab(self, notebook) -> None:
        ttk = self.ttk
        tab = ttk.Frame(notebook, padding=12)
        notebook.add(tab, text="AUTO")
        ttk.Label(tab, text="AUTO health scenarios", font=("TkDefaultFont", 11, "bold")).pack(anchor="w")
        ttk.Label(
            tab, text="Simulated ROS data for design review.\nChoose a case, then try the mode switch.",
            foreground=UI_MUTED, justify="left",
        ).pack(anchor="w", pady=(6, 10))
        ttk.Button(tab, text="Cycle all cases · 4 seconds each",
                   command=lambda: self._load_auto_case("health")).pack(fill="x", pady=(0, 8))
        for case in ("healthy", "drive_feedback_lost", "drive_mismatch", "steering_trip"):
            ttk.Button(tab, text=AUTO_CASE_LABELS[case],
                       command=lambda case=case: self._load_auto_case(case)).pack(fill="x", pady=3)
        ttk.Separator(tab).pack(fill="x", pady=12)
        ttk.Label(tab, text="All health cases").pack(anchor="w")
        cases = tuple(AUTO_CASE_LABELS)
        case_box = ttk.Combobox(tab, textvariable=self.auto_case_var,
                               values=tuple(AUTO_CASE_LABELS.values()), state="readonly")
        case_box.pack(fill="x", pady=(5, 12))
        case_box.bind("<<ComboboxSelected>>", lambda _event: self._load_auto_case(cases[case_box.current()]))
        ttk.Label(
            tab,
            text=("DRIVE: command + motor feedback\n"
                  "STEER: command + position + fault\n"
                  "BRAKE CMD: STM_B output request\n\n"
                  "Blue: actual / violet: target\n"
                  "Heading: north 0°, clockwise\n"
                  "Steering motor: KEYA zero, left positive\n\n"
                  "Low-speed and trip cases are synthetic\n"
                  "examples, not bag replay or control logic."),
            foreground=UI_MUTED, justify="left", wraplength=290,
        ).pack(anchor="w", pady=6)

    def _load_auto_case(self, case: str) -> None:
        self.scenario_var.set(f"auto_{case}")
        self._scenario_changed()

    def _set_mode(self, auto: bool) -> None:
        if not self.manual_var.get():
            self._copy_to_manual()
        self.auto_var.set(auto)

    def _scale(self, parent, label, variable, minimum, maximum, resolution=1.0) -> None:
        tk = self.tk
        tk.Label(parent, text=label, anchor="w").pack(fill="x", pady=(5, 0))
        tk.Scale(
            parent,
            variable=variable,
            from_=minimum,
            to=maximum,
            resolution=resolution,
            orient="horizontal",
            length=285,
            showvalue=True,
        ).pack(fill="x")

    def _build_values_tab(self, notebook) -> None:
        ttk = self.ttk
        tab = ttk.Frame(notebook, padding=10)
        notebook.add(tab, text="Values")
        ttk.Label(
            tab,
            text="Enable 'Use manual controls' to apply these values.",
            foreground=UI_MUTED,
            wraplength=290,
        ).pack(anchor="w")
        self._scale(tab, "Pedal [%]", self.pedal_var, 0, 100)
        self._scale(tab, "Speed [km/h]", self.speed_var, -30, 90, 0.5)
        self._scale(tab, "Bus voltage [V]", self.voltage_var, 38, 60, 0.1)
        self._scale(tab, "Bus current [A]", self.bus_current_var, -120, 400, 1)
        self._scale(tab, "Phase current [A]", self.phase_current_var, -520, 520, 1)
        self._scale(tab, "Controller temp [C]", self.controller_temp_var, 20, 90, 1)
        self._scale(tab, "Motor temp [C]", self.motor_temp_var, 20, 160, 1)

    def _build_states_tab(self, notebook) -> None:
        ttk = self.ttk
        tab = ttk.Frame(notebook, padding=12)
        notebook.add(tab, text="States")
        pairs = (
            ("AUTO mode", self.auto_var),
            ("Reverse gear", self.reverse_var),
            ("Vehicle valid", self.vehicle_valid_var),
            ("Drive detail valid", self.detail_valid_var),
            ("Part I valid", self.part1_valid_var),
            ("Part II valid", self.part2_valid_var),
            ("Tractive feedback", self.tractive_var),
            ("AUTO command received", self.command_received_var),
            ("AUTO command timeout", self.command_timeout_var),
            ("Drive stop", self.drive_stop_var),
            ("Brake active", self.brake_var),
            ("E-stop active", self.estop_var),
            ("Dashboard CAN lost", self.can_lost_var),
        )
        for label, variable in pairs:
            ttk.Checkbutton(tab, text=label, variable=variable).pack(
                anchor="w", pady=4
            )
        ttk.Label(tab, text="Vehicle GNSS state (MANUAL page)").pack(anchor="w", pady=(10, 2))
        ttk.Combobox(
            tab,
            textvariable=self.gnss_var,
            values=tuple(GNSS_STATE_VALUES),
            state="readonly",
        ).pack(fill="x")

    def _build_faults_tab(self, notebook) -> None:
        tk = self.tk
        ttk = self.ttk
        tab = ttk.Frame(notebook, padding=10)
        notebook.add(tab, text="Faults")
        ttk.Label(tab, text="Motor fault bits (multi-select)").pack(anchor="w")
        list_frame = ttk.Frame(tab)
        list_frame.pack(fill="both", expand=True, pady=(4, 8))
        self.fault_list = tk.Listbox(
            list_frame,
            selectmode="multiple",
            exportselection=False,
            height=16,
        )
        scrollbar = ttk.Scrollbar(
            list_frame, orient="vertical", command=self.fault_list.yview
        )
        self.fault_list.configure(yscrollcommand=scrollbar.set)
        self.fault_list.pack(side="left", fill="both", expand=True)
        scrollbar.pack(side="right", fill="y")
        for index, name in enumerate(FAULT_NAMES):
            self.fault_list.insert("end", f"{index:02d}  {name}")

    def _scenario_changed(self) -> None:
        self.timeline_elapsed = 0.0
        self.sample = SCENARIO_BUILDERS[self.scenario_var.get()](0.0)
        self.auto_case_var.set(AUTO_CASE_LABELS.get(self.scenario_var.get().removeprefix("auto_"), ""))
        self.manual_var.set(False)

    def _register_manual_traces(self) -> None:
        variables = (
            self.manual_var, self.pedal_var, self.speed_var, self.voltage_var,
            self.bus_current_var, self.phase_current_var,
            self.controller_temp_var, self.motor_temp_var, self.auto_var,
            self.reverse_var, self.vehicle_valid_var, self.detail_valid_var,
            self.part1_valid_var, self.part2_valid_var, self.estop_var,
            self.brake_var, self.tractive_var, self.command_received_var,
            self.command_timeout_var, self.drive_stop_var, self.can_lost_var,
            self.gnss_var,
        )
        for variable in variables:
            variable.trace_add("write", self._manual_value_changed)
        self.fault_list.bind("<<ListboxSelect>>", self._manual_value_changed)

    def _manual_value_changed(self, *_args) -> None:
        if self.manual_change_pending:
            return
        self.manual_change_pending = True
        self.root.after_idle(self._record_manual_change)

    def _record_manual_change(self) -> None:
        self.manual_change_pending = False
        effective_us = self._virtual_target_us(time.monotonic_ns())
        self.manual_changes.append(
            (effective_us, self.manual_var.get(), self._manual_sample())
        )

    def _replay_startup(self) -> None:
        self.native.replay_startup()
        self.startup_active = True
        self.native_replay_requested = True

    def _present_image(self, image: Image.Image, status: str) -> None:
        present_ns = time.monotonic_ns()
        if self.last_present_ns is not None and present_ns > self.last_present_ns:
            instant_hz = 1_000_000_000 / (present_ns - self.last_present_ns)
            self.display_hz = (
                instant_hz if self.display_hz == 0.0
                else self.display_hz * 0.9 + instant_hz * 0.1
            )
        self.last_present_ns = present_ns
        self.current_image = image
        scaled = image.resize(
            (LCD_WIDTH * 2, LCD_HEIGHT * 2), Image.Resampling.NEAREST
        )
        self.photo = self.ImageTk.PhotoImage(scaled)
        self.preview_label.configure(image=self.photo)
        self.preview_status.configure(text=status)

    def _virtual_target_us(self, wall_ns: int) -> int:
        return self.virtual_origin_us + (wall_ns - self.wall_origin_ns) // 1000

    def _schedule_next_display(self) -> None:
        now_ns = time.monotonic_ns()
        while self.next_display_ns <= now_ns:
            self.next_display_ns += self.display_period_ns
        delay_ms = max(1, (self.next_display_ns - now_ns + 999_999) // 1_000_000)
        self.root.after(delay_ms, self._tick)

    def _tick(self) -> None:
        wall_ns = time.monotonic_ns()
        target_us = self._virtual_target_us(wall_ns)

        if self.startup_active:
            self.native.run_until(target_us)
            self._present_image(
                self.native.image(),
                "480x320 logical pixels | NATIVE C++/SPI STARTUP | 1.4 s",
            )
            replay_done = (
                self.native_replay_requested and not self.native.replay_active
            )
            if not self.native.ready or (
                self.native_replay_requested and not replay_done
            ):
                self._schedule_next_display()
                return
            self.startup_active = False
            if self.native_replay_requested:
                self.next_sample_us = self.native.now_us
            self.native_replay_requested = False
            if self.next_sample_us is None:
                self.next_sample_us = self.native.now_us

        period_us = round(1_000_000 / max(1, self.telemetry_var.get()))
        if self.next_sample_us is None:
            self.next_sample_us = self.native.now_us

        def sample_at(sample_us: int) -> ScenarioSample:
            while self.manual_changes and self.manual_changes[0][0] <= sample_us:
                _, self.native_manual_enabled, self.native_manual_sample = (
                    self.manual_changes.popleft()
                )
            if self.native_manual_enabled:
                self.sample = self.native_manual_sample
                return self.native_manual_sample
            sample = SCENARIO_BUILDERS[self.scenario_var.get()](
                self.timeline_elapsed
            )
            if not self.paused_var.get():
                self.timeline_elapsed += period_us / 1_000_000
            self.sample = sample
            return sample

        self.next_sample_us, self.native_alive = schedule_samples(
            self.native, self.next_sample_us, target_us, period_us,
            self.native_alive, sample_at,
        )
        self.native.run_until(target_us)

        mode_auto = self.sample.values.status(STATUS_MODE_AUTO)
        mode_valid = self.sample.values.vehicle_valid and not self.sample.dashboard_can_lost
        if mode_valid:
            self.mode_switch_var.set(mode_auto)
        image = self.native.image()
        page = "AUTO" if self.native.auto_displayed else "MANUAL"
        source = "manual" if self.manual_var.get() else self.scenario_var.get()
        self._present_image(
            image,
            (
                "480x320 logical pixels | "
                f"{page} | "
                f"{source} | {self.telemetry_var.get()} Hz data | "
                f"{self.display_hz:.1f} Hz display (60 Hz target)"
                " | NATIVE C++/SPI"
            ),
        )
        self._schedule_next_display()

    def _manual_sample(self) -> ScenarioSample:
        status_flags = 0
        drive_flags = 0
        if self.vehicle_valid_var.get():
            status_flags |= STATUS_VEHICLE_VALID
        if self.detail_valid_var.get():
            status_flags |= STATUS_DETAIL_VALID
        if self.part1_valid_var.get():
            status_flags |= STATUS_PART1_VALID
        if self.part2_valid_var.get():
            status_flags |= STATUS_PART2_VALID
        if self.auto_var.get():
            status_flags |= STATUS_MODE_AUTO
        if self.reverse_var.get():
            status_flags |= STATUS_GEAR_REVERSE
        if self.estop_var.get():
            status_flags |= STATUS_ESTOP_ACTIVE
        if self.brake_var.get():
            status_flags |= STATUS_BRAKE_ACTIVE
        if self.tractive_var.get():
            drive_flags |= DRIVE_TRACTIVE_FEEDBACK
        if self.command_received_var.get():
            drive_flags |= DRIVE_COMMAND_RECEIVED
        if self.command_timeout_var.get():
            drive_flags |= DRIVE_COMMAND_TIMEOUT
        if self.drive_stop_var.get():
            drive_flags |= DRIVE_STOP

        fault_bits = 0
        for index in self.fault_list.curselection():
            fault_bits |= 1 << index
        values = DashboardValues(
            status_flags=status_flags,
            drive_flags=drive_flags,
            pedal_percent=self.pedal_var.get(),
            gnss_state=GNSS_STATE_VALUES[self.gnss_var.get()],
            speed_centi_mps=_speed_centi_mps(self.speed_var.get()),
            bus_voltage_decivolt=round(self.voltage_var.get() * 10.0),
            bus_current_deciamp=round(self.bus_current_var.get() * 10.0),
            phase_current_deciamp=round(self.phase_current_var.get() * 10.0),
            controller_temperature_decic=round(
                self.controller_temp_var.get() * 10.0
            ),
            motor_temperature_decic=round(self.motor_temp_var.get() * 10.0),
            motor_fault_bits=fault_bits,
        )
        return ScenarioSample(values, self.can_lost_var.get(), self.manual_auto_telemetry)

    def _copy_to_manual(self) -> None:
        values = self.sample.values
        self.manual_auto_telemetry = self.sample.auto
        self.pedal_var.set(values.pedal_percent)
        self.speed_var.set(values.speed_centi_mps * 0.036)
        self.voltage_var.set(values.bus_voltage_decivolt / 10.0)
        self.bus_current_var.set(values.bus_current_deciamp / 10.0)
        self.phase_current_var.set(values.phase_current_deciamp / 10.0)
        self.controller_temp_var.set(values.controller_temperature_decic / 10.0)
        self.motor_temp_var.set(values.motor_temperature_decic / 10.0)
        self.auto_var.set(values.status(STATUS_MODE_AUTO))
        self.reverse_var.set(values.status(STATUS_GEAR_REVERSE))
        self.vehicle_valid_var.set(values.vehicle_valid)
        self.detail_valid_var.set(values.detail_valid)
        self.part1_valid_var.set(values.status(STATUS_PART1_VALID))
        self.part2_valid_var.set(values.status(STATUS_PART2_VALID))
        self.estop_var.set(values.status(STATUS_ESTOP_ACTIVE))
        self.brake_var.set(values.status(STATUS_BRAKE_ACTIVE))
        self.tractive_var.set(values.drive(DRIVE_TRACTIVE_FEEDBACK))
        self.command_received_var.set(values.drive(DRIVE_COMMAND_RECEIVED))
        self.command_timeout_var.set(values.drive(DRIVE_COMMAND_TIMEOUT))
        self.drive_stop_var.set(values.drive(DRIVE_STOP))
        self.can_lost_var.set(self.sample.dashboard_can_lost)
        self.gnss_var.set(
            GNSS_STATE_NAMES.get(values.gnss_state, "UNAVAILABLE")
        )
        self.fault_list.selection_clear(0, "end")
        for index in range(len(FAULT_NAMES)):
            if values.motor_fault_bits & (1 << index):
                self.fault_list.selection_set(index)
        self.manual_var.set(True)

    def _save_png(self) -> None:
        if self.current_image is None:
            return
        filename = self.filedialog.asksaveasfilename(
            title="Save dashboard preview",
            defaultextension=".png",
            filetypes=(("PNG image", "*.png"),),
        )
        if filename:
            self.current_image.save(filename)

    def run(self) -> None:
        self.root.mainloop()

    def _close(self) -> None:
        self.native.close()
        self.root.destroy()

def _build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Preview the UNO R4 480x320 dashboard without CAN hardware."
    )
    parser.add_argument(
        "--variant", choices=("manual_tft", "auto_gt3", "startup"),
        default="manual_tft"
    )
    parser.add_argument("--scenario", choices=SCENARIOS, default="road")
    parser.add_argument("--telemetry-hz", type=int, choices=(10, 20), default=20)
    parser.add_argument("--timing-profile", type=Path)
    parser.add_argument(
        "--snapshot",
        type=Path,
        help="Write one exact 480x320 PNG instead of opening the GUI.",
    )
    parser.add_argument(
        "--time",
        type=float,
        default=0.0,
        help="Scenario time used with --snapshot (seconds).",
    )
    return parser

def main() -> int:
    args = _build_parser().parse_args()
    if args.snapshot is not None:
        native = NativeDashboard(args.timing_profile)
        try:
            if args.variant == "startup":
                native.run_until(round(max(0.0, args.time) * 1_000_000))
            else:
                ready_us = native.run_until_ready()
                period_us = round(1_000_000 / args.telemetry_hz)
                end_us = ready_us + round(max(0.0, args.time) * 1_000_000)
                arrival_us = ready_us
                alive = 0
                initial_sample = SCENARIO_BUILDERS[args.scenario](0.0)
                if initial_sample.dashboard_can_lost:
                    native.schedule_sample(
                        arrival_us,
                        replace(initial_sample, dashboard_can_lost=False),
                        alive,
                    )
                    arrival_us += period_us
                    alive = (alive + 1) & 0xFF
                arrival_us, alive = schedule_samples(
                    native, arrival_us, end_us, period_us, alive,
                    lambda sample_us: SCENARIO_BUILDERS[args.scenario](
                        (sample_us - ready_us) / 1_000_000
                    ),
                )
                native.run_until(end_us)
            image = native.image()
            image.save(args.snapshot)
            calibration = "calibrated" if native.profile_calibrated else "unprofiled"
            print(
                f"saved {args.snapshot} ({args.variant}, {args.scenario}, "
                f"t={args.time:.3f}s, native C++/SPI, {calibration}, 480x320)"
            )
        finally:
            native.close()
        return 0
    app = PreviewApp(
        scenario=args.scenario,
        telemetry_hz=args.telemetry_hz,
        timing_profile=args.timing_profile,
    )
    app.run()
    return 0
