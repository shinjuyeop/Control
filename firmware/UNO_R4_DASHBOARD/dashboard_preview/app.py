"""Tk GUI and snapshot CLI for the dashboard preview."""

from __future__ import annotations

import argparse
from pathlib import Path
import time

from PIL import Image

from .model import *
from .palette import Color
from .registry import MODE_PAGE_VARIANTS, VARIANTS
from .renderer import DashboardRenderer
from .startup import STARTUP_DURATION_S, StartupRenderer


class PreviewApp:
    def __init__(
        self, *, variant: str, scenario: str, telemetry_hz: int
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
        self.root.configure(bg=Color.BG)

        self.renderer = DashboardRenderer()
        self.startup_renderer = StartupRenderer()
        self.dynamics = GaugeDynamics()
        self.policy = PreviewPolicy()
        self.variant_var = tk.StringVar(value=variant)
        self.scenario_var = tk.StringVar(value=scenario)
        self.telemetry_var = tk.IntVar(value=telemetry_hz)
        self.paused_var = tk.BooleanVar(value=False)
        self.manual_var = tk.BooleanVar(value=False)
        self.motor_bands_var = tk.BooleanVar(value=False)
        self.motor_warn_var = tk.DoubleVar(value=100.0)
        self.motor_critical_var = tk.DoubleVar(value=130.0)

        self.timeline_elapsed = 0.0
        self.last_tick = time.monotonic()
        self.last_sample_at = -1.0
        self.sample = SCENARIO_BUILDERS[scenario](0.0)
        self.last_mode_auto: bool | None = None
        self.dynamics.snap(self.sample.values, self.last_tick)
        self.current_image: Image.Image | None = None
        self.photo = None
        self.startup_active = True
        self.startup_started_at = self.last_tick

        self._make_manual_variables()
        self._build_ui()
        self._scenario_changed()
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

        notebook = ttk.Notebook(outer, width=330, height=610)
        notebook.grid(row=0, column=1, sticky="ns", padx=(14, 0))
        self._build_scenario_tab(notebook)
        self._build_values_tab(notebook)
        self._build_states_tab(notebook)
        self._build_faults_tab(notebook)

    def _build_scenario_tab(self, notebook) -> None:
        ttk = self.ttk
        tab = ttk.Frame(notebook, padding=12)
        notebook.add(tab, text="Scenario")

        ttk.Label(tab, text="Display variant").pack(anchor="w")
        variant_frame = ttk.Frame(tab)
        variant_frame.pack(fill="x", pady=(4, 0))
        variant_box = ttk.Combobox(
            variant_frame,
            textvariable=self.variant_var,
            values=VARIANTS,
            state="readonly",
        )
        variant_box.pack(fill="x")
        variant_steps = ttk.Frame(variant_frame)
        variant_steps.pack(fill="x", pady=(5, 0))
        ttk.Button(
            variant_steps,
            text="Previous",
            command=lambda: self._step_variant(-1),
        ).pack(side="left", fill="x", expand=True, padx=(0, 3))
        ttk.Button(
            variant_steps,
            text="Next",
            command=lambda: self._step_variant(1),
        ).pack(side="left", fill="x", expand=True, padx=(3, 0))

        ttk.Separator(tab).pack(fill="x", pady=10)
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
            foreground=Color.MUTED,
            justify="left",
        ).pack(anchor="w", pady=(18, 0))

    def _step_variant(self, delta: int) -> None:
        current = self.variant_var.get()
        index = VARIANTS.index(current) if current in VARIANTS else 0
        self.variant_var.set(VARIANTS[(index + delta) % len(VARIANTS)])

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
            foreground=Color.MUTED,
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

        ttk.Separator(tab).pack(fill="x", pady=8)
        ttk.Checkbutton(
            tab,
            text="Enable preview-only motor temp bands",
            variable=self.motor_bands_var,
        ).pack(anchor="w")
        self._scale(tab, "Motor warning [C]", self.motor_warn_var, 50, 140, 1)
        self._scale(tab, "Motor critical [C]", self.motor_critical_var, 60, 170, 1)
        ttk.Label(
            tab,
            text="These motor temperature numbers are not vehicle authority.",
            foreground=Color.FAULT,
            wraplength=290,
        ).pack(anchor="w", pady=(8, 0))

    def _scenario_changed(self) -> None:
        self.timeline_elapsed = 0.0
        self.last_sample_at = -1.0
        self.last_mode_auto = None
        self.renderer.clear_history()
        self.manual_var.set(False)

    def _elapsed(self, dt: float) -> float:
        if not self.paused_var.get():
            self.timeline_elapsed += dt
        return self.timeline_elapsed

    def _replay_startup(self) -> None:
        self.startup_active = True
        self.startup_started_at = time.monotonic()

    def _present_image(self, image: Image.Image, status: str) -> None:
        self.current_image = image
        scaled = image.resize(
            (LCD_WIDTH * 2, LCD_HEIGHT * 2), Image.Resampling.NEAREST
        )
        self.photo = self.ImageTk.PhotoImage(scaled)
        self.preview_label.configure(image=self.photo)
        self.preview_status.configure(text=status)

    def _tick(self) -> None:
        now = time.monotonic()
        dt = min(0.1, max(0.0, now - self.last_tick))
        self.last_tick = now

        if self.startup_active:
            startup_elapsed = now - self.startup_started_at
            if startup_elapsed < STARTUP_DURATION_S:
                self._present_image(
                    self.startup_renderer.render(startup_elapsed),
                    "480x320 logical pixels | KAI STARTUP | 1.4 s",
                )
                self.root.after(33, self._tick)
                return
            self.startup_active = False
            self.last_tick = now
            dt = 0.0

        elapsed = self._elapsed(dt)
        sample_period = 1.0 / max(1, self.telemetry_var.get())
        if not self.manual_var.get() and (
            self.last_sample_at < 0.0 or elapsed - self.last_sample_at >= sample_period
        ):
            self.sample = SCENARIO_BUILDERS[self.scenario_var.get()](elapsed)
            self.last_sample_at = elapsed
        elif self.manual_var.get():
            self.sample = self._manual_sample()

        self.policy = PreviewPolicy(
            motor_bands_enabled=self.motor_bands_var.get(),
            motor_warning_c=self.motor_warn_var.get(),
            motor_critical_c=max(
                self.motor_warn_var.get() + 1.0, self.motor_critical_var.get()
            ),
        )
        self.dynamics.update(self.sample.values, dt, now)
        selected_variant = self.variant_var.get()
        mode_auto = self.sample.values.status(STATUS_MODE_AUTO)
        if (
            selected_variant in MODE_PAGE_VARIANTS
            and self.last_mode_auto is not None
            and mode_auto != self.last_mode_auto
        ):
            selected_variant = "auto_gt3" if mode_auto else "manual_tft"
            self.variant_var.set(selected_variant)
        self.last_mode_auto = mode_auto
        image = self.renderer.render(
            self.sample,
            selected_variant,
            self.dynamics,
            self.policy,
            now,
        )
        source = "manual" if self.manual_var.get() else self.scenario_var.get()
        self._present_image(
            image,
            (
                "480x320 logical pixels | "
                f"{selected_variant.replace('_', ' ').upper()} | "
                f"{source} | {self.telemetry_var.get()} Hz data / 30 Hz display"
            ),
        )
        self.root.after(33, self._tick)

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
        return ScenarioSample(values, self.can_lost_var.get())

    def _copy_to_manual(self) -> None:
        values = self.sample.values
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

def _build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Preview the UNO R4 480x320 dashboard without CAN hardware."
    )
    parser.add_argument("--variant", choices=VARIANTS, default="manual_tft")
    parser.add_argument("--scenario", choices=SCENARIOS, default="road")
    parser.add_argument("--telemetry-hz", type=int, choices=(10, 20), default=20)
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
        sample = SCENARIO_BUILDERS[args.scenario](max(0.0, args.time))
        dynamics = GaugeDynamics()
        dynamics.snap(sample.values, args.time)
        renderer = DashboardRenderer()
        renderer.seed_history(
            SCENARIO_BUILDERS[args.scenario], max(0.0, args.time)
        )
        image = renderer.render(
            sample,
            args.variant,
            dynamics,
            PreviewPolicy(),
            args.time,
        )
        image.save(args.snapshot)
        print(
            f"saved {args.snapshot} "
            f"({args.variant}, {args.scenario}, t={args.time:.1f}s, 480x320)"
        )
        return 0

    app = PreviewApp(
        variant=args.variant,
        scenario=args.scenario,
        telemetry_hz=args.telemetry_hz,
    )
    app.run()
    return 0
