"""디버그용 HMI 계층.

runtime이 만든 상태 스냅샷과 raw frame 로그를 화면에 보여준다.
"""

from __future__ import annotations

import math
import queue
import time
from collections import deque

from can_io import CanFrame, ID_MOTOR_CMD, ID_STEER_CMD


class DashboardApp:
    """Tkinter 기반 CAN lab 디버그 화면."""

    PLOT_REFRESH_HZ = 20
    PLOT_WINDOW_S = 30.0
    TICK_MS = int(1000 / PLOT_REFRESH_HZ)
    LOG_MAX_LINES = 500

    def __init__(self, root, runtime) -> None:
        """창, 플롯, 로그 큐를 준비하고 주기 갱신을 시작한다."""
        import tkinter as tk
        from tkinter import messagebox, ttk

        import matplotlib

        matplotlib.use("TkAgg")
        from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
        from matplotlib.figure import Figure

        self.tk = tk
        self.ttk = ttk
        self.messagebox = messagebox
        self.Figure = Figure
        self.FigureCanvasTkAgg = FigureCanvasTkAgg

        self.root = root
        self.runtime = runtime
        self.motor_iface = runtime.motor_iface
        self.steer_iface = runtime.steer_iface

        # frame 수신 callback에서는 UI를 직접 건드리지 않고 큐에만 넣는다.
        self.frame_queue: queue.Queue[CanFrame] = queue.Queue()
        self.log_lines = deque(maxlen=self.LOG_MAX_LINES)
        self._log_line_count = 0
        # 플롯은 최근 30초만 유지하므로 deque로 충분하다.
        self.hist_t = deque()
        self.hist_rpm = deque()
        self.hist_rpm_tgt = deque()
        self.hist_steer = deque()
        self.hist_steer_tgt = deque()
        self.t0 = time.monotonic()
        self._closing = False
        self._tick_after_id: str | None = None

        self.root.title("Team KAI CAN Lab")
        self.root.geometry("1600x880")
        self.root.minsize(1360, 760)

        self.runtime.add_frame_listener(self._on_frame)
        self._build_ui()
        self._schedule_tick()
        self.root.protocol("WM_DELETE_WINDOW", self.on_close)

    def _build_ui(self) -> None:
        """상단 상태바, 좌측 시각화, 우측 도구/로그 패널을 구성한다."""
        tk = self.tk
        ttk = self.ttk

        outer = ttk.Frame(self.root, padding=8)
        outer.pack(fill="both", expand=True)
        outer.columnconfigure(0, weight=3)
        outer.columnconfigure(1, weight=2)
        outer.rowconfigure(1, weight=1)

        top = ttk.Frame(outer)
        top.grid(row=0, column=0, columnspan=2, sticky="ew", pady=(0, 8))
        top.columnconfigure(0, weight=0)
        top.columnconfigure(1, weight=0)
        top.columnconfigure(2, weight=0)
        top.columnconfigure(3, weight=1)

        self.lbl_motor_hs = ttk.Label(top, text="Motor handshake: waiting", font=("Arial", 11, "bold"))
        self.lbl_motor_hs.grid(row=0, column=0, sticky="w", padx=(0, 12))
        self.lbl_motor_rpm = ttk.Label(top, text="RPM: 0 / tgt 0", font=("Arial", 11))
        self.lbl_motor_rpm.grid(row=0, column=1, sticky="w", padx=(0, 12))
        self.lbl_steer = ttk.Label(top, text="Steer: disabled", font=("Arial", 11))
        self.lbl_steer.grid(row=0, column=2, sticky="w", padx=(0, 12))
        self.lbl_info = ttk.Label(top, text=f"Interfaces: motor={self.motor_iface}, steer={self.steer_iface}")
        self.lbl_info.grid(row=0, column=3, sticky="e")

        # 좌측은 계기와 플롯, 우측은 수동 송신과 로그를 둔다.
        visual = ttk.Frame(outer)
        visual.grid(row=1, column=0, sticky="nsew", padx=(0, 6))
        visual.columnconfigure(0, weight=1)
        visual.rowconfigure(1, weight=1)
        visual.rowconfigure(2, weight=1)

        cards = ttk.Frame(visual)
        cards.grid(row=0, column=0, sticky="ew", pady=(0, 8))
        cards.columnconfigure(0, weight=1)
        cards.columnconfigure(1, weight=1)

        # 상단 좌측 카드: 모터 계기 + 핵심 수치
        motor_card = ttk.LabelFrame(cards, text="Motor / Drive")
        motor_card.grid(row=0, column=0, sticky="nsew", padx=(0, 4))
        motor_card.columnconfigure(1, weight=1)

        self.motor_canvas = tk.Canvas(motor_card, width=210, height=170, bg="white", highlightthickness=0)
        self.motor_canvas.grid(row=0, column=0, padx=6, pady=6, sticky="nw")
        motor_metrics = ttk.Frame(motor_card)
        motor_metrics.grid(row=0, column=1, padx=6, pady=6, sticky="nsew")
        self.var_vdc = tk.StringVar(value="Bus V: 52.0 V")
        self.var_ibus = tk.StringVar(value="Bus I: 0.0 A")
        self.var_iph = tk.StringVar(value="Phase I: 0.0 A")
        self.var_temp = tk.StringVar(value="Ctrl/Motor T: 28 / 30 C")
        self.var_ctrl = tk.StringVar(value="Ctrl: RUN=0 SPEED=0")
        self.var_err = tk.StringVar(value="Err: 0x000000")
        for idx, var in enumerate([self.var_vdc, self.var_ibus, self.var_iph, self.var_temp, self.var_ctrl, self.var_err]):
            ttk.Label(motor_metrics, textvariable=var, font=("Consolas", 10)).grid(row=idx, column=0, sticky="w", pady=1)

        # 상단 우측 카드: 조향 계기 + 핵심 수치
        steer_card = ttk.LabelFrame(cards, text="Steering")
        steer_card.grid(row=0, column=1, sticky="nsew", padx=(4, 0))
        steer_card.columnconfigure(1, weight=1)

        self.steer_canvas = tk.Canvas(steer_card, width=210, height=170, bg="white", highlightthickness=0)
        self.steer_canvas.grid(row=0, column=0, padx=6, pady=6, sticky="nw")
        steer_metrics = ttk.Frame(steer_card)
        steer_metrics.grid(row=0, column=1, padx=6, pady=6, sticky="nsew")
        self.var_steer_enable = tk.StringVar(value="Enabled: False")
        self.var_handle = tk.StringVar(value="Handle: 0.0 deg / tgt 0.0 deg")
        self.var_wheel = tk.StringVar(value="Wheel: 0.0 deg / tgt 0.0 deg")
        for idx, var in enumerate([self.var_steer_enable, self.var_handle, self.var_wheel]):
            ttk.Label(steer_metrics, textvariable=var, font=("Consolas", 10)).grid(row=idx, column=0, sticky="w", pady=1)

        motor_plot_card = ttk.LabelFrame(visual, text="Motor RPM Plot")
        motor_plot_card.grid(row=1, column=0, sticky="nsew", pady=(0, 8))
        self.fig_motor = self.Figure(figsize=(7.2, 1.8), dpi=100)
        self.ax_motor = self.fig_motor.add_subplot(111)
        (self.motor_actual_line,) = self.ax_motor.plot([], [], label="actual rpm")
        (self.motor_target_line,) = self.ax_motor.plot([], [], label="target rpm")
        self.ax_motor.set_xlabel("t [s]")
        self.ax_motor.set_ylabel("rpm")
        self.ax_motor.grid(True, alpha=0.3)
        self.ax_motor.legend(loc="upper right")
        self.fig_motor.tight_layout()
        self.canvas_motor_plot = self.FigureCanvasTkAgg(self.fig_motor, master=motor_plot_card)
        self.canvas_motor_plot.get_tk_widget().pack(fill="both", expand=True, padx=6, pady=6)

        steer_plot_card = ttk.LabelFrame(visual, text="Steering Plot")
        steer_plot_card.grid(row=2, column=0, sticky="nsew")
        self.fig_steer = self.Figure(figsize=(7.2, 1.8), dpi=100)
        self.ax_steer = self.fig_steer.add_subplot(111)
        (self.steer_actual_line,) = self.ax_steer.plot([], [], label="wheel deg")
        (self.steer_target_line,) = self.ax_steer.plot([], [], label="wheel target")
        self.ax_steer.set_xlabel("t [s]")
        self.ax_steer.set_ylabel("deg")
        self.ax_steer.grid(True, alpha=0.3)
        self.ax_steer.legend(loc="upper right")
        self.fig_steer.tight_layout()
        self.canvas_steer_plot = self.FigureCanvasTkAgg(self.fig_steer, master=steer_plot_card)
        self.canvas_steer_plot.get_tk_widget().pack(fill="both", expand=True, padx=6, pady=6)

        # 우측 패널: preset, raw frame helper, CAN 로그
        tools = ttk.Frame(outer)
        tools.grid(row=1, column=1, sticky="nsew", padx=(6, 0))
        tools.columnconfigure(0, weight=1)
        tools.rowconfigure(2, weight=1)

        ctrl_card = ttk.LabelFrame(tools, text="Presets / Manual TX")
        ctrl_card.grid(row=0, column=0, sticky="ew", pady=(0, 8))
        presets = ttk.Frame(ctrl_card)
        presets.pack(fill="x", padx=6, pady=6)
        buttons = [
            ("Motor ACK", self.runtime.send_motor_ack),
            ("RPM +1000", lambda: self.runtime.send_motor_rpm(1000)),
            ("RPM 0", lambda: self.runtime.send_motor_rpm(0)),
            ("RPM -500", lambda: self.runtime.send_motor_rpm(-500)),
            ("Steer Enable", lambda: self.runtime.send_steer_enabled(True)),
            ("Steer Disable", lambda: self.runtime.send_steer_enabled(False)),
            ("Steer +10 deg", lambda: self.runtime.send_steer_wheel_deg(10.0)),
            ("Steer -10 deg", lambda: self.runtime.send_steer_wheel_deg(-10.0)),
        ]
        for idx, (label, command) in enumerate(buttons):
            ttk.Button(presets, text=label, command=command).grid(
                row=idx // 4,
                column=idx % 4,
                padx=3,
                pady=3,
                sticky="ew",
            )
            presets.columnconfigure(idx % 4, weight=1)

        form = ttk.Frame(ctrl_card)
        form.pack(fill="x", padx=6, pady=(0, 6))
        ttk.Label(form, text="Iface").grid(row=0, column=0, sticky="w")
        ttk.Label(form, text="CAN ID").grid(row=0, column=1, sticky="w")
        ttk.Label(form, text="Data (hex)").grid(row=0, column=2, sticky="w")
        self.cmb_iface = ttk.Combobox(form, values=[self.motor_iface, self.steer_iface], width=10, state="readonly")
        self.cmb_iface.set(self.motor_iface)
        self.cmb_iface.grid(row=1, column=0, padx=(0, 8), pady=2)
        self.ent_id = ttk.Entry(form, width=14)
        self.ent_id.insert(0, f"{ID_MOTOR_CMD:08X}")
        self.ent_id.grid(row=1, column=1, padx=(0, 8), pady=2)
        self.ent_data = ttk.Entry(form, width=34)
        self.ent_data.insert(0, "AAAAAAAAAAAAAAAA")
        self.ent_data.grid(row=1, column=2, padx=(0, 8), pady=2, sticky="ew")
        ttk.Button(form, text="Send", command=self.cmd_send_manual).grid(row=1, column=3, pady=2)
        form.columnconfigure(2, weight=1)

        ttk.Button(ctrl_card, text="Reset sims", command=self.cmd_reset_sims).pack(anchor="e", padx=6, pady=(0, 6))

        help_card = ttk.LabelFrame(tools, text="Useful raw frames")
        help_card.grid(row=1, column=0, sticky="ew", pady=(0, 8))
        help_text = tk.Text(help_card, height=6, wrap="word")
        help_text.pack(fill="both", expand=True, padx=6, pady=6)
        help_text.insert(
            "1.0",
            f"Motor ACK: {ID_MOTOR_CMD:08X}#AAAAAAAAAAAAAAAA\n"
            f"Steer enable: {ID_STEER_CMD:08X}#230D200100000000\n"
            f"Steer disable: {ID_STEER_CMD:08X}#230D200101000000\n"
            f"Steer +10deg wheel: {ID_STEER_CMD:08X}#23022001FFFFFB3F\n"
            f"Motor +1000rpm (500A cap): {ID_MOTOR_CMD:08X}#8890E88003000000\n"
        )
        help_text.config(state="disabled")

        log_card = ttk.LabelFrame(tools, text="CAN frame log")
        log_card.grid(row=2, column=0, sticky="nsew")
        self.txt_log = tk.Text(log_card, height=18, wrap="none", font=("Consolas", 9))
        self.txt_log.pack(fill="both", expand=True, padx=6, pady=6)

    def _on_frame(self, frame: CanFrame) -> None:
        """runtime에서 받은 raw frame을 UI 큐에 적재한다."""
        if self._closing:
            return
        self.frame_queue.put(frame)

    def _schedule_tick(self) -> None:
        """다음 UI 갱신 tick을 예약한다."""
        if self._closing:
            return
        self._tick_after_id = self.root.after(self.TICK_MS, self._tick)

    def _tick(self) -> None:
        """20 Hz 주기로 로그, 상태, 플롯을 한 번에 갱신한다."""
        if self._closing:
            return
        self._tick_after_id = None
        try:
            self._drain_frames()
            self._refresh_state()
            self._refresh_plots()
        except self.tk.TclError:
            # 종료 직전 위젯이 사라진 경우 조용히 빠진다.
            if not self._closing:
                raise
        finally:
            self._schedule_tick()

    def _drain_frames(self) -> None:
        """수신된 raw frame을 로그 텍스트 박스에 반영한다."""
        while True:
            try:
                frame = self.frame_queue.get_nowait()
            except queue.Empty:
                break
            self._append_log_line(frame.pretty())

    def _append_log_line(self, line: str) -> None:
        """로그 한 줄만 추가하고, 초과분은 앞에서 잘라낸다."""
        dropped = len(self.log_lines) == self.LOG_MAX_LINES
        self.log_lines.append(line)
        if dropped and self._log_line_count > 0:
            self.txt_log.delete("1.0", "2.0")
            self._log_line_count -= 1
        self.txt_log.insert("end", line + "\n")
        self._log_line_count += 1
        self.txt_log.see("end")

    def _refresh_state(self) -> None:
        """runtime snapshot을 읽어 라벨, 계기, 플롯 버퍼를 갱신한다."""
        motor, steer = self.runtime.get_snapshots()
        diagnostics = self.runtime.get_diagnostics()
        self.lbl_motor_hs.configure(text=f"Motor handshake: {'OK' if motor.handshake_done else 'waiting'}")
        self.lbl_motor_rpm.configure(text=f"RPM: {motor.actual_rpm:7.1f} / tgt {motor.target_rpm:7.1f}")
        self.lbl_steer.configure(text=f"Steer: {'enabled' if steer.enabled else 'disabled'}")
        if diagnostics.bus_health is None:
            self.lbl_info.configure(
                text=(
                    f"Interfaces: motor={self.motor_iface}, steer={self.steer_iface} "
                    f"| listener_err={diagnostics.listener_errors}"
                )
            )
        else:
            self.lbl_info.configure(
                text=(
                    f"Interfaces: motor={self.motor_iface}, steer={self.steer_iface} "
                    f"| tx={diagnostics.bus_health.sent_frames} "
                    f"rx={diagnostics.bus_health.received_frames} "
                    f"cb_err={diagnostics.bus_health.callback_errors} "
                    f"listener_err={diagnostics.listener_errors}"
                )
            )

        self.var_vdc.set(f"Bus V: {motor.bus_voltage_v:5.1f} V")
        self.var_ibus.set(f"Bus I: {motor.bus_current_a:6.1f} A")
        self.var_iph.set(f"Phase I: {motor.phase_current_a:6.1f} A")
        self.var_temp.set(f"Ctrl/Motor T: {motor.ctrl_temp_c:5.1f} / {motor.motor_temp_c:5.1f} C")
        self.var_ctrl.set(f"Ctrl: RUN={int(motor.ctrl_run)} SPEED={int(motor.ctrl_speed_mode)}")
        self.var_err.set(f"Err: 0x{motor.error_group:06X}")
        self.var_steer_enable.set(f"Enabled: {steer.enabled}")
        self.var_handle.set(f"Handle: {steer.handle_deg:6.1f} deg / tgt {steer.target_handle_deg:6.1f} deg")
        self.var_wheel.set(f"Wheel:  {steer.wheel_deg:6.1f} deg / tgt {steer.target_wheel_deg:6.1f} deg")

        # 플롯 시간축은 앱 시작 또는 마지막 reset 이후 경과 시간이다.
        t = time.monotonic() - self.t0
        self.hist_t.append(t)
        self.hist_rpm.append(motor.actual_rpm)
        self.hist_rpm_tgt.append(motor.target_rpm)
        self.hist_steer.append(steer.wheel_deg)
        self.hist_steer_tgt.append(steer.target_wheel_deg)
        self._trim_history(t)

        self._draw_motor_canvas(motor.actual_rpm, motor.target_rpm)
        self._draw_steer_canvas(
            steer.enabled,
            steer.handle_deg,
            steer.target_handle_deg,
            steer.wheel_deg,
            steer.target_wheel_deg,
        )

    def _trim_history(self, now_s: float) -> None:
        """최근 30초를 벗어난 샘플은 모두 버린다."""
        cutoff = max(0.0, now_s - self.PLOT_WINDOW_S)
        while self.hist_t and self.hist_t[0] < cutoff:
            self.hist_t.popleft()
            self.hist_rpm.popleft()
            self.hist_rpm_tgt.popleft()
            self.hist_steer.popleft()
            self.hist_steer_tgt.popleft()

    def _draw_motor_canvas(self, rpm: float, target: float) -> None:
        """모터 RPM 게이지를 겹침 없이 읽히는 아크 계기판으로 그린다."""
        canvas = self.motor_canvas
        canvas.delete("all")
        min_rpm = -1500.0
        max_rpm = 4500.0
        cx, cy, radius = 105, 102, 56
        start_deg = 210.0
        sweep_deg = 240.0

        canvas.create_text(cx, 16, text="Motor RPM", font=("Arial", 13, "bold"), fill="#1a202c")
        canvas.create_arc(
            cx - radius,
            cy - radius,
            cx + radius,
            cy + radius,
            start=start_deg,
            extent=-sweep_deg,
            style="arc",
            width=4,
            outline="#cbd5e0",
        )

        for tick in [-1500, 0, 1500, 3000, 4500]:
            angle = self._rpm_angle(float(tick), min_rpm, max_rpm, start_deg, sweep_deg)
            inner = radius - 7
            outer = radius + 3
            label_r = radius + 18
            x1, y1 = self._polar_to_canvas(cx, cy, inner, angle)
            x2, y2 = self._polar_to_canvas(cx, cy, outer, angle)
            xl, yl = self._polar_to_canvas(cx, cy, label_r, angle)
            canvas.create_line(x1, y1, x2, y2, width=2, fill="#4a5568")
            canvas.create_text(xl, yl, text=str(tick), font=("Arial", 7), fill="#4a5568")

        target_angle = self._rpm_angle(target, min_rpm, max_rpm, start_deg, sweep_deg)
        tx1, ty1 = self._polar_to_canvas(cx, cy, radius - 1, target_angle)
        tx2, ty2 = self._polar_to_canvas(cx, cy, radius + 8, target_angle)
        canvas.create_line(tx1, ty1, tx2, ty2, width=3, fill="#dd6b20")

        actual_angle = self._rpm_angle(rpm, min_rpm, max_rpm, start_deg, sweep_deg)
        nx, ny = self._polar_to_canvas(cx, cy, radius - 16, actual_angle)
        canvas.create_line(cx, cy, nx, ny, width=4, fill="#c53030")
        canvas.create_oval(cx - 5, cy - 5, cx + 5, cy + 5, fill="#1a202c", outline="")
        canvas.create_text(cx, cy + 24, text=f"{rpm:+,.0f} rpm", font=("Arial", 12, "bold"), fill="#1a202c")
        canvas.create_text(cx, cy + 44, text=f"target {target:+,.0f}", font=("Arial", 9), fill="#dd6b20")

    def _draw_steer_canvas(
        self,
        enabled: bool,
        handle_deg: float,
        target_handle_deg: float,
        wheel_deg: float,
        target_wheel_deg: float,
    ) -> None:
        """조향은 겹치는 그림 대신 handle/wheel 두 개의 스케일로 분리해서 보여준다."""
        canvas = self.steer_canvas
        canvas.delete("all")
        cx = 105
        canvas.create_text(cx, 16, text="Steering", font=("Arial", 13, "bold"), fill="#1a202c")
        canvas.create_text(
            cx,
            34,
            text="ENABLED" if enabled else "DISABLED",
            font=("Arial", 9, "bold"),
            fill="#2f855a" if enabled else "#c53030",
        )

        self._draw_linear_gauge(
            canvas,
            label="Handle",
            value=handle_deg,
            target=target_handle_deg,
            min_value=-180.0,
            max_value=180.0,
            y=78,
            value_color="#1f4e79",
        )
        self._draw_linear_gauge(
            canvas,
            label="Wheel",
            value=wheel_deg,
            target=target_wheel_deg,
            min_value=-40.0,
            max_value=40.0,
            y=128,
            value_color="#2f855a",
        )

    def _draw_linear_gauge(
        self,
        canvas,
        label: str,
        value: float,
        target: float,
        min_value: float,
        max_value: float,
        y: int,
        value_color: str,
    ) -> None:
        """actual/target을 한 줄 스케일 위에 같이 표시한다."""
        x0, x1 = 28, 182
        zero_x = self._scale_x(0.0, min_value, max_value, x0, x1)
        actual_x = self._scale_x(value, min_value, max_value, x0, x1)
        target_x = self._scale_x(target, min_value, max_value, x0, x1)

        canvas.create_text(x0, y - 16, text=label, anchor="w", font=("Arial", 10, "bold"), fill="#1a202c")
        canvas.create_text(
            x1,
            y - 16,
            text=f"A {value:+.1f} / T {target:+.1f}",
            anchor="e",
            font=("Arial", 8),
            fill="#4a5568",
        )
        canvas.create_line(x0, y, x1, y, width=6, fill="#e2e8f0")
        canvas.create_line(zero_x, y - 8, zero_x, y + 8, width=1, fill="#718096")
        canvas.create_line(target_x, y - 10, target_x, y + 10, width=2, fill="#dd6b20")
        canvas.create_line(actual_x, y - 12, actual_x, y + 12, width=4, fill=value_color)
        canvas.create_oval(actual_x - 4, y - 4, actual_x + 4, y + 4, fill=value_color, outline="")
        canvas.create_text(x0, y + 14, text=f"{min_value:+.0f}", anchor="w", font=("Arial", 7), fill="#718096")
        canvas.create_text(zero_x, y + 14, text="0", font=("Arial", 7), fill="#718096")
        canvas.create_text(x1, y + 14, text=f"{max_value:+.0f}", anchor="e", font=("Arial", 7), fill="#718096")

    def _refresh_plots(self) -> None:
        """최근 30초 히스토리를 motor/steering plot에 반영한다."""
        if self._closing or len(self.hist_t) < 2:
            return

        t = list(self.hist_t)
        # 시작 30초 전에는 0초부터, 그 뒤로는 항상 최근 30초 창만 보인다.
        left = max(0.0, t[-1] - self.PLOT_WINDOW_S)
        right = max(self.PLOT_WINDOW_S, t[-1]) if t[-1] < self.PLOT_WINDOW_S else t[-1]

        self.motor_actual_line.set_data(t, list(self.hist_rpm))
        self.motor_target_line.set_data(t, list(self.hist_rpm_tgt))
        self.ax_motor.set_xlim(left, right)
        self.ax_motor.relim()
        self.ax_motor.autoscale_view(scalex=False, scaley=True)
        # draw_idle로 그려서 tick callback이 오래 붙잡히지 않게 한다.
        self.canvas_motor_plot.draw_idle()

        self.steer_actual_line.set_data(t, list(self.hist_steer))
        self.steer_target_line.set_data(t, list(self.hist_steer_tgt))
        self.ax_steer.set_xlim(left, right)
        self.ax_steer.relim()
        self.ax_steer.autoscale_view(scalex=False, scaley=True)
        self.canvas_steer_plot.draw_idle()

    def cmd_send_manual(self) -> None:
        """사용자가 입력한 raw CAN frame을 그대로 송신한다."""
        iface = self.cmb_iface.get().strip()
        sid = self.ent_id.get().strip().replace("0x", "")
        sdata = self.ent_data.get().strip().replace(" ", "")
        try:
            can_id = int(sid, 16)
            data = bytes.fromhex(sdata)
            self.runtime.send_frame(CanFrame(iface, can_id, data, True, source="manual"))
        except Exception as exc:
            self.messagebox.showerror("Send failed", f"Invalid frame: {exc}")

    def cmd_reset_sims(self) -> None:
        """plant 상태와 플롯 이력을 함께 초기화한다."""
        self.runtime.reset()
        self.hist_t.clear()
        self.hist_rpm.clear()
        self.hist_rpm_tgt.clear()
        self.hist_steer.clear()
        self.hist_steer_tgt.clear()
        self.t0 = time.monotonic()
        self._append_log_line("--- simulators reset ---")

    def on_close(self) -> None:
        """Tk 창만 닫고, 실제 runtime 정리는 launcher가 맡는다."""
        self.request_close()

    def request_close(self) -> None:
        """주기 갱신을 멈추고 mainloop를 정상적으로 빠져나오게 한다."""
        if self._closing:
            return
        self._closing = True
        if self._tick_after_id is not None:
            try:
                self.root.after_cancel(self._tick_after_id)
            except self.tk.TclError:
                pass
            self._tick_after_id = None
        try:
            self.root.quit()
        except self.tk.TclError:
            pass

    @staticmethod
    def _rpm_angle(value: float, min_rpm: float, max_rpm: float, start_deg: float, sweep_deg: float) -> float:
        """RPM 값을 아크 계기판의 각도로 바꾼다."""
        clamped = max(min_rpm, min(max_rpm, value))
        ratio = (clamped - min_rpm) / (max_rpm - min_rpm)
        return start_deg - sweep_deg * ratio

    @staticmethod
    def _polar_to_canvas(cx: float, cy: float, radius: float, angle_deg: float) -> tuple[float, float]:
        """극좌표를 Tk canvas 좌표로 바꾼다."""
        radians = math.radians(angle_deg)
        return cx + radius * math.cos(radians), cy - radius * math.sin(radians)

    @staticmethod
    def _scale_x(value: float, min_value: float, max_value: float, x0: float, x1: float) -> float:
        """실수 값을 선형 스케일 위 x 좌표로 바꾼다."""
        clamped = max(min_value, min(max_value, value))
        ratio = (clamped - min_value) / (max_value - min_value)
        return x0 + (x1 - x0) * ratio
