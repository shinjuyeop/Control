from __future__ import annotations

from PIL import ImageDraw

from ..model import *
from ..palette import *


class AnalysisVariants:

    def _draw_history_scope(
        self,
        draw: ImageDraw.ImageDraw,
        sample: ScenarioSample,
        dynamics: GaugeDynamics,
        policy: PreviewPolicy,
        now: float,
    ) -> None:
        values = sample.values
        tokens = self._status_tokens(values)
        self._fill_screen(draw, "#05090C")
        self._inline_tag(draw, (10, 7, 98, 27), tokens["link"])
        self._inline_tag(draw, (106, 7, 232, 27), tokens["tractive"])
        self._inline_tag(draw, (382, 7, 470, 27), tokens["safety"])
        current_box = (12, 36, 468, 148)
        thermal_box = (12, 158, 468, 242)
        self._scope_grid(draw, current_box)
        self._scope_grid(draw, thermal_box)
        self._scope_trace(
            draw, (12, 74, 468, 148), "bus_current_a", BUS_10_SECOND_A,
            DarkColor.YELLOW, "power_valid",
        )
        self._scope_trace(
            draw, (12, 74, 468, 148), "phase_current_a", PHASE_RATED_A,
            DarkColor.RED, "power_valid",
        )
        motor_scale = policy.motor_critical_c if policy.motor_bands_enabled else 150.0
        self._scope_trace(
            draw, (12, 196, 468, 242), "motor_temp_c", motor_scale,
            Color.BLUE,
            "thermal_valid",
        )
        self._scope_trace(
            draw, (12, 196, 468, 242), "controller_temp_c", CONTROLLER_OVERHEAT_C,
            Color.ORANGE,
            "thermal_valid",
        )
        draw.text(
            (20, 47), "5 s LOAD HISTORY", font=self.fonts.bold(8),
            fill=DarkColor.MUTED, anchor="la",
        )
        draw.text(
            (20, 68), "BUS", font=self.fonts.bold(9),
            fill=DarkColor.YELLOW, anchor="la",
        )
        draw.text(
            (78, 68),
            self._current_text(values.bus_current_deciamp, values.power_valid),
            font=self.fonts.bold(12), fill=DarkColor.TEXT, anchor="la",
        )
        draw.text(
            (340, 68), "PHS", font=self.fonts.bold(9),
            fill=DarkColor.RED, anchor="la",
        )
        draw.text(
            (458, 68),
            self._current_text(values.phase_current_deciamp, values.power_valid),
            font=self.fonts.bold(12), fill=DarkColor.TEXT, anchor="ra",
        )
        draw.text(
            (20, 170), "THERMAL HISTORY", font=self.fonts.bold(8),
            fill=DarkColor.MUTED, anchor="la",
        )
        draw.text(
            (20, 190), "MOTOR", font=self.fonts.bold(9),
            fill=Color.BLUE, anchor="la",
        )
        draw.text(
            (92, 190),
            self._temp_text(values.motor_temperature_decic, values.thermal_valid),
            font=self.fonts.bold(12), fill=DarkColor.TEXT, anchor="la",
        )
        draw.text(
            (340, 190), "CTRL", font=self.fonts.bold(9),
            fill=Color.ORANGE, anchor="la",
        )
        draw.text(
            (458, 190),
            self._temp_text(values.controller_temperature_decic, values.thermal_valid),
            font=self.fonts.bold(12), fill=DarkColor.TEXT, anchor="ra",
        )
        self._inline_tag(draw, (12, 252, 100, 276), tokens["mode"])
        self._inline_tag(draw, (106, 252, 164, 276), tokens["gear"])
        draw.text(
            (178, 264), self._speed_text(values), font=self.fonts.bold(20),
            fill=DarkColor.TEXT, anchor="lm",
        )
        draw.text(
            (252, 264), "KM/H", font=self.fonts.bold(8),
            fill=DarkColor.MUTED, anchor="lm",
        )
        draw.text(
            (300, 264), self._voltage_text(values), font=self.fonts.bold(14),
            fill=Color.BLUE, anchor="lm",
        )
        draw.text(
            (458, 264), f"PEDAL {values.pedal_percent}%",
            font=self.fonts.bold(12), fill=DarkColor.YELLOW, anchor="rm",
        )
        left, right, color = self._status_message(sample, now)
        if left:
            draw.rectangle((150, 287, 330, 316), fill=color)
            draw.text(
                (240, 301), left, font=self.fonts.bold(10),
                fill="#FFFFFF", anchor="mm",
            )

    def _draw_load_envelope(
        self,
        draw: ImageDraw.ImageDraw,
        sample: ScenarioSample,
        dynamics: GaugeDynamics,
        policy: PreviewPolicy,
        now: float,
    ) -> None:
        values = sample.values
        tokens = self._status_tokens(values)
        self._fill_screen(draw, "#F7F9F5")
        plot = (30, 24, 326, 286)
        draw.rectangle(plot, fill="#FFFFFF", outline="#536165", width=2)
        for bus_level in (BUS_CONTINUOUS_A, BUS_30_SECOND_A):
            x = plot[0] + (plot[2] - plot[0]) * bus_level / BUS_10_SECOND_A
            draw.line((x, plot[1], x, plot[3]), fill="#D3DAD8", width=1)
        for phase_level in (PHASE_RATED_A * 0.60, PHASE_RATED_A * 0.80):
            y = plot[3] - (plot[3] - plot[1]) * phase_level / PHASE_RATED_A
            draw.line((plot[0], y, plot[2], y), fill="#D3DAD8", width=1)
        draw.text(
            (plot[0], 10), "PHASE / BUS OPERATING POINT",
            font=self.fonts.bold(9), fill=Color.MUTED, anchor="la",
        )
        draw.text(
            (plot[0] - 7, plot[1]), "480", font=self.fonts.bold(7),
            fill=Color.MUTED, anchor="ra",
        )
        draw.text(
            (plot[2], plot[3] + 8), "350 A BUS", font=self.fonts.bold(7),
            fill=Color.MUTED, anchor="ra",
        )
        self._envelope_trail(draw, plot)
        bus = max(0.0, values.bus_current_deciamp / 10.0)
        phase = abs(values.phase_current_deciamp) / 10.0
        point_x = plot[0] + (plot[2] - plot[0]) * min(1.0, bus / BUS_10_SECOND_A)
        point_y = plot[3] - (plot[3] - plot[1]) * min(1.0, phase / PHASE_RATED_A)
        radius = 5 + round(values.pedal_percent * 0.07)
        point_color = self._phase_load_color(phase)
        if values.power_valid:
            draw.ellipse(
                (point_x - radius, point_y - radius, point_x + radius, point_y + radius),
                fill=point_color, outline=Color.TEXT, width=2,
            )
            label_on_right = point_x < plot[2] - 76
            draw.text(
                (point_x + 10 if label_on_right else point_x - 10, point_y - 10),
                f"B{bus:.0f} P{phase:.0f}",
                font=self.fonts.bold(9),
                fill=Color.TEXT,
                anchor="la" if label_on_right else "ra",
            )

        self._inline_tag(draw, (342, 8, 402, 32), tokens["gear"])
        self._inline_tag(draw, (408, 8, 472, 32), tokens["mode"])
        draw.text(
            (407, 64), self._speed_text(values), font=self.fonts.bold(34),
            fill=Color.TEXT if values.vehicle_valid else Color.MUTED,
            anchor="mm",
        )
        draw.text(
            (407, 89), "KM/H", font=self.fonts.bold(8),
            fill=Color.MUTED, anchor="mm",
        )
        self._envelope_metric(draw, 105, "HV", self._voltage_text(values), Color.BLUE)
        self._envelope_metric(
            draw, 143, "MOTOR",
            self._temp_text(values.motor_temperature_decic, values.thermal_valid),
            self._motor_temp_color(
                values.motor_temperature_decic / 10.0,
                bool(values.motor_fault_bits & (1 << 5)), policy,
            ),
        )
        self._envelope_metric(
            draw, 181, "CTRL",
            self._temp_text(values.controller_temperature_decic, values.thermal_valid),
            self._controller_temp_color(
                values.controller_temperature_decic / 10.0,
                bool(values.motor_fault_bits & (1 << 4)),
            ),
        )
        draw.text(
            (344, 228), "PEDAL", font=self.fonts.bold(8),
            fill=Color.MUTED, anchor="la",
        )
        draw.text(
            (470, 228), f"{values.pedal_percent}%",
            font=self.fonts.bold(15), fill=Color.MANUAL, anchor="ra",
        )
        self._inline_tag(draw, (342, 246, 406, 270), tokens["link"])
        self._inline_tag(draw, (410, 246, 472, 270), tokens["safety"])
        self._inline_tag(draw, (342, 278, 472, 310), tokens["tractive"])
        left, _, color = self._status_message(sample, now)
        if left:
            draw.rectangle((342, 278, 472, 319), fill=color)
            draw.text(
                (407, 298), left, font=self.fonts.bold(9),
                fill="#FFFFFF", anchor="mm",
            )




    def _scope_grid(
        self,
        draw: ImageDraw.ImageDraw,
        box: tuple[int, int, int, int],
    ) -> None:
        draw.rectangle(box, outline=DarkColor.GRID, width=1)
        x0, y0, x1, y1 = box
        for index in range(1, 5):
            x = x0 + (x1 - x0) * index / 5
            draw.line((x, y0, x, y1), fill="#162126", width=1)
        for index in range(1, 4):
            y = y0 + (y1 - y0) * index / 4
            draw.line((x0, y, x1, y), fill="#162126", width=1)

    def _scope_trace(
        self,
        draw: ImageDraw.ImageDraw,
        box: tuple[int, int, int, int],
        field: str,
        maximum: float,
        color: str,
        valid_field: str,
    ) -> None:
        if len(self.history) < 2:
            return
        end_time = self.history[-1].time_s
        start_time = end_time - 5.0
        x0, y0, x1, y1 = box
        points = []
        for point in self.history:
            if point.time_s < start_time or not getattr(point, valid_field):
                continue
            value = abs(float(getattr(point, field)))
            x = x0 + (x1 - x0) * (point.time_s - start_time) / 5.0
            y = y1 - (y1 - y0) * min(1.0, value / maximum)
            points.append((x, y))
        if len(points) >= 2:
            draw.line(points, fill=color, width=2)

    def _envelope_trail(
        self,
        draw: ImageDraw.ImageDraw,
        box: tuple[int, int, int, int],
    ) -> None:
        points = []
        x0, y0, x1, y1 = box
        for point in self.history:
            if not point.power_valid:
                continue
            x = x0 + (x1 - x0) * min(
                1.0, max(0.0, point.bus_current_a) / BUS_10_SECOND_A
            )
            y = y1 - (y1 - y0) * min(
                1.0, abs(point.phase_current_a) / PHASE_RATED_A
            )
            points.append((x, y))
        if len(points) >= 2:
            draw.line(points, fill="#6F9890", width=2)
        for point in points[-12:]:
            draw.ellipse(
                (point[0] - 1, point[1] - 1, point[0] + 1, point[1] + 1),
                fill="#6F9890",
            )

    def _envelope_metric(
        self,
        draw: ImageDraw.ImageDraw,
        y: int,
        label: str,
        reading: str,
        color: str,
    ) -> None:
        draw.text(
            (344, y), label, font=self.fonts.bold(8),
            fill=Color.MUTED, anchor="la",
        )
        draw.text(
            (470, y + 20), reading, font=self.fonts.bold(14),
            fill=color, anchor="ra",
        )
        draw.line((344, y + 31, 470, y + 31), fill="#B5BEBC", width=1)
