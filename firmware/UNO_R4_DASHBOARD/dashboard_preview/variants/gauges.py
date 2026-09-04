from __future__ import annotations

from PIL import ImageDraw

from ..model import *
from ..palette import *


class GaugeVariants:
    def _draw_dual_dial(
        self,
        draw: ImageDraw.ImageDraw,
        sample: ScenarioSample,
        dynamics: GaugeDynamics,
        policy: PreviewPolicy,
        now: float,
    ) -> None:
        values = sample.values
        self._fill_screen(draw, "#05090B")
        self._race_header(draw, values, dark=True)

        self._arc_gauge(
            draw,
            (40, 68, 210, 238),
            dynamics.bus_a,
            dynamics.bus_peak_a,
            BUS_10_SECOND_A,
            self._bus_load_color(values.bus_current_deciamp / 10.0),
            values.power_valid,
        )
        self._arc_gauge(
            draw,
            (270, 68, 440, 238),
            dynamics.phase_a,
            dynamics.phase_peak_a,
            PHASE_RATED_A,
            self._phase_load_color(dynamics.phase_a),
            values.power_valid,
        )
        draw.text(
            (125, 91),
            "BATTERY LOAD",
            font=self.fonts.bold(10),
            fill=DarkColor.MUTED,
            anchor="mm",
        )
        draw.text(
            (125, 153),
            self._current_text(values.bus_current_deciamp, values.power_valid),
            font=self.fonts.bold(25),
            fill=DarkColor.TEXT if values.power_valid else DarkColor.MUTED,
            anchor="mm",
        )
        draw.text(
            (125, 190),
            self._voltage_text(values),
            font=self.fonts.bold(15),
            fill=DarkColor.CYAN if values.power_valid else DarkColor.MUTED,
            anchor="mm",
        )
        draw.text(
            (355, 91),
            "MOTOR LOAD",
            font=self.fonts.bold(10),
            fill=DarkColor.MUTED,
            anchor="mm",
        )
        draw.text(
            (355, 153),
            self._current_text(values.phase_current_deciamp, values.power_valid),
            font=self.fonts.bold(25),
            fill=DarkColor.TEXT if values.power_valid else DarkColor.MUTED,
            anchor="mm",
        )
        motor_color = self._motor_temp_color(
            values.motor_temperature_decic / 10.0,
            bool(values.motor_fault_bits & (1 << 5)),
            policy,
        ) if values.thermal_valid else DarkColor.MUTED
        draw.text(
            (355, 190),
            self._temp_text(values.motor_temperature_decic, values.thermal_valid),
            font=self.fonts.bold(15),
            fill=motor_color,
            anchor="mm",
        )

        draw.text(
            (240, 92),
            "SPEED",
            font=self.fonts.bold(9),
            fill=DarkColor.MUTED,
            anchor="mm",
        )
        draw.text(
            (240, 127),
            self._speed_text(values),
            font=self.fonts.bold(28),
            fill=DarkColor.TEXT if values.vehicle_valid else DarkColor.MUTED,
            anchor="mm",
        )
        draw.text(
            (240, 150),
            "KM/H",
            font=self.fonts.bold(8),
            fill=DarkColor.MUTED,
            anchor="mm",
        )
        ctrl_color = self._controller_temp_color(
            values.controller_temperature_decic / 10.0,
            bool(values.motor_fault_bits & (1 << 4)),
        ) if values.thermal_valid else DarkColor.MUTED
        draw.text(
            (240, 196),
            "CTRL",
            font=self.fonts.bold(8),
            fill=DarkColor.MUTED,
            anchor="mm",
        )
        draw.text(
            (240, 215),
            self._temp_text(
                values.controller_temperature_decic, values.thermal_valid
            ),
            font=self.fonts.bold(13),
            fill=ctrl_color,
            anchor="mm",
        )

        pedal = values.pedal_percent if values.detail_valid else 0
        draw.text(
            (24, 260),
            "PEDAL",
            font=self.fonts.bold(9),
            fill=DarkColor.MUTED,
            anchor="lm",
        )
        self._horizontal_bar(
            draw,
            (76, 256, 424, 265),
            pedal,
            100.0,
            DarkColor.YELLOW,
            values.detail_valid,
            inactive=DarkColor.INACTIVE,
        )
        draw.text(
            (456, 260),
            f"{pedal}%" if values.detail_valid else "--%",
            font=self.fonts.bold(12),
            fill=DarkColor.TEXT if values.detail_valid else DarkColor.MUTED,
            anchor="rm",
        )
        self._alert_strip(draw, sample, now, dark=True)




    def _draw_tape_gauges(
        self,
        draw: ImageDraw.ImageDraw,
        sample: ScenarioSample,
        dynamics: GaugeDynamics,
        policy: PreviewPolicy,
        now: float,
    ) -> None:
        values = sample.values
        self._fill_screen(draw, "#F7F9F5")
        self._race_header(draw, values, dark=False)
        draw.rectangle((8, 37, 472, 278), outline="#899598", width=2)
        self._vertical_tape(
            draw, (12, 41, 105, 274), dynamics.bus_a, BUS_10_SECOND_A,
            self._bus_load_color(values.bus_current_deciamp / 10.0),
            values.power_valid, "BUS",
            self._current_text(values.bus_current_deciamp, values.power_valid),
            right=False,
        )
        self._vertical_tape(
            draw, (375, 41, 468, 274), dynamics.phase_a, PHASE_RATED_A,
            self._phase_load_color(dynamics.phase_a), values.power_valid,
            "PHASE",
            self._current_text(values.phase_current_deciamp, values.power_valid),
            right=True,
        )
        draw.line((112, 41, 112, 274), fill="#A1ACAE", width=1)
        draw.line((368, 41, 368, 274), fill="#A1ACAE", width=1)

        draw.text(
            (240, 58), "SPEED", font=self.fonts.bold(9),
            fill=Color.MUTED, anchor="mm",
        )
        draw.text(
            (240, 105), self._speed_text(values), font=self.fonts.bold(46),
            fill=Color.TEXT if values.vehicle_valid else Color.MUTED,
            anchor="mm",
        )
        draw.text(
            (240, 137), "KM/H", font=self.fonts.bold(9),
            fill=Color.MUTED, anchor="mm",
        )
        draw.line((126, 151, 354, 151), fill="#A1ACAE", width=1)
        draw.text(
            (128, 170), "HV BUS", font=self.fonts.bold(9),
            fill=Color.MUTED, anchor="la",
        )
        draw.text(
            (352, 170), self._voltage_text(values), font=self.fonts.bold(18),
            fill=self._voltage_color(values.bus_voltage_decivolt / 10.0)
            if values.power_valid else Color.MUTED,
            anchor="ra",
        )
        draw.text(
            (128, 202), "MOTOR", font=self.fonts.bold(9),
            fill=Color.MUTED, anchor="la",
        )
        draw.text(
            (230, 202),
            self._temp_text(values.motor_temperature_decic, values.thermal_valid),
            font=self.fonts.bold(15),
            fill=self._motor_temp_color(
                values.motor_temperature_decic / 10.0,
                bool(values.motor_fault_bits & (1 << 5)), policy,
            ) if values.thermal_valid else Color.MUTED,
            anchor="ra",
        )
        draw.text(
            (250, 202), "CTRL", font=self.fonts.bold(9),
            fill=Color.MUTED, anchor="la",
        )
        draw.text(
            (352, 202),
            self._temp_text(values.controller_temperature_decic, values.thermal_valid),
            font=self.fonts.bold(15),
            fill=self._controller_temp_color(
                values.controller_temperature_decic / 10.0,
                bool(values.motor_fault_bits & (1 << 4)),
            ) if values.thermal_valid else Color.MUTED,
            anchor="ra",
        )
        pedal = values.pedal_percent if values.detail_valid else 0
        draw.text(
            (128, 246), "PEDAL", font=self.fonts.bold(9),
            fill=Color.MUTED, anchor="lm",
        )
        self._horizontal_bar(
            draw, (181, 241, 316, 251), pedal, 100.0, Color.MANUAL,
            values.detail_valid,
        )
        draw.text(
            (352, 246), f"{pedal}%" if values.detail_valid else "--%",
            font=self.fonts.bold(13), fill=Color.TEXT, anchor="rm",
        )
        self._alert_strip(draw, sample, now, dark=False)
