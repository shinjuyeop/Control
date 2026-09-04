from __future__ import annotations

from PIL import ImageDraw

from ..model import *
from ..palette import *


class FlowVariants:


    def _draw_gt3_vitals(
        self,
        draw: ImageDraw.ImageDraw,
        sample: ScenarioSample,
        dynamics: GaugeDynamics,
        policy: PreviewPolicy,
        now: float,
    ) -> None:
        values = sample.values
        self._fill_screen(draw, "#090D10")
        self._race_header(draw, values, dark=True)
        grid = "#3A454B"
        draw.rectangle((8, 38, 472, 278), outline=grid, width=2)
        draw.line((160, 38, 160, 278), fill=grid, width=2)
        draw.line((320, 38, 320, 278), fill=grid, width=2)
        for y in (118, 198):
            draw.line((8, y, 160, y), fill=grid, width=1)
            draw.line((320, y, 472, y), fill=grid, width=1)

        self._vital_cell(
            draw,
            (10, 40, 158, 116),
            "HV BUS",
            self._voltage_text(values),
            self._voltage_color(values.bus_voltage_decivolt / 10.0),
            values.power_valid,
            left_bar=True,
        )
        self._vital_cell(
            draw,
            (10, 120, 158, 196),
            "BUS CURRENT",
            self._current_text(values.bus_current_deciamp, values.power_valid),
            self._bus_load_color(values.bus_current_deciamp / 10.0),
            values.power_valid,
            left_bar=True,
        )
        self._vital_cell(
            draw,
            (10, 200, 158, 276),
            "PEDAL",
            f"{values.pedal_percent}%" if values.detail_valid else "--%",
            DarkColor.YELLOW,
            values.detail_valid,
            left_bar=True,
        )

        speed = self._speed_text(values)
        draw.text(
            (240, 69),
            "SPEED",
            font=self.fonts.bold(11),
            fill=DarkColor.MUTED,
            anchor="mm",
        )
        draw.text(
            (240, 147),
            speed,
            font=self.fonts.bold(65),
            fill=DarkColor.TEXT if values.vehicle_valid else DarkColor.MUTED,
            anchor="mm",
        )
        draw.text(
            (240, 195),
            "KM/H",
            font=self.fonts.bold(11),
            fill=DarkColor.MUTED,
            anchor="mm",
        )
        self._vital_cell(
            draw,
            (322, 40, 470, 116),
            "PHASE CURRENT",
            self._current_text(values.phase_current_deciamp, values.power_valid),
            self._phase_load_color(dynamics.phase_a),
            values.power_valid,
            left_bar=False,
        )
        self._vital_cell(
            draw,
            (322, 120, 470, 196),
            "MOTOR TEMP",
            self._temp_text(values.motor_temperature_decic, values.thermal_valid),
            self._motor_temp_color(
                values.motor_temperature_decic / 10.0,
                bool(values.motor_fault_bits & (1 << 5)),
                policy,
            ),
            values.thermal_valid,
            left_bar=False,
        )
        self._vital_cell(
            draw,
            (322, 200, 470, 276),
            "CTRL TEMP",
            self._temp_text(
                values.controller_temperature_decic, values.thermal_valid
            ),
            self._controller_temp_color(
                values.controller_temperature_decic / 10.0,
                bool(values.motor_fault_bits & (1 << 4)),
            ),
            values.thermal_valid,
            left_bar=False,
        )
        self._alert_strip(draw, sample, now, dark=True)

    def _draw_cup_light(
        self,
        draw: ImageDraw.ImageDraw,
        sample: ScenarioSample,
        dynamics: GaugeDynamics,
        policy: PreviewPolicy,
        now: float,
    ) -> None:
        values = sample.values
        self._fill_screen(draw, "#FCFCF8")
        self._race_header(draw, values, dark=False)
        grid = "#77858B"
        draw.rectangle((8, 40, 472, 278), outline=grid, width=2)
        draw.line((172, 40, 172, 218), fill=grid, width=2)
        draw.line((308, 40, 308, 218), fill=grid, width=2)
        draw.line((8, 218, 472, 218), fill=grid, width=2)
        draw.line((240, 218, 240, 278), fill=grid, width=2)

        bus_value = values.bus_current_deciamp / 10.0
        self._vertical_bar(
            draw,
            (18, 67, 31, 194),
            dynamics.bus_a,
            BUS_10_SECOND_A,
            self._bus_load_color(bus_value),
            values.power_valid,
            inactive="#E1E6E5",
        )
        draw.text(
            (48, 57),
            "BATTERY",
            font=self.fonts.bold(11),
            fill=Color.MUTED,
            anchor="la",
        )
        draw.text(
            (48, 93),
            self._voltage_text(values),
            font=self.fonts.bold(24),
            fill=Color.TEXT if values.power_valid else Color.MUTED,
            anchor="la",
        )
        draw.text(
            (48, 113),
            "HV BUS",
            font=self.fonts.bold(9),
            fill=Color.MUTED,
            anchor="la",
        )
        draw.text(
            (48, 157),
            self._current_text(values.bus_current_deciamp, values.power_valid),
            font=self.fonts.bold(21),
            fill=Color.TEXT if values.power_valid else Color.MUTED,
            anchor="la",
        )
        draw.text(
            (48, 177),
            "BUS CURRENT",
            font=self.fonts.bold(9),
            fill=Color.MUTED,
            anchor="la",
        )

        draw.text(
            (240, 57),
            "ROAD SPEED",
            font=self.fonts.bold(11),
            fill=Color.MUTED,
            anchor="ma",
        )
        draw.text(
            (240, 128),
            self._speed_text(values),
            font=self.fonts.bold(55),
            fill=Color.TEXT if values.vehicle_valid else Color.MUTED,
            anchor="mm",
        )
        draw.text(
            (240, 170),
            "KM/H",
            font=self.fonts.bold(10),
            fill=Color.MUTED,
            anchor="mm",
        )

        self._vertical_bar(
            draw,
            (449, 67, 462, 194),
            dynamics.phase_a,
            PHASE_RATED_A,
            self._phase_load_color(dynamics.phase_a),
            values.power_valid,
            inactive="#E1E6E5",
        )
        draw.text(
            (432, 57),
            "MOTOR",
            font=self.fonts.bold(11),
            fill=Color.MUTED,
            anchor="ra",
        )
        draw.text(
            (432, 99),
            self._current_text(values.phase_current_deciamp, values.power_valid),
            font=self.fonts.bold(21),
            fill=Color.TEXT if values.power_valid else Color.MUTED,
            anchor="ra",
        )
        draw.text(
            (432, 119),
            "PHASE CURRENT",
            font=self.fonts.bold(9),
            fill=Color.MUTED,
            anchor="ra",
        )
        motor_color = self._motor_temp_color(
            values.motor_temperature_decic / 10.0,
            bool(values.motor_fault_bits & (1 << 5)),
            policy,
        ) if values.thermal_valid else Color.MUTED
        draw.text(
            (432, 163),
            self._temp_text(values.motor_temperature_decic, values.thermal_valid),
            font=self.fonts.bold(19),
            fill=motor_color,
            anchor="ra",
        )
        draw.text(
            (432, 181),
            "MOTOR TEMP",
            font=self.fonts.bold(9),
            fill=Color.MUTED,
            anchor="ra",
        )

        pedal = values.pedal_percent if values.detail_valid else 0
        draw.text(
            (20, 235),
            "PEDAL",
            font=self.fonts.bold(10),
            fill=Color.MUTED,
            anchor="la",
        )
        draw.text(
            (222, 235),
            f"{pedal}%" if values.detail_valid else "--%",
            font=self.fonts.bold(15),
            fill=Color.TEXT if values.detail_valid else Color.MUTED,
            anchor="ra",
        )
        self._horizontal_bar(
            draw,
            (20, 255, 222, 264),
            pedal,
            100.0,
            Color.MANUAL,
            values.detail_valid,
            inactive="#E1E6E5",
        )
        ctrl_color = self._controller_temp_color(
            values.controller_temperature_decic / 10.0,
            bool(values.motor_fault_bits & (1 << 4)),
        ) if values.thermal_valid else Color.MUTED
        draw.text(
            (260, 235),
            "CONTROLLER",
            font=self.fonts.bold(10),
            fill=Color.MUTED,
            anchor="la",
        )
        draw.text(
            (460, 249),
            self._temp_text(
                values.controller_temperature_decic, values.thermal_valid
            ),
            font=self.fonts.bold(21),
            fill=ctrl_color,
            anchor="rm",
        )
        self._alert_strip(draw, sample, now, dark=False)
