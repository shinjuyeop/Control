from __future__ import annotations

import math

from PIL import ImageDraw

from ..model import *
from ..palette import *


class BaselineVariant:
    def _draw_baseline(
        self, draw: ImageDraw.ImageDraw, sample: ScenarioSample
    ) -> None:
        values = sample.values
        bg = BaselineColor.WHITE
        text = BaselineColor.BLACK
        card = BaselineColor.LIGHT_GRAY
        border = BaselineColor.DARK_GRAY
        draw.rectangle((0, 0, LCD_WIDTH - 1, LCD_HEIGHT - 1), fill=bg)

        self._baseline_rect(draw, 8, 50, 292, 146, card)
        self._baseline_outline(draw, 8, 50, 292, 146, 2, border)
        self._baseline_rect(draw, 308, 50, 164, 146, card)
        self._baseline_outline(draw, 308, 50, 164, 146, 2, border)
        for x in (8, 166, 324):
            self._baseline_rect(draw, x, 204, 148, 62, card)
            self._baseline_outline(draw, x, 204, 148, 62, 2, border)

        self._baseline_text(draw, 18, 60, "HV BUS", 2, text, card)
        self._baseline_text(draw, 320, 60, "SPEED", 2, text, card)
        self._baseline_text(draw, 18, 212, "MOTOR", 2, text, card)
        self._baseline_text(draw, 176, 212, "CTRL", 2, text, card)
        self._baseline_text(draw, 334, 212, "FAULT", 2, text, card)

        badge_x = (8, 120, 208, 328)
        badge_width = (104, 80, 112, 144)
        for index, (label, color) in enumerate(
            self._baseline_badges(values)
        ):
            self._baseline_rect(
                draw, badge_x[index], 6, badge_width[index], 34, color
            )
            label_color = (
                text
                if color in (BaselineColor.ORANGE, BaselineColor.GREEN)
                else BaselineColor.WHITE
            )
            self._baseline_text(
                draw,
                badge_x[index] + 8,
                16,
                label,
                2,
                label_color,
                color,
            )

        voltage, bus_current, phase_current = self._baseline_power(values)
        speed = self._baseline_speed(values)
        motor, controller, fault, fault_color = self._baseline_metrics(values)
        self._baseline_rect(draw, 18, 88, 270, 96, card)
        self._baseline_text(draw, 18, 96, voltage, 3, text, card)
        self._baseline_text(draw, 18, 128, bus_current, 3, text, card)
        self._baseline_text(draw, 18, 160, phase_current, 3, text, card)
        self._baseline_rect(draw, 320, 88, 140, 96, card)
        self._baseline_text(draw, 320, 98, speed, 5, text, card)
        self._baseline_text(draw, 320, 158, "KM/H", 2, text, card)
        self._baseline_rect(draw, 18, 236, 128, 20, card)
        self._baseline_text(draw, 18, 238, motor, 2, text, card)
        self._baseline_rect(draw, 176, 236, 128, 20, card)
        self._baseline_text(draw, 176, 238, controller, 2, text, card)
        self._baseline_rect(draw, 334, 236, 128, 20, card)
        self._baseline_text(draw, 334, 238, fault, 2, fault_color, card)

        (
            bottom_left,
            bottom_middle,
            bottom_right,
            bottom_color,
            bottom_text,
            gnss_color,
            gnss_text_color,
            show_status_row,
        ) = self._baseline_bottom(sample)
        self._baseline_rect(draw, 8, 274, 464, 38, bottom_color)
        if show_status_row:
            self._baseline_rect(draw, 330, 274, 142, 38, gnss_color)
            self._baseline_text(
                draw, 18, 286, bottom_left, 2, bottom_text, bottom_color
            )
            self._baseline_text(
                draw, 178, 286, bottom_middle, 2, bottom_text, bottom_color
            )
            self._baseline_text(
                draw, 342, 286, bottom_right, 2, gnss_text_color, gnss_color
            )
        else:
            self._baseline_text(
                draw, 18, 286, bottom_left, 2, bottom_text, bottom_color
            )
            self._baseline_text(
                draw, 270, 286, bottom_middle, 2, bottom_text, bottom_color
            )

    @staticmethod
    def _baseline_rect(
        draw: ImageDraw.ImageDraw,
        x: int,
        y: int,
        width: int,
        height: int,
        color: tuple[int, int, int],
    ) -> None:
        draw.rectangle((x, y, x + width - 1, y + height - 1), fill=color)

    def _baseline_outline(
        self,
        draw: ImageDraw.ImageDraw,
        x: int,
        y: int,
        width: int,
        height: int,
        thickness: int,
        color: tuple[int, int, int],
    ) -> None:
        self._baseline_rect(draw, x, y, width, thickness, color)
        self._baseline_rect(
            draw, x, y + height - thickness, width, thickness, color
        )
        self._baseline_rect(draw, x, y, thickness, height, color)
        self._baseline_rect(
            draw, x + width - thickness, y, thickness, height, color
        )

    def _baseline_text(
        self,
        draw: ImageDraw.ImageDraw,
        x: int,
        y: int,
        value: str,
        scale: int,
        foreground: tuple[int, int, int],
        background: tuple[int, int, int],
    ) -> None:
        advance = 6 * scale
        for character in value:
            columns = FONT5X7.get(character, FONT5X7["?"])
            draw.rectangle(
                (x, y, x + advance - 1, y + 7 * scale - 1),
                fill=background,
            )
            for column in range(5):
                for row in range(7):
                    if columns[column] & (1 << row):
                        left = x + column * scale
                        top = y + row * scale
                        draw.rectangle(
                            (
                                left,
                                top,
                                left + scale - 1,
                                top + scale - 1,
                            ),
                            fill=foreground,
                        )
            x += advance

    @staticmethod
    def _baseline_badges(
        values: DashboardValues,
    ) -> tuple[tuple[str, tuple[int, int, int]], ...]:
        if values.vehicle_valid:
            if values.status(STATUS_MODE_AUTO):
                mode, mode_color = "AUTO", BaselineColor.BLUE
            else:
                mode, mode_color = "MANUAL", BaselineColor.ORANGE
            gear = "REV" if values.status(STATUS_GEAR_REVERSE) else "FWD"
        else:
            mode, mode_color, gear = "MODE --", BaselineColor.DARK_GRAY, "--"
        comm = "A-B OK" if values.vehicle_valid else "A-B LOST"
        comm_color = (
            BaselineColor.GREEN if values.vehicle_valid else BaselineColor.RED
        )
        if values.status(STATUS_ESTOP_ACTIVE):
            safety, safety_color = "E-STOP", BaselineColor.RED
        elif values.status(STATUS_BRAKE_ACTIVE):
            safety, safety_color = "BRAKE", BaselineColor.RED
        else:
            safety, safety_color = "SAFE", BaselineColor.GREEN
        return (
            (mode.ljust(7)[:7], mode_color),
            (gear.ljust(4)[:4], BaselineColor.DARK_GRAY),
            (comm.ljust(8)[:8], comm_color),
            (safety.ljust(6)[:6], safety_color),
        )

    @staticmethod
    def _baseline_labeled(label: str, raw: str, width: int) -> str:
        return (label + " " * width)[: width - len(raw)] + raw

    def _baseline_power(self, values: DashboardValues) -> tuple[str, str, str]:
        if values.power_valid:
            voltage = f"{values.bus_voltage_decivolt / 10.0:.1f} V"
            bus = f"{values.bus_current_deciamp / 10.0:.1f} A"
            phase = f"{values.phase_current_deciamp / 10.0:.1f} A"
        else:
            voltage, bus, phase = "--.- V", "--.- A", "--.- A"
        return (
            self._baseline_labeled("VOLT", voltage, 14),
            self._baseline_labeled("BUS", bus, 14),
            self._baseline_labeled("PHS", phase, 14),
        )

    @staticmethod
    def _baseline_speed(values: DashboardValues) -> str:
        raw = "--"
        if values.vehicle_valid:
            magnitude = abs(values.speed_centi_mps)
            tenths_kmh = (magnitude * 36 + 50) // 100
            raw = f"{tenths_kmh // 10}.{tenths_kmh % 10}"
        return raw[-5:].rjust(5)

    @staticmethod
    def _baseline_metrics(
        values: DashboardValues,
    ) -> tuple[str, str, str, tuple[int, int, int]]:
        if not values.thermal_valid:
            return (
                "--".rjust(8),
                "--".rjust(8),
                "--".rjust(8),
                BaselineColor.BLACK,
            )
        motor = f"{values.motor_temperature_decic / 10.0:.1f} C".rjust(8)
        controller = (
            f"{values.controller_temperature_decic / 10.0:.1f} C".rjust(8)
        )
        bits = values.motor_fault_bits & 0x00FFFFFF
        if bits:
            return motor, controller, f"0X{bits:06X}", BaselineColor.RED
        return motor, controller, "OK".rjust(8), BaselineColor.GREEN

    @staticmethod
    def _baseline_gnss(
        values: DashboardValues,
    ) -> tuple[str, tuple[int, int, int], tuple[int, int, int]]:
        if values.gnss_state == GNSS_RTK_FIXED:
            return "RTK FIXED", BaselineColor.GREEN, BaselineColor.BLACK
        if values.gnss_state == GNSS_FIX:
            return "GNSS FIX", BaselineColor.LIGHT_GRAY, BaselineColor.BLACK
        if values.gnss_state == GNSS_NO_FIX:
            return "GNSS NOFIX", BaselineColor.RED, BaselineColor.WHITE
        return "GNSS --", BaselineColor.DARK_GRAY, BaselineColor.WHITE

    @staticmethod
    def _baseline_bottom(
        sample: ScenarioSample,
    ) -> tuple:
        values = sample.values
        mode_auto = values.status(STATUS_MODE_AUTO)
        color = BaselineColor.BLUE if mode_auto else BaselineColor.ORANGE
        show_status_row = False
        if sample.dashboard_can_lost:
            left, middle, color = (
                "DASH CAN LOST",
                "VALUES STALE",
                BaselineColor.RED,
            )
        elif values.status(STATUS_ESTOP_ACTIVE):
            left, middle, color = (
                "E-STOP ACTIVE",
                "BRAKE ENGAGED",
                BaselineColor.RED,
            )
        elif not values.vehicle_valid:
            left, middle, color = (
                "STM A COMM LOST",
                "VALUES STALE",
                BaselineColor.RED,
            )
        elif values.detail_valid and values.status(STATUS_PART2_VALID) and (
            values.motor_fault_bits & 0x00FFFFFF
        ):
            left = "MOTOR FAULT"
            middle = f"0X{values.motor_fault_bits & 0x00FFFFFF:06X}"
            color = BaselineColor.RED
        elif values.detail_valid and values.drive(DRIVE_STOP):
            left, middle, color = (
                "DRIVE STOP",
                "TRACTIVE OFF",
                BaselineColor.RED,
            )
        elif values.status(STATUS_BRAKE_ACTIVE):
            left, middle, color = (
                "BRAKE ENGAGED",
                "TRACTIVE STOP",
                BaselineColor.RED,
            )
        elif mode_auto:
            show_status_row = True
            if not values.detail_valid:
                left, middle = "CMD --", "TRACT --"
            elif values.drive(DRIVE_COMMAND_TIMEOUT):
                left = "CMD TIMEOUT"
                middle = "TRACT RUN" if values.drive(
                    DRIVE_TRACTIVE_FEEDBACK
                ) else "TRACT STOP"
            elif values.drive(DRIVE_COMMAND_RECEIVED):
                left = "CMD OK"
                middle = "TRACT RUN" if values.drive(
                    DRIVE_TRACTIVE_FEEDBACK
                ) else "TRACT STOP"
            else:
                left = "CMD WAIT"
                middle = "TRACT RUN" if values.drive(
                    DRIVE_TRACTIVE_FEEDBACK
                ) else "TRACT STOP"
        else:
            show_status_row = True
            left = (
                f"PEDAL {values.pedal_percent}%"
                if values.detail_valid
                else "PEDAL --"
            )
            if not values.detail_valid:
                middle = "TRACT --"
            else:
                middle = "TRACT RUN" if values.drive(
                    DRIVE_TRACTIVE_FEEDBACK
                ) else "TRACT STOP"
        text_color = (
            BaselineColor.BLACK
            if color == BaselineColor.ORANGE
            else BaselineColor.WHITE
        )
        if show_status_row:
            gnss, gnss_color, gnss_text_color = BaselineVariant._baseline_gnss(
                values
            )
            return (
                left.ljust(11)[:11],
                middle.ljust(10)[:10],
                gnss.ljust(10)[:10],
                color,
                text_color,
                gnss_color,
                gnss_text_color,
                True,
            )
        return (
            left.ljust(20)[:20],
            middle.ljust(16)[:16],
            "",
            color,
            text_color,
            color,
            text_color,
            False,
        )
