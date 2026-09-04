"""Renderer lifecycle, semantic state synthesis, and shared primitives."""

from __future__ import annotations

from collections import deque
import math
from typing import Callable

from PIL import Image, ImageDraw

from .model import *
from .palette import *
from .registry import VARIANT_METHODS


class RendererCore:
    def __init__(self) -> None:
        self.fonts = FontBook()
        self.history: deque[HistoryPoint] = deque(maxlen=180)
        self._last_history_time: float | None = None

    def clear_history(self) -> None:
        self.history.clear()
        self._last_history_time = None

    def seed_history(
        self,
        builder: Callable[[float], ScenarioSample],
        end_time: float,
        *,
        duration: float = 5.0,
        step: float = 0.1,
    ) -> None:
        self.clear_history()
        start = max(0.0, end_time - duration)
        current = start
        while current < end_time:
            self._record_history(builder(current), current)
            current += step

    def render(
        self,
        sample: ScenarioSample,
        variant: str,
        dynamics: GaugeDynamics,
        policy: PreviewPolicy,
        now: float,
    ) -> Image.Image:
        self._record_history(sample, now)
        if variant == "baseline":
            image = Image.new(
                "RGB", (LCD_WIDTH, LCD_HEIGHT), BaselineColor.WHITE
            )
            self._draw_baseline(ImageDraw.Draw(image), sample)
            return image

        image = Image.new("RGB", (LCD_WIDTH, LCD_HEIGHT), Color.BG)
        draw = ImageDraw.Draw(image)
        renderer = getattr(self, VARIANT_METHODS[variant])
        renderer(draw, sample, dynamics, policy, now)
        return image

    def _record_history(self, sample: ScenarioSample, now: float) -> None:
        if self._last_history_time is not None:
            if now < self._last_history_time:
                self.clear_history()
            elif now - self._last_history_time < 0.05:
                return
        values = sample.values
        self.history.append(
            HistoryPoint(
                time_s=now,
                bus_current_a=values.bus_current_deciamp / 10.0,
                phase_current_a=values.phase_current_deciamp / 10.0,
                motor_temp_c=values.motor_temperature_decic / 10.0,
                controller_temp_c=values.controller_temperature_decic / 10.0,
                power_valid=values.power_valid,
                thermal_valid=values.thermal_valid,
            )
        )
        self._last_history_time = now

    def _semantic_drive(
        self, sample: ScenarioSample, now: float
    ) -> DriveSemantic:
        values = sample.values
        direction = -1 if values.status(STATUS_GEAR_REVERSE) else 1
        auto = values.status(STATUS_MODE_AUTO)
        source = "HOST" if auto else "PEDAL"
        active = values.drive(DRIVE_TRACTIVE_FEEDBACK)
        if sample.dashboard_can_lost:
            return DriveSemantic(
                "DASH DATA LOST", "LAST VALUES", Color.FAULT,
                source, direction, False, True,
            )
        if values.status(STATUS_ESTOP_ACTIVE):
            return DriveSemantic(
                "E-STOP BRAKING", "PHYSICAL STOP", Color.FAULT,
                source, direction, False, True,
            )
        if not values.vehicle_valid:
            return DriveSemantic(
                "VEHICLE DATA LOST", "STM A LINK", Color.FAULT,
                source, direction, False, True,
            )
        if not values.detail_valid:
            return DriveSemantic(
                "DRIVE DATA LOST", "VALUES STALE", Color.FAULT,
                source, direction, False, True,
            )
        if not values.status(STATUS_PART1_VALID):
            return DriveSemantic(
                "POWER DATA LOST", "PART I", Color.ORANGE,
                source, direction, active, True,
            )
        if not values.status(STATUS_PART2_VALID):
            return DriveSemantic(
                "THERMAL DATA LOST", "PART II", Color.ORANGE,
                source, direction, active, True,
            )
        if values.motor_fault_bits:
            return DriveSemantic(
                self._fault_name(values.motor_fault_bits, now),
                "MOTOR FAULT",
                Color.FAULT,
                source,
                direction,
                active,
                True,
            )
        if values.drive(DRIVE_STOP):
            return DriveSemantic(
                "DRIVE STOPPED", "TRACTIVE OFF", Color.FAULT,
                source, direction, False, True,
            )
        if values.status(STATUS_BRAKE_ACTIVE):
            return DriveSemantic(
                "BRAKING", "TRACTIVE STOP", Color.FAULT,
                source, direction, False, True,
            )
        if auto:
            if values.drive(DRIVE_COMMAND_TIMEOUT):
                return DriveSemantic(
                    "AUTO TIMEOUT", "COMMAND LOST", Color.FAULT,
                    source, direction, False, True,
                )
            if not values.drive(DRIVE_COMMAND_RECEIVED):
                return DriveSemantic(
                    "AUTO WAITING", "NO COMMAND", Color.AUTO,
                    source, direction, False, False,
                )
            if active:
                return DriveSemantic(
                    "AUTO TRACKING", "HOST CONTROL", Color.AUTO,
                    source, direction, True, False,
                )
            return DriveSemantic(
                "AUTO READY", "DRIVE IDLE", Color.AUTO,
                source, direction, False, False,
            )
        if active:
            return DriveSemantic(
                "DRIVER POWER", "PEDAL CONTROL", Color.MANUAL,
                source, direction, True, False,
            )
        return DriveSemantic(
            "DRIVER COAST", "PEDAL RELEASED", Color.MANUAL,
            source, direction, False, False,
        )

    def _status_tokens(
        self, values: DashboardValues
    ) -> dict[str, tuple[str, str]]:
        if values.vehicle_valid:
            if values.status(STATUS_MODE_AUTO):
                mode = ("AUTO", Color.AUTO)
            else:
                mode = ("MANUAL", Color.MANUAL)
            gear = (
                "REV" if values.status(STATUS_GEAR_REVERSE) else "FWD",
                "#1B252A",
            )
        else:
            mode = ("MODE --", Color.MUTED)
            gear = ("--", Color.MUTED)
        if not values.vehicle_valid:
            link = ("A-B LOST", Color.FAULT)
        elif not values.detail_valid:
            link = ("DETAIL LOST", Color.FAULT)
        elif not values.status(STATUS_PART1_VALID) or not values.status(
            STATUS_PART2_VALID
        ):
            link = ("DATA PART", Color.ORANGE)
        else:
            link = ("LINK OK", Color.OK)
        if values.status(STATUS_ESTOP_ACTIVE):
            safety = ("E-STOP", Color.FAULT)
        elif values.status(STATUS_BRAKE_ACTIVE):
            safety = ("BRAKE", Color.FAULT)
        else:
            safety = ("SAFE", Color.OK)
        tractive = (
            ("TRACTIVE RUN", Color.OK)
            if values.drive(DRIVE_TRACTIVE_FEEDBACK)
            else ("TRACTIVE STOP", Color.FAULT)
        )
        return {
            "mode": mode,
            "gear": gear,
            "link": link,
            "safety": safety,
            "tractive": tractive,
        }

    def _inline_tag(
        self,
        draw: ImageDraw.ImageDraw,
        box: tuple[int, int, int, int],
        token: tuple[str, str],
    ) -> None:
        text, color = token
        draw.rectangle(box, fill=color)
        draw.text(
            ((box[0] + box[2]) // 2, (box[1] + box[3]) // 2),
            text,
            font=self.fonts.bold(10),
            fill=self._token_text_color(color),
            anchor="mm",
        )

    @staticmethod
    def _token_text_color(color: str) -> str:
        return "#12191C" if color == Color.MANUAL else "#FFFFFF"



    def _vertical_tape(
        self,
        draw: ImageDraw.ImageDraw,
        box: tuple[int, int, int, int],
        value: float,
        maximum: float,
        color: str,
        valid: bool,
        label: str,
        reading: str,
        *,
        right: bool,
    ) -> None:
        x0, y0, x1, y1 = box
        center_x = (x0 + x1) // 2
        rail_x = x0 + 18 if right else x1 - 18
        draw.text(
            (center_x, y0 + 10), label, font=self.fonts.bold(9),
            fill=Color.MUTED, anchor="mm",
        )
        draw.text(
            (center_x, y0 + 31), reading, font=self.fonts.bold(13),
            fill=Color.TEXT if valid else Color.MUTED, anchor="mm",
        )
        tape_top = y0 + 50
        tape_bottom = y1 - 8
        draw.line((rail_x, tape_top, rail_x, tape_bottom), fill="#6F7D80", width=2)
        for index in range(11):
            y = tape_bottom - round((tape_bottom - tape_top) * index / 10)
            length = 11 if index % 5 == 0 else 7
            if right:
                draw.line((rail_x, y, rail_x + length, y), fill="#6F7D80", width=1)
            else:
                draw.line((rail_x - length, y, rail_x, y), fill="#6F7D80", width=1)
        if valid:
            ratio = max(0.0, min(1.0, value / maximum))
            marker_y = tape_bottom - round((tape_bottom - tape_top) * ratio)
            if right:
                points = (
                    (rail_x + 2, marker_y),
                    (rail_x + 15, marker_y - 7),
                    (rail_x + 15, marker_y + 7),
                )
            else:
                points = (
                    (rail_x - 2, marker_y),
                    (rail_x - 15, marker_y - 7),
                    (rail_x - 15, marker_y + 7),
                )
            draw.polygon(points, fill=color)
            draw.line(
                (rail_x, marker_y, rail_x, tape_bottom), fill=color, width=4
            )


    def _fill_screen(self, draw: ImageDraw.ImageDraw, color: str) -> None:
        draw.rectangle((0, 0, LCD_WIDTH - 1, LCD_HEIGHT - 1), fill=color)

    def _race_header(
        self, draw: ImageDraw.ImageDraw, values: DashboardValues, *, dark: bool
    ) -> None:
        bg = DarkColor.SURFACE if dark else "#E7ECEA"
        muted = DarkColor.MUTED if dark else Color.MUTED
        text = DarkColor.TEXT if dark else Color.TEXT
        if values.vehicle_valid:
            if values.status(STATUS_MODE_AUTO):
                mode, mode_color = "AUTO", Color.AUTO
            else:
                mode, mode_color = "MANUAL", Color.MANUAL
            gear = "REV" if values.status(STATUS_GEAR_REVERSE) else "FWD"
        else:
            mode, mode_color, gear = "MODE --", muted, "--"
        if not values.vehicle_valid:
            link, link_color = "A-B LOST", Color.FAULT
        elif not values.detail_valid:
            link, link_color = "DETAIL LOST", Color.FAULT
        elif not values.status(STATUS_PART1_VALID) or not values.status(
            STATUS_PART2_VALID
        ):
            link, link_color = "DATA PART", Color.ORANGE
        else:
            link, link_color = "LINK OK", Color.OK
        if values.status(STATUS_ESTOP_ACTIVE):
            safety, safety_color = "E-STOP", Color.FAULT
        elif values.status(STATUS_BRAKE_ACTIVE):
            safety, safety_color = "BRAKE", Color.FAULT
        else:
            safety, safety_color = "SAFE", Color.OK

        segments = (
            (0, 100, mode, mode_color),
            (100, 165, gear, bg),
            (165, 315, link, link_color),
            (315, 480, safety, safety_color),
        )
        for x0, x1, label, color in segments:
            draw.rectangle((x0, 0, x1 - 1, 29), fill=color)
            label_color = text if color in (Color.MANUAL, bg) else "#FFFFFF"
            draw.text(
                ((x0 + x1) // 2, 15),
                label,
                font=self.fonts.bold(12),
                fill=label_color,
                anchor="mm",
            )

    def _alert_strip(
        self,
        draw: ImageDraw.ImageDraw,
        sample: ScenarioSample,
        now: float,
        *,
        dark: bool,
    ) -> None:
        left, right, color = self._status_message(sample, now)
        draw.rectangle((0, 286, 479, 319), fill=color)
        text_color = "#101719" if color in (Color.MANUAL, Color.WARN) else "#FFFFFF"
        if left:
            draw.text(
                (14, 303),
                left,
                font=self.fonts.bold(13),
                fill=text_color,
                anchor="lm",
            )
            draw.text(
                (466, 303),
                right,
                font=self.fonts.bold(11),
                fill=text_color,
                anchor="rm",
            )
        else:
            draw.text(
                (240, 303),
                right,
                font=self.fonts.bold(12),
                fill=text_color,
                anchor="mm",
            )

    def _status_message(
        self, sample: ScenarioSample, now: float
    ) -> tuple[str, str, str]:
        values = sample.values
        if sample.dashboard_can_lost:
            return "DASH CAN LOST", "VALUES STALE", Color.FAULT
        if values.status(STATUS_ESTOP_ACTIVE):
            return "E-STOP ACTIVE", "BRAKE ENGAGED", Color.FAULT
        if not values.vehicle_valid:
            return "STM A COMM LOST", "VALUES STALE", Color.FAULT
        if not values.detail_valid:
            return "DRIVE DETAIL LOST", "VALUES STALE", Color.FAULT
        if not values.status(STATUS_PART1_VALID):
            return "POWER DATA LOST", "PART I", Color.ORANGE
        if not values.status(STATUS_PART2_VALID):
            return "THERMAL DATA LOST", "PART II", Color.ORANGE
        if values.motor_fault_bits:
            return self._fault_name(values.motor_fault_bits, now), "MOTOR FAULT", Color.FAULT
        if values.drive(DRIVE_STOP):
            return "DRIVE STOP", "TRACTIVE OFF", Color.FAULT
        if values.status(STATUS_BRAKE_ACTIVE):
            return "BRAKE ENGAGED", "TRACTIVE STOP", Color.FAULT
        if values.status(STATUS_MODE_AUTO):
            if values.drive(DRIVE_COMMAND_TIMEOUT):
                return "AUTO CMD TIMEOUT", "TRACTIVE STOP", Color.FAULT
            if values.drive(DRIVE_COMMAND_RECEIVED):
                return "AUTO CMD OK", self._tractive_text(values), Color.AUTO
            return "AUTO CMD WAIT", "TRACTIVE STOP", Color.AUTO
        return "", self._tractive_text(values), Color.MANUAL

    def _vital_cell(
        self,
        draw: ImageDraw.ImageDraw,
        box: tuple[int, int, int, int],
        label: str,
        value: str,
        color: str,
        valid: bool,
        *,
        left_bar: bool,
    ) -> None:
        x0, y0, x1, y1 = box
        bar_x = x0 if left_bar else x1 - 4
        draw.rectangle((bar_x, y0, bar_x + 3, y1), fill=color if valid else DarkColor.GRID)
        text_x = x0 + 13 if left_bar else x0 + 8
        draw.text(
            (text_x, y0 + 13),
            label,
            font=self.fonts.bold(9),
            fill=DarkColor.MUTED,
            anchor="la",
        )
        draw.text(
            (text_x, y0 + 42),
            value,
            font=self.fonts.bold(20),
            fill=DarkColor.TEXT if valid else DarkColor.MUTED,
            anchor="la",
        )





    def _horizontal_bar(
        self,
        draw: ImageDraw.ImageDraw,
        box: tuple[int, int, int, int],
        value: float,
        maximum: float,
        color: str,
        valid: bool,
        *,
        inactive: str = "#DDE4E2",
    ) -> None:
        draw.rectangle(box, fill=inactive)
        if not valid:
            return
        x0, y0, x1, y1 = box
        ratio = max(0.0, min(1.0, value / maximum))
        draw.rectangle((x0, y0, x0 + round((x1 - x0) * ratio), y1), fill=color)

    def _vertical_bar(
        self,
        draw: ImageDraw.ImageDraw,
        box: tuple[int, int, int, int],
        value: float,
        maximum: float,
        color: str,
        valid: bool,
        *,
        inactive: str = DarkColor.INACTIVE,
    ) -> None:
        draw.rectangle(box, fill=inactive)
        if not valid:
            return
        x0, y0, x1, y1 = box
        ratio = max(0.0, min(1.0, value / maximum))
        top = y1 - round((y1 - y0) * ratio)
        draw.rectangle((x0, top, x1, y1), fill=color)


    def _arc_gauge(
        self,
        draw: ImageDraw.ImageDraw,
        box: tuple[int, int, int, int],
        value: float,
        peak: float,
        maximum: float,
        color: str,
        valid: bool,
    ) -> None:
        draw.arc(box, start=200, end=340, fill=DarkColor.GRID, width=10)
        if not valid:
            return
        end = 200 + 140 * max(0.0, min(1.0, value / maximum))
        draw.arc(box, start=200, end=end, fill=color, width=10)
        for index in range(9):
            angle = math.radians(200 + index * 17.5)
            cx = (box[0] + box[2]) / 2
            cy = (box[1] + box[3]) / 2
            radius = (box[2] - box[0]) / 2
            x0 = cx + (radius - 15) * math.cos(angle)
            y0 = cy + (radius - 15) * math.sin(angle)
            x1 = cx + (radius - 4) * math.cos(angle)
            y1 = cy + (radius - 4) * math.sin(angle)
            draw.line((x0, y0, x1, y1), fill=DarkColor.MUTED, width=1)
        peak_angle = math.radians(
            200 + 140 * max(0.0, min(1.0, peak / maximum))
        )
        cx = (box[0] + box[2]) / 2
        cy = (box[1] + box[3]) / 2
        radius = (box[2] - box[0]) / 2
        peak_x0 = cx + (radius - 18) * math.cos(peak_angle)
        peak_y0 = cy + (radius - 18) * math.sin(peak_angle)
        peak_x1 = cx + (radius + 1) * math.cos(peak_angle)
        peak_y1 = cy + (radius + 1) * math.sin(peak_angle)
        draw.line(
            (peak_x0, peak_y0, peak_x1, peak_y1),
            fill=DarkColor.TEXT,
            width=2,
        )

    @staticmethod
    def _ratio(value: float, minimum: float, maximum: float) -> float:
        return max(0.0, min(1.0, (value - minimum) / (maximum - minimum)))

    @staticmethod
    def _speed_text(values: DashboardValues) -> str:
        return f"{abs(values.speed_centi_mps) * 0.036:.1f}" if values.vehicle_valid else "--.-"

    @staticmethod
    def _voltage_text(values: DashboardValues) -> str:
        return f"{values.bus_voltage_decivolt / 10.0:.1f} V" if values.power_valid else "--.- V"

    @staticmethod
    def _current_text(value_deciamp: int, valid: bool) -> str:
        return f"{value_deciamp / 10.0:+.1f} A" if valid else "--.- A"

    @staticmethod
    def _temp_text(value_decic: int, valid: bool) -> str:
        return f"{value_decic / 10.0:.1f} C" if valid else "--.- C"

    @staticmethod
    def _voltage_color(voltage_v: float) -> str:
        if voltage_v < 42.0:
            return Color.FAULT
        if voltage_v < 47.0:
            return Color.ORANGE
        return Color.BLUE

    @staticmethod
    def _bus_load_color(bus_current_a: float) -> str:
        if bus_current_a < 0.0:
            return Color.BLUE
        if bus_current_a <= BUS_CONTINUOUS_A:
            return Color.OK
        if bus_current_a <= BUS_30_SECOND_A:
            return Color.WARN
        return Color.FAULT

    @staticmethod
    def _phase_load_color(phase_current_a: float) -> str:
        if phase_current_a <= PHASE_RATED_A * 0.60:
            return Color.OK
        if phase_current_a <= PHASE_RATED_A * 0.80:
            return Color.WARN
        return Color.FAULT

    @staticmethod
    def _controller_temp_color(temp_c: float, overheat_fault: bool) -> str:
        if overheat_fault or temp_c >= CONTROLLER_OVERHEAT_C:
            return Color.FAULT
        if temp_c >= CONTROLLER_OVERHEAT_C * 0.8:
            return Color.ORANGE
        return Color.BLUE

    @staticmethod
    def _motor_temp_color(
        temp_c: float, overheat_fault: bool, policy: PreviewPolicy
    ) -> str:
        if overheat_fault:
            return Color.FAULT
        if not policy.motor_bands_enabled:
            return Color.BLUE
        if temp_c >= policy.motor_critical_c:
            return Color.FAULT
        if temp_c >= policy.motor_warning_c:
            return Color.ORANGE
        return Color.BLUE

    @staticmethod
    def _tractive_text(values: DashboardValues) -> str:
        return "TRACTIVE RUN" if values.drive(DRIVE_TRACTIVE_FEEDBACK) else "TRACTIVE STOP"


    @staticmethod
    def _fault_name(bits: int, now: float) -> str:
        names = [name for index, name in enumerate(FAULT_NAMES) if bits & (1 << index)]
        if not names:
            return ""
        return names[int(now / 1.5) % len(names)]
