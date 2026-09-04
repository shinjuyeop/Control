"""Paired MANUAL TFT and AUTO GT3/CUP dashboard pages."""

from __future__ import annotations

import math

from PIL import Image, ImageChops, ImageDraw

from ..model import *
from ..palette import *


MODE_BG = "#F4F6F2"
MODE_RULE = "#AEB8B4"
MODE_TRACK = "#D8DFDB"
BUS_REGEN_DISPLAY_A = 120.0
REPLICA_ORANGE = "#ED4A0A"
REPLICA_BLACK = "#171818"
REPLICA_MUTED = "#777A7A"
REPLICA_REGEN = "#78D4F4"


class ModePageVariants:
    def _draw_manual_tft(
        self,
        draw: ImageDraw.ImageDraw,
        sample: ScenarioSample,
        dynamics: GaugeDynamics,
        policy: PreviewPolicy,
        now: float,
    ) -> None:
        values = sample.values
        state = self._semantic_drive(sample, now)
        vehicle_valid = values.vehicle_valid and not sample.dashboard_can_lost
        detail_valid = values.detail_valid and not sample.dashboard_can_lost
        power_valid = values.power_valid and not sample.dashboard_can_lost
        thermal_valid = values.thermal_valid and not sample.dashboard_can_lost

        self._replica_background(draw)
        gear = "-"
        if vehicle_valid:
            gear = "R" if values.status(STATUS_GEAR_REVERSE) else "F"
        self._replica_gear_panel(draw, gear)
        self._replica_current_tracks(draw, values, dynamics, power_valid)

        draw.text(
            (361, 111), "KM/H", font=self.fonts.medium(12),
            fill="#4D4F4F", anchor="ra",
        )

        speed_text = self._speed_text(values) if vehicle_valid else "--.-"
        speed_size = 64 if len(speed_text) <= 4 else 57
        draw.text(
            (361, 162), speed_text, font=self.fonts.heavy(speed_size),
            fill=REPLICA_BLACK, anchor="rm",
        )

        bus_voltage = values.bus_voltage_decivolt / 10.0
        controller_temp = values.controller_temperature_decic / 10.0
        motor_temp = values.motor_temperature_decic / 10.0
        controller_fault = bool(values.motor_fault_bits & (1 << 4))
        motor_fault = bool(values.motor_fault_bits & (1 << 5))
        self._replica_segment_gauge(
            draw, 376, "PED", float(values.pedal_percent), 0.0, 100.0,
            detail_valid,
            f"{values.pedal_percent}%" if detail_valid else "--%",
            False,
        )
        self._replica_segment_gauge(
            draw, 402, "HV", bus_voltage, HV_MIN_V, HV_MAX_V, power_valid,
            f"{bus_voltage:.1f}V" if power_valid else "--.-V",
            power_valid and bus_voltage < 42.0,
        )
        self._replica_segment_gauge(
            draw, 428, "MTR", motor_temp, 0.0, policy.motor_critical_c,
            thermal_valid,
            f"{motor_temp:.0f}°" if thermal_valid else "--°",
            motor_fault or (
                policy.motor_bands_enabled and
                motor_temp >= policy.motor_critical_c
            ),
        )
        self._replica_segment_gauge(
            draw, 454, "CTRL", controller_temp, 0.0, 70.0, thermal_valid,
            f"{controller_temp:.0f}°" if thermal_valid else "--°",
            controller_fault or controller_temp >= CONTROLLER_OVERHEAT_C,
        )

        self._replica_bottom_bar(draw, sample)
        self._replica_alert(draw, state)

    def _replica_background(self, draw: ImageDraw.ImageDraw) -> None:
        for y in range(320):
            fraction = y / 319.0
            red = round(246 - 18 * fraction)
            green = round(247 - 18 * fraction)
            blue = round(247 - 17 * fraction)
            draw.line((0, y, 479, y), fill=(red, green, blue))
        for inset, color in (
            (0, "#DDDEDE"), (1, "#E4E5E5"), (2, "#EAEBEB")
        ):
            draw.rectangle(
                (inset, inset, 479 - inset, 319 - inset),
                outline=color,
            )

    def _replica_gear_panel(
        self,
        draw: ImageDraw.ImageDraw,
        gear: str,
    ) -> None:
        mask = getattr(self, "_replica_gear_mask", None)
        if mask is None:
            offset_path = self._replica_offset_path(
                self._replica_bus_path(), 44.0
            )
            visible_boundary = [offset_path[0]]
            for point in offset_path[1:]:
                previous = visible_boundary[-1]
                if point[1] <= 0.0:
                    amount = previous[1] / (previous[1] - point[1])
                    visible_boundary.append(
                        (
                            previous[0] + (point[0] - previous[0]) * amount,
                            0.0,
                        )
                    )
                    break
                visible_boundary.append(point)

            supersample = 4
            mask = Image.new("L", (480 * supersample, 320 * supersample), 0)
            mask_draw = ImageDraw.Draw(mask)
            polygon = (
                (0.0, 0.0),
                visible_boundary[-1],
                *reversed(visible_boundary[:-1]),
                (0.0, 263.0),
            )
            mask_draw.polygon(
                tuple(
                    (round(x * supersample), round(y * supersample))
                    for x, y in polygon
                ),
                fill=255,
            )
            mask = mask.resize((480, 320), Image.Resampling.LANCZOS)
            self._replica_gear_mask = mask
        draw._image.paste(REPLICA_BLACK, mask=mask)
        draw.text(
            (35, 48), gear,
            font=self.fonts.heavy(59), fill="#F7F7F7", anchor="mm",
        )

    @staticmethod
    def _replica_cubic(
        start: tuple[float, float],
        control_a: tuple[float, float],
        control_b: tuple[float, float],
        end: tuple[float, float],
    ) -> tuple[tuple[float, float], ...]:
        points = []
        for index in range(17):
            amount = index / 16.0
            inverse = 1.0 - amount
            points.append(
                (
                    inverse ** 3 * start[0]
                    + 3.0 * inverse ** 2 * amount * control_a[0]
                    + 3.0 * inverse * amount ** 2 * control_b[0]
                    + amount ** 3 * end[0],
                    inverse ** 3 * start[1]
                    + 3.0 * inverse ** 2 * amount * control_a[1]
                    + 3.0 * inverse * amount ** 2 * control_b[1]
                    + amount ** 3 * end[1],
                )
            )
        return tuple(points)

    @classmethod
    def _replica_bus_path(cls) -> tuple[tuple[float, float], ...]:
        return (
            (54.0, 240.0),
            (54.0, 190.0),
            *cls._replica_cubic(
                (54.0, 190.0),
                (54.0, 180.0),
                (59.0, 170.0),
                (66.0, 161.0),
            )[1:],
            (154.0, 53.0),
            *cls._replica_cubic(
                (154.0, 53.0),
                (163.0, 42.0),
                (177.0, 32.0),
                (193.0, 32.0),
            )[1:],
            (470.0, 32.0),
        )

    @classmethod
    def _replica_phase_path(cls) -> tuple[tuple[float, float], ...]:
        return (
            (68.0, 240.0),
            (68.0, 195.0),
            *cls._replica_cubic(
                (68.0, 195.0),
                (68.0, 185.0),
                (73.0, 175.0),
                (80.0, 166.0),
            )[1:],
            (165.0, 63.0),
            *cls._replica_cubic(
                (165.0, 63.0),
                (174.0, 52.0),
                (187.0, 48.0),
                (202.0, 48.0),
            )[1:],
            (470.0, 48.0),
        )

    @staticmethod
    def _replica_offset_path(
        path: tuple[tuple[float, float], ...],
        distance: float,
    ) -> tuple[tuple[float, float], ...]:
        result = []
        for index, (x, y) in enumerate(path):
            before = path[max(0, index - 1)]
            after = path[min(len(path) - 1, index + 1)]
            tangent_x = after[0] - before[0]
            tangent_y = after[1] - before[1]
            length = math.hypot(tangent_x, tangent_y)
            outward_x = tangent_y / length
            outward_y = -tangent_x / length
            result.append(
                (x + outward_x * distance, y + outward_y * distance)
            )
        return tuple(result)

    def _replica_current_tracks(
        self,
        draw: ImageDraw.ImageDraw,
        values: DashboardValues,
        dynamics: GaugeDynamics,
        valid: bool,
    ) -> None:
        bus_path = self._replica_bus_path()
        phase_path = self._replica_phase_path()
        self._replica_paste_track(draw, "bus-base", bus_path, 3, REPLICA_BLACK)
        self._replica_paste_track(
            draw, "phase-base", phase_path, 2, "#858888"
        )

        bus_min = -BUS_REGEN_DISPLAY_A
        bus_max = BUS_10_SECOND_A
        bus_span = bus_max - bus_min
        bus_zero = -bus_min / bus_span
        bus_redline = (BUS_30_SECOND_A - bus_min) / bus_span
        phase_redline = 0.8
        self._replica_paste_track_segment(
            draw,
            "bus-limit",
            bus_path,
            3,
            Color.FAULT,
            bus_redline,
            1.0,
            anchor_fraction=bus_redline,
            keep_after_anchor=True,
        )
        self._replica_paste_track_segment(
            draw,
            "phase-limit",
            phase_path,
            2,
            Color.FAULT,
            phase_redline,
            1.0,
            anchor_fraction=phase_redline,
            keep_after_anchor=True,
        )

        for value in (-120.0, 0.0, 150.0, 300.0, 350.0):
            fraction = (value - bus_min) / bus_span
            self._replica_track_tick(
                draw,
                bus_path,
                fraction,
                value,
                outward=True,
                color=Color.FAULT if value >= 300.0 else None,
            )
        for value in (0.0, 240.0, 480.0):
            fraction = value / PHASE_RATED_A
            self._replica_track_tick(
                draw,
                phase_path,
                fraction,
                value,
                outward=False,
                color=Color.FAULT if fraction >= phase_redline else None,
            )

        for index in range(21):
            fraction = index / 20.0
            if index % 5:
                bus_tick_value = bus_min + bus_span * fraction
                self._replica_minor_tick(
                    draw,
                    bus_path,
                    fraction,
                    True,
                    color=(
                        Color.FAULT
                        if bus_tick_value >= BUS_30_SECOND_A
                        else None
                    ),
                )
                self._replica_minor_tick(
                    draw,
                    phase_path,
                    fraction,
                    False,
                    color=Color.FAULT if fraction >= phase_redline else None,
                )

        bus_text = (
            f"{values.bus_current_deciamp / 10.0:+.1f}A"
            if valid else "--.-A"
        )
        phase_text = (
            f"{values.phase_current_deciamp / 10.0:+.1f}A"
            if valid else "--.-A"
        )
        draw.text(
            (126, 211), "BUS", font=self.fonts.medium(8),
            fill=REPLICA_MUTED, anchor="mm",
        )
        draw.text(
            (126, 229), bus_text, font=self.fonts.display_bold(13),
            fill=REPLICA_BLACK if valid else REPLICA_MUTED, anchor="mm",
        )
        draw.text(
            (197, 211), "PHASE", font=self.fonts.medium(8),
            fill=REPLICA_MUTED, anchor="mm",
        )
        draw.text(
            (197, 229), phase_text, font=self.fonts.display_bold(13),
            fill=REPLICA_BLACK if valid else REPLICA_MUTED, anchor="mm",
        )

        if not valid:
            return

        bus_a = values.bus_current_deciamp / 10.0
        bus_fraction = max(
            0.0, min(1.0, (bus_a - bus_min) / bus_span)
        )
        bus_color = REPLICA_REGEN if bus_a < 0.0 else self._bus_load_color(bus_a)
        if bus_fraction < bus_zero:
            self._replica_paste_track_segment(
                draw,
                "bus-active",
                bus_path,
                10,
                bus_color,
                bus_fraction,
                bus_zero,
                round_start=True,
                round_end=False,
                anchor_fraction=bus_zero,
                keep_after_anchor=False,
            )
        else:
            self._replica_paste_track_segment(
                draw,
                "bus-active",
                bus_path,
                10,
                bus_color,
                bus_zero,
                bus_fraction,
                anchor_fraction=bus_zero,
                keep_after_anchor=True,
            )

        phase_fraction = min(1.0, dynamics.phase_a / PHASE_RATED_A)
        phase_color = self._mode_phase_color(values, dynamics)
        self._replica_paste_track_segment(
            draw,
            "phase-active",
            phase_path,
            9,
            phase_color,
            0.0,
            phase_fraction,
            anchor_fraction=0.0,
            keep_after_anchor=True,
        )

    def _replica_paste_track(
        self,
        draw: ImageDraw.ImageDraw,
        key: str,
        path: tuple[tuple[float, float], ...],
        width: int,
        color: str,
    ) -> None:
        mask = self._replica_track_mask(key, path, width)
        draw._image.paste(color, mask=mask)

    def _replica_paste_track_segment(
        self,
        draw: ImageDraw.ImageDraw,
        key: str,
        path: tuple[tuple[float, float], ...],
        width: int,
        color: str,
        start: float,
        end: float,
        *,
        round_start: bool = False,
        round_end: bool = True,
        anchor_fraction: float | None = None,
        keep_after_anchor: bool = True,
    ) -> None:
        start = max(0.0, min(1.0, start))
        end = max(0.0, min(1.0, end))
        if end <= start:
            return
        full_mask = self._replica_track_mask(key, path, width)
        reveal = self._replica_segment_reveal_mask(
            path,
            start,
            end,
            width + 8,
            round_start=round_start,
            round_end=round_end,
        )
        segment_mask = ImageChops.multiply(full_mask, reveal)
        if anchor_fraction is not None:
            anchor_mask = self._replica_anchor_mask(
                path, anchor_fraction, keep_after_anchor
            )
            segment_mask = ImageChops.multiply(segment_mask, anchor_mask)
        draw._image.paste(color, mask=segment_mask)

    def _replica_anchor_mask(
        self,
        path: tuple[tuple[float, float], ...],
        fraction: float,
        keep_after: bool,
    ) -> Image.Image:
        cache = getattr(self, "_replica_anchor_masks", None)
        if cache is None:
            cache = {}
            self._replica_anchor_masks = cache
        cache_key = (path[0], path[-1], round(fraction, 6), keep_after)
        if cache_key in cache:
            return cache[cache_key]

        x, y, tangent_x, tangent_y = self._mode_track_location(path, fraction)
        normal_x, normal_y = -tangent_y, tangent_x
        extent = 1000.0
        direction = 1.0 if keep_after else -1.0
        edge_a = (x + normal_x * extent, y + normal_y * extent)
        edge_b = (x - normal_x * extent, y - normal_y * extent)
        shift_x = tangent_x * extent * 2.0 * direction
        shift_y = tangent_y * extent * 2.0 * direction
        mask = Image.new("L", (480, 320), 0)
        ImageDraw.Draw(mask).polygon(
            (
                edge_a,
                edge_b,
                (edge_b[0] + shift_x, edge_b[1] + shift_y),
                (edge_a[0] + shift_x, edge_a[1] + shift_y),
            ),
            fill=255,
        )
        cache[cache_key] = mask
        return mask

    def _replica_track_mask(
        self,
        key: str,
        path: tuple[tuple[float, float], ...],
        width: int,
    ) -> Image.Image:
        cache = getattr(self, "_replica_track_masks", None)
        if cache is None:
            cache = {}
            self._replica_track_masks = cache
        cache_key = (key, width)
        if cache_key in cache:
            return cache[cache_key]

        supersample = 2
        mask = Image.new("L", (480 * supersample, 320 * supersample), 0)
        mask_draw = ImageDraw.Draw(mask)
        points = tuple(
            (round(x * supersample), round(y * supersample))
            for x, y in path
        )
        mask_draw.line(
            points,
            fill=255,
            width=width * supersample,
            joint="curve",
        )
        radius = width * supersample / 2.0
        endpoints = [points[-1]]
        if not key.startswith("phase-"):
            endpoints.insert(0, points[0])
        for x, y in endpoints:
            mask_draw.ellipse(
                (x - radius, y - radius, x + radius, y + radius),
                fill=255,
            )
        mask = mask.resize((480, 320), Image.Resampling.LANCZOS)
        cache[cache_key] = mask
        return mask

    def _replica_segment_reveal_mask(
        self,
        path: tuple[tuple[float, float], ...],
        start: float,
        end: float,
        width: int,
        *,
        round_start: bool,
        round_end: bool,
    ) -> Image.Image:
        mask = Image.new("L", (480, 320), 0)
        lengths = [
            math.hypot(
                path[index + 1][0] - path[index][0],
                path[index + 1][1] - path[index][1],
            )
            for index in range(len(path) - 1)
        ]
        total = sum(lengths)
        points = [self._mode_track_location(path, start)[:2]]
        travelled = 0.0
        for index, length in enumerate(lengths):
            travelled += length
            fraction = travelled / total
            if start < fraction < end:
                points.append(path[index + 1])
        points.append(self._mode_track_location(path, end)[:2])

        mask_draw = ImageDraw.Draw(mask)
        mask_draw.line(points, fill=255, width=width, joint="curve")
        radius = width / 2.0
        endpoints = []
        if round_start:
            endpoints.append(points[0])
        if round_end:
            endpoints.append(points[-1])
        for x, y in endpoints:
            mask_draw.ellipse(
                (x - radius, y - radius, x + radius, y + radius),
                fill=255,
            )
        return mask

    def _replica_track_tick(
        self,
        draw: ImageDraw.ImageDraw,
        path: tuple[tuple[float, float], ...],
        fraction: float,
        value: float,
        *,
        outward: bool,
        color: str | None = None,
    ) -> None:
        x, y, tangent_x, tangent_y = self._mode_track_location(path, fraction)
        normal_x, normal_y = -tangent_y, tangent_x
        direction = -1.0 if outward else 1.0
        x0 = x + normal_x * direction * 4.0
        y0 = y + normal_y * direction * 4.0
        x1 = x + normal_x * direction * 10.0
        y1 = y + normal_y * direction * 10.0
        if color is None:
            color = REPLICA_BLACK if outward else REPLICA_MUTED
        draw.line((x0, y0, x1, y1), fill=color, width=2)
        label_x = x + normal_x * direction * 17.0
        label_y = y + normal_y * direction * 17.0
        label = f"{int(value)}"
        draw.text(
            (label_x, label_y), label,
            font=self.fonts.medium(7 if outward else 6),
            fill=color, anchor="mm",
        )

    def _replica_minor_tick(
        self,
        draw: ImageDraw.ImageDraw,
        path: tuple[tuple[float, float], ...],
        fraction: float,
        outward: bool,
        *,
        color: str | None = None,
    ) -> None:
        x, y, tangent_x, tangent_y = self._mode_track_location(path, fraction)
        normal_x, normal_y = -tangent_y, tangent_x
        direction = -1.0 if outward else 1.0
        draw.line(
            (
                x + normal_x * direction * 4.0,
                y + normal_y * direction * 4.0,
                x + normal_x * direction * 7.0,
                y + normal_y * direction * 7.0,
            ),
            fill=color or REPLICA_MUTED,
            width=1,
        )

    def _replica_segment_gauge(
        self,
        draw: ImageDraw.ImageDraw,
        x: int,
        label: str,
        value: float,
        minimum: float,
        maximum: float,
        valid: bool,
        value_text: str,
        critical: bool,
    ) -> None:
        top = 94
        width = 19
        height = 8
        gap = 4
        count = 8
        ratio = self._ratio(value, minimum, maximum) if valid else 0.0
        filled = round(ratio * count)
        fill_color = Color.FAULT if critical else REPLICA_BLACK
        for index in range(count):
            y = top + index * (height + gap)
            from_bottom = count - index
            box = (x, y, x + width, y + height)
            if valid and from_bottom <= filled:
                draw.rectangle(box, fill=fill_color)
            else:
                draw.rectangle(box, fill="#F2F3F3", outline=REPLICA_BLACK)
        bottom = top + count * height + (count - 1) * gap
        draw.text(
            (x + width // 2, bottom + 9), label,
            font=self.fonts.medium(7), fill=REPLICA_MUTED, anchor="mm",
        )
        draw.text(
            (x + width // 2, bottom + 21), value_text,
            font=self.fonts.display_bold(10),
            fill=Color.FAULT if critical else REPLICA_BLACK,
            anchor="mm",
        )

    def _replica_bottom_bar(
        self,
        draw: ImageDraw.ImageDraw,
        sample: ScenarioSample,
    ) -> None:
        values = sample.values
        draw.rounded_rectangle(
            (55, 263, 476, 314), radius=4, fill="#C3C5C5"
        )
        draw.polygon(
            ((3, 314), (58, 263), (63, 263), (63, 314)),
            fill="#C3C5C5",
        )
        draw.rounded_rectangle(
            (55, 258, 474, 310), radius=4,
            fill="#F4F4F3", outline="#D0D2D1",
        )
        draw.polygon(
            ((4, 310), (58, 258), (63, 258), (63, 310)),
            fill="#F4F4F3",
        )
        draw.line((4, 310, 58, 258, 470, 258), fill="#D0D2D1", width=1)
        draw.line((4, 310, 470, 310), fill="#D0D2D1", width=1)

        if sample.dashboard_can_lost or not values.vehicle_valid:
            link_value = "LOST"
            link_color = Color.FAULT
        elif not values.detail_valid:
            link_value = "DATA"
            link_color = Color.FAULT
        elif not values.status(STATUS_PART1_VALID) or not values.status(
            STATUS_PART2_VALID
        ):
            link_value = "PART"
            link_color = Color.ORANGE
        else:
            link_value = "OK"
            link_color = REPLICA_BLACK

        tractive = values.drive(DRIVE_TRACTIVE_FEEDBACK)
        brake = values.status(STATUS_BRAKE_ACTIVE)
        estop = values.status(STATUS_ESTOP_ACTIVE)
        fault = bool(values.motor_fault_bits)
        gnss_value, gnss_color = self._gnss_short(values)
        status_cells = (
            (62, 137, "LINK", link_value, link_color),
            (
                137, 212, "TRACT",
                "ON" if tractive else "OFF",
                REPLICA_BLACK if tractive else REPLICA_MUTED,
            ),
            (212, 277, "BRAKE", "ON" if brake else "OFF",
             Color.FAULT if brake else REPLICA_MUTED),
            (277, 347, "E-STOP", "ON" if estop else "OFF",
             Color.FAULT if estop else REPLICA_MUTED),
            (347, 407, "GNSS", gnss_value, gnss_color),
            (407, 471, "FAULT", "ON" if fault else "OK",
             Color.FAULT if fault else REPLICA_BLACK),
        )
        for index, (x0, x1, label, value, color) in enumerate(status_cells):
            if index:
                draw.line((x0, 267, x0, 302), fill=REPLICA_ORANGE, width=2)
            self._replica_status_item(draw, x0, x1, label, value, color)

    def _replica_status_item(
        self,
        draw: ImageDraw.ImageDraw,
        x0: int,
        x1: int,
        label: str,
        value: str,
        color: str,
    ) -> None:
        center_x = (x0 + x1) // 2
        draw.text(
            (center_x, 274), label, font=self.fonts.medium(8),
            fill=REPLICA_MUTED, anchor="mm",
        )
        draw.text(
            (center_x, 295), value, font=self.fonts.display_bold(14),
            fill=color, anchor="mm",
        )

    def _replica_alert(
        self,
        draw: ImageDraw.ImageDraw,
        state: DriveSemantic,
    ) -> None:
        if not state.critical:
            return
        draw.rounded_rectangle(
            (281, 218, 470, 242), radius=4, fill=state.color
        )
        draw.text(
            (375, 230), state.title, font=self.fonts.display_bold(11),
            fill="#FFFFFF", anchor="mm",
        )

    def _draw_auto_gt3(
        self,
        draw: ImageDraw.ImageDraw,
        sample: ScenarioSample,
        dynamics: GaugeDynamics,
        policy: PreviewPolicy,
        now: float,
    ) -> None:
        values = sample.values
        state = self._semantic_drive(sample, now)
        self._fill_screen(draw, MODE_BG)
        draw.rectangle((8, 8, 471, 311), outline="#75817D", width=2)
        draw.line((151, 8, 151, 224), fill=MODE_RULE, width=2)
        draw.line((329, 8, 329, 224), fill=MODE_RULE, width=2)
        draw.line((8, 224, 471, 224), fill=MODE_RULE, width=2)

        gnss_text, gnss_color = self._gnss_status(values)
        draw.text(
            (240, 27), gnss_text, font=self.fonts.bold(8),
            fill=gnss_color, anchor="mm",
        )

        draw.text(
            (22, 27), "HV / BUS / CTRL", font=self.fonts.bold(8),
            fill=Color.MUTED, anchor="la",
        )
        self._manual_hv_gauge(draw, (22, 67, 49, 177), values)
        draw.text(
            (61, 76), self._voltage_text(values), font=self.fonts.bold(13),
            fill=(
                self._voltage_color(values.bus_voltage_decivolt / 10.0)
                if values.power_valid else Color.MUTED
            ),
            anchor="la",
        )
        draw.text(
            (61, 109),
            self._current_text(values.bus_current_deciamp, values.power_valid),
            font=self.fonts.bold(13),
            fill=self._bus_load_color(values.bus_current_deciamp / 10.0),
            anchor="la",
        )
        self._temperature_readout(
            draw, (70, 157), "controller",
            values.controller_temperature_decic / 10.0,
            values.thermal_valid,
            bool(values.motor_fault_bits & (1 << 4)),
            policy, now,
        )

        draw.text(
            (240, 77), self._speed_text(values), font=self.fonts.bold(48),
            fill=Color.TEXT, anchor="mm",
        )
        draw.text(
            (240, 114), "KM/H", font=self.fonts.bold(8),
            fill=Color.MUTED, anchor="mm",
        )
        draw.text(
            (296, 77), self._mode_gear_text(values), font=self.fonts.bold(11),
            fill=Color.TEXT if values.vehicle_valid else Color.MUTED,
            anchor="lm",
        )
        command_text, command_color = self._auto_command_state(values, state)
        draw.text(
            (240, 178), command_text, font=self.fonts.bold(12),
            fill=command_color, anchor="mm",
        )
        draw.text(
            (240, 202), "HOST CONTROL", font=self.fonts.bold(7),
            fill=Color.MUTED, anchor="mm",
        )

        phase_color = self._mode_phase_color(values, dynamics)
        draw.text(
            (344, 27), "PHASE / MOTOR", font=self.fonts.bold(8),
            fill=Color.MUTED, anchor="la",
        )
        draw.text(
            (458, 77),
            self._current_text(values.phase_current_deciamp, values.power_valid),
            font=self.fonts.bold(16), fill=phase_color, anchor="ra",
        )
        self._auto_motor_load(draw, (347, 98, 455, 117), values, dynamics)
        self._temperature_readout(
            draw, (364, 165), "motor",
            values.motor_temperature_decic / 10.0,
            values.thermal_valid,
            bool(values.motor_fault_bits & (1 << 5)),
            policy, now,
        )

        self._auto_progress(draw, values, state)
        self._mode_alert(draw, state)

    def _manual_hv_gauge(
        self,
        draw: ImageDraw.ImageDraw,
        box: tuple[int, int, int, int],
        values: DashboardValues,
    ) -> None:
        x0, y0, x1, y1 = box
        draw.rectangle(box, outline=Color.TEXT, width=2)
        inner = (x0 + 4, y0 + 4, x1 - 4, y1 - 4)
        draw.rectangle(inner, fill=MODE_TRACK)
        for fraction in (0.25, 0.5, 0.75):
            y = inner[3] - (inner[3] - inner[1]) * fraction
            draw.line((x0 - 4, y, x0, y), fill=MODE_RULE, width=1)
        if values.power_valid:
            voltage = values.bus_voltage_decivolt / 10.0
            ratio = self._ratio(voltage, HV_MIN_V, HV_MAX_V)
            top = inner[3] - round((inner[3] - inner[1]) * ratio)
            draw.rectangle(
                (inner[0], top, inner[2], inner[3]),
                fill=self._voltage_color(voltage),
            )

    def _temperature_readout(
        self,
        draw: ImageDraw.ImageDraw,
        position: tuple[int, int],
        kind: str,
        temperature_c: float,
        valid: bool,
        fault: bool,
        policy: PreviewPolicy,
        now: float,
    ) -> None:
        x, y = position
        color = self._temperature_band(
            kind, temperature_c, fault, policy, now
        ) if valid else MODE_TRACK
        if kind == "controller":
            draw.rectangle((x - 10, y - 10, x + 10, y + 10), fill=color, outline=Color.TEXT, width=1)
            for offset in (-6, 0, 6):
                draw.line((x - 14, y + offset, x - 10, y + offset), fill=Color.TEXT, width=1)
                draw.line((x + 10, y + offset, x + 14, y + offset), fill=Color.TEXT, width=1)
            label = "CTRL"
        else:
            draw.ellipse((x - 11, y - 11, x + 11, y + 11), fill=color, outline=Color.TEXT, width=1)
            draw.text((x, y), "M", font=self.fonts.bold(8), fill=Color.TEXT, anchor="mm")
            label = "MOTOR"
        draw.text(
            (x + 20, y - 7), label, font=self.fonts.bold(7),
            fill=Color.MUTED, anchor="la",
        )
        draw.text(
            (x + 20, y + 10),
            f"{temperature_c:.1f}°C" if valid else "--.-°C",
            font=self.fonts.bold(11), fill=Color.TEXT, anchor="la",
        )

    def _temperature_band(
        self,
        kind: str,
        temperature_c: float,
        fault: bool,
        policy: PreviewPolicy,
        now: float,
    ) -> str:
        if kind == "controller":
            critical = CONTROLLER_OVERHEAT_C
            green_start = critical * 0.50
            yellow_start = critical * 0.70
            orange_start = critical * 0.85
        else:
            critical = policy.motor_critical_c
            green_start = critical * 0.40
            yellow_start = critical * 0.65
            orange_start = policy.motor_warning_c
        if fault or temperature_c >= critical:
            return Color.FAULT if int(now * 4.0) % 2 == 0 else MODE_BG
        if temperature_c >= orange_start:
            return Color.ORANGE
        if temperature_c >= yellow_start:
            return Color.WARN
        if temperature_c >= green_start:
            return Color.OK
        return Color.BLUE

    def _auto_motor_load(
        self,
        draw: ImageDraw.ImageDraw,
        box: tuple[int, int, int, int],
        values: DashboardValues,
        dynamics: GaugeDynamics,
    ) -> None:
        draw.rectangle(box, fill=MODE_TRACK)
        if not values.power_valid:
            return
        x0, y0, x1, y1 = box
        ratio = min(1.0, dynamics.phase_a / PHASE_RATED_A)
        draw.rectangle(
            (x0, y0, x0 + round((x1 - x0) * ratio), y1),
            fill=self._mode_phase_color(values, dynamics),
        )

    def _auto_progress(
        self,
        draw: ImageDraw.ImageDraw,
        values: DashboardValues,
        state: DriveSemantic,
    ) -> None:
        command_live = values.drive(DRIVE_COMMAND_RECEIVED) and not values.drive(
            DRIVE_COMMAND_TIMEOUT
        )
        drive_live = state.active
        motion_live = values.vehicle_valid and abs(values.speed_centi_mps) > 10
        colors = (
            Color.FAULT if values.drive(DRIVE_COMMAND_TIMEOUT) else Color.AUTO if command_live else "#A8B2AE",
            Color.FAULT if state.critical and not state.active else Color.AUTO if drive_live else "#A8B2AE",
            Color.AUTO if motion_live else "#A8B2AE",
        )
        nodes = ((80, "HOST"), (240, "DRIVE"), (400, "MOTION"))
        for index, ((x, label), color) in enumerate(zip(nodes, colors)):
            draw.rectangle((x - 8, 252, x + 8, 268), fill=color)
            draw.text(
                (x, 292), label, font=self.fonts.bold(8),
                fill=Color.MUTED, anchor="mm",
            )
            if index < 2:
                next_x = nodes[index + 1][0]
                line_color = colors[index + 1]
                draw.line((x + 11, 260, next_x - 16, 260), fill=line_color, width=4)
                self._mode_direction_arrow_raw(
                    draw, (next_x - 22, 260), (next_x - 11, 260),
                    line_color, 7,
                )

    @staticmethod
    def _auto_command_state(
        values: DashboardValues,
        state: DriveSemantic,
    ) -> tuple[str, str]:
        if state.critical:
            return state.title, state.color
        if values.drive(DRIVE_COMMAND_RECEIVED):
            return "TRACKING", Color.AUTO
        return "WAITING", Color.MUTED

    @staticmethod
    def _mode_phase_color(
        values: DashboardValues,
        dynamics: GaugeDynamics,
    ) -> str:
        if values.motor_fault_bits & ~(1 << 4):
            return Color.FAULT
        if dynamics.phase_a <= PHASE_RATED_A * 0.60:
            return Color.OK
        if dynamics.phase_a <= PHASE_RATED_A * 0.80:
            return Color.WARN
        return Color.FAULT

    @staticmethod
    def _mode_gear_text(values: DashboardValues) -> str:
        if not values.vehicle_valid:
            return "--"
        return "REV" if values.status(STATUS_GEAR_REVERSE) else "FWD"

    @staticmethod
    def _mode_gear_letter(values: DashboardValues) -> str:
        if not values.vehicle_valid:
            return "-"
        return "R" if values.status(STATUS_GEAR_REVERSE) else "F"

    @staticmethod
    def _mode_direction_arrow_raw(
        draw: ImageDraw.ImageDraw,
        start: tuple[float, float],
        end: tuple[float, float],
        color: str,
        thickness: float,
    ) -> None:
        dx = end[0] - start[0]
        dy = end[1] - start[1]
        length = math.hypot(dx, dy)
        if length < 2.0:
            return
        ux, uy = dx / length, dy / length
        px, py = -uy, ux
        head_length = min(length * 0.42, max(11.0, thickness * 1.35))
        shaft_end = (end[0] - ux * head_length, end[1] - uy * head_length)
        shaft_half = max(1.5, thickness / 2.0)
        head_half = max(6.0, thickness * 0.9)
        draw.polygon(
            (
                (start[0] + px * shaft_half, start[1] + py * shaft_half),
                (shaft_end[0] + px * shaft_half, shaft_end[1] + py * shaft_half),
                (shaft_end[0] - px * shaft_half, shaft_end[1] - py * shaft_half),
                (start[0] - px * shaft_half, start[1] - py * shaft_half),
            ),
            fill=color,
        )
        draw.polygon(
            (
                end,
                (shaft_end[0] + px * head_half, shaft_end[1] + py * head_half),
                (shaft_end[0] - px * head_half, shaft_end[1] - py * head_half),
            ),
            fill=color,
        )

    @staticmethod
    def _mode_track_location(
        path: tuple[tuple[float, float], ...],
        fraction: float,
    ) -> tuple[float, float, float, float]:
        fraction = max(0.0, min(1.0, fraction))
        lengths = [
            math.hypot(
                path[index + 1][0] - path[index][0],
                path[index + 1][1] - path[index][1],
            )
            for index in range(len(path) - 1)
        ]
        target = sum(lengths) * fraction
        travelled = 0.0
        for index, segment_length in enumerate(lengths):
            if travelled + segment_length >= target or index == len(lengths) - 1:
                local = (
                    0.0 if segment_length == 0.0
                    else (target - travelled) / segment_length
                )
                x0, y0 = path[index]
                x1, y1 = path[index + 1]
                tangent_x = (x1 - x0) / segment_length
                tangent_y = (y1 - y0) / segment_length
                return (
                    x0 + (x1 - x0) * local,
                    y0 + (y1 - y0) * local,
                    tangent_x,
                    tangent_y,
                )
            travelled += segment_length
        x0, y0 = path[-2]
        x1, y1 = path[-1]
        segment_length = math.hypot(x1 - x0, y1 - y0)
        return x1, y1, (x1 - x0) / segment_length, (y1 - y0) / segment_length

    def _mode_alert(
        self,
        draw: ImageDraw.ImageDraw,
        state: DriveSemantic,
    ) -> None:
        if not state.critical:
            return
        draw.rectangle((135, 0, 345, 30), fill=state.color)
        draw.text(
            (240, 15), state.title, font=self.fonts.bold(11),
            fill="#FFFFFF", anchor="mm",
        )
