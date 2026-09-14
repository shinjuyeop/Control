"""Static MANUAL dashboard layout and current-track masks."""

from __future__ import annotations

import math

from PIL import Image, ImageChops, ImageDraw

BUS_10_SECOND_A = 350.0
BUS_30_SECOND_A = 300.0
PHASE_RATED_A = 480.0
REDLINE_COLOR = "#D92D20"
BUS_REGEN_DISPLAY_A = 120.0
REPLICA_ORANGE = "#ED4A0A"
REPLICA_BLACK = "#171818"
REPLICA_MUTED = "#777A7A"


class ManualLayout:
    def _draw_manual_static(self, draw: ImageDraw.ImageDraw) -> None:
        self._replica_background(draw)
        self._replica_gear_panel(draw)
        self._replica_current_tracks(draw)

        draw.text(
            (361, 111), "KM/H", font=self.fonts.medium(12),
            fill="#4D4F4F", anchor="ra",
        )
        for x, label in ((376, "PED"), (402, "HV"), (428, "MTR"), (454, "CTRL")):
            self._replica_segment_gauge(draw, x, label)
        self._replica_bottom_bar(draw)

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
            REDLINE_COLOR,
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
            REDLINE_COLOR,
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
                color=REDLINE_COLOR if value >= 300.0 else None,
            )
        for value in (0.0, 240.0, 480.0):
            fraction = value / PHASE_RATED_A
            self._replica_track_tick(
                draw,
                phase_path,
                fraction,
                value,
                outward=False,
                color=REDLINE_COLOR if fraction >= phase_redline else None,
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
                        REDLINE_COLOR
                        if bus_tick_value >= BUS_30_SECOND_A
                        else None
                    ),
                )
                self._replica_minor_tick(
                    draw,
                    phase_path,
                    fraction,
                    False,
                    color=REDLINE_COLOR if fraction >= phase_redline else None,
                )

        draw.text(
            (126, 211), "BUS", font=self.fonts.medium(8),
            fill=REPLICA_MUTED, anchor="mm",
        )
        draw.text(
            (197, 211), "PHASE", font=self.fonts.medium(8),
            fill=REPLICA_MUTED, anchor="mm",
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
    ) -> None:
        top = 94
        width = 19
        height = 8
        gap = 4
        count = 8
        for index in range(count):
            y = top + index * (height + gap)
            box = (x, y, x + width, y + height)
            draw.rectangle(box, fill="#F2F3F3", outline=REPLICA_BLACK)
        bottom = top + count * height + (count - 1) * gap
        draw.text(
            (x + width // 2, bottom + 9), label,
            font=self.fonts.medium(7), fill=REPLICA_MUTED, anchor="mm",
        )

    def _replica_bottom_bar(
        self,
        draw: ImageDraw.ImageDraw,
    ) -> None:
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

        status_cells = (
            (62, 137, "LINK"),
            (137, 212, "TRACT"),
            (212, 277, "BRAKE"),
            (277, 347, "E-STOP"),
            (347, 407, "GNSS"),
            (407, 471, "FAULT"),
        )
        for index, (x0, x1, label) in enumerate(status_cells):
            if index:
                draw.line((x0, 267, x0, 302), fill=REPLICA_ORANGE, width=2)
            center_x = (x0 + x1) // 2
            draw.text(
                (center_x, 274), label, font=self.fonts.medium(8),
                fill="#777A7A", anchor="mm",
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
