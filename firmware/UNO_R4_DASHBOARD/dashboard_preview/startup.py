"""KAI symbol startup sequence for the 480x320 preview."""

from __future__ import annotations

from PIL import Image, ImageChops, ImageDraw, ImageFilter

from .model import LCD_HEIGHT, LCD_WIDTH


STARTUP_DURATION_S = 1.4
_REVEAL_END_S = 0.35
_HOLD_END_S = 1.10


def _smoothstep(value: float) -> float:
    value = max(0.0, min(1.0, value))
    return value * value * (3.0 - 2.0 * value)


class StartupRenderer:
    """Render a fixed KAI symbol with a restrained outline reveal."""

    def __init__(self) -> None:
        self._outline = self._build_outline_mask()
        self._haze = self._outline.filter(ImageFilter.GaussianBlur(4.5))

    def render(self, elapsed_s: float) -> Image.Image:
        elapsed_s = max(0.0, min(STARTUP_DURATION_S, elapsed_s))
        image = Image.new("RGB", (LCD_WIDTH, LCD_HEIGHT), "#000000")

        if elapsed_s < _REVEAL_END_S:
            progress = _smoothstep(elapsed_s / _REVEAL_END_S)
            reveal = self._reveal_mask(progress)
            outline = ImageChops.multiply(self._outline, reveal)
            haze = ImageChops.multiply(self._haze, reveal)
            haze_strength = 0.12
            frontier = self._frontier_mask(progress)
        elif elapsed_s < _HOLD_END_S:
            progress = (elapsed_s - _REVEAL_END_S) / (
                _HOLD_END_S - _REVEAL_END_S
            )
            outline = self._outline
            haze = self._haze
            haze_strength = 0.12 * (1.0 - _smoothstep(progress)) + 0.02
            frontier = None
        else:
            fade = 1.0 - _smoothstep(
                (elapsed_s - _HOLD_END_S) /
                (STARTUP_DURATION_S - _HOLD_END_S)
            )
            outline = self._outline.point(lambda value: round(value * fade))
            haze = self._haze
            haze_strength = 0.02 * fade
            frontier = None

        haze_mask = haze.point(
            lambda value: round(value * haze_strength)
        )
        image.paste((205, 216, 224), mask=haze_mask)
        image.paste((238, 242, 245), mask=outline)

        if frontier is not None:
            leading_edge = ImageChops.multiply(outline, frontier)
            image.paste((255, 255, 255), mask=leading_edge)
        return image

    @staticmethod
    def _build_outline_mask() -> Image.Image:
        supersample = 2
        mask = Image.new(
            "L", (LCD_WIDTH * supersample, LCD_HEIGHT * supersample), 0
        )
        draw = ImageDraw.Draw(mask)

        # Traced from the supplied 401x138 high-resolution KAI symbol.
        source_width = 401.0
        target_width = 320.0 * supersample
        scale = target_width / source_width
        origin_x = (LCD_WIDTH * supersample - target_width) / 2.0
        origin_y = 105.0 * supersample

        def points(source: tuple[tuple[int, int], ...]) -> list[tuple[int, int]]:
            return [
                (
                    round(origin_x + x * scale),
                    round(origin_y + y * scale),
                )
                for x, y in source
            ]

        long_stroke = (
            (400, 0),
            (0, 1),
            (0, 20),
            (350, 20),
            (259, 111),
            (238, 98),
            (202, 98),
            (262, 137),
        )
        short_stroke = (
            (99, 38),
            (296, 38),
            (276, 58),
            (99, 58),
        )
        draw.polygon(points(long_stroke), fill=255)
        draw.polygon(points(short_stroke), fill=255)

        # A two-pixel inner edge keeps the original silhouette hollow.
        eroded = mask.filter(ImageFilter.MinFilter(9))
        outline = ImageChops.subtract(mask, eroded)
        return outline.resize(
            (LCD_WIDTH, LCD_HEIGHT), Image.Resampling.LANCZOS
        )

    @staticmethod
    def _reveal_mask(progress: float) -> Image.Image:
        mask = Image.new("L", (LCD_WIDTH, LCD_HEIGHT), 0)
        draw = ImageDraw.Draw(mask)
        boundary_x = 60.0 + progress * 370.0
        draw.polygon(
            (
                (0, 0),
                (boundary_x + 20, 0),
                (boundary_x - 20, LCD_HEIGHT),
                (0, LCD_HEIGHT),
            ),
            fill=255,
        )
        return mask

    @staticmethod
    def _frontier_mask(progress: float) -> Image.Image:
        mask = Image.new("L", (LCD_WIDTH, LCD_HEIGHT), 0)
        draw = ImageDraw.Draw(mask)
        boundary_x = 60.0 + progress * 370.0
        band = 10.0
        draw.polygon(
            (
                (boundary_x + 20 - band, 0),
                (boundary_x + 20 + band, 0),
                (boundary_x - 20 + band, LCD_HEIGHT),
                (boundary_x - 20 - band, LCD_HEIGHT),
            ),
            fill=150,
        )
        return mask
