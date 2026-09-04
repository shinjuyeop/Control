"""Shared dashboard colors and fonts."""

from __future__ import annotations

from pathlib import Path

from PIL import ImageFont

FONT5X7 = {
    " ": (0x00, 0x00, 0x00, 0x00, 0x00),
    "!": (0x00, 0x00, 0x5F, 0x00, 0x00),
    "%": (0x63, 0x13, 0x08, 0x64, 0x63),
    "+": (0x08, 0x08, 0x3E, 0x08, 0x08),
    "-": (0x08, 0x08, 0x08, 0x08, 0x08),
    ".": (0x00, 0x60, 0x60, 0x00, 0x00),
    "/": (0x20, 0x10, 0x08, 0x04, 0x02),
    "0": (0x3E, 0x51, 0x49, 0x45, 0x3E),
    "1": (0x00, 0x42, 0x7F, 0x40, 0x00),
    "2": (0x42, 0x61, 0x51, 0x49, 0x46),
    "3": (0x21, 0x41, 0x45, 0x4B, 0x31),
    "4": (0x18, 0x14, 0x12, 0x7F, 0x10),
    "5": (0x27, 0x45, 0x45, 0x45, 0x39),
    "6": (0x3C, 0x4A, 0x49, 0x49, 0x30),
    "7": (0x01, 0x71, 0x09, 0x05, 0x03),
    "8": (0x36, 0x49, 0x49, 0x49, 0x36),
    "9": (0x06, 0x49, 0x49, 0x29, 0x1E),
    ":": (0x00, 0x36, 0x36, 0x00, 0x00),
    "?": (0x02, 0x01, 0x51, 0x09, 0x06),
    "A": (0x7E, 0x11, 0x11, 0x11, 0x7E),
    "B": (0x7F, 0x49, 0x49, 0x49, 0x36),
    "C": (0x3E, 0x41, 0x41, 0x41, 0x22),
    "D": (0x7F, 0x41, 0x41, 0x22, 0x1C),
    "E": (0x7F, 0x49, 0x49, 0x49, 0x41),
    "F": (0x7F, 0x09, 0x09, 0x09, 0x01),
    "G": (0x3E, 0x41, 0x49, 0x49, 0x7A),
    "H": (0x7F, 0x08, 0x08, 0x08, 0x7F),
    "I": (0x00, 0x41, 0x7F, 0x41, 0x00),
    "J": (0x20, 0x40, 0x41, 0x3F, 0x01),
    "K": (0x7F, 0x08, 0x14, 0x22, 0x41),
    "L": (0x7F, 0x40, 0x40, 0x40, 0x40),
    "M": (0x7F, 0x02, 0x0C, 0x02, 0x7F),
    "N": (0x7F, 0x04, 0x08, 0x10, 0x7F),
    "O": (0x3E, 0x41, 0x41, 0x41, 0x3E),
    "P": (0x7F, 0x09, 0x09, 0x09, 0x06),
    "Q": (0x3E, 0x41, 0x51, 0x21, 0x5E),
    "R": (0x7F, 0x09, 0x19, 0x29, 0x46),
    "S": (0x46, 0x49, 0x49, 0x49, 0x31),
    "T": (0x01, 0x01, 0x7F, 0x01, 0x01),
    "U": (0x3F, 0x40, 0x40, 0x40, 0x3F),
    "V": (0x1F, 0x20, 0x40, 0x20, 0x1F),
    "W": (0x7F, 0x20, 0x18, 0x20, 0x7F),
    "X": (0x63, 0x14, 0x08, 0x14, 0x63),
    "Y": (0x03, 0x04, 0x78, 0x04, 0x03),
    "Z": (0x61, 0x51, 0x49, 0x45, 0x43),
}

def _rgb565(value: int) -> tuple[int, int, int]:
    red = ((value >> 11) & 0x1F) * 255 // 31
    green = ((value >> 5) & 0x3F) * 255 // 63
    blue = (value & 0x1F) * 255 // 31
    return red, green, blue

class BaselineColor:
    BLACK = _rgb565(0x0000)
    WHITE = _rgb565(0xFFFF)
    RED = _rgb565(0xF800)
    GREEN = _rgb565(0x07E0)
    BLUE = _rgb565(0x045F)
    ORANGE = _rgb565(0xFC60)
    LIGHT_GRAY = _rgb565(0xE71C)
    DARK_GRAY = _rgb565(0x4208)

class Color:
    BG = "#F2F5F4"
    TEXT = "#142128"
    MUTED = "#66757D"
    BORDER = "#D5DEE1"
    AUTO = "#1769D2"
    MANUAL = "#F2A900"
    OK = "#00A878"
    BLUE = "#2F80ED"
    WARN = "#F2B705"
    ORANGE = "#F27922"
    FAULT = "#D92D20"

class DarkColor:
    BG = "#060A0C"
    SURFACE = "#0D1418"
    TEXT = "#F4F7F8"
    MUTED = "#829198"
    GRID = "#263238"
    INACTIVE = "#1A2429"
    CYAN = "#00D6D1"
    GREEN = "#18C37E"
    YELLOW = "#F2D63D"
    RED = "#FF3B30"

class FontBook:
    _DISPLAY_MEDIUM_PATHS = (
        str(Path.home() / ".local/share/fonts/BlenderPro-Medium.ttf"),
        "/usr/share/fonts/truetype/roboto/unhinted/RobotoCondensed-Medium.ttf",
        "/usr/share/fonts/truetype/dejavu/DejaVuSansCondensed.ttf",
    )
    _DISPLAY_BOLD_PATHS = (
        str(Path.home() / ".local/share/fonts/BlenderPro-Bold.ttf"),
        "/usr/share/fonts/truetype/roboto/unhinted/RobotoCondensed-Bold.ttf",
        "/usr/share/fonts/truetype/dejavu/DejaVuSansCondensed-Bold.ttf",
    )
    _DISPLAY_HEAVY_PATHS = (
        str(Path.home() / ".local/share/fonts/BlenderPro-Heavy.ttf"),
        "/usr/share/fonts/truetype/roboto/unhinted/RobotoCondensed-Bold.ttf",
        "/usr/share/fonts/truetype/dejavu/DejaVuSansCondensed-Bold.ttf",
    )
    _BOLD_PATHS = (
        "/usr/share/fonts/truetype/roboto/unhinted/RobotoCondensed-Bold.ttf",
        "/usr/share/fonts/truetype/dejavu/DejaVuSansCondensed-Bold.ttf",
        "/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf",
    )
    _REGULAR_PATHS = (
        "/usr/share/fonts/truetype/roboto/unhinted/RobotoCondensed-Regular.ttf",
        "/usr/share/fonts/truetype/liberation/LiberationSansNarrow-Regular.ttf",
        "/usr/share/fonts/truetype/dejavu/DejaVuSansCondensed.ttf",
    )
    def __init__(self) -> None:
        self._cache: dict[tuple[str, int], ImageFont.ImageFont] = {}

    def bold(self, size: int) -> ImageFont.ImageFont:
        return self._load("bold", size, self._BOLD_PATHS)

    def regular(self, size: int) -> ImageFont.ImageFont:
        return self._load("regular", size, self._REGULAR_PATHS)

    def medium(self, size: int) -> ImageFont.ImageFont:
        return self._load("display-medium", size, self._DISPLAY_MEDIUM_PATHS)

    def display_bold(self, size: int) -> ImageFont.ImageFont:
        return self._load("display-bold", size, self._DISPLAY_BOLD_PATHS)

    def heavy(self, size: int) -> ImageFont.ImageFont:
        return self._load("display-heavy", size, self._DISPLAY_HEAVY_PATHS)

    def _load(
        self, family: str, size: int, candidates: tuple[str, ...]
    ) -> ImageFont.ImageFont:
        key = (family, size)
        if key in self._cache:
            return self._cache[key]
        for candidate in candidates:
            if Path(candidate).is_file():
                font = ImageFont.truetype(candidate, size=size)
                self._cache[key] = font
                return font
        font = ImageFont.load_default()
        self._cache[key] = font
        return font
