"""Generate UNO R4 raster assets from the canonical dashboard preview."""

from __future__ import annotations

from dataclasses import dataclass
import math
from pathlib import Path

from PIL import Image, ImageChops, ImageDraw, ImageFilter, ImageFont

from .auto_layout import AutoLayout
from .manual_layout import ManualLayout


LCD_WIDTH = 480
LCD_HEIGHT = 320
ASSET_DIR = Path(__file__).resolve().parents[1] / "src" / "assets"
OUTPUT = ASSET_DIR / "manual_tft_assets.h"
AUTO_OUTPUT = ASSET_DIR / "auto_tft_assets.h"
AUTO_DYNAMIC_OUTPUT = (
    ASSET_DIR / "auto_tft_dynamic_assets.h"
)
STARTUP_OUTPUT = ASSET_DIR / "startup_tft_assets.h"


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

    def __init__(self) -> None:
        self._cache: dict[tuple[str, int], ImageFont.ImageFont] = {}

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


class AssetRenderer(ManualLayout, AutoLayout):
    def __init__(self) -> None:
        self.fonts = FontBook()


def rgb565(color: tuple[int, int, int]) -> int:
    red, green, blue = color
    return ((red >> 3) << 11) | ((green >> 2) << 5) | (blue >> 3)


def build_manual_static(renderer: AssetRenderer) -> Image.Image:
    image = Image.new("RGB", (480, 320))
    draw = ImageDraw.Draw(image)
    renderer._draw_manual_static(draw)
    return image


def build_auto_static(renderer: AssetRenderer) -> Image.Image:
    image = Image.new("RGB", (480, 320), "#FFFFFF")
    draw = ImageDraw.Draw(image)
    renderer._auto_static_shell(draw)
    draw.ellipse((396, 141, 404, 149), fill="#131C25")
    return image


def rle_pixels(image: Image.Image) -> bytes:
    pixels = [
        rgb565(color)
        for color in image.convert("RGB").get_flattened_data()
    ]
    output = bytearray()
    current = pixels[0]
    count = 1
    for value in pixels[1:]:
        if value == current and count < 255:
            count += 1
            continue
        output.extend((count, current >> 8, current & 0xFF))
        current = value
        count = 1
    output.extend((count, current >> 8, current & 0xFF))
    return bytes(output)


def rle_rows(image: Image.Image) -> tuple[list[int], bytes]:
    image = image.convert("RGB")
    offsets = [0]
    data = bytearray()
    for y in range(image.height):
        row = image.crop((0, y, image.width, y + 1))
        data.extend(rle_pixels(row))
        offsets.append(len(data))
    return offsets, bytes(data)


@dataclass(frozen=True)
class FontSpec:
    name: str
    family: str
    size: int
    characters: str


FONT_SPECS = (
    FontSpec("gear", "heavy", 59, "-FR"),
    FontSpec("speed64", "heavy", 64, "-.0123456789"),
    FontSpec("speed57", "heavy", 57, "-.0123456789"),
    FontSpec("current", "display_bold", 13, "+-.A0123456789"),
    FontSpec("gauge", "display_bold", 10, "-.%VC°0123456789"),
    FontSpec("status", "display_bold", 14,
             "-ABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789"),
    FontSpec("alert", "display_bold", 11,
             " -ABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789"),
)


def used_characters(*values: str) -> str:
    return "".join(sorted(set("".join(values))))


AUTO_FONT_SPECS = (
    FontSpec("badge_label", "medium", 11, used_characters(
        "GPS", "LIDAR", "CAMERA L", "CAMERA R", "DRIVE", "STEER",
        "E-STOP", "BRAKE CMD", "NOW", "TARGET")),
    FontSpec("badge_state", "display_bold", 12, used_characters(
        "FIXED", "FLOAT", "NO FIX", "WAIT", "RX", "LOST", "RUN", "HOLD",
        "CMD LOST", "NO FEEDBACK", "RUN MISMATCH", "FAULT", "STATUS LOST",
        "WAIT CMD", "ACTIVE", "MOTOR FAULT", "CAN LOST", "POS LOST",
        "TRACK FAULT", "TRACK WARN", "BRAKE", "E-STOP", "DISABLED", "OFF", "ON")),
    FontSpec("message", "display_bold", 13, used_characters(
        "DASH CAN LOST", "E-STOP ACTIVE", "AUTO DATA LOST", "STEERING STATUS",
        "STEER MOTOR FAULT", "STEER CAN LOST", "STEER POSITION", "FEEDBACK LOST",
        "STEER COMMAND OR", "MODE LOST", "STEER TRACKING", "DRIVE COMMAND LOST",
        "DRIVE MOTOR", "DRIVE RUN FEEDBACK", "MISMATCH", "DRIVE STATUS LOST",
        "BRAKE COMMAND", "INVALID", "BRAKE STATUS LOST", "LIDAR INPUT LOST",
        "CAMERA L INPUT", "CAMERA R INPUT", "GPS NO FIX", "VEHICLE HEADING",
        "UNAVAILABLE", "TARGET HEADING", "WARNING", "FOLLOW ROUTE", "REJOIN ROUTE",
        "TRAFFIC CONTROL", "AVOID OBSTACLE", "INTERSECTION", "FINISH",
        "EMERGENCY STOP", "AUTO ACTIVE", "WAITING FOR")),
    FontSpec("speed", "heavy", 44, "-.0123456789"),
    FontSpec("target_speed", "display_bold", 32, "-.0123456789"),
    FontSpec("reading", "display_bold", 24, "+-.°0123456789"),
    FontSpec("current_large", "heavy", 36, " -A0123456789"),
    FontSpec("current_small", "heavy", 30, " -A0123456789"),
    FontSpec("steer_state", "display_bold", 17, used_characters(
        "ACTIVE", "WAIT CMD", "STATUS LOST", "MOTOR FAULT", "CAN LOST", "POS LOST",
        "CMD LOST", "TRACK FAULT", "TRACK WARN", "BRAKE", "E-STOP", "DISABLED", "WAIT")),
    FontSpec("trace", "medium", 10, " 5sPEAK0123456789-"),
)


def font_for(book: FontBook, spec: FontSpec):
    return getattr(book, spec.family)(spec.size)


def glyph_code(character: str) -> int:
    return 0xB0 if character == "°" else ord(character)


def emit_bytes(name: str, data: bytes) -> str:
    lines = [f"static const uint8_t {name}[] = {{"]
    for index in range(0, len(data), 16):
        chunk = data[index:index + 16]
        lines.append("    " + ", ".join(f"0x{value:02X}U" for value in chunk) + ",")
    lines.append("};")
    return "\n".join(lines)


def emit_u32(name: str, values: list[int]) -> str:
    lines = [f"static const uint32_t {name}[] = {{"]
    for index in range(0, len(values), 8):
        chunk = values[index:index + 8]
        lines.append("    " + ", ".join(f"{value}UL" for value in chunk) + ",")
    lines.append("};")
    return "\n".join(lines)


def emit_u16(name: str, values: list[int]) -> str:
    lines = [f"static const uint16_t {name}[] = {{"]
    for index in range(0, len(values), 12):
        chunk = values[index:index + 12]
        lines.append("    " + ", ".join(f"{value}U" for value in chunk) + ",")
    lines.append("};")
    return "\n".join(lines)


def emit_font(book: FontBook, spec: FontSpec) -> str:
    font = font_for(book, spec)
    raw_glyphs = []
    raw_size = 0
    for character in spec.characters:
        mask, offset = font.getmask2(character, mode="L", anchor="mm")
        data = bytes(mask)
        raw_glyphs.append((character, offset, mask.size, data))
        raw_size += len(data)
    compressed_size = sum(
        sum(len(rle_alpha(item[3][row * item[2][0]:(row + 1) * item[2][0]]))
            for row in range(item[2][1]))
        for item in raw_glyphs
    )
    use_rle = compressed_size < raw_size

    alpha = bytearray()
    glyph_rows = []
    row_offsets: list[int] = []
    minimum_top = 127
    maximum_bottom = -128
    for character, offset, size, raw in raw_glyphs:
        width, height = size
        row_index = len(row_offsets)
        if use_rle:
            encoded_buffer = bytearray()
            row_offsets.append(0)
            for row in range(height):
                encoded_buffer.extend(rle_alpha(raw[row * width:(row + 1) * width]))
                row_offsets.append(len(encoded_buffer))
            encoded = bytes(encoded_buffer)
        else:
            encoded = raw
        glyph_rows.append(
            (glyph_code(character), offset[0], offset[1], width, height,
             round(font.getlength(character) * 64), len(alpha), len(encoded), row_index)
        )
        alpha.extend(encoded)
        minimum_top = min(minimum_top, offset[1])
        maximum_bottom = max(maximum_bottom, offset[1] + height)
    lines = [emit_bytes(f"manual_font_{spec.name}_alpha", bytes(alpha)), ""]
    if use_rle:
        lines.extend((emit_u16(f"manual_font_{spec.name}_row_offsets", row_offsets), ""))
    lines.append(f"static const TftGlyphAsset manual_font_{spec.name}_glyphs[] = {{")
    for code, left, top, width, height, advance, offset, length, row_index in glyph_rows:
        lines.append(
            f"    {{{code}U, {left}, {top}, {width}U, {height}U, "
            f"{advance}U, {offset}UL, {length}U, {row_index}U}},"
        )
    lines.append("};")
    lines.append(
        f"static const TftFontAsset manual_font_{spec.name} = {{"
        f"manual_font_{spec.name}_glyphs, "
        f"sizeof(manual_font_{spec.name}_glyphs) / "
        f"sizeof(manual_font_{spec.name}_glyphs[0]), "
        f"manual_font_{spec.name}_alpha, "
        f"{'manual_font_' + spec.name + '_row_offsets' if use_rle else 'nullptr'}, "
        f"{1 if use_rle else 0}U, "
        f"{minimum_top}, {maximum_bottom}}};"
    )
    return "\n".join(lines)


def rle_alpha(data: bytes) -> bytes:
    if not data:
        return b""
    output = bytearray()
    value = data[0]
    count = 1
    for next_value in data[1:]:
        if next_value == value and count < 255:
            count += 1
        else:
            output.extend((count, value))
            value = next_value
            count = 1
    output.extend((count, value))
    return bytes(output)


def _build_startup_outline() -> Image.Image:
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


def build_startup_masks() -> tuple[
    tuple[int, int, int, int], int, list[tuple[int, int, int, int]], bytes, bytes
]:
    outline = _build_startup_outline()
    haze = outline.filter(ImageFilter.GaussianBlur(4.5))
    haze_maximum = max(haze.get_flattened_data())
    haze_quantized = haze.point(
        lambda value: round(value * 15 / haze_maximum) if haze_maximum else 0
    )
    bbox = haze.getbbox()
    if bbox is None:
        raise RuntimeError("startup masks are empty")

    x0, y0, x1, y1 = bbox
    rows: list[tuple[int, int, int, int]] = []
    outline_data = bytearray()
    haze_data = bytearray()
    for y in range(y0, y1):
        nonzero = [
            x for x in range(x0, x1)
            if outline.getpixel((x, y)) or haze_quantized.getpixel((x, y))
        ]
        if not nonzero:
            rows.append((0, 0, len(outline_data), len(haze_data)))
            continue
        row_x0 = min(nonzero)
        row_x1 = max(nonzero) + 1
        outline_row = bytes(
            outline.crop((row_x0, y, row_x1, y + 1)).get_flattened_data()
        )
        haze_row = bytes(
            haze_quantized.crop((row_x0, y, row_x1, y + 1)).get_flattened_data()
        )
        rows.append((row_x0, row_x1 - row_x0, len(outline_data), len(haze_data)))
        outline_data.extend(rle_alpha(outline_row))
        haze_data.extend(rle_alpha(haze_row))
    rows.append((0, 0, len(outline_data), len(haze_data)))
    return bbox, haze_maximum, rows, bytes(outline_data), bytes(haze_data)


def emit_startup_assets() -> tuple[int, tuple[int, int, int, int]]:
    bbox, haze_maximum, rows, outline_data, haze_data = build_startup_masks()
    sections = [
        "#ifndef STARTUP_TFT_ASSETS_H",
        "#define STARTUP_TFT_ASSETS_H",
        "",
        "#include <stdint.h>",
        "",
        "struct StartupTftMaskRow {",
        "  uint16_t x; uint16_t length; uint16_t outline_offset; uint16_t haze_offset;",
        "};",
        "",
        f"static constexpr uint16_t STARTUP_BBOX_X0 = {bbox[0]}U;",
        f"static constexpr uint16_t STARTUP_BBOX_Y0 = {bbox[1]}U;",
        f"static constexpr uint16_t STARTUP_BBOX_X1 = {bbox[2] - 1}U;",
        f"static constexpr uint16_t STARTUP_BBOX_Y1 = {bbox[3] - 1}U;",
        f"static constexpr uint8_t STARTUP_HAZE_MAXIMUM = {haze_maximum}U;",
        "",
        emit_bytes("startup_outline_data", outline_data),
        "",
        emit_bytes("startup_haze_data", haze_data),
        "",
        "static const StartupTftMaskRow startup_mask_rows[] = {",
    ]
    sections.extend(
        f"    {{{x}U, {length}U, {outline_offset}U, {haze_offset}U}},"
        for x, length, outline_offset, haze_offset in rows
    )
    sections.extend(("};", "", "#endif  // STARTUP_TFT_ASSETS_H", ""))
    STARTUP_OUTPUT.write_text("\n".join(sections), encoding="utf-8")
    return len(outline_data) + len(haze_data) + len(rows) * 8, bbox


def emit_auto_font(book: FontBook, spec: FontSpec) -> str:
    font = font_for(book, spec)
    raw_glyphs = []
    raw_size = 0
    for character in spec.characters:
        mask, offset = font.getmask2(character, mode="L", anchor="mm")
        data = bytes(mask)
        raw_glyphs.append((character, offset, mask.size, data))
        raw_size += len(data)
    compressed_size = sum(
        sum(len(rle_alpha(item[3][row * item[2][0]:(row + 1) * item[2][0]]))
            for row in range(item[2][1]))
        for item in raw_glyphs
    )
    use_rle = compressed_size < raw_size

    data = bytearray()
    glyph_rows = []
    row_offsets: list[int] = []
    minimum_top = 127
    maximum_bottom = -128
    for character, offset, size, raw in raw_glyphs:
        width, height = size
        row_index = len(row_offsets)
        if use_rle:
            encoded_buffer = bytearray()
            row_offsets.append(0)
            for row in range(height):
                encoded_buffer.extend(rle_alpha(raw[row * width:(row + 1) * width]))
                row_offsets.append(len(encoded_buffer))
            encoded = bytes(encoded_buffer)
        else:
            encoded = raw
        glyph_rows.append((glyph_code(character), offset[0], offset[1], width, height,
                           round(font.getlength(character) * 64), len(data), len(encoded),
                           row_index))
        data.extend(encoded)
        minimum_top = min(minimum_top, offset[1])
        maximum_bottom = max(maximum_bottom, offset[1] + height)

    prefix = f"auto_font_{spec.name}"
    lines = [emit_bytes(f"{prefix}_data", bytes(data)), ""]
    if use_rle:
        lines.extend((emit_u16(f"{prefix}_row_offsets", row_offsets), ""))
    lines.append(f"static const TftGlyphAsset {prefix}_glyphs[] = {{")
    for code, left, top, width, height, advance, offset, length, row_index in glyph_rows:
        lines.append(
            f"    {{{code}U, {left}, {top}, {width}U, {height}U, {advance}U, "
            f"{offset}UL, {length}U, {row_index}U}},"
        )
    lines.append("};")
    lines.append(
        f"static const TftFontAsset {prefix} = {{"
        f"{prefix}_glyphs, sizeof({prefix}_glyphs) / sizeof({prefix}_glyphs[0]), "
        f"{prefix}_data, {prefix + '_row_offsets' if use_rle else 'nullptr'}, "
        f"{1 if use_rle else 0}U, {minimum_top}, {maximum_bottom}}};"
    )
    return "\n".join(lines)


def dense_path(renderer: AssetRenderer, path) -> list[tuple[float, float]]:
    return [renderer._mode_track_location(path, index / 255.0)[:2]
            for index in range(256)]


def track_rows(renderer: AssetRenderer, path, width: int, key: str):
    mask = renderer._replica_track_mask(key, path, width)
    samples = dense_path(renderer, path)
    rows = []
    data = bytearray()
    for y in range(320):
        xs = [x for x in range(480) if mask.getpixel((x, y)) != 0]
        if not xs:
            rows.append((0, 0, len(data)))
            continue
        x0, x1 = min(xs), max(xs)
        offset = len(data)
        for x in range(x0, x1 + 1):
            alpha = mask.getpixel((x, y))
            if alpha == 0:
                data.extend((0, 0))
                continue
            progress = min(
                range(256),
                key=lambda index: (
                    (samples[index][0] - x) ** 2
                    + (samples[index][1] - y) ** 2
                ),
            )
            data.extend((alpha, progress))
        rows.append((x0, x1 - x0 + 1, offset))
    return rows, bytes(data)


def emit_track(name: str, rows, data: bytes) -> str:
    lines = [emit_bytes(f"manual_track_{name}_data", data), ""]
    lines.append(f"static const ManualTftTrackRow manual_track_{name}_rows[] = {{")
    for x, length, offset in rows:
        lines.append(f"    {{{x}U, {length}U, {offset}UL}},")
    lines.append("};")
    return "\n".join(lines)


def main() -> None:
    renderer = AssetRenderer()
    static = build_manual_static(renderer)
    row_offsets, row_data = rle_rows(static)

    bus_rows, bus_data = track_rows(
        renderer, renderer._replica_bus_path(), 10, "firmware-bus-active")
    phase_rows, phase_data = track_rows(
        renderer, renderer._replica_phase_path(), 9, "firmware-phase-active")

    sections = [
        "#ifndef MANUAL_TFT_ASSETS_H",
        "#define MANUAL_TFT_ASSETS_H",
        "",
        "#include <stddef.h>",
        "#include <stdint.h>",
        "#include \"../include/tft_font_renderer.h\"",
        "",
        "struct ManualTftTrackRow { uint16_t x; uint16_t length; uint32_t offset; };",
        "",
        emit_u32("manual_static_row_offsets", row_offsets),
        "",
        emit_bytes("manual_static_row_rle", row_data),
        "",
    ]
    book = FontBook()
    for spec in FONT_SPECS:
        sections.extend((emit_font(book, spec), ""))
    sections.extend((emit_track("bus", bus_rows, bus_data), ""))
    sections.extend((emit_track("phase", phase_rows, phase_data), ""))
    sections.extend(("#endif  // MANUAL_TFT_ASSETS_H", ""))
    OUTPUT.write_text("\n".join(sections), encoding="utf-8")
    auto_offsets, auto_data = rle_rows(build_auto_static(renderer))
    auto_sections = [
        "#ifndef AUTO_TFT_ASSETS_H",
        "#define AUTO_TFT_ASSETS_H",
        "",
        "#include <stdint.h>",
        "",
        emit_u32("auto_static_row_offsets", auto_offsets),
        "",
        emit_bytes("auto_static_row_rle", auto_data),
        "",
        "#endif  // AUTO_TFT_ASSETS_H",
        "",
    ]
    AUTO_OUTPUT.write_text("\n".join(auto_sections), encoding="utf-8")
    auto_dynamic_sections = [
        "#ifndef AUTO_TFT_DYNAMIC_ASSETS_H",
        "#define AUTO_TFT_DYNAMIC_ASSETS_H",
        "",
        "#include <stddef.h>",
        "#include <stdint.h>",
        "#include \"../include/tft_font_renderer.h\"",
        "",
    ]
    for spec in AUTO_FONT_SPECS:
        auto_dynamic_sections.extend((emit_auto_font(book, spec), ""))
    auto_dynamic_sections.extend(("#endif  // AUTO_TFT_DYNAMIC_ASSETS_H", ""))
    AUTO_DYNAMIC_OUTPUT.write_text("\n".join(auto_dynamic_sections), encoding="utf-8")
    startup_size, startup_bbox = emit_startup_assets()
    print(
        f"wrote {OUTPUT} ({OUTPUT.stat().st_size} bytes source; "
        f"static={len(row_data)}, tracks={len(bus_data) + len(phase_data)}); "
        f"{AUTO_OUTPUT} ({AUTO_OUTPUT.stat().st_size} bytes source; "
        f"static={len(auto_data)}); {AUTO_DYNAMIC_OUTPUT}; "
        f"{STARTUP_OUTPUT} (payload+directory={startup_size}, bbox={startup_bbox})"
    )


if __name__ == "__main__":
    main()
