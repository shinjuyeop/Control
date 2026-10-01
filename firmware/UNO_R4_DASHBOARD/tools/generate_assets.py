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


STATIC_KEYFRAME_ROWS = 8
STATIC_RUN = 0x80
STATIC_MAX_OP = 128


def _encode_static_row(row: list[int], above: list[int] | None) -> bytes:
    """Encode one palette-index row with the fewest run/copy-up ops."""
    width = len(row)
    cost = [0] * (width + 1)
    choice: list[tuple[int, int]] = [(0, 0)] * width
    for start in range(width - 1, -1, -1):
        best = None
        run = 1
        while (start + run < width and run < STATIC_MAX_OP
               and row[start + run] == row[start]):
            run += 1
        for length in range(1, run + 1):
            candidate = 2 + cost[start + length]
            if best is None or candidate < best[0]:
                best = (candidate, STATIC_RUN, length)
        if above is not None:
            length = 0
            while (start + length < width and length < STATIC_MAX_OP
                   and row[start + length] == above[start + length]):
                length += 1
                candidate = 1 + cost[start + length]
                if candidate < best[0]:
                    best = (candidate, 0, length)
        cost[start] = best[0]
        choice[start] = (best[1], best[2])
    output = bytearray()
    position = 0
    while position < width:
        kind, length = choice[position]
        output.append(kind | (length - 1))
        if kind == STATIC_RUN:
            output.append(row[position])
        position += length
    return bytes(output)


def encode_static(image: Image.Image) -> tuple[list[int], list[int], bytes]:
    """Palette-index rows; non-keyframe rows may keep spans of the row above."""
    image = image.convert("RGB")
    pixels = [rgb565(color) for color in image.get_flattened_data()]
    counts: dict[int, int] = {}
    for value in pixels:
        counts[value] = counts.get(value, 0) + 1
    palette = sorted(counts, key=lambda value: (-counts[value], value))
    if len(palette) > 256:
        raise RuntimeError(f"static layer has {len(palette)} colors (max 256)")
    index = {value: position for position, value in enumerate(palette)}
    width = image.width
    rows = [
        [index[value] for value in pixels[y * width:(y + 1) * width]]
        for y in range(image.height)
    ]
    offsets = [0]
    data = bytearray()
    for y, row in enumerate(rows):
        above = rows[y - 1] if y % STATIC_KEYFRAME_ROWS else None
        data.extend(_encode_static_row(row, above))
        offsets.append(len(data))
    if len(data) > 0xFFFF:
        raise RuntimeError("static layer exceeds 16-bit offsets")
    return palette, offsets, bytes(data)


def emit_static(prefix: str, image: Image.Image) -> tuple[str, int]:
    palette, offsets, data = encode_static(image)
    text = "\n".join((
        emit_u16(f"{prefix}_static_palette", palette),
        "",
        emit_u16(f"{prefix}_static_row_offsets", offsets),
        "",
        emit_bytes(f"{prefix}_static_rows", data),
        "",
        f"static constexpr AssetCodec::StaticLayer {prefix}_static = {{"
        f"{prefix}_static_palette, {prefix}_static_row_offsets, "
        f"{prefix}_static_rows, {STATIC_KEYFRAME_ROWS}U}};",
    ))
    return text, len(palette) * 2 + len(offsets) * 2 + len(data)


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


def emit_u16(name: str, values: list[int]) -> str:
    lines = [f"static const uint16_t {name}[] = {{"]
    for index in range(0, len(values), 12):
        chunk = values[index:index + 12]
        lines.append("    " + ", ".join(f"{value}U" for value in chunk) + ",")
    lines.append("};")
    return "\n".join(lines)


# One font glyph before encoding: (code, left, top, width, height, advance_q6, alpha).
RawGlyph = tuple[int, int, int, int, int, int, bytes]

FONT_PACKED = 0
FONT_TOKENS = 1
FONT_ZERO_TOKEN = 0
FONT_FULL_TOKEN = 15


def font_alpha_levels(values: list[int]) -> list[int]:
    """Sixteen alpha levels: exact 0 and 255 plus 14 least-squares interior levels."""
    histogram: dict[int, int] = {}
    for value in values:
        if 0 < value < 255:
            histogram[value] = histogram.get(value, 0) + 1
    points = sorted(histogram)
    clusters = 14
    if len(points) <= clusters:
        interior = points + [points[-1] if points else 128] * (clusters - len(points))
        return [0, *interior, 255]

    count = len(points)
    weight = [0] * (count + 1)
    first = [0] * (count + 1)
    second = [0] * (count + 1)
    for index, value in enumerate(points):
        frequency = histogram[value]
        weight[index + 1] = weight[index] + frequency
        first[index + 1] = first[index] + frequency * value
        second[index + 1] = second[index] + frequency * value * value

    def error(begin: int, end: int) -> float:
        total = weight[end] - weight[begin]
        mean = first[end] - first[begin]
        return second[end] - second[begin] - mean * mean / total

    infinity = float("inf")
    cost = [[infinity] * (count + 1) for _ in range(clusters + 1)]
    split = [[0] * (count + 1) for _ in range(clusters + 1)]
    cost[0][0] = 0.0
    for cluster in range(1, clusters + 1):
        for end in range(cluster, count + 1):
            for begin in range(cluster - 1, end):
                candidate = cost[cluster - 1][begin] + error(begin, end)
                if candidate < cost[cluster][end]:
                    cost[cluster][end] = candidate
                    split[cluster][end] = begin
    levels = []
    end = count
    for cluster in range(clusters, 0, -1):
        begin = split[cluster][end]
        total = weight[end] - weight[begin]
        levels.append(round((first[end] - first[begin]) / total))
        end = begin
    return [0, *sorted(levels), 255]


def _nearest_level(levels: list[int], value: int) -> int:
    if value == 0:
        return FONT_ZERO_TOKEN
    if value == 255:
        return FONT_FULL_TOKEN
    return min(range(16), key=lambda index: (abs(levels[index] - value), index))


def _pack_nibbles(nibbles: list[int]) -> bytes:
    if len(nibbles) % 2:
        nibbles = [*nibbles, 0]
    return bytes((nibbles[index] << 4) | nibbles[index + 1]
                 for index in range(0, len(nibbles), 2))


def _font_token_row(codes: list[int]) -> bytes:
    nibbles: list[int] = []
    position = 0
    while position < len(codes):
        code = codes[position]
        if code in (FONT_ZERO_TOKEN, FONT_FULL_TOKEN):
            run = 1
            while (position + run < len(codes) and run < 16
                   and codes[position + run] == code):
                run += 1
            nibbles.extend((code, run - 1))
            position += run
        else:
            nibbles.append(code)
            position += 1
    return _pack_nibbles(nibbles)


def emit_font_asset(prefix: str, data_name: str, glyphs: list[RawGlyph]) -> str:
    """Emit 4-bit glyph rows, packed or tokenized, whichever is smaller."""
    levels = font_alpha_levels([value for glyph in glyphs for value in glyph[6]])
    coded = [
        (glyph, [_nearest_level(levels, value) for value in glyph[6]])
        for glyph in glyphs
    ]
    packed = bytearray()
    packed_rows = []
    tokens = bytearray()
    token_rows = []
    row_offsets: list[int] = []
    for (code, left, top, width, height, advance, _), codes in coded:
        rows = [codes[row * width:(row + 1) * width] for row in range(height)]
        packed_rows.append(
            (code, left, top, width, height, advance, len(packed), 0))
        for row in rows:
            packed.extend(_pack_nibbles(row))
        token_rows.append(
            (code, left, top, width, height, advance, len(tokens), len(row_offsets)))
        glyph_begin = len(tokens)
        row_offsets.append(0)
        for row in rows:
            tokens.extend(_font_token_row(row))
            row_offsets.append(len(tokens) - glyph_begin)
    if len(tokens) + len(row_offsets) * 2 < len(packed):
        encoding, data, glyph_rows = FONT_TOKENS, tokens, token_rows
    else:
        encoding, data, glyph_rows, row_offsets = FONT_PACKED, packed, packed_rows, []
    if len(data) > 0xFFFF:
        raise RuntimeError(f"{prefix} exceeds 16-bit glyph offsets")

    minimum_top = min(glyph[2] for glyph in glyphs)
    maximum_bottom = max(glyph[2] + glyph[4] for glyph in glyphs)
    lines = [
        emit_bytes(f"{prefix}_levels", bytes(levels)),
        "",
        emit_bytes(f"{prefix}_{data_name}", bytes(data)),
        "",
    ]
    if row_offsets:
        lines.extend((emit_u16(f"{prefix}_row_offsets", row_offsets), ""))
    lines.append(f"static const TftGlyphAsset {prefix}_glyphs[] = {{")
    for code, left, top, width, height, advance, offset, row_index in glyph_rows:
        lines.append(
            f"    {{{code}U, {left}, {top}, {width}U, {height}U, "
            f"{advance}U, {offset}U, {row_index}U}},"
        )
    lines.append("};")
    lines.append(
        f"static const TftFontAsset {prefix} = {{"
        f"{prefix}_glyphs, sizeof({prefix}_glyphs) / sizeof({prefix}_glyphs[0]), "
        f"{prefix}_{data_name}, "
        f"{prefix + '_row_offsets' if row_offsets else 'nullptr'}, "
        f"{prefix}_levels, {encoding}U, {minimum_top}, {maximum_bottom}}};"
    )
    return "\n".join(lines)


def render_font(book: FontBook, spec: FontSpec) -> list[RawGlyph]:
    font = font_for(book, spec)
    glyphs = []
    for character in spec.characters:
        mask, offset = font.getmask2(character, mode="L", anchor="mm")
        width, height = mask.size
        glyphs.append((glyph_code(character), offset[0], offset[1], width, height,
                       round(font.getlength(character) * 64), bytes(mask)))
    return glyphs


SPAN_ZERO = 0x00
SPAN_FULL = 0x40
SPAN_LITERAL = 0x80
SPAN_VALUE = 0xC0
SPAN_MAX = 64


def encode_alpha_spans(values: list[int]) -> bytes:
    """Fewest bytes of: 00nnnnnn n+1 zeros, 01nnnnnn n+1 x 255,
    10nnnnnn n+1 literal bytes, 11nnnnnn n+1 x the following byte."""
    count = len(values)
    cost = [0] * (count + 1)
    choice: list[tuple[int, int]] = [(0, 0)] * count
    for start in range(count - 1, -1, -1):
        value = values[start]
        run = 1
        while (start + run < count and run < SPAN_MAX
               and values[start + run] == value):
            run += 1
        run_cost = 1 if value in (0, 255) else 2
        run_kind = (SPAN_ZERO if value == 0 else
                    SPAN_FULL if value == 255 else SPAN_VALUE)
        best = (run_cost + cost[start + 1], run_kind, 1)
        for length in range(2, run + 1):
            candidate = run_cost + cost[start + length]
            if candidate < best[0]:
                best = (candidate, run_kind, length)
        for length in range(1, min(SPAN_MAX, count - start) + 1):
            candidate = 1 + length + cost[start + length]
            if candidate < best[0]:
                best = (candidate, SPAN_LITERAL, length)
        cost[start] = best[0]
        choice[start] = (best[1], best[2])
    output = bytearray()
    position = 0
    while position < count:
        kind, length = choice[position]
        output.append(kind | (length - 1))
        if kind == SPAN_LITERAL:
            output.extend(values[position:position + length])
        elif kind == SPAN_VALUE:
            output.append(values[position])
        position += length
    return bytes(output)


def encode_haze_runs(values: list[int]) -> bytes:
    """One byte per run: 4-bit haze level, 4-bit run length - 1."""
    output = bytearray()
    position = 0
    while position < len(values):
        value = values[position]
        if value > 15:
            raise RuntimeError("haze level exceeds 4 bits")
        run = 1
        while (position + run < len(values) and run < 16
               and values[position + run] == value):
            run += 1
        output.append((value << 4) | (run - 1))
        position += run
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


# A startup row before encoding: (x, outline alpha, 4-bit haze level).
StartupRow = tuple[int, list[int], list[int]]


def build_startup_masks() -> tuple[
    tuple[int, int, int, int], int, list[StartupRow]
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
    rows: list[StartupRow] = []
    for y in range(y0, y1):
        nonzero = [
            x for x in range(x0, x1)
            if outline.getpixel((x, y)) or haze_quantized.getpixel((x, y))
        ]
        if not nonzero:
            rows.append((0, [], []))
            continue
        row_x0 = min(nonzero)
        row_x1 = max(nonzero) + 1
        rows.append((
            row_x0,
            list(outline.crop((row_x0, y, row_x1, y + 1)).get_flattened_data()),
            list(haze_quantized.crop((row_x0, y, row_x1, y + 1)).get_flattened_data()),
        ))
    return bbox, haze_maximum, rows


def write_startup_header(bbox: tuple[int, int, int, int], haze_maximum: int,
                         rows: list[StartupRow]) -> int:
    directory = []
    outline_data = bytearray()
    haze_data = bytearray()
    for x, outline_row, haze_row in rows:
        directory.append((x, len(outline_row), len(outline_data), len(haze_data)))
        outline_data.extend(encode_alpha_spans(outline_row))
        haze_data.extend(encode_haze_runs(haze_row))
    directory.append((0, 0, len(outline_data), len(haze_data)))
    if max(len(outline_data), len(haze_data)) > 0xFFFF:
        raise RuntimeError("startup masks exceed 16-bit offsets")
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
        "// Outline: AssetCodec alpha spans. Haze: one byte per run,",
        "// high nibble = level 0..15, low nibble = run length - 1.",
        emit_bytes("startup_outline_data", bytes(outline_data)),
        "",
        emit_bytes("startup_haze_data", bytes(haze_data)),
        "",
        "static const StartupTftMaskRow startup_mask_rows[] = {",
    ]
    sections.extend(
        f"    {{{x}U, {length}U, {outline_offset}U, {haze_offset}U}},"
        for x, length, outline_offset, haze_offset in directory
    )
    sections.extend(("};", "", "#endif  // STARTUP_TFT_ASSETS_H", ""))
    STARTUP_OUTPUT.write_text("\n".join(sections), encoding="utf-8")
    return len(outline_data) + len(haze_data) + len(directory) * 8


def dense_path(renderer: AssetRenderer, path) -> list[tuple[float, float]]:
    return [renderer._mode_track_location(path, index / 255.0)[:2]
            for index in range(256)]


# A track row before encoding: (x, alpha, progress); progress is unused where alpha is 0.
TrackRow = tuple[int, list[int], list[int]]


def track_rows(renderer: AssetRenderer, path, width: int, key: str) -> list[TrackRow]:
    mask = renderer._replica_track_mask(key, path, width)
    samples = dense_path(renderer, path)
    rows: list[TrackRow] = []
    for y in range(320):
        xs = [x for x in range(480) if mask.getpixel((x, y)) != 0]
        if not xs:
            rows.append((0, [], []))
            continue
        x0, x1 = min(xs), max(xs)
        alphas = []
        progress = []
        for x in range(x0, x1 + 1):
            alpha = mask.getpixel((x, y))
            alphas.append(alpha)
            if alpha == 0:
                progress.append(0)
                continue
            progress.append(min(
                range(256),
                key=lambda index: (
                    (samples[index][0] - x) ** 2
                    + (samples[index][1] - y) ** 2
                ),
            ))
        rows.append((x0, alphas, progress))
    return rows


def encode_track_row(alphas: list[int], progress: list[int]) -> bytes:
    """First progress, alpha spans, absolute exceptions, then one +0/+1 bit per pixel."""
    visible = [index for index, alpha in enumerate(alphas) if alpha]
    output = bytearray((progress[visible[0]],))
    output.extend(encode_alpha_spans(alphas))
    exceptions = []
    bits = bytearray((len(visible) - 1 + 7) // 8)
    for order in range(1, len(visible)):
        current = progress[visible[order]]
        step = current - progress[visible[order - 1]]
        if step == 1:
            bits[(order - 1) >> 3] |= 1 << ((order - 1) & 7)
        elif step != 0:
            exceptions.append((order, current))
    if len(exceptions) > 255:
        raise RuntimeError("too many track progress exceptions")
    output.append(len(exceptions))
    for order, current in exceptions:
        output.extend((order & 0xFF, order >> 8, current))
    output.extend(bits)
    return bytes(output)


def emit_track(name: str, rows: list[TrackRow]) -> str:
    data = bytearray()
    directory = []
    for x, alphas, progress in rows:
        if not any(alphas):
            directory.append((0, 0, len(data)))
            continue
        directory.append((x, len(alphas), len(data)))
        data.extend(encode_track_row(alphas, progress))
    if len(data) > 0xFFFF:
        raise RuntimeError("track exceeds 16-bit offsets")
    lines = [emit_bytes(f"manual_track_{name}_data", bytes(data)), ""]
    lines.append(f"static const ManualTftTrackRow manual_track_{name}_rows[] = {{")
    for x, length, offset in directory:
        lines.append(f"    {{{x}U, {length}U, {offset}U}},")
    lines.append("};")
    return "\n".join(lines)


FontBlock = tuple[str, list[RawGlyph]]


def write_manual_header(static: Image.Image, fonts: list[FontBlock],
                        bus: list[TrackRow], phase: list[TrackRow]) -> None:
    static_text, _ = emit_static("manual", static)
    sections = [
        "#ifndef MANUAL_TFT_ASSETS_H",
        "#define MANUAL_TFT_ASSETS_H",
        "",
        "#include <stddef.h>",
        "#include <stdint.h>",
        "#include \"../include/asset_codec.h\"",
        "#include \"../include/tft_font_renderer.h\"",
        "",
        "// Row data: first progress, alpha spans, exception count, exceptions",
        "// (uint16 visible-pixel index, absolute progress), then one bit per",
        "// visible pixel after the first: 1 = progress + 1, 0 = unchanged.",
        "struct ManualTftTrackRow { uint16_t x; uint16_t length; uint16_t offset; };",
        "",
        static_text,
        "",
    ]
    for name, glyphs in fonts:
        sections.extend((emit_font_asset(f"manual_font_{name}", "alpha", glyphs), ""))
    sections.extend((emit_track("bus", bus), ""))
    sections.extend((emit_track("phase", phase), ""))
    sections.extend(("#endif  // MANUAL_TFT_ASSETS_H", ""))
    OUTPUT.write_text("\n".join(sections), encoding="utf-8")


def write_auto_headers(static: Image.Image, fonts: list[FontBlock]) -> None:
    static_text, _ = emit_static("auto", static)
    AUTO_OUTPUT.write_text("\n".join((
        "#ifndef AUTO_TFT_ASSETS_H",
        "#define AUTO_TFT_ASSETS_H",
        "",
        "#include <stdint.h>",
        "#include \"../include/asset_codec.h\"",
        "",
        static_text,
        "",
        "#endif  // AUTO_TFT_ASSETS_H",
        "",
    )), encoding="utf-8")
    sections = [
        "#ifndef AUTO_TFT_DYNAMIC_ASSETS_H",
        "#define AUTO_TFT_DYNAMIC_ASSETS_H",
        "",
        "#include <stddef.h>",
        "#include <stdint.h>",
        "#include \"../include/tft_font_renderer.h\"",
        "",
    ]
    for name, glyphs in fonts:
        sections.extend((emit_font_asset(f"auto_font_{name}", "data", glyphs), ""))
    sections.extend(("#endif  // AUTO_TFT_DYNAMIC_ASSETS_H", ""))
    AUTO_DYNAMIC_OUTPUT.write_text("\n".join(sections), encoding="utf-8")


def main() -> None:
    renderer = AssetRenderer()
    book = FontBook()
    write_manual_header(
        build_manual_static(renderer),
        [(spec.name, render_font(book, spec)) for spec in FONT_SPECS],
        track_rows(renderer, renderer._replica_bus_path(), 10, "firmware-bus-active"),
        track_rows(renderer, renderer._replica_phase_path(), 9,
                   "firmware-phase-active"),
    )
    write_auto_headers(
        build_auto_static(renderer),
        [(spec.name, render_font(book, spec)) for spec in AUTO_FONT_SPECS],
    )
    bbox, haze_maximum, startup_rows = build_startup_masks()
    startup_size = write_startup_header(bbox, haze_maximum, startup_rows)
    print(
        f"wrote {OUTPUT} ({OUTPUT.stat().st_size} bytes source); "
        f"{AUTO_OUTPUT} ({AUTO_OUTPUT.stat().st_size} bytes source); "
        f"{AUTO_DYNAMIC_OUTPUT}; "
        f"{STARTUP_OUTPUT} (payload+directory={startup_size}, bbox={bbox})"
    )


if __name__ == "__main__":
    main()
