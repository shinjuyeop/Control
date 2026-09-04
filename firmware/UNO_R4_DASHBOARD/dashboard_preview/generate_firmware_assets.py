"""Generate UNO R4 raster assets from the canonical dashboard preview."""

from __future__ import annotations

from dataclasses import dataclass
import math
from pathlib import Path

from PIL import Image

from .model import (
    DRIVE_TRACTIVE_FEEDBACK,
    GNSS_RTK_FIXED,
    STATUS_DETAIL_VALID,
    STATUS_PART1_VALID,
    STATUS_PART2_VALID,
    STATUS_VEHICLE_VALID,
    DashboardValues,
    GaugeDynamics,
    PreviewPolicy,
    ScenarioSample,
)
from .palette import FontBook
from .renderer import DashboardRenderer
from .startup import StartupRenderer


OUTPUT = Path(__file__).resolve().parents[1] / "manual_tft_assets.h"


def rgb565(color: tuple[int, int, int]) -> int:
    red, green, blue = color
    return ((red >> 3) << 11) | ((green >> 2) << 5) | (blue >> 3)


def background_color(y: int) -> tuple[int, int, int]:
    fraction = y / 319.0
    return (
        round(246 - 18 * fraction),
        round(247 - 18 * fraction),
        round(247 - 17 * fraction),
    )


def restore_gradient(image: Image.Image, box: tuple[int, int, int, int]) -> None:
    x0, y0, x1, y1 = box
    for y in range(y0, y1):
        image.paste(background_color(y), (x0, y, x1, y + 1))


def build_manual_static(renderer: DashboardRenderer) -> Image.Image:
    flags = (
        STATUS_VEHICLE_VALID
        | STATUS_DETAIL_VALID
        | STATUS_PART1_VALID
        | STATUS_PART2_VALID
    )
    values = DashboardValues(
        status_flags=flags,
        drive_flags=DRIVE_TRACTIVE_FEEDBACK,
        gnss_state=GNSS_RTK_FIXED,
    )
    sample = ScenarioSample(values)
    dynamics = GaugeDynamics()
    dynamics.snap(values)
    image = renderer.render(sample, "manual_tft", dynamics, PreviewPolicy(), 0.0)

    image.paste("#171818", (4, 12, 67, 82))
    restore_gradient(image, (185, 130, 367, 193))
    restore_gradient(image, (88, 214, 238, 244))
    for x in (376, 402, 428, 454):
        restore_gradient(image, (x - 5, 200, min(480, x + 25), 226))
    for x0, x1 in ((62, 137), (137, 212), (212, 277),
                   (277, 347), (347, 407), (407, 471)):
        image.paste("#F4F4F3", (x0 + 2, 284, x1 - 1, 306))
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


def emit_font(book: FontBook, spec: FontSpec) -> str:
    font = font_for(book, spec)
    alpha = bytearray()
    glyph_rows = []
    for character in spec.characters:
        mask, offset = font.getmask2(character, mode="L", anchor="mm")
        width, height = mask.size
        glyph_rows.append(
            (glyph_code(character), offset[0], offset[1], width, height,
             round(font.getlength(character) * 64), len(alpha))
        )
        alpha.extend(bytes(mask))
    lines = [emit_bytes(f"manual_font_{spec.name}_alpha", bytes(alpha)), ""]
    lines.append(f"static const ManualTftGlyphAsset manual_font_{spec.name}_glyphs[] = {{")
    for code, left, top, width, height, advance, offset in glyph_rows:
        lines.append(
            f"    {{{code}U, {left}, {top}, {width}U, {height}U, "
            f"{advance}U, {offset}UL}},"
        )
    lines.append("};")
    lines.append(
        f"static const ManualTftFontAsset manual_font_{spec.name} = {{"
        f"manual_font_{spec.name}_glyphs, "
        f"sizeof(manual_font_{spec.name}_glyphs) / "
        f"sizeof(manual_font_{spec.name}_glyphs[0]), "
        f"manual_font_{spec.name}_alpha}};"
    )
    return "\n".join(lines)


def dense_path(renderer: DashboardRenderer, path) -> list[tuple[float, float]]:
    return [renderer._mode_track_location(path, index / 255.0)[:2]
            for index in range(256)]


def track_rows(renderer: DashboardRenderer, path, width: int, key: str):
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
    renderer = DashboardRenderer()
    static = build_manual_static(renderer)
    row_offsets, row_data = rle_rows(static)

    startup = StartupRenderer()
    startup_times = (0.0, 0.12, 0.24, 0.35, 0.80, 1.10, 1.25, 1.40)
    startup_frames = [rle_pixels(startup.render(value)) for value in startup_times]

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
        "",
        "struct ManualTftGlyphAsset {",
        "  uint8_t code; int8_t left; int8_t top; uint8_t width; uint8_t height;",
        "  uint16_t advance_q6; uint32_t alpha_offset;",
        "};",
        "struct ManualTftFontAsset {",
        "  const ManualTftGlyphAsset *glyphs; size_t glyph_count; const uint8_t *alpha;",
        "};",
        "struct ManualTftTrackRow { uint16_t x; uint16_t length; uint32_t offset; };",
        "",
        emit_u32("manual_static_row_offsets", row_offsets),
        "",
        emit_bytes("manual_static_row_rle", row_data),
        "",
    ]
    for index, frame in enumerate(startup_frames):
        sections.extend((emit_bytes(f"manual_startup_{index}", frame), ""))
    sections.append("static const uint8_t *const manual_startup_frames[] = {")
    for index in range(len(startup_frames)):
        sections.append(f"    manual_startup_{index},")
    sections.append("};")
    sections.append("static const size_t manual_startup_sizes[] = {")
    for index in range(len(startup_frames)):
        sections.append(f"    sizeof(manual_startup_{index}),")
    sections.extend(("};", ""))

    book = FontBook()
    for spec in FONT_SPECS:
        sections.extend((emit_font(book, spec), ""))
    sections.extend((emit_track("bus", bus_rows, bus_data), ""))
    sections.extend((emit_track("phase", phase_rows, phase_data), ""))
    sections.extend(("#endif  // MANUAL_TFT_ASSETS_H", ""))
    OUTPUT.write_text("\n".join(sections), encoding="utf-8")
    print(
        f"wrote {OUTPUT} ({OUTPUT.stat().st_size} bytes source; "
        f"static={len(row_data)}, startup={sum(map(len, startup_frames))}, "
        f"tracks={len(bus_data) + len(phase_data)})"
    )


if __name__ == "__main__":
    main()
