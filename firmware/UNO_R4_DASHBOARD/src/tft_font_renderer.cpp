#include "include/tft_font_renderer.h"

namespace
{

const TftGlyphAsset *findGlyph(const TftFontAsset &font, uint8_t code)
{
  for (size_t index = 0U; index < font.glyph_count; ++index)
  {
    if (font.glyphs[index].code == code) return &font.glyphs[index];
  }
  return nullptr;
}

uint16_t blend(uint16_t background, uint16_t foreground, uint8_t alpha)
{
  if (alpha == 0U) return background;
  if (alpha == 255U) return foreground;
  const uint16_t inverse = (uint16_t)(255U - alpha);
  const uint8_t br = (uint8_t)((background >> 11) & 0x1FU);
  const uint8_t bg = (uint8_t)((background >> 5) & 0x3FU);
  const uint8_t bb = (uint8_t)(background & 0x1FU);
  const uint8_t fr = (uint8_t)((foreground >> 11) & 0x1FU);
  const uint8_t fg = (uint8_t)((foreground >> 5) & 0x3FU);
  const uint8_t fb = (uint8_t)(foreground & 0x1FU);
  return (uint16_t)((((uint16_t)fr * alpha + (uint16_t)br * inverse + 127U) /
                     255U) << 11) |
         (uint16_t)((((uint16_t)fg * alpha + (uint16_t)bg * inverse + 127U) /
                     255U) << 5) |
         (uint16_t)(((uint16_t)fb * alpha + (uint16_t)bb * inverse + 127U) /
                    255U);
}

void drawPixel(uint16_t *row_pixels,
               uint16_t row_width,
               int16_t x,
               uint16_t color,
               uint8_t alpha)
{
  if (x >= 0 && x < (int16_t)row_width)
    row_pixels[x] = blend(row_pixels[x], color, alpha);
}

void drawGlyphRow(uint16_t *row_pixels,
                  uint16_t row_width,
                  uint16_t y,
                  int16_t left,
                  int16_t anchor_y,
                  const TftGlyphAsset &glyph,
                  const TftFontAsset &font,
                  uint16_t color)
{
  const int16_t top = anchor_y + glyph.top;
  if ((int16_t)y < top || (int16_t)y >= top + glyph.height) return;
  const uint16_t row = (uint16_t)((int16_t)y - top);
  const uint8_t *levels = font.alpha_levels;
  if (font.encoding == TFT_FONT_PACKED)
  {
    const uint8_t *codes = font.data + glyph.data_offset +
                           (uint32_t)row * ((glyph.width + 1U) / 2U);
    for (uint8_t x = 0U; x < glyph.width; ++x)
    {
      const uint8_t pair = codes[x >> 1];
      const uint8_t code = (x & 1U) != 0U ? (uint8_t)(pair & 0x0FU)
                                          : (uint8_t)(pair >> 4);
      if (code != 0U)
        drawPixel(row_pixels, row_width, (int16_t)(left + x), color, levels[code]);
    }
    return;
  }

  const uint8_t *codes = font.data + glyph.data_offset +
                         font.row_offsets[glyph.row_offset_index + row];
  uint16_t nibble = 0U;
  auto next = [&]() -> uint8_t {
    const uint8_t pair = codes[nibble >> 1];
    const uint8_t code = (nibble & 1U) != 0U ? (uint8_t)(pair & 0x0FU)
                                             : (uint8_t)(pair >> 4);
    nibble++;
    return code;
  };
  uint16_t position = 0U;
  while (position < glyph.width)
  {
    const uint8_t code = next();
    if (code == 0U)
    {
      position = (uint16_t)(position + next() + 1U);
    }
    else if (code == 15U)
    {
      const uint16_t end = (uint16_t)(position + next() + 1U);
      for (; position < end; ++position)
        drawPixel(row_pixels, row_width, (int16_t)(left + position), color, 255U);
    }
    else
    {
      drawPixel(row_pixels, row_width, (int16_t)(left + position), color,
                levels[code]);
      position++;
    }
  }
}

}  // namespace

namespace TftFontRenderer
{

void drawTextRow(uint16_t *row_pixels,
                 uint16_t row_width,
                 uint16_t y,
                 int16_t anchor_x,
                 int16_t anchor_y,
                 const char *text,
                 const TftFontAsset &font,
                 uint16_t color,
                 TftHorizontalAnchor anchor)
{
  if ((int16_t)y < anchor_y + font.minimum_top ||
      (int16_t)y >= anchor_y + font.maximum_bottom) return;
  uint32_t width_q6 = 0U;
  for (const uint8_t *cursor = (const uint8_t *)text; *cursor != 0U; ++cursor)
  {
    const TftGlyphAsset *glyph = findGlyph(font, *cursor);
    if (glyph != nullptr) width_q6 += glyph->advance_q6;
  }
  int32_t cursor_q6 = (int32_t)anchor_x * 64;
  if (anchor == TftHorizontalAnchor::Center)
    cursor_q6 -= (int32_t)(width_q6 / 2U);
  else if (anchor == TftHorizontalAnchor::Right)
    cursor_q6 -= (int32_t)width_q6;
  for (const uint8_t *cursor = (const uint8_t *)text; *cursor != 0U; ++cursor)
  {
    const TftGlyphAsset *glyph = findGlyph(font, *cursor);
    if (glyph == nullptr) continue;
    const int32_t center_q6 = cursor_q6 + ((int32_t)glyph->advance_q6 / 2);
    const int16_t left = (int16_t)((center_q6 + 32) / 64) + glyph->left;
    drawGlyphRow(row_pixels, row_width, y, left, anchor_y, *glyph, font, color);
    cursor_q6 += glyph->advance_q6;
  }
}

}  // namespace TftFontRenderer
