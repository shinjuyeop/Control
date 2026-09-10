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
  const uint32_t row_begin = (uint32_t)((int16_t)y - top) * glyph.width;
  const uint32_t row_end = row_begin + glyph.width;
  if (font.rle == 0U)
  {
    const uint8_t *alpha = font.data + glyph.data_offset + row_begin;
    for (uint8_t x = 0U; x < glyph.width; ++x)
    {
      const int16_t destination_x = left + x;
      if (destination_x >= 0 && destination_x < (int16_t)row_width)
        row_pixels[destination_x] =
            blend(row_pixels[destination_x], color, alpha[x]);
    }
    return;
  }

  const uint16_t row = (uint16_t)((int16_t)y - top);
  const uint16_t encoded_begin =
      font.row_offsets[glyph.row_offset_index + row];
  const uint16_t encoded_end =
      font.row_offsets[glyph.row_offset_index + row + 1U];
  const uint8_t *encoded = font.data + glyph.data_offset + encoded_begin;
  uint32_t position = 0U;
  for (uint16_t offset = 0U; offset + 1U < encoded_end - encoded_begin; offset += 2U)
  {
    const uint32_t run_end = position + encoded[offset];
    if (encoded[offset + 1U] != 0U)
    {
      for (uint32_t pixel = position; pixel < run_end; ++pixel)
      {
        const int16_t destination_x = left + (int16_t)pixel;
        if (destination_x >= 0 && destination_x < (int16_t)row_width)
          row_pixels[destination_x] = blend(
              row_pixels[destination_x], color, encoded[offset + 1U]);
      }
    }
    position = run_end;
    if (position >= glyph.width) break;
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
