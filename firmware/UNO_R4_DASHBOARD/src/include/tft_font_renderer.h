#ifndef TFT_FONT_RENDERER_H
#define TFT_FONT_RENDERER_H

#include <stddef.h>
#include <stdint.h>

struct TftGlyphAsset
{
  uint8_t code;
  int8_t left;
  int8_t top;
  uint8_t width;
  uint8_t height;
  uint16_t advance_q6;
  uint32_t data_offset;
  uint16_t data_length;
  uint16_t row_offset_index;
} __attribute__((packed));

struct TftFontAsset
{
  const TftGlyphAsset *glyphs;
  size_t glyph_count;
  const uint8_t *data;
  const uint16_t *row_offsets;
  uint8_t rle;
  int8_t minimum_top;
  int8_t maximum_bottom;
} __attribute__((packed));

enum class TftHorizontalAnchor : int8_t
{
  Left = -1,
  Center = 0,
  Right = 1
};

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
                 TftHorizontalAnchor anchor);

}  // namespace TftFontRenderer

#endif  // TFT_FONT_RENDERER_H
