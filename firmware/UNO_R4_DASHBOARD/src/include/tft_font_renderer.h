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
  uint16_t data_offset;
  uint16_t row_offset_index;
} __attribute__((packed));

// Glyph pixels are 4-bit codes mapped through alpha_levels (0 = 0, 15 = 255).
// Packed: rows of two codes per byte, high nibble first.
// Tokens: byte-aligned rows of nibbles; 0,n = n+1 clear pixels, 15,n = n+1
// opaque pixels, any other code = one pixel. row_offsets locate each row.
enum : uint8_t
{
  TFT_FONT_PACKED = 0U,
  TFT_FONT_TOKENS = 1U
};

struct TftFontAsset
{
  const TftGlyphAsset *glyphs;
  size_t glyph_count;
  const uint8_t *data;
  const uint16_t *row_offsets;
  const uint8_t *alpha_levels;
  uint8_t encoding;
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
