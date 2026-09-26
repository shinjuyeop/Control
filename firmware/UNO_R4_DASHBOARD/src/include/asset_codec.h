#ifndef ASSET_CODEC_H
#define ASSET_CODEC_H

#include <stdint.h>
#include <string.h>

// Decoders for the compact formats written by tools/generate_assets.py.
namespace AssetCodec
{

static constexpr uint16_t STATIC_WIDTH = 480U;

// A 480-pixel static layer stored as palette indices. Row op bit 7 set: run of
// (low 7 bits + 1) pixels followed by one palette index. Bit 7 clear: keep the
// same number of indices from the row above. Every keyframe_rows-th row holds
// runs only, so any row decodes from the nearest keyframe above it.
struct StaticLayer
{
  const uint16_t *palette;
  const uint16_t *row_offsets;
  const uint8_t *rows;
  uint8_t keyframe_rows;
};

class StaticRowDecoder
{
public:
  constexpr explicit StaticRowDecoder(const StaticLayer &layer) : layer_(layer) {}

  void decode(uint16_t y, uint16_t *pixels)
  {
    if (y != decoded_y_)
    {
      const uint16_t keyframe = (uint16_t)(y - y % layer_.keyframe_rows);
      // Sequential rows reuse the cached row; other rows restart at the keyframe.
      uint16_t row = (decoded_y_ != NONE && decoded_y_ >= keyframe &&
                      decoded_y_ < y)
                         ? (uint16_t)(decoded_y_ + 1U)
                         : keyframe;
      for (; row <= y; ++row) decodeIndices(row);
      decoded_y_ = y;
    }
    for (uint16_t x = 0U; x < STATIC_WIDTH; ++x)
      pixels[x] = layer_.palette[indices_[x]];
  }

private:
  static constexpr uint16_t NONE = 0xFFFFU;

  void decodeIndices(uint16_t y)
  {
    const uint8_t *data = layer_.rows + layer_.row_offsets[y];
    const uint8_t *end = layer_.rows + layer_.row_offsets[y + 1U];
    uint16_t x = 0U;
    while (data < end)
    {
      const uint8_t op = *data++;
      const uint8_t count = (uint8_t)((op & 0x7FU) + 1U);
      if ((op & 0x80U) != 0U) memset(&indices_[x], *data++, count);
      x = (uint16_t)(x + count);
    }
  }

  StaticLayer layer_;
  uint8_t indices_[STATIC_WIDTH] = {};
  uint16_t decoded_y_ = NONE;
};

// Alpha spans: 00nnnnnn = n+1 zero pixels, 01nnnnnn = n+1 pixels at 255,
// 10nnnnnn = n+1 literal alpha bytes, 11nnnnnn = n+1 pixels at the next byte.
class AlphaSpanReader
{
public:
  explicit AlphaSpanReader(const uint8_t *data) : data_(data) {}

  // Returns the next run; literal bytes are returned one pixel at a time.
  void nextRun(uint8_t *count, uint8_t *value)
  {
    if (remaining_ == 0U) load();
    if (literal_)
    {
      *count = 1U;
      *value = *data_++;
      --remaining_;
      return;
    }
    *count = remaining_;
    *value = value_;
    remaining_ = 0U;
  }

  uint8_t next()
  {
    if (remaining_ == 0U) load();
    --remaining_;
    return literal_ ? *data_++ : value_;
  }

  // Byte following the spans that cover `pixels`.
  static const uint8_t *skip(const uint8_t *data, uint16_t pixels)
  {
    uint16_t covered = 0U;
    while (covered < pixels)
    {
      const uint8_t op = *data++;
      const uint8_t count = (uint8_t)((op & 0x3FU) + 1U);
      if ((op & 0xC0U) == LITERAL) data += count;
      else if ((op & 0xC0U) == VALUE) data++;
      covered = (uint16_t)(covered + count);
    }
    return data;
  }

private:
  static constexpr uint8_t FULL = 0x40U;
  static constexpr uint8_t LITERAL = 0x80U;
  static constexpr uint8_t VALUE = 0xC0U;

  void load()
  {
    const uint8_t op = *data_++;
    const uint8_t kind = (uint8_t)(op & 0xC0U);
    remaining_ = (uint8_t)((op & 0x3FU) + 1U);
    literal_ = kind == LITERAL;
    if (kind == VALUE) value_ = *data_++;
    else if (!literal_) value_ = kind == FULL ? 255U : 0U;
  }

  const uint8_t *data_;
  uint8_t remaining_ = 0U;
  uint8_t value_ = 0U;
  bool literal_ = false;
};

}  // namespace AssetCodec

#endif  // ASSET_CODEC_H
