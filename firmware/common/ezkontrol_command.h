#ifndef EZKONTROL_COMMAND_H
#define EZKONTROL_COMMAND_H
#include <math.h>
#include <stdint.h>

/* Existing STM_A CAN 0x0C01EFD0 contract, shared by native Speed, AUTO PI and
 * MANUAL. Keep the deployed bit1 contract despite the 2023 instruction wording. */
static inline void EZkontrol_EncodeCommand(int32_t clamped_rpm, float current,
    uint8_t run, uint8_t speed_mode, uint8_t life, uint8_t data[8])
{
  double raw_current = (current + 3200.0) / 0.1;
  uint16_t amps = (uint16_t)fmax(0.0, fmin(65535.0, raw_current));
  uint16_t rpm = (uint16_t)(clamped_rpm + 32000);
  data[0] = (uint8_t)(amps & 0xffu);
  data[1] = (uint8_t)(amps >> 8);
  data[2] = (uint8_t)(rpm & 0xffu);
  data[3] = (uint8_t)(rpm >> 8);
  data[4] = (uint8_t)((run ? 1u : 0u) | (speed_mode ? 2u : 0u));
  data[5] = data[6] = 0u;
  data[7] = life;
}
#endif
