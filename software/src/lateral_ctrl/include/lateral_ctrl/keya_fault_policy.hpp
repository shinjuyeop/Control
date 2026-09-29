#pragma once

#include <array>
#include <cstdint>

namespace lateral_ctrl
{
// KEYA V1.1, section 5.2 (PDF pp.19-20). Decoding preserves every raw bit.
enum class KeyaReset { NONE, SELF, DISABLE, ENABLE, POWER };
struct KeyaFaultDefinition { const char * name; KeyaReset reset; };
inline constexpr std::array<KeyaFaultDefinition, 16> keyaFaultDefinitions{{
  {"disabled", KeyaReset::NONE}, {"over_voltage", KeyaReset::SELF},
  {"hardware_overcurrent", KeyaReset::POWER}, {"eeprom_error", KeyaReset::POWER},
  {"under_voltage", KeyaReset::DISABLE}, {"reserved", KeyaReset::NONE},
  {"software_overcurrent", KeyaReset::DISABLE}, {"control_mode_error", KeyaReset::POWER},
  {"phase_failure", KeyaReset::POWER}, {"stall", KeyaReset::DISABLE},
  {"temperature", KeyaReset::SELF}, {"hall_failure", KeyaReset::DISABLE},
  {"current_sensor_failure", KeyaReset::POWER}, {"spare", KeyaReset::NONE},
  {"can_disconnect", KeyaReset::ENABLE}, {"rs232_disconnect", KeyaReset::ENABLE}
}};

// Display labels only; the Host does not gate output on these classes.
enum class KeyaFaultClass { INFO, RECOVERABLE, HARD };
inline KeyaFaultClass keyaFaultClass(unsigned bit)
{
  // Hall failure is unsafe even though KEYA permits a Disable reset.
  if ((uint16_t{1} << bit) & 0x198c) return KeyaFaultClass::HARD;
  // Control Ways=CAN: RS232 loss is irrelevant, reserved/spare are diagnostic only.
  if ((uint16_t{1} << bit) & 0x4652) return KeyaFaultClass::RECOVERABLE;
  return KeyaFaultClass::INFO;
}
inline const char * keyaFaultClassName(KeyaFaultClass c)
{
  return c == KeyaFaultClass::HARD ? "HARD" :
    c == KeyaFaultClass::RECOVERABLE ? "RECOVERABLE" : "INFO";
}
}  // namespace lateral_ctrl
