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
struct KeyaFaults {
  uint16_t raw, hard{0}, recoverable{0}, info{0};
  explicit KeyaFaults(uint16_t bits) : raw(bits) {
    for (unsigned bit = 0; bit < 16; ++bit) {
      const uint16_t mask = bits & (uint16_t{1} << bit);
      switch (keyaFaultClass(bit)) {
        case KeyaFaultClass::HARD: hard |= mask; break;
        case KeyaFaultClass::RECOVERABLE: recoverable |= mask; break;
        case KeyaFaultClass::INFO: info |= mask; break;
      }
    }
  }
};

// Pure policy; callers supply monotonic seconds and fresh fault + position evidence.
// No ROS, I/O, or motor commands here. A latch requires operator investigation/restart.
class KeyaRecoveryPolicy
{
public:
  unsigned max_attempts{3};
  double clear_hold_sec{0.5}, retry_sec{1.0}, persistence_sec{30.0}, healthy_reset_sec{60.0};
  bool recovering{false}, latched{false};
  unsigned attempts{0};
  uint64_t total_attempts{0};
  uint16_t reason_bits{0};
  const char * latch_reason{"none"};

  // Returns true only on recovery completion: caller must discard ALL queued targets.
  bool update(uint16_t raw, bool fresh, double now) {
    const KeyaFaults faults(raw);
    if (faults.hard) {
      reason_bits |= faults.hard;
      latched = true;
      latch_reason = "hard_fault";
    }
    if (latched) { recovering = false; return false; }
    if (faults.recoverable && !recovering) {
      recovering = true;
      started_ = now;
      clear_since_ = -1;
      healthy_since_ = -1;
      reason_bits = faults.recoverable;
      consumeAttempt(now);
    }
    if (recovering) {
      // Cached faults are not persistence evidence. CAN-disconnect needs Enable:
      // waiting in MANUAL for an eligible command must not spend a wall-time budget.
      // Its actual Enable attempts remain bounded below.
      if (!fresh || !(faults.recoverable & ~uint16_t{0x4000})) started_ = now;
      reason_bits |= faults.recoverable;
      if (now - started_ >= persistence_sec) {
        latched = true; recovering = false; latch_reason = "persistent_fault";
        return false;
      }
      if (!fresh || faults.recoverable) clear_since_ = -1;
      else if (clear_since_ < 0) clear_since_ = now;
      else if (now - clear_since_ >= clear_hold_sec) {
        recovering = false; healthy_since_ = now; return true;
      }
    } else if (fresh) {
      if (healthy_since_ < 0) healthy_since_ = now;
      if (now - healthy_since_ >= healthy_reset_sec) attempts = 0;
    } else healthy_since_ = -1;
    return false;
  }
  bool probeDue(double now) const {
    return recovering && !latched && now - last_attempt_ >= retry_sec;
  }
  bool consumeAttempt(double now) {
    if (attempts >= max_attempts) {
      latched = true; recovering = false; latch_reason = "retry_exhausted";
      return false;
    }
    ++attempts; ++total_attempts; last_attempt_ = now; return true;
  }
private:
  double started_{0}, last_attempt_{0}, clear_since_{-1}, healthy_since_{-1};
};
}  // namespace lateral_ctrl
