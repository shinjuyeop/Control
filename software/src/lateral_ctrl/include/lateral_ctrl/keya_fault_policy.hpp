#pragma once

#include <algorithm>
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

// Pure policy: monotonic time, no I/O. Only hardware HARD bits can latch.
class KeyaRecoveryPolicy
{
public:
  double clear_hold_sec{0.5}, retry_sec{1.0};
  bool recovering{false}, latched{false};
  uint64_t attempts{0}, total_attempts{0};
  uint16_t reason_bits{0};
  const char * latch_reason{"none"};
  double last_recovery_sec{-1.0};

  // True on recovery completion: caller must discard all queued targets.
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
      clear_since_ = -1;
      reason_bits = faults.recoverable;
      attempts = 0;
      recordAttempt(now);  // Initial zero/Disable reset for this episode.
    }
    if (recovering) {
      reason_bits |= faults.recoverable;
      if (!fresh || faults.recoverable) clear_since_ = -1;
      else if (clear_since_ < 0) clear_since_ = now;
      else if (now - clear_since_ >= clear_hold_sec) {
        recovering = false;
        last_recovery_sec = now;
        return true;
      }
    }
    return false;
  }
  bool probeDue(double now) const {
    return recovering && !latched && now - last_attempt_ >= retry_delay_sec_;
  }
  void recordAttempt(double now) {
    ++attempts;
    ++total_attempts;
    last_attempt_ = now;
    // Rate limit attempts indefinitely; never convert a retry statistic to FAULT.
    retry_delay_sec_ = attempts == 1 ? retry_sec :
      std::min(std::max(5.0, retry_sec), retry_delay_sec_ * 2.0);
  }
  double retryDelaySec() const { return retry_delay_sec_; }
  // The node must check MANUAL/stop/freshness/active bits before acknowledging.
  void acknowledgeHardLatch() {
    latched = recovering = false;
    latch_reason = "none";
    attempts = 0;
    last_attempt_ = retry_delay_sec_ = 0;
    clear_since_ = -1;
    // Keep lifetime count, last reason and last recovery time for diagnosis.
  }
private:
  double last_attempt_{0}, retry_delay_sec_{0}, clear_since_{-1};
};
}  // namespace lateral_ctrl
