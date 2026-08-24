#pragma once

#include "lateral_ctrl/steering_types.hpp"

#include <chrono>
#include <cstdint>

namespace lateral_ctrl::detail
{
bool applyKeyaCanFrame(
  KeyaMotorState & state, uint32_t feedback_id, uint32_t heartbeat_id,
  uint32_t can_id, uint8_t dlc, const uint8_t data[8],
  std::chrono::steady_clock::time_point received_at);
}  // namespace lateral_ctrl::detail
