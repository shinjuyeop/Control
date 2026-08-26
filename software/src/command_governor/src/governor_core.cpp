#include "command_governor/governor_core.hpp"

#include <cmath>

namespace command_governor
{

const char * to_string(InputVerdict verdict)
{
  switch (verdict) {
    case InputVerdict::kAccept: return "ACCEPT";
    case InputVerdict::kRejectInvalidStamp: return "REJECT_INVALID_STAMP";
    case InputVerdict::kRejectStale: return "REJECT_STALE";
    case InputVerdict::kRejectFutureStamp: return "REJECT_FUTURE_STAMP";
    case InputVerdict::kRejectNonFinite: return "REJECT_NON_FINITE";
    default: return "UNKNOWN";
  }
}

InputVerdict evaluate_input(
  bool stamp_invalid, double age_ms, double timeout_ms,
  float speed_target_mps, float steering_target_rad)
{
  if (stamp_invalid) {
    return InputVerdict::kRejectInvalidStamp;
  }
  if (!std::isfinite(speed_target_mps) || !std::isfinite(steering_target_rad)) {
    return InputVerdict::kRejectNonFinite;
  }
  if (age_ms > timeout_ms) {
    return InputVerdict::kRejectStale;
  }
  // 미래 stamp 허용 폭도 timeout과 같게 둔다. 이보다 큰 음수 나이는 발행자
  // 클럭이 어긋난 것이므로, 재-stamp로 가려지기 전에 여기서 드러낸다.
  if (age_ms < -timeout_ms) {
    return InputVerdict::kRejectFutureStamp;
  }
  return InputVerdict::kAccept;
}

}  // namespace command_governor
