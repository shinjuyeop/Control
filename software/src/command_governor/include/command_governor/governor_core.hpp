#ifndef COMMAND_GOVERNOR__GOVERNOR_CORE_HPP_
#define COMMAND_GOVERNOR__GOVERNOR_CORE_HPP_

namespace command_governor
{

// 입력 명령 한 건을 내보낼지에 대한 판정.
// 거버너는 출력을 자기 클럭으로 재-stamp하므로, 여기서 거른 입력만이
// "신선한 명령"으로 하류에 나간다. 이 게이트가 없으면 오래된 입력이
// 재-stamp로 되살아나 조향의 200 ms freshness 검사를 무력화한다.
enum class InputVerdict
{
  kAccept = 0,
  kRejectInvalidStamp,  // stamp가 0(미기록) 또는 sec가 음수(손상·언더플로)
  kRejectStale,         // 입력 나이가 timeout 초과
  kRejectFutureStamp,   // 입력 stamp가 허용 범위 밖의 미래 — 클럭 불일치
  kRejectNonFinite,     // speed/steering 필드에 NaN 또는 Inf
};

const char * to_string(InputVerdict verdict);

// age_ms는 거버너 클럭 기준 (now - input stamp)이며 미래 stamp면 음수다.
// stamp_invalid가 참이면 age_ms는 판정에 쓰이지 않는다. 호출자는 음수 sec를
// rclcpp::Time으로 만들기 전에 반드시 stamp_invalid로 걸러야 한다 — 그
// 생성자는 음수 시각에 예외를 던진다.
// 판정 순서: invalid stamp → non-finite → stale → future.
// 경계값: age_ms == timeout_ms는 수락한다. 조향의 신선도 검사(age > timeout
// 거부)와 같은 부등호 방향을 쓴다.
InputVerdict evaluate_input(
  bool stamp_invalid, double age_ms, double timeout_ms,
  float speed_target_mps, float steering_target_rad);

// 수락한 명령의 속도 목표. brake면 0, 그 밖에는 0·음수·계단 변화까지 그대로 전달한다.
// 변화율 제한은 두지 않는다: 종방향 변화율은 STM_A와 상위 계획기가 맡는다.
float forwarded_speed(float speed_target_mps, bool brake_engage);

}  // namespace command_governor

#endif  // COMMAND_GOVERNOR__GOVERNOR_CORE_HPP_
