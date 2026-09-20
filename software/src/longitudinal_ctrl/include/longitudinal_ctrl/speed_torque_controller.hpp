#pragma once

#include <cstdint>

namespace longitudinal_ctrl
{
// 차속 → EZkontrol 상전류(torque control) 명령을 만드는 외부 속도 루프.
//
// 플랜트 모델은 2026-09-20 오전 18개 기록(GNSS 속도 vs Part I phase current)에서
// 식별했다: dv/dt = current_gain * I_phase - rolling_resistance - speed_damping * v.
// 이 모델로 feedforward를 만들고, PI는 모델 오차(하중·경사·노면·SOC)만 보정한다.
// 단위: 속도 m/s, 가속도 m/s², 전류 A, 시간 s. 전류 부호는 +구동, -회생.
struct SpeedTorqueConfig
{
  double control_period_sec{0.02};

  // 플랜트 (식별값). 1 m/s²에 약 195 A, 정속 유지 전류 ≈ 45 + 5.6·v A.
  double current_gain_mps2_per_a{0.00512};
  double rolling_resistance_mps2{0.228};
  double speed_damping_per_s{0.0288};

  // feedforward 배율. 0이면 해당 항을 끈다.
  double feedforward_accel_scale{1.0};
  double feedforward_resistance_scale{1.0};

  // PI. ωn=1 rad/s, ζ=1 기준: kp = 2ζωn/K ≈ 380, ki = ωn²/K ≈ 190.
  double kp_a_per_mps{380.0};
  double ki_a_per_m{190.0};
  double integrator_limit_a{150.0};

  // 목표 형상. 외부 목표가 이미 ramp라면 그대로 통과한다.
  double accel_limit_mps2{1.0};
  double decel_limit_mps2{1.0};
  // 형상 목표가 실측 차속에서 이보다 멀어지지 않게 잡는다(reference governor).
  // 출력 포화 중 목표만 앞서가서 생기는 windup·undershoot를 막는다.
  double tracking_error_limit_mps{1.0};

  // 출력 한계. STM_A가 ±480 A로 다시 자른다.
  double max_drive_current_a{300.0};
  double max_regen_current_a{80.0};       // 크기. 0이면 감속 시 타력(0 A)만 사용
  double regen_bus_current_limit_a{35.0}; // 실측 DC bus 충전전류가 넘으면 회생 상한을 줄인다
  double current_slew_a_per_s{800.0};
  double regen_min_speed_mps{0.5};        // 이보다 느리면 음의 전류를 내지 않는다
  double stop_speed_mps{0.3};             // 목표·실측이 모두 이하이면 0 A, 적분 초기화

  // 차속 추정. RPM 기반 차속의 spike(9→12→171 m/s)와 0 dropout을 거른다.
  // 같은 필터를 형상 목표에도 걸어 오차를 만들므로, ramp 중 필터 지연이 오차·적분으로
  // 쌓이지 않는다(feedforward는 필터 전 목표 가속도를 쓴다).
  double speed_filter_hz{2.0};
  double speed_jump_reject_mps{1.0};
  int speed_jump_reject_max_samples{8};
  double feedback_timeout_sec{0.3};

  void validate() const;
};

struct SpeedTorqueOutput
{
  double target_mps{0.0}, shaped_target_mps{0.0}, filtered_target_mps{0.0}, target_accel_mps2{0.0};
  double speed_mps{0.0}, error_mps{0.0};
  double ff_a{0.0}, p_a{0.0}, i_a{0.0}, unsaturated_a{0.0}, command_a{0.0};
  double regen_cap_a{0.0};
  bool saturated{false}, slew_limited{false}, regen_blocked{false}, stopped{false};
};

class SpeedTorqueController
{
public:
  explicit SpeedTorqueController(const SpeedTorqueConfig & config = {});
  // 실행 중 재설정. 적분기·차속 추정은 유지하고 새 한계로 다시 자른다.
  void configure(const SpeedTorqueConfig & config);
  const SpeedTorqueConfig & config() const {return config_;}

  // 원시 차속 샘플 하나를 반영한다. spike로 거른 경우 false.
  bool observeSpeed(double speed_mps, double time_sec);
  void observeBusCurrent(double bus_current_a);
  bool haveSpeed() const {return have_speed_;}
  bool feedbackFresh(double now_sec) const;
  double filteredSpeed() const {return speed_mps_;}
  double rawSpeed() const {return raw_speed_mps_;}
  int rejectedStreak() const {return rejected_streak_;}
  uint64_t rejectedTotal() const {return rejected_total_;}
  void resetSpeedEstimate();

  // bumpless 진입: 형상 목표는 현재 차속에서, 출력은 현재 실측 전류에서 시작한다.
  void start(double now_sec, double initial_current_a);
  void stop();
  bool active() const {return active_;}
  SpeedTorqueOutput update(double target_mps, double now_sec);

private:
  double resistanceCurrentA(double speed_mps) const;

  SpeedTorqueConfig config_;
  bool have_speed_{false};
  double speed_mps_{0.0}, raw_speed_mps_{0.0}, last_sample_time_sec_{0.0};
  int rejected_streak_{0};
  uint64_t rejected_total_{0};
  double bus_current_a_{0.0};

  bool active_{false};
  double shaped_target_mps_{0.0}, filtered_target_mps_{0.0}, integrator_a_{0.0}, last_command_a_{0.0};
  double regen_cap_a_{0.0}, last_update_time_sec_{0.0};
};
}  // namespace longitudinal_ctrl
