#include "longitudinal_ctrl/speed_torque_controller.hpp"

#include <gtest/gtest.h>

#include <cmath>
#include <limits>

using namespace longitudinal_ctrl;

namespace
{
constexpr double kDt = 0.02;

// 일정 차속을 여러 샘플 넣어 필터가 수렴한 상태를 만든다.
void seedSpeed(SpeedTorqueController & c, double speed, double & t, int samples = 200)
{
  for (int i = 0; i < samples; ++i) {
    t += kDt;
    c.observeSpeed(speed, t);
  }
}

// 식별 모델을 그대로 적분하는 단순 플랜트.
struct Plant
{
  double v{0.0};
  void step(double current, const SpeedTorqueConfig & c, double dt = kDt)
  {
    const double a = c.current_gain_mps2_per_a * current - c.rolling_resistance_mps2 -
      c.speed_damping_per_s * v;
    v = std::max(0.0, v + a * dt);
  }
};
}  // namespace

TEST(SpeedTorqueConfig, ValidateRejectsInconsistentValues)
{
  SpeedTorqueConfig c;
  EXPECT_NO_THROW(c.validate());
  c.current_gain_mps2_per_a = 0.0;
  EXPECT_THROW(c.validate(), std::invalid_argument);
  c = {};
  c.speed_filter_hz = 30.0;  // > 25 Hz Nyquist at 20 ms
  EXPECT_THROW(c.validate(), std::invalid_argument);
  c = {};
  c.feedback_timeout_sec = 0.01;
  EXPECT_THROW(c.validate(), std::invalid_argument);
  c = {};
  c.max_regen_current_a = -1.0;
  EXPECT_THROW(c.validate(), std::invalid_argument);
  c = {};
  c.kp_a_per_mps = std::numeric_limits<double>::quiet_NaN();
  EXPECT_THROW(c.validate(), std::invalid_argument);
  c = {};
  c.max_regen_current_a = 0.0;  // coast-only is allowed
  EXPECT_NO_THROW(c.validate());
}

TEST(SpeedTorqueController, SpeedSpikesAreRejectedAndFilterConverges)
{
  SpeedTorqueController c;
  double t = 0.0;
  seedSpeed(c, 5.0, t);
  EXPECT_NEAR(c.filteredSpeed(), 5.0, 1e-3);
  // 9.3, 12.4, 171 m/s 같은 RPM spike와 0 dropout은 버려진다.
  for (double spike : {9.3, 12.4, 171.0, 0.0}) {
    t += kDt;
    EXPECT_FALSE(c.observeSpeed(spike, t));
    EXPECT_NEAR(c.filteredSpeed(), 5.0, 1e-3);
  }
  EXPECT_EQ(c.rejectedTotal(), 4u);
  // 정상 샘플은 다시 받아들인다.
  t += kDt;
  EXPECT_TRUE(c.observeSpeed(5.05, t));
  EXPECT_EQ(c.rejectedStreak(), 0);
}

TEST(SpeedTorqueController, PersistentJumpIsReacquiredAfterMaxRejections)
{
  SpeedTorqueConfig config;
  config.speed_jump_reject_max_samples = 3;
  SpeedTorqueController c(config);
  double t = 0.0;
  seedSpeed(c, 5.0, t);
  for (int i = 0; i < 3; ++i) {
    t += kDt;
    EXPECT_FALSE(c.observeSpeed(1.0, t));
  }
  t += kDt;
  EXPECT_TRUE(c.observeSpeed(1.0, t));
  EXPECT_NEAR(c.filteredSpeed(), 1.0, 1e-9);
}

TEST(SpeedTorqueController, RejectedSamplesDoNotRefreshFeedback)
{
  SpeedTorqueController c;
  double t = 0.0;
  seedSpeed(c, 5.0, t);
  const double last_good = t;
  for (int i = 0; i < 8; ++i) {
    t += kDt;
    c.observeSpeed(50.0, t);
  }
  EXPECT_TRUE(c.feedbackFresh(last_good + 0.29));
  EXPECT_FALSE(c.feedbackFresh(last_good + 0.31));
}

TEST(SpeedTorqueController, BumplessStartMatchesMeasuredCurrent)
{
  SpeedTorqueController c;
  double t = 0.0;
  seedSpeed(c, 5.0, t);
  c.start(t, 120.0);
  const auto out = c.update(5.0, t + kDt);
  EXPECT_NEAR(out.command_a, 120.0, 1.0);
  EXPECT_NEAR(out.shaped_target_mps, 5.0, 1e-6);
  EXPECT_FALSE(out.saturated);
}

TEST(SpeedTorqueController, SteadyStateFeedforwardMatchesIdentifiedMap)
{
  SpeedTorqueController c;
  const auto & cfg = c.config();
  double t = 0.0;
  seedSpeed(c, 6.0, t);
  c.start(t, 0.0);
  // 적분기가 -ff0로 시작하므로 첫 명령은 0이고, 오차가 0이면 ff만 남아야 한다.
  SpeedTorqueOutput out;
  for (int i = 0; i < 5; ++i) {
    t += kDt;
    c.observeSpeed(6.0, t);
    out = c.update(6.0, t);
  }
  const double expected = (cfg.rolling_resistance_mps2 + cfg.speed_damping_per_s * 6.0) /
    cfg.current_gain_mps2_per_a;
  EXPECT_NEAR(out.ff_a, expected, 1e-6);
  EXPECT_NEAR(expected, 78.3, 0.5);  // ≈ 45 + 5.6 * 6 A
  EXPECT_NEAR(out.error_mps, 0.0, 1e-9);
  EXPECT_NEAR(out.p_a, 0.0, 1e-9);
}

TEST(SpeedTorqueController, AccelerationStepIsShapedAndFeedforwardIsBounded)
{
  SpeedTorqueController c;
  const auto & cfg = c.config();
  double t = 0.0;
  seedSpeed(c, 2.0, t);
  c.start(t, 60.0);
  t += kDt;
  auto out = c.update(6.0, t);
  EXPECT_NEAR(out.shaped_target_mps, 2.0 + cfg.accel_limit_mps2 * kDt, 1e-9);
  EXPECT_NEAR(out.target_accel_mps2, cfg.accel_limit_mps2, 1e-9);
  const double accel_ff = cfg.accel_limit_mps2 / cfg.current_gain_mps2_per_a;
  EXPECT_NEAR(accel_ff, 195.3, 0.5);
  EXPECT_GT(out.ff_a, accel_ff);  // + 저항 항
  EXPECT_LE(out.command_a, cfg.max_drive_current_a);
  // 목표가 실측 차속에서 tracking_error_limit 이상 앞서지 않는다.
  for (int i = 0; i < 200; ++i) {
    t += kDt;
    c.observeSpeed(2.0, t);
    out = c.update(6.0, t);
  }
  EXPECT_NEAR(out.shaped_target_mps, 2.0 + cfg.tracking_error_limit_mps, 1e-9);
  EXPECT_NEAR(out.target_accel_mps2, 0.0, 1e-9);
}

TEST(SpeedTorqueController, ConditionalIntegrationStopsWindupWhileSaturated)
{
  SpeedTorqueController c;
  const auto & cfg = c.config();
  double t = 0.0;
  seedSpeed(c, 1.0, t);
  c.start(t, 0.0);
  double integrator = 0.0;
  for (int i = 0; i < 300; ++i) {
    t += kDt;
    c.observeSpeed(1.0, t);
    const auto out = c.update(6.0, t);
    if (out.saturated && out.error_mps > 0.0) {
      // 포화 방향으로는 적분이 더 자라지 않는다.
      EXPECT_LE(out.i_a, integrator + 1e-9);
    }
    integrator = out.i_a;
    EXPECT_LE(out.command_a, cfg.max_drive_current_a + 1e-9);
    EXPECT_LE(std::fabs(out.i_a), cfg.integrator_limit_a + 1e-9);
  }
}

TEST(SpeedTorqueController, RegenIsClampedAndBlockedNearStandstill)
{
  SpeedTorqueController c;
  const auto & cfg = c.config();
  double t = 0.0;
  seedSpeed(c, 6.0, t);
  c.start(t, 80.0);
  SpeedTorqueOutput out;
  for (int i = 0; i < 100; ++i) {
    t += kDt;
    c.observeSpeed(6.0, t);
    out = c.update(2.0, t);
  }
  EXPECT_NEAR(out.command_a, -cfg.max_regen_current_a, 1e-6);
  EXPECT_TRUE(out.saturated);
  EXPECT_FALSE(out.regen_blocked);
  // 차속이 regen_min_speed 아래이면 음의 전류를 내지 않는다.
  SpeedTorqueController slow;
  t = 0.0;
  seedSpeed(slow, 0.4, t);
  slow.start(t, 0.0);
  for (int i = 0; i < 50; ++i) {
    t += kDt;
    slow.observeSpeed(0.4, t);
    out = slow.update(0.0, t);
  }
  EXPECT_TRUE(out.regen_blocked);
  EXPECT_GE(out.command_a, 0.0);
}

TEST(SpeedTorqueController, CoastOnlyWhenRegenLimitIsZero)
{
  SpeedTorqueConfig config;
  config.max_regen_current_a = 0.0;
  SpeedTorqueController c(config);
  double t = 0.0;
  seedSpeed(c, 6.0, t);
  c.start(t, 80.0);
  SpeedTorqueOutput out;
  for (int i = 0; i < 100; ++i) {
    t += kDt;
    c.observeSpeed(6.0, t);
    out = c.update(2.0, t);
  }
  EXPECT_DOUBLE_EQ(out.command_a, 0.0);
  EXPECT_TRUE(out.saturated);
}

TEST(SpeedTorqueController, StopWhenTargetAndSpeedAreBelowThreshold)
{
  SpeedTorqueController c;
  double t = 0.0;
  seedSpeed(c, 0.2, t);
  c.start(t, 40.0);
  SpeedTorqueOutput out;
  for (int i = 0; i < 20; ++i) {
    t += kDt;
    c.observeSpeed(0.2, t);
    out = c.update(0.0, t);
  }
  EXPECT_TRUE(out.stopped);
  EXPECT_DOUBLE_EQ(out.command_a, 0.0);
  EXPECT_DOUBLE_EQ(out.i_a, 0.0);
}

TEST(SpeedTorqueController, SlewRateLimitsCommandSteps)
{
  SpeedTorqueController c;
  const auto & cfg = c.config();
  double t = 0.0;
  seedSpeed(c, 3.0, t);
  c.start(t, 0.0);
  t += kDt;
  const auto out = c.update(6.0, t);
  EXPECT_TRUE(out.slew_limited);
  EXPECT_NEAR(out.command_a, cfg.current_slew_a_per_s * kDt, 1e-6);
}

TEST(SpeedTorqueController, BusCurrentGuardReducesRegenCap)
{
  SpeedTorqueController c;
  const auto & cfg = c.config();
  double t = 0.0;
  seedSpeed(c, 6.0, t);
  c.start(t, 0.0);
  c.observeBusCurrent(-60.0);  // 충전 한계 35 A 초과
  SpeedTorqueOutput out;
  for (int i = 0; i < 100; ++i) {
    t += kDt;
    c.observeSpeed(6.0, t);
    out = c.update(2.0, t);
  }
  EXPECT_LT(out.regen_cap_a, cfg.max_regen_current_a);
  EXPECT_GE(out.regen_cap_a, 10.0);
  EXPECT_GE(out.command_a, -out.regen_cap_a - 1e-9);
  c.observeBusCurrent(-5.0);
  for (int i = 0; i < 200; ++i) {
    t += kDt;
    c.observeSpeed(6.0, t);
    out = c.update(2.0, t);
  }
  EXPECT_NEAR(out.regen_cap_a, cfg.max_regen_current_a, 1e-6);
}

TEST(SpeedTorqueController, ClosedLoopOnIdentifiedPlantTracksRampWithoutOvershoot)
{
  SpeedTorqueController c;
  const auto & cfg = c.config();
  Plant plant;
  plant.v = 3.0;
  double t = 0.0;
  seedSpeed(c, plant.v, t);
  c.start(t, (cfg.rolling_resistance_mps2 + cfg.speed_damping_per_s * plant.v) /
    cfg.current_gain_mps2_per_a);
  double peak = 0.0;
  SpeedTorqueOutput out;
  for (int i = 0; i < 1500; ++i) {  // 30 s
    t += kDt;
    out = c.update(6.0, t);
    plant.step(out.command_a, cfg);
    c.observeSpeed(plant.v, t);
    peak = std::max(peak, plant.v);
  }
  EXPECT_NEAR(plant.v, 6.0, 0.05);
  EXPECT_LT(peak, 6.15);
  // 감속: 회생 한계 안에서 undershoot 없이 내려온다.
  double trough = plant.v;
  for (int i = 0; i < 1500; ++i) {
    t += kDt;
    out = c.update(3.0, t);
    plant.step(out.command_a, cfg);
    c.observeSpeed(plant.v, t);
    trough = std::min(trough, plant.v);
  }
  EXPECT_NEAR(plant.v, 3.0, 0.05);
  EXPECT_GT(trough, 2.85);
}

TEST(SpeedTorqueController, ReconfigureKeepsStateWithinNewLimits)
{
  SpeedTorqueController c;
  double t = 0.0;
  seedSpeed(c, 5.0, t);
  c.start(t, 100.0);
  SpeedTorqueConfig config = c.config();
  config.integrator_limit_a = 10.0;
  config.max_regen_current_a = 20.0;
  c.configure(config);
  t += kDt;
  const auto out = c.update(5.0, t);
  EXPECT_LE(std::fabs(out.i_a), 10.0 + 1e-9);
  EXPECT_LE(out.regen_cap_a, 20.0 + 1e-9);
}

TEST(SpeedTorqueController, StopClearsControllerButKeepsSpeedEstimate)
{
  SpeedTorqueController c;
  double t = 0.0;
  seedSpeed(c, 5.0, t);
  c.start(t, 100.0);
  c.update(5.0, t + kDt);
  c.stop();
  EXPECT_FALSE(c.active());
  EXPECT_NEAR(c.filteredSpeed(), 5.0, 1e-3);
  EXPECT_TRUE(c.feedbackFresh(t + 0.1));
}
