#include "lateral_ctrl/torque_steering_controller.hpp"

#include <gtest/gtest.h>
#include <limits>

using namespace lateral_ctrl;
namespace
{
constexpr double rad = 3.14159265358979323846 / 180.0;
void seed(TorqueSteeringController & controller, double angle = 0.0)
{
  controller.observeFeedback(angle, 1.0);
  controller.observeFeedback(angle, 1.02);
}
}

TEST(TorqueSteeringController, FullScaleAndSubScaleError)
{
  TorqueSteeringController c;
  seed(c);
  for (const double sign : {-1.0, 1.0}) {
    EXPECT_EQ(c.update(sign * 0.1 * rad, 0.0, 1.02).command_raw, sign * 400);
    EXPECT_EQ(c.update(sign * 0.25 * rad, 0.0, 1.02).command_raw, sign * 1000);
    EXPECT_EQ(c.update(sign * 0.5 * rad, 0.0, 1.02).command_raw, sign * 2000);
    const auto saturated = c.update(sign * 5.0 * rad, 0.0, 1.02);
    EXPECT_EQ(saturated.command_raw, sign * 2000);
    EXPECT_TRUE(saturated.saturated);
  }
}

TEST(TorqueSteeringController, FullRawRangeIsAvailable)
{
  TorqueSteeringConfig config;
  config.peak_limit_raw = 10000;
  TorqueSteeringController c(config);
  seed(c);
  EXPECT_EQ(c.update(0.5 * rad, 0.0, 1.02).command_raw, 10000);
  EXPECT_EQ(c.update(-0.5 * rad, 0.0, 1.02).command_raw, -10000);
}

TEST(TorqueSteeringController, MeasuredVelocityBrakesAtZeroErrorWithoutSetpointKick)
{
  for (const double sign : {-1.0, 1.0}) {
    TorqueSteeringController c;
    c.observeFeedback(0.0, 1.0);
    c.observeFeedback(sign * rad, 1.02);
    const auto out = c.update(sign * rad, 0.0, 1.02);
    EXPECT_DOUBLE_EQ(out.p_raw, 0.0);
    EXPECT_LT(sign * out.d_raw, 0.0);
    EXPECT_LT(sign * out.command_raw, 0.0);
    EXPECT_FALSE(out.settled);
    EXPECT_DOUBLE_EQ(c.update(sign * 5 * rad, 0.0, 1.02).d_raw, out.d_raw);
  }
}

TEST(TorqueSteeringController, FilterUsesFreshSamplesAndTheirIntervals)
{
  TorqueSteeringController c;
  c.observeFeedback(0.0, 1.0);
  c.observeFeedback(rad, 1.02);
  const auto first = c.update(rad, 0.0, 1.02);
  EXPECT_NEAR(first.unfiltered_rate_rad_s, 50 * rad, 1e-10);
  EXPECT_GT(first.rate_rad_s, 0.0);
  EXPECT_LT(first.rate_rad_s, first.unfiltered_rate_rad_s);
  c.observeFeedback(rad, 1.02);
  EXPECT_DOUBLE_EQ(c.update(rad, 0.0, 1.03).rate_rad_s, first.rate_rad_s);
  c.observeFeedback(2 * rad, 1.06);
  EXPECT_NEAR(c.update(2 * rad, 0.0, 1.06).unfiltered_rate_rad_s, 25 * rad, 1e-10);
}

TEST(TorqueSteeringController, SettleRequiresBothSmallErrorAndLowRate)
{
  TorqueSteeringController c;
  seed(c);
  EXPECT_TRUE(c.update(0.04 * rad, 0.0, 1.02).settled);
  EXPECT_EQ(c.update(0.04 * rad, 0.0, 1.02).command_raw, 0);
  EXPECT_GT(c.update(0.06 * rad, 0.0, 1.02).command_raw, 0);
  c.observeFeedback(rad, 1.04);
  EXPECT_FALSE(c.update(rad, 0.0, 1.04).settled);
  EXPECT_LT(c.update(rad, 0.0, 1.04).command_raw, 0);
}

TEST(TorqueSteeringController, SoftClampAndDirectionBlockingKeepInwardRecovery)
{
  TorqueSteeringConfig cfg;
  TorqueSteeringController c(cfg);
  seed(c, cfg.max_angle_rad);
  auto out = c.update(100.0, 10.0, 1.02);
  EXPECT_DOUBLE_EQ(out.target_rad, cfg.max_angle_rad - cfg.soft_limit_margin_rad);
  EXPECT_EQ(out.command_raw, 0);
  EXPECT_TRUE(out.direction_blocked);
  EXPECT_LT(c.update(0.0, 0.0, 1.02).command_raw, 0);
  c.reset(2.0);
  c.observeFeedback(cfg.min_angle_rad, 2.0);
  c.observeFeedback(cfg.min_angle_rad, 2.02);
  EXPECT_EQ(c.update(-100.0, -10.0, 2.02).command_raw, 0);
  EXPECT_GT(c.update(0.0, 0.0, 2.02).command_raw, 0);
  EXPECT_EQ(c.observeFeedback(cfg.min_angle_rad - 1e-6, 2.04), TorqueSafetyFault::ANGLE_LIMIT);
  EXPECT_EQ(c.update(0.0, 0.0, 2.04).command_raw, 0);
}

TEST(TorqueSteeringController, TaperAffectsOnlyOutwardEffort)
{
  TorqueSteeringConfig cfg;
  TorqueSteeringController c(cfg);
  const double angle = cfg.max_angle_rad - cfg.soft_limit_margin_rad / 2;
  seed(c, angle);
  EXPECT_EQ(c.update(100.0, 10.0, 1.02).command_raw, cfg.peak_limit_raw / 2);
  EXPECT_EQ(c.update(0.0, 0.0, 1.02).command_raw, -cfg.peak_limit_raw);
}

TEST(TorqueSteeringController, HardRateSafetyCannotBeFilteredAway)
{
  TorqueSteeringConfig cfg;
  cfg.rate_filter_hz = 0.01;
  TorqueSteeringController c(cfg);
  c.observeFeedback(0.0, 1.0);
  EXPECT_EQ(c.observeFeedback(3 * rad, 1.02), TorqueSafetyFault::RATE_LIMIT);
  EXPECT_EQ(c.observeFeedback(3 * rad, 1.02), TorqueSafetyFault::RATE_LIMIT);
  const auto out = c.update(10 * rad, 0.0, 1.02);
  EXPECT_EQ(out.fault, TorqueSafetyFault::RATE_LIMIT);
  EXPECT_EQ(out.command_raw, 0);
}

TEST(TorqueSteeringController, ResetRequiresFreshRateHistory)
{
  TorqueSteeringController c;
  seed(c);
  EXPECT_NE(c.update(rad, 0.0, 1.02).command_raw, 0);
  c.reset(1.03);
  EXPECT_EQ(c.update(rad, 0.0, 1.03).fault, TorqueSafetyFault::FEEDBACK_TIMEOUT);
  c.observeFeedback(0.0, 1.04);
  EXPECT_EQ(c.update(rad, 0.0, 1.04).command_raw, 0);
  c.observeFeedback(0.0, 1.06);
  EXPECT_NE(c.update(rad, 0.0, 1.06).command_raw, 0);
}

TEST(TorqueSteeringController, PeakBudgetSurvivesReverseResetAndReconfigure)
{
  TorqueSteeringConfig cfg;
  cfg.peak_duration_sec = 0.04;
  TorqueSteeringController c(cfg);
  seed(c);
  EXPECT_EQ(c.update(rad, 0.0, 1.02).command_raw, 2000);
  EXPECT_EQ(c.update(-rad, 0.0, 1.04).command_raw, -2000);
  auto out = c.update(rad, 0.0, 1.06);
  EXPECT_EQ(out.command_raw, 1000);
  EXPECT_TRUE(out.peak_limited);
  c.reset(1.06);
  c.configure(cfg);
  c.observeFeedback(0.0, 1.06);
  c.observeFeedback(0.0, 1.08);
  EXPECT_EQ(c.update(rad, 0.0, 1.08).command_raw, 1000);
  c.idle(1.10);
  c.idle(3.11);
  c.observeFeedback(0.0, 3.11);
  c.observeFeedback(0.0, 3.13);
  EXPECT_EQ(c.update(rad, 0.0, 3.13).command_raw, 2000);
}

TEST(TorqueSteeringController, FiveSecondOperatingEnvelopeDropsToSustainedLimit)
{
  TorqueSteeringConfig cfg;
  cfg.peak_limit_raw = 10000;
  cfg.sustained_limit_raw = 5000;
  cfg.peak_duration_sec = 5.0;
  cfg.peak_recovery_sec = 5.0;
  TorqueSteeringController c(cfg);
  seed(c);

  for (int tick = 0; tick < 250; ++tick) {
    const double time = 1.02 + tick * 0.02;
    c.observeFeedback(0.0, time);
    const auto out = c.update(rad, 0.0, time);
    EXPECT_EQ(out.command_raw, 10000) << "tick=" << tick;
    EXPECT_FALSE(out.peak_limited) << "tick=" << tick;
  }

  c.observeFeedback(0.0, 6.02);
  const auto limited = c.update(rad, 0.0, 6.02);
  EXPECT_EQ(limited.command_raw, 5000);
  EXPECT_TRUE(limited.peak_limited);
}

TEST(TorqueSteeringController, ContinuousSaturationNeverRestoresPeak)
{
  TorqueSteeringController c;
  seed(c);
  c.update(rad, 0.0, 1.02);
  for (int i = 1; i < 500; ++i) {
    const double t = 1.02 + i * 0.02;
    c.observeFeedback(0.0, t);
    const auto out = c.update(rad, 0.0, t);
    if (i >= 10) {
      EXPECT_EQ(out.command_raw, 1000);
    }
  }
}

TEST(TorqueSteeringController, InvalidInputAndStaleFeedbackFailClosed)
{
  TorqueSteeringController c;
  seed(c);
  const auto nan = std::numeric_limits<double>::quiet_NaN();
  EXPECT_EQ(c.update(nan, 0.0, 1.02).fault, TorqueSafetyFault::INVALID_INPUT);
  EXPECT_EQ(c.update(0.0, nan, 1.02).fault, TorqueSafetyFault::INVALID_INPUT);
  EXPECT_EQ(c.update(0.0, 0.0, nan).command_raw, 0);
  EXPECT_EQ(c.update(rad, 0.0, 1.2).fault, TorqueSafetyFault::FEEDBACK_TIMEOUT);
  EXPECT_EQ(c.observeFeedback(nan, 1.21), TorqueSafetyFault::INVALID_INPUT);
  EXPECT_EQ(c.observeFeedback(0.0, 0.5), TorqueSafetyFault::INVALID_INPUT);
}

TEST(TorqueSteeringController, InvalidConfigurationIsRejected)
{
  TorqueSteeringConfig cfg;
  cfg.peak_limit_raw = 10001;
  EXPECT_THROW(cfg.validate(), std::invalid_argument);
  cfg = {};
  cfg.sustained_limit_raw = 3000;
  EXPECT_THROW(cfg.validate(), std::invalid_argument);
  cfg = {};
  cfg.kd = std::numeric_limits<double>::quiet_NaN();
  EXPECT_THROW(cfg.validate(), std::invalid_argument);
  cfg = {};
  cfg.soft_limit_margin_rad = 0.5;
  EXPECT_THROW(cfg.validate(), std::invalid_argument);
  cfg = {};
  cfg.feedback_timeout_sec = 0.01;
  EXPECT_THROW(cfg.validate(), std::invalid_argument);
  cfg = {};
  cfg.peak_duration_sec = 5.0;
  cfg.peak_recovery_sec = 5.0;
  EXPECT_NO_THROW(cfg.validate());
  cfg.peak_duration_sec = 5.001;
  cfg.peak_recovery_sec = 5.001;
  EXPECT_THROW(cfg.validate(), std::invalid_argument);
}

TEST(TorqueSteeringController, FreshArrivalDoesNotHideExcessiveFeedbackGap)
{
  TorqueSteeringController c;
  seed(c);
  EXPECT_EQ(c.observeFeedback(0.0, 1.13), TorqueSafetyFault::FEEDBACK_TIMEOUT);
  const auto out = c.update(rad, 0.0, 1.13);
  EXPECT_EQ(out.fault, TorqueSafetyFault::FEEDBACK_TIMEOUT);
  EXPECT_EQ(out.command_raw, 0);
  EXPECT_FALSE(out.rate_ready);
}

TEST(TorqueSteeringController, MotorSignIsAppliedOnceAtBoundary)
{
  EXPECT_EQ(keyaTorqueCommand(1000, 1), 1000);
  EXPECT_EQ(keyaTorqueCommand(1000, -1), -1000);
  EXPECT_EQ(keyaTorqueCommand(-1000, -1), 1000);
  EXPECT_EQ(keyaTorqueCommand(0, -1), 0);
  EXPECT_THROW(keyaTorqueCommand(10, 0), std::invalid_argument);
  EXPECT_THROW(keyaTorqueCommand(10001, 1), std::invalid_argument);
}

TEST(TorqueSteeringController, DefaultMethodATermsLeavePdUnchanged)
{
  TorqueSteeringController c;
  seed(c);
  const auto out = c.update(0.1 * rad, 0.0, 50.0, 3.0, 1.02);
  EXPECT_DOUBLE_EQ(out.ff_raw, 0.0);
  EXPECT_DOUBLE_EQ(out.friction_raw, 0.0);
  EXPECT_EQ(out.command_raw, 400);
}

TEST(TorqueSteeringController, FrictionFollowsErrorSignOutsideDeadbandOnly)
{
  TorqueSteeringConfig cfg;
  cfg.full_scale_error_rad = 5.0 * rad;  // P = 400 raw/deg keeps the sum below the limit
  cfg.friction_raw = 106.0;
  cfg.friction_deadband_rad = 0.2 * rad;
  TorqueSteeringController c(cfg);
  seed(c);
  for (const double sign : {-1.0, 1.0}) {
    const auto out = c.update(sign * 1.0 * rad, 0.0, 1.02);
    EXPECT_DOUBLE_EQ(out.friction_raw, sign * 106.0);
    EXPECT_EQ(out.command_raw, sign * (400 + 106));
    const auto inside = c.update(sign * 0.15 * rad, 0.0, 1.02);
    EXPECT_DOUBLE_EQ(inside.friction_raw, 0.0);
    EXPECT_EQ(inside.command_raw, sign * 60);
  }
}

TEST(TorqueSteeringController, SettledTargetStillSendsZeroWithFriction)
{
  TorqueSteeringConfig cfg;
  cfg.friction_raw = 500.0;
  cfg.friction_deadband_rad = 0.0;
  TorqueSteeringController c(cfg);
  seed(c);
  const auto out = c.update(0.04 * rad, 0.0, 1.02);
  EXPECT_TRUE(out.settled);
  EXPECT_EQ(out.command_raw, 0);
}

TEST(TorqueSteeringController, FrictionScheduleInterpolatesAndHoldsEnds)
{
  TorqueSteeringConfig cfg;
  cfg.friction_raw = 999.0;  // ignored once a schedule exists
  cfg.friction_schedule_speed_mps = {0.5, 1.5, 3.5};
  cfg.friction_schedule_raw = {300.0, 200.0, 100.0};
  EXPECT_NO_THROW(cfg.validate());
  EXPECT_DOUBLE_EQ(cfg.frictionRawAt(0.0), 300.0);
  EXPECT_DOUBLE_EQ(cfg.frictionRawAt(1.0), 250.0);
  EXPECT_DOUBLE_EQ(cfg.frictionRawAt(-2.5), 150.0);
  EXPECT_DOUBLE_EQ(cfg.frictionRawAt(9.0), 100.0);
  cfg.full_scale_error_rad = 5.0 * rad;
  TorqueSteeringController c(cfg);
  seed(c);
  EXPECT_EQ(c.update(-1.0 * rad, 0.0, 0.0, 2.5, 1.02).command_raw, -(400 + 150));
}

TEST(TorqueSteeringController, InertiaFeedforwardAddsAndIsClamped)
{
  TorqueSteeringConfig cfg;
  cfg.inertia_ff = 20.0;
  cfg.feedforward_limit_raw = 300.0;
  TorqueSteeringController c(cfg);
  seed(c);
  // At zero error the term is still reported, but a settled target sends no torque.
  auto out = c.update(0.0, 0.0, 10.0, 0.0, 1.02);
  EXPECT_DOUBLE_EQ(out.ff_raw, 200.0);
  EXPECT_TRUE(out.settled);
  out = c.update(0.1 * rad, 0.0, -50.0, 0.0, 1.02);
  EXPECT_DOUBLE_EQ(out.ff_raw, -300.0);
  EXPECT_EQ(out.command_raw, 400 - 300);
  EXPECT_DOUBLE_EQ(out.unsaturated_raw, out.p_raw + out.d_raw + out.ff_raw + out.friction_raw);
}

TEST(TorqueSteeringController, NonFiniteAccelerationOrSpeedFailClosed)
{
  TorqueSteeringController c;
  seed(c);
  const auto nan = std::numeric_limits<double>::quiet_NaN();
  EXPECT_EQ(c.update(rad, 0.0, nan, 0.0, 1.02).fault, TorqueSafetyFault::INVALID_INPUT);
  EXPECT_EQ(c.update(rad, 0.0, 0.0, nan, 1.02).command_raw, 0);
}

TEST(TorqueSteeringController, MethodAConfigurationIsValidated)
{
  TorqueSteeringConfig cfg;
  cfg.friction_raw = -1.0;
  EXPECT_THROW(cfg.validate(), std::invalid_argument);
  cfg = {};
  cfg.inertia_ff = -0.1;
  EXPECT_THROW(cfg.validate(), std::invalid_argument);
  cfg = {};
  cfg.feedforward_limit_raw = 10001.0;
  EXPECT_THROW(cfg.validate(), std::invalid_argument);
  cfg = {};
  cfg.feedforward_filter_hz = 0.0;
  EXPECT_THROW(cfg.validate(), std::invalid_argument);
  cfg = {};
  cfg.friction_deadband_rad = std::numeric_limits<double>::quiet_NaN();
  EXPECT_THROW(cfg.validate(), std::invalid_argument);
  cfg = {};
  cfg.friction_schedule_speed_mps = {0.0, 1.0};
  cfg.friction_schedule_raw = {100.0};
  EXPECT_THROW(cfg.validate(), std::invalid_argument);
  cfg.friction_schedule_raw = {100.0, 100.0};
  EXPECT_NO_THROW(cfg.validate());
  cfg.friction_schedule_speed_mps = {1.0, 1.0};
  EXPECT_THROW(cfg.validate(), std::invalid_argument);
}

TEST(TargetAccelerationEstimator, RecoversConstantAccelerationFromCommandStamps)
{
  TargetAccelerationEstimator e;
  const double accel = 2.0;
  double estimate = 0.0;
  for (int i = 0; i < 40; ++i) {
    const double t = 10.0 + i * 0.05;  // 20 Hz planning cadence
    const double s = t - 10.0;
    estimate = e.observe(0.5 * accel * s * s, t, 5.0, 0.2);
    if (i < 2) {
      EXPECT_DOUBLE_EQ(estimate, 0.0);
    }
  }
  EXPECT_NEAR(estimate, accel, 1e-6);
}

TEST(TargetAccelerationEstimator, HeldTargetGivesZeroAndGapsOrReversedStampsRestart)
{
  TargetAccelerationEstimator e;
  for (int i = 0; i < 10; ++i) EXPECT_DOUBLE_EQ(e.observe(0.1, 1.0 + i * 0.02, 5.0, 0.2), 0.0);
  e.observe(0.2, 1.20, 50.0, 0.2);
  EXPECT_NE(e.acceleration(), 0.0);
  EXPECT_DOUBLE_EQ(e.observe(0.3, 1.50, 50.0, 0.2), 0.0);  // 300 ms gap
  e.observe(0.4, 1.52, 50.0, 0.2);
  e.observe(0.6, 1.54, 50.0, 0.2);
  EXPECT_NE(e.acceleration(), 0.0);
  EXPECT_DOUBLE_EQ(e.observe(0.6, 1.54, 50.0, 0.2), 0.0);  // duplicate stamp
  EXPECT_DOUBLE_EQ(e.observe(std::numeric_limits<double>::quiet_NaN(), 1.56, 50.0, 0.2), 0.0);
}
