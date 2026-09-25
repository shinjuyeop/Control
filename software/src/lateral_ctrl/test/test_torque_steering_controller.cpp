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
  for (const double sign : {-1.0, 1.0}) {
    TorqueSteeringController c(cfg);
    const double boundary = sign > 0 ? cfg.max_angle_rad : cfg.min_angle_rad;
    seed(c, boundary);
    auto out = c.update(sign * 100.0, sign * 10.0, 1.02);
    EXPECT_DOUBLE_EQ(out.target_rad, boundary - sign * cfg.soft_limit_margin_rad);
    EXPECT_EQ(out.command_raw, 0);
    EXPECT_TRUE(out.direction_blocked);
    EXPECT_LT(sign * c.update(0.0, 0.0, 1.02).command_raw, 0);

    // Cross the calibration boundary slowly: retain inward recovery without a
    // fault, while an outward D term must still be blocked outside the range.
    EXPECT_EQ(c.observeFeedback(boundary + sign * 0.001, 1.04), TorqueSafetyFault::NONE);
    out = c.update(0.0, 0.0, 1.04);
    EXPECT_EQ(out.fault, TorqueSafetyFault::NONE);
    EXPECT_LT(sign * out.command_raw, 0);
    out = c.update(sign * 100.0, sign * 10.0, 1.04);
    EXPECT_EQ(out.command_raw, 0);
    EXPECT_TRUE(out.direction_blocked);
  }
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

TEST(TorqueSteeringController, HighRateRemainsTelemetryWithoutFault)
{
  TorqueSteeringConfig cfg;
  cfg.rate_filter_hz = 0.01;
  TorqueSteeringController c(cfg);
  c.observeFeedback(0.0, 1.0);
  EXPECT_EQ(c.observeFeedback(3 * rad, 1.02), TorqueSafetyFault::NONE);
  EXPECT_EQ(c.observeFeedback(3 * rad, 1.02), TorqueSafetyFault::NONE);
  const auto out = c.update(10 * rad, 0.0, 1.02);
  EXPECT_EQ(out.fault, TorqueSafetyFault::NONE);
  EXPECT_NE(out.command_raw, 0);
  EXPECT_NEAR(out.unfiltered_rate_rad_s, 150.0 * rad, 1e-12);
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
  cfg.settle_rate_rad_s = 0.0;
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

TEST(TorqueSteeringController, ExternalLawRetainsOutputLimitsAndFeedbackStop)
{
  TorqueSteeringController c;
  seed(c);
  int calls=0;
  auto law=[&calls](double,double,double,double limit) { ++calls; return limit*2; };
  const auto out=c.update(5*rad,0,1.02,law);
  EXPECT_EQ(out.command_raw,2000);
  EXPECT_TRUE(out.saturated);
  const auto stale=c.update(5*rad,0,1.3,law);
  EXPECT_EQ(stale.command_raw,0);
  EXPECT_EQ(stale.fault,TorqueSafetyFault::FEEDBACK_TIMEOUT);
  EXPECT_EQ(calls,1);
}
TEST(TorqueSteeringController, FailedExternalLawStopsOutput)
{
  TorqueSteeringController c;
  seed(c);
  const auto out=c.update(5*rad,0,1.02,[](double,double,double,double) {
    return std::numeric_limits<double>::quiet_NaN();
  });
  EXPECT_EQ(out.command_raw,0);
  EXPECT_EQ(out.fault,TorqueSafetyFault::INVALID_INPUT);
}
