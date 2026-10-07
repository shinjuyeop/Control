#include <cmath>
#include <limits>

#include <gtest/gtest.h>

#include "command_governor/governor_core.hpp"

namespace
{

using command_governor::InputVerdict;
using command_governor::evaluate_input;
using command_governor::to_string;

constexpr double kTimeoutMs = 200.0;
constexpr float kNan = std::numeric_limits<float>::quiet_NaN();
constexpr float kInf = std::numeric_limits<float>::infinity();

TEST(SpeedSlew, AccelerationStartsAtZeroAndUsesIndependentUpAndDownRates)
{
  command_governor::SpeedSlew slew;
  EXPECT_FLOAT_EQ(slew.update(5, false, 0, .2, .5, 1), 0);
  for (int i = 1; i <= 50; ++i) {
    EXPECT_NEAR(slew.update(5, false, i * .1, .2, .5, 1), i * .1, 1e-5);
  }
  EXPECT_NEAR(slew.update(2, false, 5.1, .2, .5, 1), 4.95, 1e-5);
  EXPECT_FLOAT_EQ(slew.update(0, false, 5.15, .2, .5, 1), 0);
  EXPECT_NEAR(slew.update(5, false, 5.2, .2, .5, 1), .05, 1e-5);
  EXPECT_FLOAT_EQ(slew.update(5, true, 5.25, .2, .5, 1), 0);
  EXPECT_NEAR(slew.update(5, false, 5.3, .2, .5, 1), .05, 1e-5);
  EXPECT_FLOAT_EQ(slew.update(5, false, 6, .2, .5, 1), 0);
  EXPECT_NEAR(slew.update(5, false, 6.1, .2, .5, 1), .1, 1e-5);
  EXPECT_FLOAT_EQ(slew.update(5, false, 6.15, .2, .5, 0), 5);
}

TEST(SpeedSlew, DownwardStepConvergesAtConfiguredRateWithVariablePeriods)
{
  command_governor::SpeedSlew slew;
  EXPECT_FLOAT_EQ(slew.update(5, false, 0, .2, .5), 5);
  EXPECT_FLOAT_EQ(slew.update(2, false, .1, .2, .5), 4.95f);
  EXPECT_FLOAT_EQ(slew.update(2, false, .15, .2, .5), 4.925f);
  for (int i = 2; i <= 60; ++i) {
    const float output = slew.update(2, false, i * .1, .2, .5);
    EXPECT_NEAR(output, std::max(2.0, 5.0 - i * .05), 1e-5);
  }
  EXPECT_FLOAT_EQ(slew.update(2, false, 6.1, .2, .5), 2);
  EXPECT_FLOAT_EQ(slew.update(5, false, 6.15, .2, .5), 5);
}

TEST(SpeedSlew, ZeroBrakeReverseAndStreamRecoveryNeverRetainOldSpeed)
{
  command_governor::SpeedSlew slew;
  slew.update(5, false, 0, .2, .5);
  EXPECT_FLOAT_EQ(slew.update(0, false, .05, .2, .5), 0);
  EXPECT_FLOAT_EQ(slew.update(2, false, .1, .2, .5), 2);
  EXPECT_FLOAT_EQ(slew.update(5, true, .15, .2, .5), 0);
  EXPECT_FLOAT_EQ(slew.update(1, false, .2, .2, .5), 1);
  EXPECT_FLOAT_EQ(slew.update(-1, false, .25, .2, .5), -1);
  slew.update(5, false, .3, .2, .5);
  EXPECT_FLOAT_EQ(slew.update(1, false, 1, .2, .5), 1);
  slew.update(5, false, 1.05, .2, .5);
  EXPECT_FLOAT_EQ(slew.update(1, false, 1.1, .2, 0), 1);
}

TEST(EvaluateInput, AcceptsFreshFiniteCommand)
{
  EXPECT_EQ(
    evaluate_input(false, 5.0, kTimeoutMs, 1.5f, 0.1f), InputVerdict::kAccept);
}

TEST(EvaluateInput, AcceptsZeroAgeAndZeroTargets)
{
  EXPECT_EQ(
    evaluate_input(false, 0.0, kTimeoutMs, 0.0f, 0.0f), InputVerdict::kAccept);
}

TEST(EvaluateInput, RejectsInvalidStamp)
{
  // stamp가 0이거나 음수면 나이 계산 자체가 무의미하므로 다른 판정보다 먼저
  // 거른다. 0/음수 판별 자체는 노드가 rclcpp::Time 생성 전에 수행한다 —
  // 음수 시각은 그 생성자가 예외를 던지기 때문이다.
  EXPECT_EQ(
    evaluate_input(true, 0.0, kTimeoutMs, 0.0f, 0.0f),
    InputVerdict::kRejectInvalidStamp);
  // invalid stamp는 다른 어떤 필드 상태보다 우선한다.
  EXPECT_EQ(
    evaluate_input(true, kTimeoutMs + 100.0, kTimeoutMs, kNan, kInf),
    InputVerdict::kRejectInvalidStamp);
}

TEST(EvaluateInput, RejectsStaleInput)
{
  EXPECT_EQ(
    evaluate_input(false, kTimeoutMs + 0.1, kTimeoutMs, 0.0f, 0.0f),
    InputVerdict::kRejectStale);
}

TEST(EvaluateInput, AcceptsAgeExactlyAtTimeout)
{
  // 조향의 신선도 검사(age > timeout 거부)와 같은 부등호 방향.
  EXPECT_EQ(
    evaluate_input(false, kTimeoutMs, kTimeoutMs, 0.0f, 0.0f),
    InputVerdict::kAccept);
}

TEST(EvaluateInput, AcceptsSlightlyFutureStamp)
{
  // 같은 호스트에서도 작은 음수 나이는 정상 범위다.
  EXPECT_EQ(
    evaluate_input(false, -5.0, kTimeoutMs, 0.0f, 0.0f), InputVerdict::kAccept);
}

TEST(EvaluateInput, RejectsFarFutureStamp)
{
  // 허용 폭(-timeout)을 넘는 미래 stamp는 발행자 클럭 불일치다.
  EXPECT_EQ(
    evaluate_input(false, -kTimeoutMs - 0.1, kTimeoutMs, 0.0f, 0.0f),
    InputVerdict::kRejectFutureStamp);
}

TEST(EvaluateInput, RejectsNonFiniteSpeed)
{
  EXPECT_EQ(
    evaluate_input(false, 5.0, kTimeoutMs, kNan, 0.0f),
    InputVerdict::kRejectNonFinite);
  EXPECT_EQ(
    evaluate_input(false, 5.0, kTimeoutMs, kInf, 0.0f),
    InputVerdict::kRejectNonFinite);
}

TEST(EvaluateInput, RejectsNonFiniteSteering)
{
  EXPECT_EQ(
    evaluate_input(false, 5.0, kTimeoutMs, 0.0f, kNan),
    InputVerdict::kRejectNonFinite);
  EXPECT_EQ(
    evaluate_input(false, 5.0, kTimeoutMs, 0.0f, -kInf),
    InputVerdict::kRejectNonFinite);
}

TEST(EvaluateInput, NonFiniteBeatsStale)
{
  // 판정 순서 고정: non-finite가 stale보다 먼저 보고된다.
  EXPECT_EQ(
    evaluate_input(false, kTimeoutMs + 100.0, kTimeoutMs, kNan, 0.0f),
    InputVerdict::kRejectNonFinite);
}

TEST(ToString, CoversEveryVerdict)
{
  EXPECT_STREQ(to_string(InputVerdict::kAccept), "ACCEPT");
  EXPECT_STREQ(to_string(InputVerdict::kRejectInvalidStamp), "REJECT_INVALID_STAMP");
  EXPECT_STREQ(to_string(InputVerdict::kRejectStale), "REJECT_STALE");
  EXPECT_STREQ(
    to_string(InputVerdict::kRejectFutureStamp), "REJECT_FUTURE_STAMP");
  EXPECT_STREQ(to_string(InputVerdict::kRejectNonFinite), "REJECT_NON_FINITE");
}

}  // namespace
