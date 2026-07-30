#include "lateral_ctrl/bench_test_utils.hpp"
#include "lateral_ctrl/steering_types.hpp"

#include <gtest/gtest.h>

using lateral_ctrl::DriveMode;
using lateral_ctrl::SafetyInputs;
using lateral_ctrl::SteeringDisableReason;

TEST(SteeringSafetyLogic, ModeAndFeedbackGatesAuto)
{
  SafetyInputs safety;
  EXPECT_EQ(
    lateral_ctrl::selectDisableReason(false, DriveMode::UNKNOWN, safety, false, false, true),
    SteeringDisableReason::MODE_NOT_RECEIVED);
  EXPECT_EQ(
    lateral_ctrl::selectDisableReason(true, DriveMode::MANUAL, safety, false, false, true),
    SteeringDisableReason::MANUAL_MODE);
  EXPECT_EQ(
    lateral_ctrl::selectDisableReason(true, DriveMode::AUTO, safety, false, false, true),
    SteeringDisableReason::NONE);
  EXPECT_EQ(
    lateral_ctrl::selectDisableReason(true, DriveMode::AUTO, safety, false, false, false),
    SteeringDisableReason::FEEDBACK_INVALID);
}
TEST(SteeringSafetyLogic, RealAndTestEstopAreMonotonicOr)
{
  SafetyInputs safety;
  safety.real_estop = true;
  safety.test_estop = false;
  EXPECT_TRUE(safety.effectiveEstop());
}
TEST(SteeringSafetyLogic, RealOrTestEstopActivates)
{
  SafetyInputs safety;
  safety.test_estop = true;
  EXPECT_TRUE(safety.effectiveEstop());
  safety.test_estop = false;
  safety.real_estop = true;
  EXPECT_TRUE(safety.effectiveEstop());
}
TEST(SteeringSafetyLogic, CommandBrakeDisablesAutoSteering)
{
  SafetyInputs safety;
  safety.command_brake = true;
  EXPECT_EQ(
    lateral_ctrl::selectDisableReason(
      true, DriveMode::AUTO, safety, false, false, true),
    SteeringDisableReason::COMMAND_BRAKE);
}

TEST(SteeringSafetyLogic, CommandBrakeReleasesOnlyWithFreshStamp)
{
  EXPECT_FALSE(lateral_ctrl::nextCommandBrakeState(false, false, false));
  EXPECT_TRUE(lateral_ctrl::nextCommandBrakeState(true, false, false));
  EXPECT_FALSE(lateral_ctrl::nextCommandBrakeState(true, false, true));
  EXPECT_TRUE(lateral_ctrl::nextCommandBrakeState(false, true, false));
  EXPECT_TRUE(lateral_ctrl::nextCommandBrakeState(true, true, true));
}

TEST(SteeringSafetyLogic, TestEstopFalseCannotClearRealEstop)
{
  SafetyInputs safety;
  safety.real_estop = true;
  safety.test_estop = false;
  EXPECT_TRUE(safety.effectiveEstop());
}
TEST(SteeringSafetyLogic, AutoWithoutInhibitMapsToWaitCommand)
{
  SafetyInputs safety;
  const auto reason =
    lateral_ctrl::selectDisableReason(true, DriveMode::AUTO, safety, false, false, true);
  EXPECT_EQ(reason, SteeringDisableReason::NONE);
  EXPECT_EQ(
    lateral_ctrl::stateForDisableReason(reason), lateral_ctrl::SteeringState::WAIT_CMD);
}
TEST(SteeringSafetyLogic, InvalidModeDisables)
{
  SafetyInputs safety;
  EXPECT_EQ(
    lateral_ctrl::selectDisableReason(true, DriveMode::UNKNOWN, safety, false, false, true),
    SteeringDisableReason::INVALID_MODE);
}
TEST(SteeringSafetyLogic, InvalidFeedbackDisables)
{
  SafetyInputs safety;
  EXPECT_EQ(
    lateral_ctrl::selectDisableReason(true, DriveMode::AUTO, safety, false, false, false),
    SteeringDisableReason::FEEDBACK_INVALID);
}
TEST(SteeringSafetyLogic, FaultDisables)
{
  SafetyInputs safety;
  EXPECT_EQ(
    lateral_ctrl::selectDisableReason(true, DriveMode::AUTO, safety, true, false, true),
    SteeringDisableReason::MOTOR_FAULT);
}
TEST(SteeringSafetyLogic, CanTimeoutDisables)
{
  SafetyInputs safety;
  EXPECT_EQ(
    lateral_ctrl::selectDisableReason(true, DriveMode::AUTO, safety, false, true, true),
    SteeringDisableReason::CAN_TIMEOUT);
}
TEST(SteeringSafetyLogic, PriorityIsDeterministic)
{
  SafetyInputs safety;
  safety.real_estop = true;
  safety.command_brake = true;
  EXPECT_EQ(
    lateral_ctrl::selectDisableReason(true, DriveMode::AUTO, safety, true, true, false),
    SteeringDisableReason::ESTOP);
  safety.real_estop = false;
  EXPECT_EQ(
    lateral_ctrl::selectDisableReason(true, DriveMode::AUTO, safety, true, true, false),
    SteeringDisableReason::CAN_TIMEOUT);
}
TEST(SteeringSafetyLogic, BenchSequenceAlternatesFiveDegrees)
{
  EXPECT_EQ(
    lateral_ctrl::defaultBenchSequence(),
    (std::vector<double>{5.0, -5.0}));
}
TEST(SteeringSafetyLogic, RateLimiterBoundsMovement)
{
  EXPECT_DOUBLE_EQ(lateral_ctrl::rateLimit(0.0, 10.0, 5.0, 0.1), 0.5);
  EXPECT_DOUBLE_EQ(lateral_ctrl::rateLimit(0.5, -10.0, 5.0, 0.1), 0.0);
}
TEST(SteeringSafetyLogic, RecoveryDiscardsOldTarget)
{
  double target = 20.0;
  double commanded = 10.0;
  bool valid = true;
  lateral_ctrl::resetMotionTargetForRecovery(2.0, target, commanded, valid);
  EXPECT_DOUBLE_EQ(target, 2.0);
  EXPECT_DOUBLE_EQ(commanded, 2.0);
  EXPECT_FALSE(valid);
}
TEST(SteeringSafetyLogic, DisableReasonsMapToStates)
{
  EXPECT_EQ(
    lateral_ctrl::stateForDisableReason(SteeringDisableReason::NONE),
    lateral_ctrl::SteeringState::WAIT_CMD);
  EXPECT_EQ(
    lateral_ctrl::stateForDisableReason(SteeringDisableReason::ESTOP),
    lateral_ctrl::SteeringState::ESTOP);
  EXPECT_EQ(
    lateral_ctrl::stateForDisableReason(SteeringDisableReason::COMMAND_BRAKE),
    lateral_ctrl::SteeringState::BRAKE);
  EXPECT_EQ(
    lateral_ctrl::stateForDisableReason(SteeringDisableReason::CAN_TIMEOUT),
    lateral_ctrl::SteeringState::CAN_LOST);
  EXPECT_EQ(
    lateral_ctrl::stateForDisableReason(SteeringDisableReason::MOTOR_FAULT),
    lateral_ctrl::SteeringState::FAULT);
}

TEST(SteeringSafetyLogic, ActiveIsTheOnlyMotorEnabledState)
{
  EXPECT_TRUE(lateral_ctrl::shouldMotorBeEnabled(lateral_ctrl::SteeringState::ACTIVE));
  EXPECT_FALSE(lateral_ctrl::shouldMotorBeEnabled(lateral_ctrl::SteeringState::WAIT_CMD));
  EXPECT_FALSE(lateral_ctrl::shouldMotorBeEnabled(lateral_ctrl::SteeringState::DISABLED));
  EXPECT_FALSE(lateral_ctrl::shouldMotorBeEnabled(lateral_ctrl::SteeringState::ESTOP));
}

TEST(SteeringSafetyLogic, CommandMustBeReceivedAfterRecoveryBoundary)
{
  EXPECT_FALSE(lateral_ctrl::pendingCommandIsEligible(true, 4, 5, true));
  EXPECT_TRUE(lateral_ctrl::pendingCommandIsEligible(true, 5, 5, true));
  EXPECT_FALSE(lateral_ctrl::pendingCommandIsEligible(true, 5, 5, false));
  EXPECT_FALSE(lateral_ctrl::pendingCommandIsEligible(false, 5, 5, true));
}

TEST(SteeringSafetyLogic, ModeAndSteerTimeoutsMapSafely)
{
  SafetyInputs safety;
  EXPECT_EQ(
    lateral_ctrl::selectDisableReason(
      true, true, DriveMode::AUTO, safety, false, false, true, false),
    SteeringDisableReason::MODE_TIMEOUT);
  EXPECT_EQ(
    lateral_ctrl::selectDisableReason(
      true, false, DriveMode::AUTO, safety, false, false, true, true),
    SteeringDisableReason::STEER_COMMAND_TIMEOUT);
  EXPECT_EQ(
    lateral_ctrl::stateForDisableReason(SteeringDisableReason::STEER_COMMAND_TIMEOUT),
    lateral_ctrl::SteeringState::WAIT_CMD);
  EXPECT_EQ(
    lateral_ctrl::stateForDisableReason(SteeringDisableReason::MODE_TIMEOUT),
    lateral_ctrl::SteeringState::DISABLED);
  EXPECT_EQ(
    lateral_ctrl::stateForDisableReason(SteeringDisableReason::FEEDBACK_INVALID),
    lateral_ctrl::SteeringState::FEEDBACK_LOST);
}
