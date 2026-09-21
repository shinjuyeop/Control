#include "lateral_ctrl/bench_test_utils.hpp"
#include "lateral_ctrl/steering_types.hpp"

#include <gtest/gtest.h>

#include <limits>

using lateral_ctrl::DriveMode;
using lateral_ctrl::SafetyInputs;
using lateral_ctrl::SteeringDisableReason;

namespace
{
// 노드는 항상 8인자 selectDisableReason을 호출한다. mode_timeout과
// steer_command_timeout을 따로 보지 않는 경우를 읽기 쉽게 쓰기 위한 test 전용
// helper이며, production header에는 축약 overload를 두지 않는다.
SteeringDisableReason reasonWithoutTimeouts(
  bool mode_received, DriveMode mode, const SafetyInputs & safety, bool motor_fault,
  bool can_timeout, bool feedback_valid)
{
  return lateral_ctrl::selectDisableReason(
    mode_received, false, mode, safety, motor_fault, can_timeout, feedback_valid, false);
}
}  // namespace

TEST(SteeringSafetyLogic, ModeAndFeedbackGatesAuto)
{
  SafetyInputs safety;
  EXPECT_EQ(
    reasonWithoutTimeouts(false, DriveMode::UNKNOWN, safety, false, false, true),
    SteeringDisableReason::MODE_NOT_RECEIVED);
  EXPECT_EQ(
    reasonWithoutTimeouts(true, DriveMode::MANUAL, safety, false, false, true),
    SteeringDisableReason::MANUAL_MODE);
  EXPECT_EQ(
    reasonWithoutTimeouts(true, DriveMode::AUTO, safety, false, false, true),
    SteeringDisableReason::NONE);
  EXPECT_EQ(
    reasonWithoutTimeouts(true, DriveMode::AUTO, safety, false, false, false),
    SteeringDisableReason::FEEDBACK_INVALID);
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
    reasonWithoutTimeouts(
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

TEST(SteeringSafetyLogic, DriveSpeedMustReachOneKmphInEitherDirection)
{
  constexpr float threshold = 1.0F / 3.6F;
  for (const float sign : {1.0F, -1.0F}) {
    EXPECT_FALSE(lateral_ctrl::driveSpeedPermitsSteering(0.0F, true, threshold));
    EXPECT_FALSE(lateral_ctrl::driveSpeedPermitsSteering(
        sign * std::nextafter(threshold, 0.0F), true, threshold));
    EXPECT_TRUE(lateral_ctrl::driveSpeedPermitsSteering(sign * threshold, true, threshold));
    EXPECT_TRUE(lateral_ctrl::driveSpeedPermitsSteering(sign * 2.0F, true, threshold));
  }
}

TEST(SteeringSafetyLogic, ZeroThresholdPermitsStandstillWithValidFeedback)
{
  EXPECT_TRUE(lateral_ctrl::driveSpeedPermitsSteering(0.0F, true, 0.0F));
}

TEST(SteeringSafetyLogic, InvalidDriveFeedbackNeverPermitsSteering)
{
  EXPECT_FALSE(lateral_ctrl::driveSpeedPermitsSteering(2.0F, false, 0.0F));
  EXPECT_FALSE(lateral_ctrl::driveSpeedPermitsSteering(
      std::numeric_limits<float>::quiet_NaN(), true, 0.0F));
  EXPECT_FALSE(lateral_ctrl::driveSpeedPermitsSteering(
      std::numeric_limits<float>::infinity(), true, 0.0F));
  EXPECT_FALSE(lateral_ctrl::driveSpeedPermitsSteering(
      -std::numeric_limits<float>::infinity(), true, 0.0F));
}

TEST(SteeringSafetyLogic, AutoWithoutInhibitMapsToWaitCommand)
{
  SafetyInputs safety;
  const auto reason =
    reasonWithoutTimeouts(true, DriveMode::AUTO, safety, false, false, true);
  EXPECT_EQ(reason, SteeringDisableReason::NONE);
  EXPECT_EQ(
    lateral_ctrl::stateForDisableReason(reason), lateral_ctrl::SteeringState::WAIT_CMD);
}
TEST(SteeringSafetyLogic, InvalidModeDisables)
{
  SafetyInputs safety;
  EXPECT_EQ(
    reasonWithoutTimeouts(true, DriveMode::UNKNOWN, safety, false, false, true),
    SteeringDisableReason::INVALID_MODE);
}
TEST(SteeringSafetyLogic, FaultDisables)
{
  SafetyInputs safety;
  EXPECT_EQ(
    reasonWithoutTimeouts(true, DriveMode::AUTO, safety, true, false, true),
    SteeringDisableReason::MOTOR_FAULT);
}
TEST(SteeringSafetyLogic, CanTimeoutDisables)
{
  SafetyInputs safety;
  EXPECT_EQ(
    reasonWithoutTimeouts(true, DriveMode::AUTO, safety, false, true, true),
    SteeringDisableReason::CAN_TIMEOUT);
}
TEST(SteeringSafetyLogic, PriorityIsDeterministic)
{
  SafetyInputs safety;
  safety.real_estop = true;
  safety.command_brake = true;
  EXPECT_EQ(
    reasonWithoutTimeouts(true, DriveMode::AUTO, safety, true, true, false),
    SteeringDisableReason::ESTOP);
  safety.real_estop = false;
  EXPECT_EQ(
    reasonWithoutTimeouts(true, DriveMode::AUTO, safety, true, true, false),
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
  EXPECT_DOUBLE_EQ(
    lateral_ctrl::rateLimit(0.5, 10.0, 5.0, -1.0), 0.5);
  EXPECT_THROW(
    lateral_ctrl::rateLimit(
      std::numeric_limits<double>::quiet_NaN(), 10.0, 5.0, 0.1),
    std::invalid_argument);
  EXPECT_THROW(
    lateral_ctrl::rateLimit(
      0.0, 10.0, std::numeric_limits<double>::quiet_NaN(), 0.1),
    std::invalid_argument);
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

#include "lateral_ctrl/keya_fault_policy.hpp"
#include "../../../../firmware/common/command_brake_order.h"

TEST(KeyaPolicy, EveryBitAndSimultaneousFaults)
{
  using namespace lateral_ctrl;
  const KeyaFaultClass expected[] = {
    KeyaFaultClass::INFO, KeyaFaultClass::RECOVERABLE, KeyaFaultClass::HARD,
    KeyaFaultClass::HARD, KeyaFaultClass::RECOVERABLE, KeyaFaultClass::INFO,
    KeyaFaultClass::RECOVERABLE, KeyaFaultClass::HARD, KeyaFaultClass::HARD,
    KeyaFaultClass::RECOVERABLE, KeyaFaultClass::RECOVERABLE, KeyaFaultClass::HARD,
    KeyaFaultClass::HARD, KeyaFaultClass::INFO, KeyaFaultClass::RECOVERABLE, KeyaFaultClass::INFO};
  for (unsigned bit = 0; bit < 16; ++bit) {
    EXPECT_EQ(keyaFaultClass(bit), expected[bit]) << bit;
    const KeyaFaults faults(uint16_t{1} << bit);
    EXPECT_EQ(faults.raw, uint16_t{1} << bit);
    EXPECT_EQ(faults.hard | faults.recoverable | faults.info, faults.raw);
    EXPECT_STRNE(keyaFaultDefinitions[bit].name, "");
  }
  const KeyaFaults all(0xffff);
  EXPECT_EQ(all.hard, 0x198c); EXPECT_EQ(all.recoverable, 0x4652); EXPECT_EQ(all.info, 0xa021);
}

TEST(KeyaPolicy, HardNeverRecoversIncludingHallAndMixedFaults)
{
  for (uint16_t bits : {0x0004, 0x0800, 0x4014}) {
    lateral_ctrl::KeyaRecoveryPolicy p;
    p.update(bits, true, 0); p.update(0, true, 100);
    EXPECT_TRUE(p.latched); EXPECT_FALSE(p.recovering); EXPECT_FALSE(p.probeDue(200));
  }
}

TEST(KeyaPolicy, RecoverableClearNeedsContinuousFreshEvidence)
{
  lateral_ctrl::KeyaRecoveryPolicy p;
  p.update(0x10, true, 0);
  EXPECT_TRUE(p.recovering); EXPECT_EQ(p.attempts, 1U);
  EXPECT_FALSE(p.update(0, true, 1));
  EXPECT_FALSE(p.update(0, false, 1.4));
  EXPECT_FALSE(p.update(0, true, 2));
  EXPECT_TRUE(p.update(0, true, 2.5));
  EXPECT_FALSE(p.recovering);
}

TEST(KeyaPolicy, RepeatedAndPersistentRecoverableFaultsNeverLatch)
{
  lateral_ctrl::KeyaRecoveryPolicy p;
  for (int i = 0; i < 1000; ++i) {
    p.update(0x4652, true, i * 2); p.update(0, true, i * 2 + 0.5);
    EXPECT_TRUE(p.update(0, true, i * 2 + 1));
    ASSERT_FALSE(p.latched);
    EXPECT_EQ(p.total_attempts, static_cast<uint64_t>(i + 1));
  }
  EXPECT_EQ(p.last_recovery_sec, 1999);
  lateral_ctrl::KeyaRecoveryPolicy persistent;
  persistent.update(0x400, true, 0);
  persistent.update(0x400, true, 86400);
  EXPECT_TRUE(persistent.recovering);
  EXPECT_FALSE(persistent.latched);
}

TEST(KeyaPolicy, InfiniteAttemptsHaveCappedBackoffAndNeverLatch)
{
  lateral_ctrl::KeyaRecoveryPolicy p;
  p.update(0x4000, true, 0);
  EXPECT_FALSE(p.probeDue(0.99)); EXPECT_TRUE(p.probeDue(1));
  p.recordAttempt(1);
  EXPECT_FALSE(p.probeDue(2.99)); EXPECT_TRUE(p.probeDue(3));
  p.recordAttempt(3);
  EXPECT_FALSE(p.probeDue(6.99)); EXPECT_TRUE(p.probeDue(7));
  double now = 7;
  for (int i = 0; i < 1000; ++i) {
    ASSERT_TRUE(p.probeDue(now)); p.recordAttempt(now);
    EXPECT_FALSE(p.probeDue(now + 4.99));
    EXPECT_FALSE(p.latched); now += 5;
  }
  EXPECT_EQ(p.attempts, 1003U);
  EXPECT_EQ(p.total_attempts, 1003U);
}

TEST(BrakeRelease, BothMcuConsumersRejectPreBrakeCommandsWithoutClockSync)
{
  for (int board = 0; board < 2; ++board) {
    int64_t barrier = 0;
    EXPECT_TRUE(CommandBrakeOrder_Accept(&barrier, 0, 10, 0));
    EXPECT_TRUE(CommandBrakeOrder_Accept(&barrier, 1, 11, 0));
    EXPECT_FALSE(CommandBrakeOrder_Accept(&barrier, 0, 10, 900000000));
    EXPECT_FALSE(CommandBrakeOrder_Accept(&barrier, 0, 11, 0));
    EXPECT_FALSE(CommandBrakeOrder_Accept(&barrier, 0, 0, 0));
    EXPECT_TRUE(CommandBrakeOrder_Accept(&barrier, 0, 11, 1));
    EXPECT_TRUE(CommandBrakeOrder_Accept(&barrier, 1, 0, 0)); // stop always wins
  }
}

TEST(KeyaPolicy, LostCommunicationDoesNotPromoteCachedFaultToPersistentLatch)
{
  lateral_ctrl::KeyaRecoveryPolicy p;
  p.update(0x10, true, 0); p.update(0x10, false, 100);
  EXPECT_FALSE(p.latched);
  p.update(0, true, 101);
  EXPECT_TRUE(p.update(0, true, 102));
}

TEST(KeyaPolicy, CanDisconnectMayWaitForAutoCommandWithoutPersistenceLatch)
{
  lateral_ctrl::KeyaRecoveryPolicy p;
  p.update(0x4000, true, 0); p.update(0x4000, true, 300);
  EXPECT_FALSE(p.latched); EXPECT_TRUE(p.recovering);
  for (int i = 0; i < 10; ++i) p.recordAttempt(301 + i * 5);
  EXPECT_FALSE(p.latched);
  EXPECT_EQ(p.attempts, 11U);
}
