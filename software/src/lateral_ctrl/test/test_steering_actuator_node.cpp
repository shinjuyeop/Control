#include "lateral_ctrl/steering_actuator_node.hpp"

#include <gtest/gtest.h>
#include <limits>
#include <utility>

namespace lateral_ctrl
{
class RecordingKeyaDriver : public KeyaCanDriver
{
public:
  RecordingKeyaDriver() : KeyaCanDriver("unused-test-interface", 1) {}
  bool open() override { opened = true; return true; }
  bool isOpen() const override { return opened; }
  bool sendEnable() override { frames.emplace_back("enable", 0); return !fail_enable; }
  bool sendDisable() override { frames.emplace_back("disable", 0); return !fail_disable; }
  bool sendPositionCount(int32_t count) override { frames.emplace_back("position", count); return true; }
  bool sendTorqueCommand(int32_t value) override {
    frames.emplace_back("torque", value);
    return !fail_torque;
  }
  bool requestEncoderPosition() override { return true; }
  void receiveAvailableFrames() override {}
  const KeyaMotorState & motorState() const override { return motor; }
  void fresh(int32_t count = 0) {
    motor.motor_position_count = count;
    motor.position_valid = motor.communication_ok = true;
    motor.last_feedback = motor.last_position_feedback = std::chrono::steady_clock::now();
  }
  KeyaMotorState motor;
  bool opened{false}, fail_enable{false}, fail_torque{false}, fail_disable{false};
  using Frames = std::vector<std::pair<std::string, int32_t>>;
  std::shared_ptr<Frames> trace{std::make_shared<Frames>()};
  Frames & frames{*trace};
};

class SteeringActuatorNodeTestPeer
{
public:
  static void cancelTimer(SteeringActuatorNode & n) { n.control_timer_->cancel(); }
  static void tick(SteeringActuatorNode & n) { n.controlTick(); }
  static void drive(SteeringActuatorNode & n, uint8_t mode = 1) {
    auto msg = std::make_shared<kaiev26_msgs::msg::DriveActuatorStatus>();
    msg->mode = mode;
    msg->part1_valid = true;
    msg->speed_mps = 1.0;
    n.onDriveStatus(msg);
  }
  static void command(SteeringActuatorNode & n, float rad = 0.0174532925F,
    bool brake = false, double age_ms = 0.0)
  {
    auto msg = std::make_shared<kaiev26_msgs::msg::ActuatorCommand>();
    msg->header.stamp = n.now() - rclcpp::Duration::from_seconds(age_ms / 1000.0);
    msg->steering_target_rad = rad;
    msg->brake_engage = brake;
    n.onCommand(msg);
  }
  static void estop(SteeringActuatorNode & n) { n.setEstop(true, false); }
  static SteeringState state(const SteeringActuatorNode & n) { return n.state_; }
  static bool enabled(const SteeringActuatorNode & n) { return n.enabled_commanded_; }
  static bool pending(const SteeringActuatorNode & n) { return n.pending_command_valid_; }
  static TorqueSafetyFault fault(const SteeringActuatorNode & n) { return n.torque_fault_latched_; }
  static void expireCommand(SteeringActuatorNode & n) {
    n.last_steer_command_time_ = std::chrono::steady_clock::now() - std::chrono::seconds(1);
  }
  static void expireStampDeadline(SteeringActuatorNode & n) {
    n.torque_command_deadline_ = std::chrono::steady_clock::now() - std::chrono::milliseconds(1);
  }
  static void delayTick(SteeringActuatorNode & n) {
    n.previous_tick_ = std::chrono::steady_clock::now() - std::chrono::milliseconds(150);
  }
  static double kd(const SteeringActuatorNode & n) { return n.torque_controller_.config().kd; }
  static double currentLimitCommandA(const SteeringActuatorNode & n) {
    return n.torqueCurrentLimitCommandA();
  }
};
}  // namespace lateral_ctrl

using namespace lateral_ctrl;
using Peer = SteeringActuatorNodeTestPeer;

class SteeringNodeTest : public ::testing::Test
{
protected:
  static void SetUpTestSuite() {
    const char * args[] = {"steering_node_test", "--ros-args", "--log-level", "fatal"};
    rclcpp::init(4, args);
  }
  static void TearDownTestSuite() { rclcpp::shutdown(); }
  void create(const std::string & mode = "torque", bool armed = true, int sign = 1,
    std::vector<rclcpp::Parameter> extra = {}, int32_t initial_count = 0)
  {
    rclcpp::NodeOptions options;
    std::vector<rclcpp::Parameter> parameters{
      {"count_scale", 3500.0}, {"equivalent_k1_deg", 45.36712598527852},
      {"equivalent_k2_deg", -3.534573536433758}, {"equivalent_k3_deg", -44.4281013734926},
      {"equivalent_k4_deg", 2.7876017898037944}, {"equivalent_k5_deg", 28.96220619772027},
      {"command_count_limit", 3279}, {"counts_per_rev", 10000.0},
      {"actuation_mode", mode}, {"hardware_can_enabled", true},
      {"actuator_output_enabled", armed}, {"torque_direction_sign", sign}};
    parameters.insert(parameters.end(), extra.begin(), extra.end());
    options.parameter_overrides(parameters);
    auto driver = std::make_unique<RecordingKeyaDriver>();
    can = driver.get();
    node = std::make_unique<SteeringActuatorNode>(options, std::move(driver));
    Peer::cancelTimer(*node);
    can->fresh(initial_count);
    Peer::drive(*node);
    Peer::tick(*node);
    can->frames.clear();
  }
  void activate() {
    Peer::command(*node);
    can->fresh();
    Peer::tick(*node);
    ASSERT_EQ(Peer::state(*node), SteeringState::ACTIVE);
  }
  void output() {
    can->fresh();
    Peer::tick(*node);
    ASSERT_TRUE(Peer::enabled(*node));
  }
  void expectStop() {
    ASSERT_GE(can->frames.size(), 2U);
    const auto n = can->frames.size();
    EXPECT_EQ(can->frames[n - 2], std::make_pair(std::string("torque"), int32_t{0}));
    EXPECT_EQ(can->frames[n - 1], std::make_pair(std::string("disable"), int32_t{0}));
    EXPECT_FALSE(Peer::enabled(*node));
    EXPECT_FALSE(Peer::pending(*node));
  }
  RecordingKeyaDriver * can{};
  std::unique_ptr<SteeringActuatorNode> node;
};

TEST_F(SteeringNodeTest, ActiveEntryZeroBeforeEnableThenFreshTickProducesTorque)
{
  create();
  activate();
  ASSERT_EQ(can->frames.size(), 2U);
  EXPECT_EQ(can->frames[0], std::make_pair(std::string("torque"), int32_t{0}));
  EXPECT_EQ(can->frames[1].first, "enable");
  can->frames.clear();
  Peer::tick(*node);  // no new position sample -> no torque
  EXPECT_EQ(can->frames.back().second, 0);
  output();
  EXPECT_GT(can->frames.back().second, 0);
  for (const auto & frame : can->frames) EXPECT_NE(frame.first, "position");
}

TEST_F(SteeringNodeTest, CurrentLimitCommandUsesWireSignAndConfiguredProtectionCurrent)
{
  create("torque", true, -1, {{"keya_protection_current_a", 20.0}});
  activate();
  output();
  ASSERT_EQ(can->frames.back(), std::make_pair(std::string("torque"), int32_t{-2000}));
  EXPECT_DOUBLE_EQ(Peer::currentLimitCommandA(*node), -3.2);
}

TEST_F(SteeringNodeTest, ShutdownSendsZeroThenDisable)
{
  create(); activate(); output();
  const auto trace = can->trace;
  node.reset();
  ASSERT_GE(trace->size(), 2U);
  EXPECT_EQ((*trace)[trace->size() - 2], std::make_pair(std::string("torque"), int32_t{0}));
  EXPECT_EQ(trace->back().first, "disable");
}

TEST_F(SteeringNodeTest, StartupRejectsInvalidModeSignAndWideInteger)
{
  EXPECT_THROW(create("invalid"), std::invalid_argument);
  EXPECT_THROW(create("torque", true, 0), std::invalid_argument);
  EXPECT_THROW(create("torque", true, 1,
    {{"torque_peak_limit_raw", int64_t{4294969296}}}), std::invalid_argument);
  EXPECT_THROW(create("torque", true, 1,
    {{"torque_control_period_ms", int64_t{4294967316}}}), std::invalid_argument);
}

TEST_F(SteeringNodeTest, TorqueStepBypassesPositionRateShaper)
{
  create(); activate(); output();
  EXPECT_EQ(can->frames.back().second, 2000);
  Peer::command(*node, -0.0174532925F);
  output();
  EXPECT_EQ(can->frames.back().second, -2000);
}

TEST_F(SteeringNodeTest, RawSignIsAppliedAtTheNodeBoundary)
{
  create("torque", true, -1);
  activate();
  output();
  EXPECT_LT(can->frames.back().second, 0);
}

TEST_F(SteeringNodeTest, EstopStopsImmediatelyAndDropsOldTarget)
{
  create(); activate(); output();
  Peer::estop(*node);
  expectStop();
  Peer::tick(*node);
  EXPECT_EQ(Peer::state(*node), SteeringState::ESTOP);
}

TEST_F(SteeringNodeTest, ManualStopsImmediately)
{
  create(); activate(); output();
  Peer::drive(*node, 0);
  expectStop();
}

TEST_F(SteeringNodeTest, ManualSteeringOutsideAutoEnvelopeDoesNotLatchAndCanReturnToAuto)
{
  create(); activate(); output();
  Peer::drive(*node, 0);
  expectStop();
  can->frames.clear();
  // Recorded MANUAL return turn: the motor was already disabled before the
  // encoder crossed -3279. Returning to center must not leave a new AUTO latch.
  for (const int32_t count : {-3247, -3292, -3398, 0}) {
    can->fresh(count); Peer::tick(*node);
    EXPECT_EQ(Peer::fault(*node), TorqueSafetyFault::NONE);
    EXPECT_EQ(Peer::state(*node), SteeringState::DISABLED);
    EXPECT_FALSE(Peer::enabled(*node));
  }
  for (const auto & frame : can->frames) {
    EXPECT_NE(frame.first, "enable");
    if (frame.first == "torque") {
      EXPECT_EQ(frame.second, 0);
    }
  }
  Peer::drive(*node);
  can->fresh(); Peer::tick(*node);
  EXPECT_EQ(Peer::state(*node), SteeringState::WAIT_CMD);
  EXPECT_FALSE(Peer::enabled(*node));
  activate();
  output();
  EXPECT_GT(can->frames.back().second, 0);
}

TEST_F(SteeringNodeTest, AutoEntryOutsideCalibrationAllowsInwardRecovery)
{
  for (const int32_t count : {-3299, 3299}) {
    node.reset();
    create();
    Peer::drive(*node, 0);
    can->fresh(count); Peer::tick(*node);
    ASSERT_EQ(Peer::fault(*node), TorqueSafetyFault::NONE);
    Peer::drive(*node);
    Peer::command(*node, 0.0F);
    can->fresh(count); Peer::tick(*node);
    ASSERT_EQ(Peer::state(*node), SteeringState::WAIT_CMD);
    can->fresh(count); Peer::tick(*node);
    ASSERT_EQ(Peer::state(*node), SteeringState::ACTIVE);
    can->fresh(count); Peer::tick(*node);
    EXPECT_EQ(Peer::fault(*node), TorqueSafetyFault::NONE);
    EXPECT_TRUE(Peer::enabled(*node));
    ASSERT_EQ(can->frames.back().first, "torque");
    EXPECT_LT(can->frames.back().second * count, 0);
  }
}

TEST_F(SteeringNodeTest, ManualRoundTripDoesNotClearAnAutoSafetyFault)
{
  create(); activate(); output();
  can->fresh(300); Peer::tick(*node);
  ASSERT_EQ(Peer::fault(*node), TorqueSafetyFault::RATE_LIMIT);
  Peer::drive(*node, 0);
  can->fresh(); Peer::tick(*node);
  EXPECT_EQ(Peer::fault(*node), TorqueSafetyFault::RATE_LIMIT);
  Peer::drive(*node);
  Peer::command(*node); can->fresh(); Peer::tick(*node);
  EXPECT_EQ(Peer::state(*node), SteeringState::FAULT);
  EXPECT_FALSE(Peer::enabled(*node));
}

TEST_F(SteeringNodeTest, BrakeStopsImmediately)
{
  create(); activate(); output();
  Peer::command(*node, 0.0F, true);
  // The brake command may be pending until the next safety tick, but cannot execute.
  Peer::tick(*node);
  expectStop();
  EXPECT_EQ(Peer::state(*node), SteeringState::BRAKE);
}

TEST_F(SteeringNodeTest, SteeringEnabledStopsAndNeedsNewCommand)
{
  create(); activate(); output();
  ASSERT_TRUE(node->set_parameter({"steering_enabled", false}).successful);
  expectStop();
  ASSERT_TRUE(node->set_parameter({"steering_enabled", true}).successful);
  can->fresh(); Peer::tick(*node);
  EXPECT_EQ(Peer::state(*node), SteeringState::WAIT_CMD);
  EXPECT_FALSE(Peer::enabled(*node));
}

TEST_F(SteeringNodeTest, CommandTimeoutStopsAndDoesNotReuseTarget)
{
  create(); activate(); output();
  Peer::expireCommand(*node);
  can->fresh(); Peer::tick(*node);
  expectStop();
  EXPECT_EQ(Peer::state(*node), SteeringState::WAIT_CMD);
  can->fresh(); Peer::tick(*node);
  EXPECT_FALSE(Peer::enabled(*node));
}

TEST_F(SteeringNodeTest, OriginalCommandStampExpiryIsNotExtendedByReceipt)
{
  create(); activate(); output();
  Peer::expireStampDeadline(*node);
  can->fresh(); Peer::tick(*node);
  expectStop();
}

TEST_F(SteeringNodeTest, StaleFutureAndNonFiniteCommandsStopImmediately)
{
  for (int input = 0; input < 3; ++input) {
    node.reset();
    create(); activate(); output();
    if (input == 0) Peer::command(*node, 0.0F, false, 250.0);
    if (input == 1) Peer::command(*node, 0.0F, false, -1000.0);
    if (input == 2) Peer::command(*node, std::numeric_limits<float>::quiet_NaN());
    expectStop();
  }
}

TEST_F(SteeringNodeTest, FeedbackTimeoutDespiteLiveCanStops)
{
  create(); activate(); output();
  can->motor.last_position_feedback -= std::chrono::milliseconds(110);
  can->motor.last_feedback = std::chrono::steady_clock::now();
  Peer::tick(*node);
  expectStop();
  EXPECT_EQ(Peer::state(*node), SteeringState::FEEDBACK_LOST);
}

TEST_F(SteeringNodeTest, InvalidPositionStops)
{
  create(); activate(); output();
  can->motor.position_valid = false;
  Peer::tick(*node);
  expectStop();
}

TEST_F(SteeringNodeTest, CanTimeoutStops)
{
  create(); activate(); output();
  can->motor.last_feedback -= std::chrono::seconds(1);
  Peer::tick(*node);
  expectStop();
  EXPECT_EQ(Peer::state(*node), SteeringState::CAN_LOST);
}

TEST_F(SteeringNodeTest, MotorFaultStops)
{
  create(); activate(); output();
  can->motor.fault_bits = 4;
  can->fresh(); Peer::tick(*node);
  expectStop();
  EXPECT_EQ(Peer::state(*node), SteeringState::FAULT);
}

TEST_F(SteeringNodeTest, OverspeedLatchClearsThroughExplicitDisable)
{
  create(); activate(); output();
  can->fresh(300); Peer::tick(*node);
  expectStop();
  ASSERT_EQ(Peer::fault(*node), TorqueSafetyFault::RATE_LIMIT);
  can->fresh(); Peer::command(*node); Peer::tick(*node);
  EXPECT_EQ(Peer::state(*node), SteeringState::FAULT);
  EXPECT_FALSE(Peer::enabled(*node));
  ASSERT_TRUE(node->set_parameter({"steering_enabled", false}).successful);
  ASSERT_TRUE(node->set_parameter({"steering_enabled", true}).successful);
  can->fresh(); Peer::tick(*node);
  EXPECT_EQ(Peer::state(*node), SteeringState::WAIT_CMD);
}

TEST_F(SteeringNodeTest, OverspeedStops)
{
  create(); activate(); output();
  can->fresh(300); Peer::tick(*node);
  expectStop();
  EXPECT_EQ(Peer::state(*node), SteeringState::FAULT);
}

TEST_F(SteeringNodeTest, OverspeedLatchClearsOnRestartAndRequiresFreshCommand)
{
  create(); activate(); output();
  can->fresh(300); Peer::tick(*node);
  expectStop();
  ASSERT_EQ(Peer::fault(*node), TorqueSafetyFault::RATE_LIMIT);

  // Stopping the wheel and sending new targets cannot clear this process's latch.
  for (int i = 0; i < 3; ++i) {
    can->fresh(300); Peer::command(*node); Peer::tick(*node);
    ASSERT_EQ(Peer::state(*node), SteeringState::FAULT);
    ASSERT_FALSE(Peer::enabled(*node));
  }

  // vehicle.launch.py starts a new node; the wheel remains at its stopped angle.
  node.reset();
  create("torque", true, 1, {}, 300);
  EXPECT_EQ(Peer::fault(*node), TorqueSafetyFault::NONE);
  EXPECT_EQ(Peer::state(*node), SteeringState::WAIT_CMD);
  EXPECT_FALSE(Peer::pending(*node));
  EXPECT_FALSE(Peer::enabled(*node));
  can->fresh(300); Peer::tick(*node);
  EXPECT_FALSE(Peer::enabled(*node));

  Peer::command(*node, 0.0F);
  can->fresh(300); Peer::tick(*node);
  ASSERT_EQ(Peer::state(*node), SteeringState::ACTIVE);
  ASSERT_EQ(can->frames.size(), 2U);
  EXPECT_EQ(can->frames[0], std::make_pair(std::string("torque"), int32_t{0}));
  EXPECT_EQ(can->frames[1].first, "enable");
  can->fresh(300); Peer::tick(*node);
  EXPECT_EQ(can->frames.back().first, "torque");
  EXPECT_LT(can->frames.back().second, 0);

  // A new overspeed still trips the unchanged safety limit after restart.
  can->fresh(600); Peer::tick(*node);
  expectStop();
  EXPECT_EQ(Peer::state(*node), SteeringState::FAULT);
  EXPECT_EQ(Peer::fault(*node), TorqueSafetyFault::RATE_LIMIT);
}

TEST_F(SteeringNodeTest, ControlStallStopsEvenWithFreshFeedback)
{
  create(); activate(); output();
  Peer::delayTick(*node);
  can->fresh(); Peer::tick(*node);
  expectStop();
}

TEST_F(SteeringNodeTest, FailedTorqueWriteStopsAndDisableIsAttempted)
{
  create(); activate(); output();
  can->fail_torque = true;
  can->fresh(); Peer::tick(*node);
  expectStop();
  Peer::command(*node);
  can->fresh(); Peer::tick(*node);
  EXPECT_FALSE(Peer::enabled(*node));
}

TEST_F(SteeringNodeTest, FailedEnableIsFollowedByZeroAndDisable)
{
  create();
  can->fail_enable = true;
  Peer::command(*node); can->fresh(); Peer::tick(*node);
  expectStop();
}

TEST_F(SteeringNodeTest, ReadOnlyNeverSendsActuation)
{
  create("torque", false);
  Peer::command(*node); can->fresh(); Peer::tick(*node);
  Peer::estop(*node);
  node->set_parameter({"steering_enabled", false});
  EXPECT_TRUE(can->frames.empty());
}

TEST_F(SteeringNodeTest, RuntimeTuningIsValidatedAtomicallyBeforeStopping)
{
  create(); activate(); output();
  can->frames.clear();
  const auto rejected = node->set_parameters_atomically({
    {"torque_kd", 2000.0}, {"torque_peak_limit_raw", 500}});
  EXPECT_FALSE(rejected.successful);
  EXPECT_DOUBLE_EQ(Peer::kd(*node), 1500.0);
  EXPECT_TRUE(can->frames.empty());
  EXPECT_TRUE(Peer::enabled(*node));
  EXPECT_FALSE(node->set_parameter({"actuation_mode", "position"}).successful);
  EXPECT_FALSE(node->set_parameter({"torque_direction_sign", -1}).successful);
  EXPECT_FALSE(node->set_parameter({"torque_kd", -1.0}).successful);
  EXPECT_FALSE(node->set_parameter({"torque_peak_limit_raw", int64_t{4294969296}}).successful);
  EXPECT_TRUE(node->set_parameter({"torque_kd", 2000.0}).successful);
  EXPECT_DOUBLE_EQ(Peer::kd(*node), 2000.0);
  expectStop();
}

TEST_F(SteeringNodeTest, PositionModeRetainsCurrentCountBeforeEnableAndRateShaper)
{
  create("position");
  activate();
  ASSERT_GE(can->frames.size(), 2U);
  EXPECT_EQ(can->frames[0], std::make_pair(std::string("position"), int32_t{0}));
  EXPECT_EQ(can->frames[1].first, "enable");
  output();
  for (const auto & frame : can->frames) EXPECT_NE(frame.first, "torque");
  EXPECT_LT(can->frames.back().second, 77); // 1 deg would be about 77 counts; tiny test dt is shaped.
  Peer::estop(*node);
  EXPECT_EQ(can->frames.back().first, "disable");
}
