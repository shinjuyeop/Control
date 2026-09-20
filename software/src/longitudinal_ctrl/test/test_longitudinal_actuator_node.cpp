#include "longitudinal_ctrl/longitudinal_actuator_node.hpp"

#include <gtest/gtest.h>

#include <memory>
#include <string>
#include <vector>

namespace longitudinal_ctrl
{
class LongitudinalActuatorNodeTestPeer
{
public:
  static void cancelTimer(LongitudinalActuatorNode & n) {n.control_timer_->cancel();}
  static void tick(LongitudinalActuatorNode & n) {n.controlTick();}
  static void drive(LongitudinalActuatorNode & n, float speed = 4.0f, uint8_t mode = 1,
    bool stop = false, uint32_t fault = 0, float phase_current = 60.0f, uint8_t gear = 0)
  {
    auto msg = std::make_shared<kaiev26_msgs::msg::DriveActuatorStatus>();
    msg->mode = mode;
    msg->gear = gear;
    msg->part1_valid = true;
    msg->fault_bits_valid = true;
    msg->fault_bits = fault;
    msg->stop_request = stop;
    msg->speed_mps = speed;
    msg->phase_current_a = phase_current;
    msg->bus_current_a = 10.0f;
    n.onDriveStatus(msg);
  }
  static void command(LongitudinalActuatorNode & n, float speed = 4.0f, bool brake = false)
  {
    auto msg = std::make_shared<kaiev26_msgs::msg::ActuatorCommand>();
    msg->header.stamp = n.now();
    msg->speed_target_mps = speed;
    msg->brake_engage = brake;
    n.onCommand(msg);
  }
  static void estop(LongitudinalActuatorNode & n, bool active)
  {
    auto msg = std::make_shared<kaiev26_msgs::msg::BrakeActuatorStatus>();
    msg->stop_reason_bits = active ? kaiev26_msgs::msg::BrakeActuatorStatus::STOP_REASON_ESTOP : 0;
    n.onBrakeStatus(msg);
  }
  static std::string reason(const LongitudinalActuatorNode & n)
  {
    return n.blockingReason(std::chrono::steady_clock::now());
  }
  static bool active(const LongitudinalActuatorNode & n) {return n.controller_.active();}
  static float rpmLimit(const LongitudinalActuatorNode & n, double v) {return n.motorRpmLimit(v);}
};
}  // namespace longitudinal_ctrl

using namespace longitudinal_ctrl;
using Peer = LongitudinalActuatorNodeTestPeer;

namespace
{
class NodeFixture : public ::testing::Test
{
protected:
  static void SetUpTestSuite() {rclcpp::init(0, nullptr);}
  static void TearDownTestSuite() {rclcpp::shutdown();}

  std::shared_ptr<LongitudinalActuatorNode> make(bool enabled = true)
  {
    rclcpp::NodeOptions options;
    options.parameter_overrides({{"torque_control_enabled", enabled}});
    auto node = std::make_shared<LongitudinalActuatorNode>(options);
    Peer::cancelTimer(*node);
    listener_ = std::make_shared<rclcpp::Node>("listener");
    commands_.clear();
    sub_ = listener_->create_subscription<kaiev26_msgs::msg::DriveTorqueCommand>(
      "/drive/torque_command", 10,
      [this](kaiev26_msgs::msg::DriveTorqueCommand::SharedPtr msg) {commands_.push_back(*msg);});
    executor_.add_node(node);
    executor_.add_node(listener_);
    return node;
  }
  void settle(LongitudinalActuatorNode & node, int ticks = 5, float speed = 4.0f)
  {
    for (int i = 0; i < ticks; ++i) {
      Peer::drive(node, speed);
      Peer::command(node, speed);
      Peer::tick(node);
      executor_.spin_some();
    }
  }
  void spin()
  {
    for (int i = 0; i < 20 && commands_.empty(); ++i) {
      executor_.spin_some(std::chrono::milliseconds(10));
    }
  }

  rclcpp::executors::SingleThreadedExecutor executor_;
  std::shared_ptr<rclcpp::Node> listener_;
  rclcpp::Subscription<kaiev26_msgs::msg::DriveTorqueCommand>::SharedPtr sub_;
  std::vector<kaiev26_msgs::msg::DriveTorqueCommand> commands_;
};
}  // namespace

TEST_F(NodeFixture, DisabledByDefaultPublishesSpeedModeOnly)
{
  auto node = make(false);
  Peer::drive(*node);
  Peer::command(*node);
  EXPECT_EQ(Peer::reason(*node), "disabled_by_parameter");
  Peer::tick(*node);
  spin();
  ASSERT_FALSE(commands_.empty());
  EXPECT_FALSE(commands_.back().torque_mode);
  EXPECT_FLOAT_EQ(commands_.back().phase_current_target_a, 0.0f);
  EXPECT_FALSE(Peer::active(*node));
}

TEST_F(NodeFixture, BlockingReasonsFollowSafetyPriority)
{
  auto node = make(true);
  EXPECT_EQ(Peer::reason(*node), "drive_status_not_received");
  Peer::drive(*node, 4.0f, 0);
  EXPECT_EQ(Peer::reason(*node), "mode_not_auto");
  Peer::drive(*node, 4.0f, 1, true);
  EXPECT_EQ(Peer::reason(*node), "stop_request");
  Peer::drive(*node, 4.0f, 1, false, 0x100);
  EXPECT_EQ(Peer::reason(*node), "motor_fault");
  Peer::drive(*node, 4.0f, 1, false, 0, 60.0f, 1);
  EXPECT_EQ(Peer::reason(*node), "gear_reverse");
  Peer::drive(*node);
  EXPECT_EQ(Peer::reason(*node), "command_not_received");
  Peer::command(*node, 4.0f, true);
  EXPECT_EQ(Peer::reason(*node), "brake_engage");
  Peer::command(*node, -1.0f);
  EXPECT_EQ(Peer::reason(*node), "target_negative");
  Peer::command(*node, 4.0f);
  EXPECT_EQ(Peer::reason(*node), "");
  Peer::estop(*node, true);
  EXPECT_EQ(Peer::reason(*node), "estop");
  Peer::estop(*node, false);
  EXPECT_EQ(Peer::reason(*node), "");
}

TEST_F(NodeFixture, ActiveTorqueCommandStartsBumplessFromMeasuredCurrent)
{
  auto node = make(true);
  settle(*node, 30, 4.0f);
  EXPECT_TRUE(Peer::active(*node));
  spin();
  ASSERT_FALSE(commands_.empty());
  const auto & last = commands_.back();
  EXPECT_TRUE(last.torque_mode);
  // 실측 60 A에서 시작해 정속 유지 전류(≈ 45 + 5.6·4 = 67 A) 근처에 머문다.
  EXPECT_GT(last.phase_current_target_a, 40.0f);
  EXPECT_LT(last.phase_current_target_a, 90.0f);
  // 회전수 상한 = (4 + 2 m/s) → 약 1121 rpm
  EXPECT_NEAR(last.motor_rpm_limit, 6.0 * 60.0 / (3.14159265358979 * 0.47) * 4.6, 5.0);
}

TEST_F(NodeFixture, BrakeReleasesTorqueModeImmediately)
{
  auto node = make(true);
  settle(*node, 10, 4.0f);
  EXPECT_TRUE(Peer::active(*node));
  Peer::command(*node, 4.0f, true);
  Peer::tick(*node);
  EXPECT_FALSE(Peer::active(*node));
  commands_.clear();
  Peer::tick(*node);
  spin();
  ASSERT_FALSE(commands_.empty());
  EXPECT_FALSE(commands_.back().torque_mode);
  EXPECT_FLOAT_EQ(commands_.back().phase_current_target_a, 0.0f);
}

TEST_F(NodeFixture, LiveParameterToggleAndValidation)
{
  auto node = make(false);
  settle(*node, 3, 4.0f);
  EXPECT_FALSE(Peer::active(*node));
  auto result = node->set_parameter(rclcpp::Parameter("torque_control_enabled", true));
  EXPECT_TRUE(result.successful);
  settle(*node, 3, 4.0f);
  EXPECT_TRUE(Peer::active(*node));
  result = node->set_parameter(rclcpp::Parameter("kp_a_per_mps", -1.0));
  EXPECT_FALSE(result.successful);
  result = node->set_parameter(rclcpp::Parameter("max_regen_current_a", 0.0));
  EXPECT_TRUE(result.successful);
  result = node->set_parameter(rclcpp::Parameter("publish_period_ms", 40));
  EXPECT_FALSE(result.successful);  // startup-only
  EXPECT_FLOAT_EQ(Peer::rpmLimit(*node, 1.0), 3.0f * 60.0f / (3.14159265f * 0.47f) * 4.6f);
  result = node->set_parameter(rclcpp::Parameter("speed_limit_margin_mps", 0.0));
  EXPECT_TRUE(result.successful);
  EXPECT_FLOAT_EQ(Peer::rpmLimit(*node, 1.0), 0.0f);
}
