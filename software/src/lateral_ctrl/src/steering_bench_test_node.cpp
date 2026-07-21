#include "lateral_ctrl/bench_test_utils.hpp"

#include <rcl_interfaces/msg/set_parameters_result.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/u_int8.hpp>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <functional>
#include <mutex>
#include <vector>

namespace lateral_ctrl
{
class SteeringBenchTestNode : public rclcpp::Node
{
public:
  SteeringBenchTestNode()
  : Node("steering_bench_test_node")
  {
    drive_mode_publish_enabled_ = declare_parameter("drive_mode_publish_enabled", true);
    test_drive_mode_ = declare_parameter("test_drive_mode", 1);
    drive_mode_publish_period_ms_ = declare_parameter("drive_mode_publish_period_ms", 100);
    steer_sequence_publish_enabled_ = declare_parameter("steer_sequence_publish_enabled", true);
    sequence_ = declare_parameter("steer_sequence_deg", defaultBenchSequence());
    steer_command_interval_ms_ = declare_parameter("steer_command_interval_ms", 3000);
    sequence_repeat_ = declare_parameter("steer_sequence_repeat", false);
    sequence_start_delay_sec_ = declare_parameter("steer_sequence_start_delay_sec", 3.0);
    safety_injection_enabled_ = declare_parameter("safety_injection_enabled", true);
    test_estop_active_ = declare_parameter("test_estop_active", false);
    test_aeb_active_ = declare_parameter("test_aeb_active", false);
    max_abs_sequence_deg_ = declare_parameter("wheel_limit_deg", 30.0);
    if (!validSequence(sequence_)) {
      steer_sequence_publish_enabled_ = false;
      RCLCPP_ERROR(get_logger(), "Initial steer_sequence_deg is invalid; sequence disabled");
    }
    if (test_drive_mode_ != 0 && test_drive_mode_ != 1) {
      drive_mode_publish_enabled_ = false;
      RCLCPP_ERROR(get_logger(), "test_drive_mode must be 0 (MANUAL) or 1 (AUTO)");
    }
    if (drive_mode_publish_period_ms_ <= 0) {
      drive_mode_publish_enabled_ = false;
      RCLCPP_ERROR(get_logger(), "drive_mode_publish_period_ms must be greater than zero");
    }
    if (steer_command_interval_ms_ <= 0 || !std::isfinite(sequence_start_delay_sec_) ||
      sequence_start_delay_sec_ < 0.0)
    {
      steer_sequence_publish_enabled_ = false;
      RCLCPP_ERROR(get_logger(), "Invalid sequence interval/start delay; sequence disabled");
    }

    mode_pub_ = create_publisher<std_msgs::msg::UInt8>("/mode", 10);
    steer_pub_ = create_publisher<std_msgs::msg::Float32>("/cmd/steer", 10);
    estop_pub_ = create_publisher<std_msgs::msg::UInt8>("/steering/test/estop", 10);
    aeb_pub_ = create_publisher<std_msgs::msg::UInt8>("/steering/test/aeb", 10);
    sequence_enabled_at_ = std::chrono::steady_clock::now();
    last_mode_publish_ = last_sequence_publish_ = sequence_enabled_at_;

    parameter_callback_ = add_on_set_parameters_callback(
      std::bind(&SteeringBenchTestNode::onParameters, this, std::placeholders::_1));
    timer_ = create_wall_timer(
      std::chrono::milliseconds(50), std::bind(&SteeringBenchTestNode::tick, this));
    RCLCPP_WARN(
      get_logger(),
      "BENCH SAFETY: confirm the wheels are near mechanical center. This sequence uses ABSOLUTE "
      "wheel angles; the first 0 deg command can cause a large movement if steering is not centered.");
  }

  ~SteeringBenchTestNode() override
  {
    std_msgs::msg::UInt8 value;
    value.data = 0;
    if (mode_pub_) mode_pub_->publish(value);  // MANUAL
    if (estop_pub_) estop_pub_->publish(value);
    if (aeb_pub_) aeb_pub_->publish(value);
  }

private:
  rcl_interfaces::msg::SetParametersResult onParameters(
    const std::vector<rclcpp::Parameter> & parameters)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;
    for (const auto & parameter : parameters) {
      const auto & name = parameter.get_name();
      if (name == "steer_sequence_deg" && !validSequence(parameter.as_double_array())) {
        result.successful = false;
        result.reason = "steer_sequence_deg contains a non-finite or out-of-limit angle";
        return result;
      }
      if (name == "test_drive_mode" &&
        parameter.as_int() != 0 && parameter.as_int() != 1)
      {
        result.successful = false;
        result.reason = "test_drive_mode must be 0 or 1";
        return result;
      }
      if ((name == "drive_mode_publish_period_ms" ||
        name == "steer_command_interval_ms") && parameter.as_int() <= 0)
      {
        result.successful = false;
        result.reason = name + " must be greater than zero";
        return result;
      }
      if (name == "steer_sequence_start_delay_sec" &&
        (!std::isfinite(parameter.as_double()) || parameter.as_double() < 0.0))
      {
        result.successful = false;
        result.reason = "steer_sequence_start_delay_sec must be finite and non-negative";
        return result;
      }
    }
    for (const auto & parameter : parameters) {
      const auto & name = parameter.get_name();
      if (name == "drive_mode_publish_enabled") {
        drive_mode_publish_enabled_ = parameter.as_bool();
      } else if (name == "test_drive_mode") {
        test_drive_mode_ = parameter.as_int();
      } else if (name == "drive_mode_publish_period_ms") {
        drive_mode_publish_period_ms_ = parameter.as_int();
      } else if (name == "steer_sequence_publish_enabled") {
        const bool enabling = !steer_sequence_publish_enabled_ && parameter.as_bool();
        steer_sequence_publish_enabled_ = parameter.as_bool();
        if (enabling) resetSequence();
      } else if (name == "steer_sequence_deg") {
        sequence_ = parameter.as_double_array();
        resetSequence();
      } else if (name == "steer_command_interval_ms") {
        steer_command_interval_ms_ = parameter.as_int();
      } else if (name == "steer_sequence_repeat") {
        sequence_repeat_ = parameter.as_bool();
      } else if (name == "steer_sequence_start_delay_sec") {
        sequence_start_delay_sec_ = parameter.as_double();
      } else if (name == "safety_injection_enabled") {
        if (safety_injection_enabled_ && !parameter.as_bool()) clearTestSafety();
        safety_injection_enabled_ = parameter.as_bool();
      } else if (name == "test_estop_active") {
        test_estop_active_ = parameter.as_bool();
      } else if (name == "test_aeb_active") {
        test_aeb_active_ = parameter.as_bool();
      }
    }
    return result;
  }

  void clearTestSafety()
  {
    std_msgs::msg::UInt8 value;
    value.data = 0;
    estop_pub_->publish(value);
    aeb_pub_->publish(value);
    test_estop_active_ = false;
    test_aeb_active_ = false;
  }

  bool validSequence(const std::vector<double> & sequence) const
  {
    return std::all_of(sequence.begin(), sequence.end(), [this](double angle) {
      return std::isfinite(angle) && std::abs(angle) <= max_abs_sequence_deg_;
    });
  }

  void resetSequence()
  {
    sequence_index_ = 0;
    sequence_enabled_at_ = std::chrono::steady_clock::now();
    last_sequence_publish_ = sequence_enabled_at_;
  }

  void tick()
  {
    std::lock_guard<std::mutex> lock(mutex_);
    const auto now = std::chrono::steady_clock::now();
    if (drive_mode_publish_enabled_ &&
      std::chrono::duration_cast<std::chrono::milliseconds>(now - last_mode_publish_).count() >=
      drive_mode_publish_period_ms_)
    {
      std_msgs::msg::UInt8 mode;
      mode.data = static_cast<uint8_t>(test_drive_mode_);
      mode_pub_->publish(mode);
      last_mode_publish_ = now;
    }
    if (safety_injection_enabled_) {
      std_msgs::msg::UInt8 value;
      value.data = test_estop_active_ ? 1 : 0;
      estop_pub_->publish(value);
      value.data = test_aeb_active_ ? 1 : 0;
      aeb_pub_->publish(value);
    }
    if (!steer_sequence_publish_enabled_ || sequence_.empty()) return;
    if (std::chrono::duration<double>(now - sequence_enabled_at_).count() <
      sequence_start_delay_sec_) return;
    if (std::chrono::duration_cast<std::chrono::milliseconds>(
        now - last_sequence_publish_).count() < steer_command_interval_ms_) return;
    if (sequence_index_ >= sequence_.size()) {
      if (!sequence_repeat_) return;
      sequence_index_ = 0;
    }
    std_msgs::msg::Float32 command;
    command.data = static_cast<float>(sequence_[sequence_index_++]);
    steer_pub_->publish(command);
    last_sequence_publish_ = now;
  }

  std::mutex mutex_;
  bool drive_mode_publish_enabled_, steer_sequence_publish_enabled_;
  bool sequence_repeat_, safety_injection_enabled_, test_estop_active_, test_aeb_active_;
  int64_t test_drive_mode_, drive_mode_publish_period_ms_, steer_command_interval_ms_;
  double sequence_start_delay_sec_, max_abs_sequence_deg_;
  std::vector<double> sequence_;
  size_t sequence_index_{0};
  std::chrono::steady_clock::time_point sequence_enabled_at_;
  std::chrono::steady_clock::time_point last_mode_publish_, last_sequence_publish_;
  rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr mode_pub_, estop_pub_, aeb_pub_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr steer_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr parameter_callback_;
};
}  // namespace lateral_ctrl

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<lateral_ctrl::SteeringBenchTestNode>());
  rclcpp::shutdown();
  return 0;
}
