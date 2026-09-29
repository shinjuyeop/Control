#include "control_tools/bench_test_utils.hpp"

#include <kaiev26_msgs/msg/actuator_command.hpp>
#include <kaiev26_msgs/msg/drive_actuator_status.hpp>
#include <rcl_interfaces/msg/set_parameters_result.hpp>
#include <rclcpp/rclcpp.hpp>
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
    sequence_ = declare_parameter("steer_sequence_deg", control_tools::defaultBenchSequence());
    steer_command_publish_period_ms_ =
      declare_parameter("steer_command_publish_period_ms", 20);
    steer_target_change_interval_ms_ =
      declare_parameter("steer_target_change_interval_ms", 3000);
    sequence_repeat_ = declare_parameter("steer_sequence_repeat", true);
    sequence_start_delay_sec_ = declare_parameter("steer_sequence_start_delay_sec", 3.0);
    safety_injection_enabled_ = declare_parameter("safety_injection_enabled", true);
    test_estop_active_ = declare_parameter("test_estop_active", false);
    max_abs_sequence_deg_ = declare_parameter("max_abs_sequence_deg", 28.0);
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
    if (steer_command_publish_period_ms_ <= 0 || steer_target_change_interval_ms_ <= 0 ||
      !std::isfinite(sequence_start_delay_sec_) ||
      sequence_start_delay_sec_ < 0.0)
    {
      steer_sequence_publish_enabled_ = false;
      RCLCPP_ERROR(get_logger(), "Invalid sequence interval/start delay; sequence disabled");
    }

    steer_pub_ =
      create_publisher<kaiev26_msgs::msg::ActuatorCommand>("/vehicle/command", 10);
    if (drive_mode_publish_enabled_) ensureModePublisher();
    if (safety_injection_enabled_) ensureSafetyPublishers();
    sequence_enabled_at_ = std::chrono::steady_clock::now();
    last_mode_publish_ = last_safety_publish_ = sequence_enabled_at_;
    next_steer_command_publish_ = last_target_change_ = sequence_enabled_at_;

    parameter_callback_ = add_on_set_parameters_callback(
      std::bind(&SteeringBenchTestNode::onParameters, this, std::placeholders::_1));
    timer_ = create_wall_timer(
      std::chrono::milliseconds(5), std::bind(&SteeringBenchTestNode::tick, this));
    RCLCPP_WARN(
      get_logger(),
      "BENCH SAFETY: confirm the wheels are near mechanical center. This sequence uses ABSOLUTE "
      "wheel angles; the first 5 deg command can cause a large movement if steering is not centered.");
  }

  ~SteeringBenchTestNode() override
  {
    if (mode_pub_) publishDriveStatus(0);  // MANUAL
    std_msgs::msg::UInt8 value;
    value.data = 0;
    if (estop_pub_) estop_pub_->publish(value);
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
        name == "steer_command_publish_period_ms" ||
        name == "steer_target_change_interval_ms") && parameter.as_int() <= 0)
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
        const bool enabled = parameter.as_bool();
        if (enabled) ensureModePublisher();
        else mode_pub_.reset();
        drive_mode_publish_enabled_ = enabled;
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
      } else if (name == "steer_command_publish_period_ms") {
        steer_command_publish_period_ms_ = parameter.as_int();
      } else if (name == "steer_target_change_interval_ms") {
        steer_target_change_interval_ms_ = parameter.as_int();
      } else if (name == "steer_sequence_repeat") {
        sequence_repeat_ = parameter.as_bool();
      } else if (name == "steer_sequence_start_delay_sec") {
        sequence_start_delay_sec_ = parameter.as_double();
      } else if (name == "safety_injection_enabled") {
        const bool enabled = parameter.as_bool();
        if (enabled) {
          ensureSafetyPublishers();
        } else if (safety_injection_enabled_) {
          clearTestSafety();
          estop_pub_.reset();
        }
        safety_injection_enabled_ = enabled;
      } else if (name == "test_estop_active") {
        test_estop_active_ = parameter.as_bool();
      }
    }
    return result;
  }

  void ensureModePublisher()
  {
    // 벤치에서는 STM_A의 drive status 발행(물리 모드 스위치)을 대신한다.
    if (!mode_pub_) {
      mode_pub_ =
        create_publisher<kaiev26_msgs::msg::DriveActuatorStatus>("/drive/status", 10);
    }
  }

  void publishDriveStatus(uint8_t mode_value)
  {
    kaiev26_msgs::msg::DriveActuatorStatus status;
    status.header.stamp = now();
    status.mode = mode_value;
    status.gear = kaiev26_msgs::msg::DriveActuatorStatus::GEAR_FORWARD;
    status.tractive_feedback_valid = false;
    status.fault_bits_valid = false;
    status.command_received = false;
    mode_pub_->publish(status);
  }

  void ensureSafetyPublishers()
  {
    if (!estop_pub_) {
      estop_pub_ = create_publisher<std_msgs::msg::UInt8>("/steering/test/estop", 10);
    }
  }

  void clearTestSafety()
  {
    std_msgs::msg::UInt8 value;
    value.data = 0;
    if (estop_pub_) estop_pub_->publish(value);
    test_estop_active_ = false;
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
    current_target_wheel_angle_deg_ = 0.0;
    target_selected_ = false;
    sequence_complete_ = false;
    sequence_enabled_at_ = std::chrono::steady_clock::now();
    next_steer_command_publish_ = last_target_change_ = sequence_enabled_at_;
  }

  void tick()
  {
    std::lock_guard<std::mutex> lock(mutex_);
    const auto now = std::chrono::steady_clock::now();
    if (drive_mode_publish_enabled_ &&
      mode_pub_ &&
      std::chrono::duration_cast<std::chrono::milliseconds>(now - last_mode_publish_).count() >=
      drive_mode_publish_period_ms_)
    {
      publishDriveStatus(static_cast<uint8_t>(test_drive_mode_));
      last_mode_publish_ = now;
    }
    if (safety_injection_enabled_ &&
      estop_pub_ &&
      std::chrono::duration_cast<std::chrono::milliseconds>(
        now - last_safety_publish_).count() >= 50)
    {
      std_msgs::msg::UInt8 value;
      value.data = test_estop_active_ ? 1 : 0;
      estop_pub_->publish(value);
      last_safety_publish_ = now;
    }
    if (!steer_sequence_publish_enabled_ || sequence_.empty()) return;
    if (std::chrono::duration<double>(now - sequence_enabled_at_).count() <
      sequence_start_delay_sec_) return;
    if (!target_selected_) {
      sequence_index_ = 0;
      current_target_wheel_angle_deg_ = sequence_[sequence_index_];
      target_selected_ = true;
      last_target_change_ = now;
      next_steer_command_publish_ = now;
    } else if (std::chrono::duration_cast<std::chrono::milliseconds>(
        now - last_target_change_).count() >= steer_target_change_interval_ms_)
    {
      ++sequence_index_;
      if (sequence_index_ >= sequence_.size()) {
        if (sequence_repeat_) sequence_index_ = 0;
        else sequence_complete_ = true;
      }
      if (!sequence_complete_) current_target_wheel_angle_deg_ = sequence_[sequence_index_];
      last_target_change_ = now;
    }
    if (sequence_complete_) return;
    if (now < next_steer_command_publish_) return;
    // 벤치 시퀀스와 canonical 명령은 모두 중앙 등가각·좌회전 양수다.
    constexpr double kDegToCanonicalRad = 3.14159265358979323846 / 180.0;
    kaiev26_msgs::msg::ActuatorCommand command;
    command.header.stamp = this->now();
    command.steering_target_rad =
      static_cast<float>(current_target_wheel_angle_deg_ * kDegToCanonicalRad);
    command.speed_target_mps = 0.0F;
    command.brake_engage = false;
    steer_pub_->publish(command);
    do {
      next_steer_command_publish_ +=
        std::chrono::milliseconds(steer_command_publish_period_ms_);
    } while (next_steer_command_publish_ <= now);
  }

  std::mutex mutex_;
  bool drive_mode_publish_enabled_, steer_sequence_publish_enabled_;
  bool sequence_repeat_, safety_injection_enabled_, test_estop_active_;
  int64_t test_drive_mode_, drive_mode_publish_period_ms_;
  int64_t steer_command_publish_period_ms_, steer_target_change_interval_ms_;
  double sequence_start_delay_sec_, max_abs_sequence_deg_, current_target_wheel_angle_deg_{0.0};
  std::vector<double> sequence_;
  size_t sequence_index_{0};
  bool target_selected_{false}, sequence_complete_{false};
  std::chrono::steady_clock::time_point sequence_enabled_at_;
  std::chrono::steady_clock::time_point last_mode_publish_, last_safety_publish_;
  std::chrono::steady_clock::time_point next_steer_command_publish_, last_target_change_;
  rclcpp::Publisher<kaiev26_msgs::msg::DriveActuatorStatus>::SharedPtr mode_pub_;
  rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr estop_pub_;
  rclcpp::Publisher<kaiev26_msgs::msg::ActuatorCommand>::SharedPtr steer_pub_;
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
