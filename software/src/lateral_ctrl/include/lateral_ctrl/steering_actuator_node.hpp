#pragma once

#include "lateral_ctrl/keya_can_driver.hpp"
#include "kaiev26_msgs/msg/steering_actuator_status.hpp"
#include "lateral_ctrl/steering_angle_converter.hpp"
#include "lateral_ctrl/steering_types.hpp"
#include "lateral_ctrl/wheel_angle_tracking_controller.hpp"

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/u_int8.hpp>

#include <memory>
#include <string>

namespace lateral_ctrl
{
class SteeringActuatorNode : public rclcpp::Node
{
public:
  explicit SteeringActuatorNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  ~SteeringActuatorNode() override;

private:
  void onSteer(const std_msgs::msg::Float32::SharedPtr msg);
  void onMode(const std_msgs::msg::UInt8::SharedPtr msg);
  void onEstop(const std_msgs::msg::UInt8::SharedPtr msg, bool test_input);
  void onAeb(const std_msgs::msg::UInt8::SharedPtr msg, bool test_input);
  void requestImmediateDisable(SteeringDisableReason reason);
  void discardAllSteeringCommands();
  void controlTick();
  void transitionTo(SteeringState next);
  void enterWaitCommand(SteeringDisableReason reason = SteeringDisableReason::NONE);
  bool tryEnterActiveFromPendingCommand(std::chrono::steady_clock::time_point now);
  void publishState(double feedback_age_ms, double mode_age_ms, double steer_age_ms);
  bool outputArmed() const { return hardware_can_enabled_ && actuator_output_enabled_; }

  std::unique_ptr<KeyaCanDriver> can_driver_;
  SteeringAngleConverter converter_;
  WheelAngleTrackingController tracking_controller_;
  DriveMode drive_mode_{DriveMode::UNKNOWN};
  SteeringState state_{SteeringState::INIT};
  SteeringDisableReason disable_reason_{SteeringDisableReason::MODE_NOT_RECEIVED};
  SteeringDisableReason requested_disable_reason_{SteeringDisableReason::NONE};
  SafetyInputs safety_;

  bool mode_received_{false};
  bool movement_target_valid_{false};
  bool pending_command_valid_{false};
  bool enabled_commanded_{false};
  bool can_timeout_{true};
  bool feedback_timeout_{true};
  bool mode_timeout_{false};
  bool steer_command_timeout_{false};
  bool tracking_enabled_{false};
  bool hardware_can_enabled_{false};
  bool actuator_output_enabled_{false};
  bool test_input_injection_enabled_{false};
  bool legacy_throttle_input_enabled_{false};
  double target_wheel_angle_deg_{0.0};
  double pending_target_wheel_angle_deg_{0.0};
  double commanded_wheel_angle_deg_{0.0};
  double corrected_wheel_angle_deg_{0.0};
  double actual_wheel_angle_deg_{0.0};
  double steering_rate_deg_s_{0.0};
  double max_steering_rate_deg_s_{60.0};
  double tracking_error_warning_deg_{1.0};
  double tracking_error_fault_deg_{3.0};
  int tracking_error_fault_duration_ms_{500};
  int control_period_ms_{20};
  int can_timeout_ms_{500};
  int feedback_timeout_ms_{500};
  int mode_timeout_ms_{1500};
  int steer_command_timeout_ms_{200};
  uint64_t tick_count_{0};
  uint64_t steer_command_generation_{0};
  uint64_t pending_command_generation_{0};
  uint64_t minimum_accepted_command_generation_{1};
  std::chrono::steady_clock::time_point previous_tick_;
  std::chrono::steady_clock::time_point last_mode_time_{};
  std::chrono::steady_clock::time_point last_steer_command_time_{};
  std::chrono::steady_clock::time_point tracking_fault_since_{};

  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr steer_sub_;
  rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr mode_sub_, estop_sub_, aeb_sub_;
  rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr test_estop_sub_, test_aeb_sub_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr angle_pub_, legacy_angle_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr enable_commanded_pub_, enabled_pub_;
  rclcpp::Publisher<kaiev26_msgs::msg::SteeringActuatorStatus>::SharedPtr status_pub_;
  rclcpp::TimerBase::SharedPtr control_timer_;
};
}  // namespace lateral_ctrl
