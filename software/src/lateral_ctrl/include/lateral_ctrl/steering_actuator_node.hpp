#pragma once

#include "lateral_ctrl/keya_can_driver.hpp"
#include "kaiev26_msgs/msg/actuator_command.hpp"
#include "kaiev26_msgs/msg/brake_actuator_status.hpp"
#include "kaiev26_msgs/msg/drive_actuator_status.hpp"
#include "kaiev26_msgs/msg/steering_actuator_status.hpp"
#include "lateral_ctrl/steering_angle_converter.hpp"
#include "lateral_ctrl/steering_types.hpp"
#include "lateral_ctrl/wheel_angle_tracking_controller.hpp"

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/int32.hpp>
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
  void onCommand(const kaiev26_msgs::msg::ActuatorCommand::SharedPtr msg);
  void onDriveStatus(const kaiev26_msgs::msg::DriveActuatorStatus::SharedPtr msg);
  void onBrakeStatus(const kaiev26_msgs::msg::BrakeActuatorStatus::SharedPtr msg);
  void setEstop(bool active, bool test_input);
  void applyMode(uint8_t mode_value);
  void requestImmediateDisable(SteeringDisableReason reason);
  void discardAllSteeringCommands();
  void controlTick();
  void transitionTo(SteeringState next);
  void enterWaitCommand(SteeringDisableReason reason = SteeringDisableReason::NONE);
  bool tryEnterActiveFromPendingCommand(std::chrono::steady_clock::time_point now);
  void publishState(double feedback_age_ms, double mode_age_ms, double steer_age_ms);
  // 버스를 열고 조회 프레임을 보내 상태를 발행하는 데까지가 canLinkEnabled의 범위다.
  // 모터를 실제로 움직이는 enable/disable/position 프레임은 outputArmed여야 나간다.
  // 벤치 관측은 hardware_can_enabled만 켜서 읽기 전용으로 돌린다.
  bool canLinkEnabled() const { return hardware_can_enabled_; }
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
  bool drive_speed_permits_steering_{false};
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
  bool steering_enabled_{true};
  bool test_input_injection_enabled_{false};
  double target_wheel_angle_deg_{0.0};
  double pending_target_wheel_angle_deg_{0.0};
  double commanded_wheel_angle_deg_{0.0};
  double corrected_wheel_angle_deg_{0.0};
  double actual_wheel_angle_deg_{0.0};
  double max_steering_rate_deg_s_{60.0};
  double minimum_steering_speed_mps_{1.0 / 3.6};
  int control_period_ms_{20};
  int can_timeout_ms_{500};
  int feedback_timeout_ms_{500};
  int mode_timeout_ms_{1500};
  int drive_status_timeout_ms_{500};
  int steer_command_timeout_ms_{200};
  uint64_t tick_count_{0};
  uint64_t steer_command_generation_{0};
  uint64_t pending_command_generation_{0};
  uint64_t minimum_accepted_command_generation_{1};
  std::chrono::steady_clock::time_point previous_tick_;
  std::chrono::steady_clock::time_point last_mode_time_{};
  std::chrono::steady_clock::time_point last_steer_command_time_{};

  rclcpp::Subscription<kaiev26_msgs::msg::ActuatorCommand>::SharedPtr command_sub_;
  rclcpp::Subscription<kaiev26_msgs::msg::DriveActuatorStatus>::SharedPtr drive_status_sub_;
  rclcpp::Subscription<kaiev26_msgs::msg::BrakeActuatorStatus>::SharedPtr brake_status_sub_;
  rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr test_estop_sub_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr angle_pub_;
  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr target_motor_count_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr enable_commanded_pub_, enabled_pub_;
  rclcpp::Publisher<kaiev26_msgs::msg::SteeringActuatorStatus>::SharedPtr status_pub_;
  rclcpp::TimerBase::SharedPtr control_timer_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr parameter_callback_;
};
}  // namespace lateral_ctrl
