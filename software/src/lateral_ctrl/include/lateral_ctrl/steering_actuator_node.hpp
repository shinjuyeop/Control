#pragma once

#include "lateral_ctrl/keya_fault_policy.hpp"
#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <std_srvs/srv/trigger.hpp>

#include "lateral_ctrl/keya_can_driver.hpp"
#include "kaiev26_msgs/msg/actuator_command.hpp"
#include "kaiev26_msgs/msg/brake_actuator_status.hpp"
#include "kaiev26_msgs/msg/drive_actuator_status.hpp"
#include "kaiev26_msgs/msg/steering_actuator_status.hpp"
#include "lateral_ctrl/steering_angle_converter.hpp"
#include "lateral_ctrl/steering_types.hpp"
#include "lateral_ctrl/wheel_angle_tracking_controller.hpp"
#include "lateral_ctrl/torque_steering_controller.hpp"

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/int32.hpp>
#include <std_msgs/msg/u_int8.hpp>
#include <std_msgs/msg/string.hpp>

#include <memory>
#include "steering_mpc/steering_mpc.hpp"
#include <string>

namespace lateral_ctrl
{
class SteeringActuatorNode : public rclcpp::Node
{
public:
  explicit SteeringActuatorNode(
    const rclcpp::NodeOptions & options = rclcpp::NodeOptions(),
    std::unique_ptr<KeyaCanDriver> driver = nullptr);
  ~SteeringActuatorNode() override;

private:
  friend class SteeringActuatorNodeTestPeer;
  void stopMotor();
  void reconnectCan(std::chrono::steady_clock::time_point now);
  void publishSafety(double can_age, double position_age, double command_age, double fault_age);
  std::string faultLatchClearRejection(std::chrono::steady_clock::time_point now) const;
  void clearFaultLatch(std_srvs::srv::Trigger::Response & response);
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr clear_fault_service_;
  KeyaRecoveryPolicy recovery_;
  double drive_speed_mps_{0.0};
  bool drive_speed_valid_{false};
  std::chrono::steady_clock::time_point last_brake_status_time_{};
  double last_recovery_time_sec_{0.0};
  bool recovery_probe_{false}, link_recovering_{false};
  double probe_started_{0};
  int reconnect_delay_ms_{100};
  uint64_t reconnect_attempts_{0};
  std::chrono::steady_clock::time_point next_reconnect_{}, link_opened_{};
  int64_t minimum_command_stamp_ns_{0}, last_brake_stamp_ns_{0};
  SteeringDisableReason last_disable_reason_{SteeringDisableReason::NONE};
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr safety_pub_;
  void torqueSafetyStop(TorqueSafetyFault fault);
  rcl_interfaces::msg::SetParametersResult onParameters(
    const std::vector<rclcpp::Parameter> & parameters);
  double torqueCurrentLimitCommandA() const;
  void publishTorqueState();
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
  TorqueSteeringController torque_controller_;
  TorqueSteeringOutput torque_output_;
  std::unique_ptr<steering_mpc::SteeringMpc> steering_mpc_;
  mpc_common::Solution mpc_result_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr mpc_diagnostics_;
  std::string actuation_mode_{"position"};
  bool torqueMode() const { return actuation_mode_ == "torque"; }
  int torque_direction_sign_{1};
  int32_t sent_torque_raw_{0};
  double keya_protection_current_a_{20.0};
  bool stop_pending_{false};
  // Active Host-side controller condition. A healthy feedback cycle clears it.
  TorqueSafetyFault torque_safety_condition_{TorqueSafetyFault::NONE};
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
  std::chrono::steady_clock::time_point torque_command_deadline_{};

  rclcpp::Subscription<kaiev26_msgs::msg::ActuatorCommand>::SharedPtr command_sub_;
  rclcpp::Subscription<kaiev26_msgs::msg::DriveActuatorStatus>::SharedPtr drive_status_sub_;
  rclcpp::Subscription<kaiev26_msgs::msg::BrakeActuatorStatus>::SharedPtr brake_status_sub_;
  rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr test_estop_sub_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr angle_pub_;
  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr target_motor_count_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr enable_commanded_pub_, enabled_pub_;
  rclcpp::Publisher<kaiev26_msgs::msg::SteeringActuatorStatus>::SharedPtr status_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr actuation_mode_pub_, torque_safety_pub_;
  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr torque_command_pub_, torque_limit_pub_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr requested_angle_pub_, angle_error_pub_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr rate_pub_, raw_rate_pub_, torque_p_pub_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr torque_d_pub_, torque_unsaturated_pub_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr torque_current_limit_command_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr torque_saturated_pub_, torque_settled_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr torque_peak_limited_pub_, torque_rate_ready_pub_;
  rclcpp::TimerBase::SharedPtr control_timer_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr parameter_callback_;
};
}  // namespace lateral_ctrl
