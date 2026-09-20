#pragma once

#include "longitudinal_ctrl/speed_torque_controller.hpp"

#include "geometry_msgs/msg/twist_with_covariance_stamped.hpp"
#include "kaiev26_msgs/msg/actuator_command.hpp"
#include "kaiev26_msgs/msg/brake_actuator_status.hpp"
#include "kaiev26_msgs/msg/drive_actuator_status.hpp"
#include "kaiev26_msgs/msg/drive_torque_command.hpp"
#include "rcl_interfaces/msg/set_parameters_result.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/string.hpp"

#include <chrono>
#include <string>
#include <vector>

namespace longitudinal_ctrl
{
// /vehicle/command의 목표 차속과 /drive/status의 피드백으로 상전류 명령을 만들어
// /drive/torque_command로 보낸다. STM_A는 이 명령이 신선하고 torque_mode=true일 때만
// EZkontrol을 torque control로 구동하고, 그 외에는 기존 speed control로 돌아간다.
// 정지·brake·timeout의 HALTED 판단은 계속 STM_A가 /vehicle/command로 수행한다.
class LongitudinalActuatorNode : public rclcpp::Node
{
public:
  explicit LongitudinalActuatorNode(
    const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  friend class LongitudinalActuatorNodeTestPeer;
  using Clock = std::chrono::steady_clock;

  SpeedTorqueConfig declareControllerParameters();
  rcl_interfaces::msg::SetParametersResult onParameters(
    const std::vector<rclcpp::Parameter> & parameters);
  void onCommand(const kaiev26_msgs::msg::ActuatorCommand::SharedPtr msg);
  void onDriveStatus(const kaiev26_msgs::msg::DriveActuatorStatus::SharedPtr msg);
  void onBrakeStatus(const kaiev26_msgs::msg::BrakeActuatorStatus::SharedPtr msg);
  void onGnssVelocity(const geometry_msgs::msg::TwistWithCovarianceStamped::SharedPtr msg);
  void controlTick();
  // torque control을 허용하지 않는 첫 번째 이유. 허용되면 빈 문자열.
  std::string blockingReason(Clock::time_point now) const;
  float motorRpmLimit(double shaped_target_mps) const;
  void publishDebug(const SpeedTorqueOutput & out, const std::string & state);

  SpeedTorqueController controller_;
  bool torque_control_enabled_{false};
  std::string speed_source_{"drive_status"};
  int publish_period_ms_{20};
  int command_timeout_ms_{200};
  int drive_status_timeout_ms_{200};
  double tire_diameter_m_{0.47};
  double gear_ratio_{4.6};
  double speed_limit_margin_mps_{2.0};
  double speed_limit_floor_mps_{3.0};

  bool command_received_{false};
  float target_speed_mps_{0.0f};
  bool brake_engage_{true};
  Clock::time_point last_command_time_{};

  bool drive_received_{false};
  kaiev26_msgs::msg::DriveActuatorStatus drive_{};
  Clock::time_point last_drive_time_{};
  bool estop_active_{false};
  std::string last_state_{};

  rclcpp::Subscription<kaiev26_msgs::msg::ActuatorCommand>::SharedPtr command_sub_;
  rclcpp::Subscription<kaiev26_msgs::msg::DriveActuatorStatus>::SharedPtr drive_status_sub_;
  rclcpp::Subscription<kaiev26_msgs::msg::BrakeActuatorStatus>::SharedPtr brake_status_sub_;
  rclcpp::Subscription<geometry_msgs::msg::TwistWithCovarianceStamped>::SharedPtr gnss_sub_;
  rclcpp::Publisher<kaiev26_msgs::msg::DriveTorqueCommand>::SharedPtr torque_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr state_pub_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr target_pub_, shaped_target_pub_, filtered_target_pub_,
    target_accel_pub_, speed_pub_, speed_raw_pub_, error_pub_, ff_pub_, p_pub_, i_pub_,
    unsaturated_pub_, command_pub_, regen_cap_pub_, rejected_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr saturated_pub_, slew_limited_pub_,
    regen_blocked_pub_, stopped_pub_, active_pub_;
  rclcpp::TimerBase::SharedPtr control_timer_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr parameter_callback_;
};
}  // namespace longitudinal_ctrl
