#include <algorithm>
#include <cmath>
#include <cstddef>
#include <memory>
#include <stdexcept>

#include <kaiev26_msgs/msg/actuator_command.hpp>
#include <kaiev26_msgs/msg/drive_current_command.hpp>
#include <rcl_interfaces/msg/parameter_descriptor.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joy.hpp>

namespace control_launch::joystick_teleop
{

class JoystickTeleopNode final : public rclcpp::Node
{
public:
  JoystickTeleopNode()
  : Node("joystick_teleop")
  {
    rcl_interfaces::msg::ParameterDescriptor startup_parameter;
    startup_parameter.read_only = true;

    max_steering_deg_ = declare_parameter("max_steering_deg", 30.0, startup_parameter);
    max_speed_kph_ = declare_parameter("max_speed_kph", 20.0, startup_parameter);
    steering_step_deg_ = declare_parameter("steering_step_deg", 1.0, startup_parameter);
    regen_max_a_ = declare_parameter("regen_max_a", 10.0, startup_parameter);
    left_steering_axis_ = declare_parameter("left_steering_axis", 0, startup_parameter);
    right_steering_axis_ = declare_parameter("right_steering_axis", 3, startup_parameter);
    lt_axis_ = declare_parameter("lt_axis", 2, startup_parameter);
    rt_axis_ = declare_parameter("rt_axis", 5, startup_parameter);
    dpad_horizontal_axis_ = declare_parameter("dpad_horizontal_axis", 6, startup_parameter);

    if (!std::isfinite(max_steering_deg_) || max_steering_deg_ < 0.0 ||
      !std::isfinite(max_speed_kph_) || max_speed_kph_ < 0.0 ||
      !std::isfinite(regen_max_a_) || regen_max_a_ < 0.0 ||
      !std::isfinite(steering_step_deg_) || steering_step_deg_ <= 0.0)
    {
      throw std::invalid_argument(
              "maxima must be finite and nonnegative; steps must be finite and positive");
    }
    const auto axes = {left_steering_axis_, right_steering_axis_, lt_axis_, rt_axis_,
      dpad_horizontal_axis_};
    if (*std::min_element(axes.begin(), axes.end()) < 0) {
      throw std::invalid_argument("axis indices must be nonnegative");
    }
    required_axes_ = static_cast<std::size_t>(*std::max_element(axes.begin(), axes.end())) + 1;

    dynamic_steering_deg_ = max_steering_deg_;
    publisher_ = create_publisher<kaiev26_msgs::msg::ActuatorCommand>(
      "/planning/command", rclcpp::QoS(10));
    // LT 회생 요청은 STM_A가 AUTO TORQUE 모드에서만 받는다.
    current_publisher_ = create_publisher<kaiev26_msgs::msg::DriveCurrentCommand>(
      "/vehicle/drive_current_command", rclcpp::QoS(1));
    subscription_ = create_subscription<sensor_msgs::msg::Joy>(
      "/joy", rclcpp::QoS(10),
      [this](sensor_msgs::msg::Joy::ConstSharedPtr joy) {onJoy(*joy);});
    reportLimits();
  }

private:
  static int direction(double axis)
  {
    return axis > 0.5 ? 1 : (axis < -0.5 ? -1 : 0);
  }

  static double triggerRatio(double axis)
  {
    return std::clamp((1.0 - axis) * 0.5, 0.0, 1.0);
  }

  void reportLimits() const
  {
    RCLCPP_INFO(
      get_logger(), "right stick: +/- %.2f deg; LT regen: 0..-%.2f A",
      dynamic_steering_deg_, regen_max_a_);
  }

  void onJoy(const sensor_msgs::msg::Joy & joy)
  {
    // Joy 장치별 축 개수는 다르므로 실제로 읽을 인덱스까지만 확인한다.
    if (joy.axes.size() < required_axes_) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 1000,
        "Joy has %zu axes; configured mapping needs at least %zu",
        joy.axes.size(), required_axes_);
      return;
    }

    const int horizontal = direction(joy.axes[dpad_horizontal_axis_]);
    const double previous_steering = dynamic_steering_deg_;
    // 기존 패드 배치에서 십자키 왼쪽이 양수다.
    if (horizontal != 0 && horizontal != previous_horizontal_) {
      dynamic_steering_deg_ = std::max(
        0.0, dynamic_steering_deg_ - horizontal * steering_step_deg_);
    }
    previous_horizontal_ = horizontal;
    if (dynamic_steering_deg_ != previous_steering) {
      reportLimits();
    }

    const double left = joy.axes[left_steering_axis_];
    const double right = joy.axes[right_steering_axis_];
    const double lt = triggerRatio(joy.axes[lt_axis_]);
    const double rt = triggerRatio(joy.axes[rt_axis_]);
    // 입력이 살아 있으면 동적 범위가 0이어도 오른쪽 스틱이 우선한다.
    const double steering_deg = right != 0.0 ?
      right * dynamic_steering_deg_ : left * max_steering_deg_;
    const double speed_kph = rt * max_speed_kph_;

    constexpr double kDegreesToRadians = 3.14159265358979323846 / 180.0;
    kaiev26_msgs::msg::ActuatorCommand command;
    command.header.stamp = now();
    command.steering_target_rad = static_cast<float>(steering_deg * kDegreesToRadians);
    command.speed_target_mps = static_cast<float>(speed_kph / 3.6);
    command.brake_engage = false;
    publisher_->publish(command);

    // 놓았을 때도 0을 보내 STM_A가 timeout을 기다리지 않고 회생을 거둔다.
    kaiev26_msgs::msg::DriveCurrentCommand regen;
    regen.header.stamp = command.header.stamp;
    regen.phase_current_a = static_cast<float>(-lt * regen_max_a_);
    current_publisher_->publish(regen);
  }

  double max_steering_deg_;
  double max_speed_kph_;
  double steering_step_deg_;
  double regen_max_a_;
  double dynamic_steering_deg_;
  int left_steering_axis_;
  int right_steering_axis_;
  int lt_axis_;
  int rt_axis_;
  int dpad_horizontal_axis_;
  std::size_t required_axes_;
  int previous_horizontal_{0};
  rclcpp::Publisher<kaiev26_msgs::msg::ActuatorCommand>::SharedPtr publisher_;
  rclcpp::Publisher<kaiev26_msgs::msg::DriveCurrentCommand>::SharedPtr current_publisher_;
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr subscription_;
};

}  // namespace control_launch::joystick_teleop

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<control_launch::joystick_teleop::JoystickTeleopNode>());
  rclcpp::shutdown();
  return 0;
}
