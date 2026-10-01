// /planning/command를 받아 /vehicle/command로 재발행하는 명령 파이프의 한 단계.
// 이 노드가 /vehicle/command의 유일한 발행자다.
//
// 양수 속도 목표의 가·감속 slew. 0은 즉시 토크컷이다.
// 수신 즉시 재발행하므로 자체 타이머가 없고,
// 입력이 끊기면 출력도 끊긴다 — 그러면 STM_A의 500 ms Host timeout과 조향의
// 200 ms freshness가 기존 그대로 안전망 역할을 한다. 마지막 값 유지(keep-alive)는
// 상류 정지를 가리므로 의도적으로 두지 않는다.
//
// 출력은 이 노드의 클럭으로 재-stamp한다. 하류(조향)가 stamp 나이로 신선도를
// 판정하므로 발행자 클럭이 어긋나면 명령 전체가 거부되는데, 재-stamp가 그
// 결합을 이 노드 한 곳에서 끊는다. 대신 입력 나이 게이트가 오래된 입력의
// 되살아남을 막는다.

#include <cstdint>
#include <chrono>
#include <cmath>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include "command_governor/governor_core.hpp"
#include "kaiev26_msgs/msg/actuator_command.hpp"
#include "rcl_interfaces/msg/set_parameters_result.hpp"
#include "rclcpp/rclcpp.hpp"

namespace command_governor
{

class CommandGovernorNode : public rclcpp::Node
{
public:
  CommandGovernorNode()
  : rclcpp::Node("command_governor")
  {
    const std::string input_topic =
      declare_parameter<std::string>("input_topic", "/planning/command");
    const std::string output_topic =
      declare_parameter<std::string>("output_topic", "/vehicle/command");
    input_timeout_ms_ = declare_parameter<double>("input_timeout_ms", 200.0);
    deceleration_slew_mps2_ = declare_parameter<double>("deceleration_slew_mps2", 0.5);
    acceleration_slew_mps2_ = declare_parameter<double>("acceleration_slew_mps2", 0.0);
    if (!std::isfinite(acceleration_slew_mps2_) || acceleration_slew_mps2_ < 0.0) {
      throw std::invalid_argument("acceleration_slew_mps2 must be finite and >= 0");
    }
    if (!std::isfinite(deceleration_slew_mps2_) || deceleration_slew_mps2_ < 0.0) {
      throw std::invalid_argument("deceleration_slew_mps2 must be finite and >= 0");
    }
    if (!std::isfinite(input_timeout_ms_) || !(input_timeout_ms_ > 0.0)) {
      // 0 이하나 NaN이면 모든 입력이 거부되어 출력이 아예 없어진다.
      // 설정 오류는 조용한 무출력 대신 launch 실패로 드러낸다.
      throw std::invalid_argument("input_timeout_ms must be > 0");
    }

    // 파라미터는 생성자에서 한 번만 읽는다. 런타임 set이 조용한
    // no-op이 되지 않도록 command_generator와 같은 방식으로 거부한다.
    parameter_callback_ = add_on_set_parameters_callback(
      [](const std::vector<rclcpp::Parameter> & parameters) {
        rcl_interfaces::msg::SetParametersResult result;
        result.successful = true;
        for (const auto & parameter : parameters) {
          const std::string & name = parameter.get_name();
          if (name == "input_topic" || name == "output_topic" ||
            name == "input_timeout_ms" || name == "deceleration_slew_mps2" ||
            name == "acceleration_slew_mps2")
          {
            result.successful = false;
            result.reason = name + " is read at startup (restart required)";
            return result;
          }
        }
        return result;
      });

    // 구독자(STM_A/B micro-ROS, 조향)가 모두 기본 RELIABLE이므로 맞춘다.
    publisher_ = create_publisher<kaiev26_msgs::msg::ActuatorCommand>(
      output_topic, rclcpp::QoS(10));
    subscription_ = create_subscription<kaiev26_msgs::msg::ActuatorCommand>(
      input_topic, rclcpp::QoS(10),
      [this](kaiev26_msgs::msg::ActuatorCommand::UniquePtr message) {
        onCommand(std::move(message));
      });

    RCLCPP_INFO(
      get_logger(), "governor: %s -> %s, input_timeout_ms=%.0f, deceleration_slew_mps2=%.3f, acceleration_slew_mps2=%.3f",
      input_topic.c_str(), output_topic.c_str(), input_timeout_ms_, deceleration_slew_mps2_,
      acceleration_slew_mps2_);
  }

private:
  void onCommand(kaiev26_msgs::msg::ActuatorCommand::UniquePtr message)
  {
    const rclcpp::Time now = this->now();
    // 0(미기록)과 음수 sec(손상·언더플로)를 함께 거른다. 음수 시각은
    // rclcpp::Time 생성자가 예외를 던지므로 반드시 생성 전에 걸러야 한다 —
    // 콜백에서 새는 예외는 유일 발행자인 이 프로세스를 통째로 죽인다.
    const bool stamp_invalid =
      (message->header.stamp.sec < 0) ||
      ((message->header.stamp.sec == 0) && (message->header.stamp.nanosec == 0));
    const double age_ms = stamp_invalid ?
      0.0 : (now - rclcpp::Time(message->header.stamp)).seconds() * 1000.0;

    const InputVerdict verdict = evaluate_input(
      stamp_invalid, age_ms, input_timeout_ms_,
      message->speed_target_mps, message->steering_target_rad);
    if (verdict != InputVerdict::kAccept) {
      ++rejected_count_;
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 1000,
        "input rejected: %s (age=%.1f ms, rejected=%lu, accepted=%lu)",
        to_string(verdict), age_ms,
        static_cast<unsigned long>(rejected_count_),
        static_cast<unsigned long>(accepted_count_));
      return;
    }

    const double receipt_s = std::chrono::duration<double>(
      std::chrono::steady_clock::now().time_since_epoch()).count();
    message->speed_target_mps = speed_slew_.update(
      message->speed_target_mps, message->brake_engage, receipt_s,
      input_timeout_ms_ / 1000.0, deceleration_slew_mps2_, acceleration_slew_mps2_);
    message->header.stamp = now;
    ++accepted_count_;
    publisher_->publish(std::move(message));
  }

  double input_timeout_ms_{200.0};
  double deceleration_slew_mps2_{0.5};
  double acceleration_slew_mps2_{0.0};
  SpeedSlew speed_slew_;
  uint64_t accepted_count_{0};
  uint64_t rejected_count_{0};
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr parameter_callback_;
  rclcpp::Publisher<kaiev26_msgs::msg::ActuatorCommand>::SharedPtr publisher_;
  rclcpp::Subscription<kaiev26_msgs::msg::ActuatorCommand>::SharedPtr subscription_;
};

}  // namespace command_governor

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<command_governor::CommandGovernorNode>());
  rclcpp::shutdown();
  return 0;
}
