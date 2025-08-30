#include <chrono>
#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int32.hpp"
#include "rcl_interfaces/msg/set_parameters_result.hpp"

using namespace std::chrono_literals;

class SpeedPublisher : public rclcpp::Node
{
public:
  SpeedPublisher()
  : Node("speed_publisher")
  {
    /* ── Declare parameter & read initial value ────────────────────── */
    declare_parameter<int>("rpm", 250);                 // default 500
    rpm_ = get_parameter("rpm").as_int();

    /* ── Dynamic parameter callback ───────────────────────────────── */
    param_cb_handle_ = add_on_set_parameters_callback(
      [this](const std::vector<rclcpp::Parameter> & params)
      {
        rcl_interfaces::msg::SetParametersResult result;
        result.successful = true;
        for (const auto & p : params)
        {
          if (p.get_name() == "rpm")
          {
            rpm_ = p.as_int();
            RCLCPP_INFO(get_logger(), "RPM updated to %d", rpm_);
          }
        }
        return result;
      });

    /* ── Publisher & Timer ─────────────────────────────────────────── */
    publisher_ = create_publisher<std_msgs::msg::Int32>("/cmd/rpm", 10);
    timer_      = create_wall_timer(10ms,
                    std::bind(&SpeedPublisher::publish_rpm, this));
  }

private:
  void publish_rpm()
  {
    std_msgs::msg::Int32 msg;
    msg.data = rpm_;
    publisher_->publish(msg);
  }

  /* ── members ────────────────────────────────────────────────────── */
  int32_t rpm_;
  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr        timer_;
  OnSetParametersCallbackHandle::SharedPtr param_cb_handle_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SpeedPublisher>());
  rclcpp::shutdown();
  return 0;
}
