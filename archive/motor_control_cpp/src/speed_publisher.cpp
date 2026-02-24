#include <chrono>
#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32.hpp"
#include "rcl_interfaces/msg/set_parameters_result.hpp"

using namespace std::chrono_literals;
using rclcpp::node_interfaces::OnSetParametersCallbackHandle;

class SpeedPublisher : public rclcpp::Node
{
public:
  SpeedPublisher()
      : Node("speed_publisher")
  {
    // Parameter
    declare_parameter<double>("rpm", 250.0);
    rpm_ = get_parameter("rpm").as_double();

    // Dynamic parameter callback
    param_cb_handle_ = add_on_set_parameters_callback(
        [this](const std::vector<rclcpp::Parameter> &params)
        {
          rcl_interfaces::msg::SetParametersResult res;
          res.successful = true;
          for (const auto &p : params)
          {
            if (p.get_name() == "rpm")
            {
              rpm_ = p.as_double();
            }
          }
          return res;
        });

    // Publisher (/cmd/rpm, Float32) and 100 Hz timer
    publisher_ = create_publisher<std_msgs::msg::Float32>("/cmd/rpm", 10);
    timer_ = create_wall_timer(10ms, std::bind(&SpeedPublisher::publish_rpm, this));
  }

private:
  void publish_rpm()
  {
    std_msgs::msg::Float32 msg;
    msg.data = static_cast<float>(rpm_);
    publisher_->publish(msg);
  }

  double rpm_{0.0};
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
  OnSetParametersCallbackHandle::SharedPtr param_cb_handle_;
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SpeedPublisher>());
  rclcpp::shutdown();
  return 0;
}
