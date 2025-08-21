#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/int32.hpp>

class SpeedMonitorNode : public rclcpp::Node {
public:
  SpeedMonitorNode() : rclcpp::Node("speed_monitor_node") {
    // 파라미터로 토픽명 설정 가능
    topic_ = this->declare_parameter<std::string>("topic", "/current_speed");

    sub_ = this->create_subscription<std_msgs::msg::Int32>(
        topic_, 10,
        [this](const std_msgs::msg::Int32::SharedPtr msg) {
          RCLCPP_INFO(this->get_logger(), "RPM: %d", msg->data);
        });

    RCLCPP_INFO(this->get_logger(), "Subscribed to %s (std_msgs/Int32)", topic_.c_str());
  }

private:
  std::string topic_;
  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr sub_;
};

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SpeedMonitorNode>());
  rclcpp::shutdown();
  return 0;
}
