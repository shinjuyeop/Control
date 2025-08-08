#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32.hpp"
#include <vector>

class SteeringCommandPublisher : public rclcpp::Node {
public:
    SteeringCommandPublisher()
    : Node("steering_command_publisher"), index_(0)
    {
        publisher_ = this->create_publisher<std_msgs::msg::Float32>("/steering_command", 10);

        // 테스트용 명령 시퀀스 (degree)
        commands_ = {0.0f, 10.0f, -15.0f, 20.0f, 0.0f};

        timer_ = this->create_wall_timer(
            std::chrono::seconds(1),
            std::bind(&SteeringCommandPublisher::publish_command, this)
        );

        RCLCPP_INFO(this->get_logger(), "SteeringCommandPublisher started");
    }

private:
    void publish_command() {
        std_msgs::msg::Float32 msg;
        msg.data = commands_[index_];
        publisher_->publish(msg);

        RCLCPP_INFO(this->get_logger(), "Published steering command: %.2f", msg.data);

        index_ = (index_ + 1) % commands_.size();  // 순환
    }

    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    std::vector<float> commands_;
    size_t index_;
};

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SteeringCommandPublisher>());
    rclcpp::shutdown();
    return 0;
}
