#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32.hpp"

#include <linux/can.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <unistd.h>
#include <cstring>
#include <string>

class SteeringFeedbackNode : public rclcpp::Node {
public:
    SteeringFeedbackNode(const std::string& can_iface = "can1")
    : Node("steering_feedback_node")
    {
        // 퍼블리셔 생성
        encoder_pub_ = this->create_publisher<std_msgs::msg::Float32>("/encoder_angle", 10);
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100),
            std::bind(&SteeringFeedbackNode::read_can_frame, this)
        );

        // 소켓 설정
        if ((can_socket_ = socket(PF_CAN, SOCK_RAW, CAN_RAW)) < 0) {
            RCLCPP_FATAL(this->get_logger(), "Failed to create CAN socket");
            rclcpp::shutdown();
            return;
        }

        struct ifreq ifr;
        std::strncpy(ifr.ifr_name, can_iface.c_str(), IFNAMSIZ - 1);
        if (ioctl(can_socket_, SIOCGIFINDEX, &ifr) < 0) {
            RCLCPP_FATAL(this->get_logger(), "Failed to get CAN interface index");
            rclcpp::shutdown();
            return;
        }

        struct sockaddr_can addr;
        addr.can_family = AF_CAN;
        addr.can_ifindex = ifr.ifr_ifindex;

        if (bind(can_socket_, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
            RCLCPP_FATAL(this->get_logger(), "Failed to bind CAN socket");
            rclcpp::shutdown();
            return;
        }

        RCLCPP_INFO(this->get_logger(), "Steering Feedback Node started");
    }

    ~SteeringFeedbackNode() {
        close(can_socket_);
    }

private:
    void read_can_frame() {
        struct can_frame frame;
        int nbytes = read(can_socket_, &frame, sizeof(frame));

        if (nbytes < 0) {
            RCLCPP_WARN(this->get_logger(), "CAN read error");
            return;
        }

        uint32_t can_id = frame.can_id & CAN_EFF_MASK;

        if (can_id == 0x07000001) {
            int16_t encoder_raw = (frame.data[6] << 8) | frame.data[7];
            if (encoder_raw & 0x8000) encoder_raw -= 0x10000;

            float angle_deg = (encoder_raw / 10000.0f) * 360.0f;

            std_msgs::msg::Float32 msg;
            msg.data = angle_deg;
            encoder_pub_->publish(msg);

            RCLCPP_INFO(this->get_logger(), "Encoder angle: %.2f deg", angle_deg);
        }
    }

    int can_socket_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr encoder_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    std::string iface = "can1";
    if (argc > 1) iface = argv[1];
    rclcpp::spin(std::make_shared<SteeringFeedbackNode>(iface));
    rclcpp::shutdown();
    return 0;
}
