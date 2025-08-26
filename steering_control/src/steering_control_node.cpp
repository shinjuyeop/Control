#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32.hpp"
#include "throttle_msgs/msg/throttle_data.hpp"

#include <linux/can.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <unistd.h>
#include <time.h>
#include <cstring>
#include <string>

class SteeringControlNode : public rclcpp::Node {
public:
    SteeringControlNode(const std::string& can_iface = "can1") : Node("steering_control_node") {
        RCLCPP_INFO(this->get_logger(), "Steering Control Node Started");

        // 구독 생성
        subscription_ = this->create_subscription<std_msgs::msg::Float32>(
            "/cmd/steer",
            10,
            std::bind(&SteeringControlNode::steering_callback, this, std::placeholders::_1)
        );

        // ESTOP 수신 (throttle_data에서 estop 플래그 사용)
        throttle_sub_ = this->create_subscription<throttle_msgs::msg::ThrottleData>(
            "throttle_data", 10,
            std::bind(&SteeringControlNode::throttle_callback, this, std::placeholders::_1)
        );

        // CAN 소켓 설정
        can_socket_ = socket(PF_CAN, SOCK_RAW, CAN_RAW);
        if (can_socket_ < 0) {
            RCLCPP_FATAL(this->get_logger(), "Failed to open CAN socket");
            rclcpp::shutdown();
            return;
        }

        struct ifreq ifr;
        std::strncpy(ifr.ifr_name, can_iface.c_str(), IFNAMSIZ - 1);
        ioctl(can_socket_, SIOCGIFINDEX, &ifr);

        struct sockaddr_can addr;
        addr.can_family = AF_CAN;
        addr.can_ifindex = ifr.ifr_ifindex;

        if (bind(can_socket_, (struct sockaddr*)&addr, sizeof(addr)) < 0) {
            RCLCPP_FATAL(this->get_logger(), "Failed to bind CAN socket");
            rclcpp::shutdown();
            return;
        }

        RCLCPP_INFO(this->get_logger(), "CAN socket initialized");
        enable_motor();
    }

    ~SteeringControlNode() {
        close(can_socket_);
    }

private:
    int can_socket_;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr subscription_;
    rclcpp::Subscription<throttle_msgs::msg::ThrottleData>::SharedPtr throttle_sub_;
    bool estop_{false};
    bool last_estop_{false};
    bool is_manual_{false};
    bool last_is_manual_{false};
    int32_t last_scaled_{0}; // 마지막으로 전송한 조향 목표값(스케일)
    int32_t pending_scaled_{0};
    bool has_pending_{false};

    void send_position_scaled(int32_t scaled_val) {
        uint8_t data[8];
        data[0] = 0x23;
        data[1] = 0x02;
        data[2] = 0x20;
        data[3] = 0x01;
        data[4] = (scaled_val >> 24) & 0xFF;
        data[5] = (scaled_val >> 16) & 0xFF;
        data[6] = (scaled_val >> 8) & 0xFF;
        data[7] = scaled_val & 0xFF;

        struct can_frame frame;
        frame.can_id = 0x06000001 | CAN_EFF_FLAG;
        frame.can_dlc = 8;
        std::memcpy(frame.data, data, 8);

        int sent = write(can_socket_, &frame, sizeof(frame));
        if (sent != sizeof(frame)) {
            RCLCPP_WARN(this->get_logger(), "Failed to send CAN frame (scaled=%d)", scaled_val);
        }
    }

    void enable_motor() {
        uint8_t data[8] = {0x23, 0x0D, 0x20, 0x01, 0x00, 0x00, 0x00, 0x00};

        struct can_frame frame;
        frame.can_id = 0x06000001 | CAN_EFF_FLAG;
        frame.can_dlc = 8;
        std::memcpy(frame.data, data, 8);

        int sent = write(can_socket_, &frame, sizeof(frame));
        if (sent == sizeof(frame)) {
            RCLCPP_INFO(this->get_logger(), "Motor enabled.");
        } else {
            RCLCPP_ERROR(this->get_logger(), "Failed to send motor enable frame.");
        }
    }

    void throttle_callback(const throttle_msgs::msg::ThrottleData::SharedPtr msg) {
        // 메시지 정의: rpm(int16), estop(bool), asms(bool)
        // 여기서는 asms=true 를 MANUAL 로 해석합니다.
        estop_ = msg->estop;
        is_manual_ = msg->asms;

        if (estop_ && !last_estop_) {
            // ESTOP 상승 에지: 추가 명령을 차단하고 현재(마지막) 각도에서 정지/유지
            RCLCPP_WARN(this->get_logger(), "ESTOP detected → holding last steering angle (scaled=%d)", last_scaled_);
        }

        // MANUAL → AUTO 전환 시: 모터 전원 복귀에 따라 재-활성 후 마지막 지령 재전송
        if (!is_manual_ && last_is_manual_) {
            if (!estop_) {
                RCLCPP_INFO(this->get_logger(), "ASMS: MANUAL -> AUTO, re-enabling motor and sending last command (scaled=%d)", has_pending_ ? pending_scaled_ : last_scaled_);
                enable_motor();
                // 짧은 안정화 대기 (전원 복귀 직후 수신기 준비 시간)
                struct timespec ts{0, 50 * 1000 * 1000}; // 50ms
                nanosleep(&ts, nullptr);
                // 보류된 명령이 있으면 우선 전송
                if (has_pending_) {
                    send_position_scaled(pending_scaled_);
                    last_scaled_ = pending_scaled_;
                    has_pending_ = false;
                } else {
                    // 보류 명령이 없으면 마지막 값 재전송(선택)
                    send_position_scaled(last_scaled_);
                }
            } else {
                RCLCPP_WARN(this->get_logger(), "ASMS: MANUAL -> AUTO but ESTOP is active; keeping hold state");
            }
        }

        last_estop_ = estop_;
        last_is_manual_ = is_manual_;
    }

    void steering_callback(const std_msgs::msg::Float32::SharedPtr msg) {
        // ESTOP 또는 MANUAL 중에는 추가 명령 차단(값은 보류)
        if (estop_ || is_manual_) {
            // 최신 명령을 버퍼링하여 AUTO 복귀 시 전송
            float y_tmp = msg->data;
            float x_tmp = -0.00125f * y_tmp * y_tmp + 3.635f * y_tmp - 3.875f;
            pending_scaled_ = static_cast<int32_t>((x_tmp / 360.0f) * 10000.0f);
            has_pending_ = true;
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000, is_manual_ ? "MANUAL active: steering commands buffered" : "ESTOP active: steering commands ignored");
            return;
        }
        float y = msg->data;  // 입력받은 조향각 (y)

        // 근사된 역함수 적용 (y → x)
        float x = -0.00125f * y * y + 3.635f * y - 3.875f;

        // x를 360도 기준 10000단위 스케일로 변환
    int32_t scaled_val = static_cast<int32_t>((x / 360.0f) * 10000.0f);
    last_scaled_ = scaled_val;
    has_pending_ = false; // 정상 전송 시 보류 해제

        send_position_scaled(scaled_val);
        RCLCPP_INFO(this->get_logger(), "Received y=%.2f → Approximated x=%.2f → Scaled=%d", y, x, scaled_val);
    }
};

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    std::string iface = "can1";
    if (argc > 1) iface = argv[1];
    rclcpp::spin(std::make_shared<SteeringControlNode>(iface));
    rclcpp::shutdown();
    return 0;
}

