/*
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
#include <cmath>   // sqrt, fabs

class SteeringControlNode : public rclcpp::Node {
public:
    SteeringControlNode(const std::string& can_iface = "can1") : Node("steering_control_node") {
        RCLCPP_INFO(this->get_logger(), "Steering Control Node Started");

        subscription_ = this->create_subscription<std_msgs::msg::Float32>(
            "/cmd/steer", 10,
            std::bind(&SteeringControlNode::steering_callback, this, std::placeholders::_1)
        );

        throttle_sub_ = this->create_subscription<throttle_msgs::msg::ThrottleData>(
            "throttle_data", 10,
            std::bind(&SteeringControlNode::throttle_callback, this, std::placeholders::_1)
        );

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

    ~SteeringControlNode() { close(can_socket_); }

private:
    // 평균식 y = A x^2 + B x + C (x>=0), x<0는 원점대칭 y = -f(-x)
    static constexpr float A = 0.0002f;
    static constexpr float B = 0.2159f;
    static constexpr float C = 0.1612f;

    // 바퀴각 물리 한계 (±30°)
    static constexpr float WHEEL_LIMIT_DEG = 30.0f;

    // |y|에 대해 f(x_abs)=|y|를 풀고, 부호는 sign(y) 적용 (원점대칭 역함수)
    float wheel_to_handle_deg_from_avg_piecewise(float y_deg) const {
        const float sgn = (y_deg >= 0.0f) ? 1.0f : -1.0f;
        float ya = std::fabs(y_deg);
        float D = B*B - 4.0f*A*(C - ya);
        if (D < 0.0f) { ya = C; D = B*B; } // 안전 클램프
        float x_abs = (-B + std::sqrt(D)) / (2.0f*A);
        if (x_abs < 0.0f) x_abs = 0.0f;
        return sgn * x_abs;
    }

    inline float clamp_wheel_deg(float y) const {
        if (y > WHEEL_LIMIT_DEG)  return WHEEL_LIMIT_DEG;
        if (y < -WHEEL_LIMIT_DEG) return -WHEEL_LIMIT_DEG;
        return y;
    }

    int can_socket_;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr subscription_;
    rclcpp::Subscription<throttle_msgs::msg::ThrottleData>::SharedPtr throttle_sub_;
    bool estop_{false}, last_estop_{false};
    bool is_manual_{false}, last_is_manual_{false};
    int32_t last_scaled_{0};
    int32_t pending_scaled_{0};
    bool has_pending_{false};

    void send_position_scaled(int32_t scaled_val) {
        uint8_t data[8];
        data[0] = 0x23; data[1] = 0x02; data[2] = 0x20; data[3] = 0x01;
        data[4] = (scaled_val >> 24) & 0xFF;
        data[5] = (scaled_val >> 16) & 0xFF;
        data[6] = (scaled_val >> 8)  & 0xFF;
        data[7] =  scaled_val        & 0xFF;

        struct can_frame frame;
        frame.can_id  = 0x06000001 | CAN_EFF_FLAG;
        frame.can_dlc = 8;
        std::memcpy(frame.data, data, 8);

        int sent = write(can_socket_, &frame, sizeof(frame));
        if (sent != sizeof(frame)) {
            RCLCPP_WARN(this->get_logger(), "Failed to send CAN frame (scaled=%d)", scaled_val);
        }
    }

    void enable_motor() {
        uint8_t data[8] = {0x23,0x0D,0x20,0x01,0x00,0x00,0x00,0x00};
        struct can_frame frame;
        frame.can_id  = 0x06000001 | CAN_EFF_FLAG;
        frame.can_dlc = 8;
        std::memcpy(frame.data, data, 8);
        int sent = write(can_socket_, &frame, sizeof(frame));
        if (sent == sizeof(frame)) RCLCPP_INFO(this->get_logger(), "Motor enabled.");
        else RCLCPP_ERROR(this->get_logger(), "Failed to send motor enable frame.");
    }

    void throttle_callback(const throttle_msgs::msg::ThrottleData::SharedPtr msg) {
        estop_ = msg->estop;
        is_manual_ = msg->asms;

        if (estop_ && !last_estop_) {
            RCLCPP_WARN(this->get_logger(), "ESTOP detected → holding last steering angle (scaled=%d)", last_scaled_);
        }

        if (!is_manual_ && last_is_manual_) {
            if (!estop_) {
                RCLCPP_INFO(this->get_logger(),
                    "ASMS: MANUAL -> AUTO, re-enabling motor and sending last command (scaled=%d)",
                    has_pending_ ? pending_scaled_ : last_scaled_);
                enable_motor();
                struct timespec ts{0, 50 * 1000 * 1000}; // 50ms
                nanosleep(&ts, nullptr);
                if (has_pending_) {
                    send_position_scaled(pending_scaled_);
                    last_scaled_ = pending_scaled_;
                    has_pending_ = false;
                } else {
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
        // 명령 토픽 정의: 좌=-°, 우=+°  (사람이 보기 편한 부호)
        // 하드웨어 raw: 좌=+, 우=-  → 역함수 뒤 x에 부호 반전해서 모터로 전송해야 함.

        if (estop_ || is_manual_) {
            float y_cmd_in = msg->data;                 // 바퀴각 명령 (deg)
            float y_cmd    = clamp_wheel_deg(y_cmd_in); // ±30° 제한
            if (y_cmd != y_cmd_in) {
                RCLCPP_WARN(this->get_logger(),
                            "Wheel cmd clamped (buffered): requested=%.2f°, used=%.2f° (±%.0f°)",
                            y_cmd_in, y_cmd, WHEEL_LIMIT_DEG);
            }

            // 역함수로 핸들각 계산
            float x = wheel_to_handle_deg_from_avg_piecewise(y_cmd);

            // ✅ 하드웨어 부호에 맞게 반전 (좌:- -> 좌:+ 로 전송)
            x = -x;

            pending_scaled_ = static_cast<int32_t>((x / 360.0f) * 10000.0f);
            has_pending_ = true;
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                is_manual_ ? "MANUAL active: steering commands buffered" : "ESTOP active: steering commands ignored");
            return;
        }

        float y_cmd_in = msg->data;               // 바퀴각 명령 (deg)
        float y_cmd    = clamp_wheel_deg(y_cmd_in);
        if (y_cmd != y_cmd_in) {
            RCLCPP_WARN(this->get_logger(),
                        "Wheel cmd clamped: requested=%.2f°, used=%.2f° (±%.0f°)",
                        y_cmd_in, y_cmd, WHEEL_LIMIT_DEG);
        }

        // 역함수로 핸들각 계산
        float x = wheel_to_handle_deg_from_avg_piecewise(y_cmd);

        // ✅ 하드웨어 부호에 맞게 반전 (좌:- -> 좌:+)
        x = -x;

        // 360도 기준 10000단위 스케일
        int32_t scaled_val = static_cast<int32_t>((x / 360.0f) * 10000.0f);
        last_scaled_ = scaled_val;
        has_pending_ = false;

        send_position_scaled(scaled_val);
        RCLCPP_INFO(this->get_logger(),
            "Wheel cmd=%.2f° (limit ±%.0f) → Handle x(after invert)=%.2f° → Scaled=%d",
            y_cmd, WHEEL_LIMIT_DEG, x, scaled_val);
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
*/
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
#include <cmath>   // sqrt, fabs

class SteeringControlNode : public rclcpp::Node {
public:
    explicit SteeringControlNode(const std::string& can_iface = "can1")
    : Node("steering_control_node")
    {
        RCLCPP_INFO(get_logger(), "Steering Control Node Started");

        subscription_ = create_subscription<std_msgs::msg::Float32>(
            "/cmd/steer", 10,
            std::bind(&SteeringControlNode::steering_callback, this, std::placeholders::_1));

        throttle_sub_ = create_subscription<throttle_msgs::msg::ThrottleData>(
            "throttle_data", 10,
            std::bind(&SteeringControlNode::throttle_callback, this, std::placeholders::_1));

        // ── CAN 소켓 열기
        can_socket_ = socket(PF_CAN, SOCK_RAW, CAN_RAW);
        if (can_socket_ < 0) {
            RCLCPP_FATAL(get_logger(), "Failed to open CAN socket");
            rclcpp::shutdown();
            return;
        }

        struct ifreq ifr {};
        std::strncpy(ifr.ifr_name, can_iface.c_str(), IFNAMSIZ - 1);
        if (ioctl(can_socket_, SIOCGIFINDEX, &ifr) < 0) {
            RCLCPP_FATAL(get_logger(), "Failed to get CAN ifindex for %s", can_iface.c_str());
            rclcpp::shutdown();
            return;
        }

        struct sockaddr_can addr {};
        addr.can_family  = AF_CAN;
        addr.can_ifindex = ifr.ifr_ifindex;

        if (bind(can_socket_, (struct sockaddr*)&addr, sizeof(addr)) < 0) {
            RCLCPP_FATAL(get_logger(), "Failed to bind CAN socket");
            rclcpp::shutdown();
            return;
        }

        RCLCPP_INFO(get_logger(), "CAN socket initialized");

        // 드라이버 Enable
        enable_motor();

        // ── 포지션 홀드 / Keep-Alive 타이머 (10 Hz = 100 ms)
        hold_timer_ = create_wall_timer(
            std::chrono::milliseconds(100),
            std::bind(&SteeringControlNode::hold_timer_tick, this));
    }

    ~SteeringControlNode() override {
        if (can_socket_ >= 0) close(can_socket_);
    }

private:
    // 평균식 y = A x^2 + B x + C (x>=0), x<0는 원점대칭 y = -f(-x)
    static constexpr float A = 0.0002f;
    static constexpr float B = 0.2159f;
    static constexpr float C = 0.1612f;

    // 바퀴각 물리 한계 (±30°)
    static constexpr float WHEEL_LIMIT_DEG = 30.0f;

    // |y|에 대해 f(x_abs)=|y|를 풀고, 부호는 sign(y) 적용 (원점대칭 역함수)
    float wheel_to_handle_deg_from_avg_piecewise(float y_deg) const {
        const float sgn = (y_deg >= 0.0f) ? 1.0f : -1.0f;
        float ya = std::fabs(y_deg);
        float D  = B*B - 4.0f*A*(C - ya);
        if (D < 0.0f) { ya = C; D = B*B; } // 안전 클램프
        float x_abs = (-B + std::sqrt(D)) / (2.0f*A);
        if (x_abs < 0.0f) x_abs = 0.0f;
        return sgn * x_abs;
    }

    inline float clamp_wheel_deg(float y) const {
        if (y >  WHEEL_LIMIT_DEG) return  WHEEL_LIMIT_DEG;
        if (y < -WHEEL_LIMIT_DEG) return -WHEEL_LIMIT_DEG;
        return y;
    }

    // ── CAN 전송: 포지션(10000/circle)
    void send_position_scaled(int32_t scaled_val) {
        uint8_t data[8];
        data[0] = 0x23; data[1] = 0x02; data[2] = 0x20; data[3] = 0x01;
        data[4] = (scaled_val >> 24) & 0xFF;
        data[5] = (scaled_val >> 16) & 0xFF;
        data[6] = (scaled_val >> 8)  & 0xFF;
        data[7] =  scaled_val        & 0xFF;

        struct can_frame frame {};
        frame.can_id  = 0x06000001 | CAN_EFF_FLAG; // Keya: 0x06000000 + node(=1)
        frame.can_dlc = 8;
        std::memcpy(frame.data, data, 8);

        const int sent = write(can_socket_, &frame, sizeof(frame));
        if (sent != (int)sizeof(frame)) {
            RCLCPP_WARN(get_logger(), "Failed to send CAN frame (scaled=%d)", scaled_val);
        }
    }

    // ── CAN 전송: Enable
    void enable_motor() {
        uint8_t data[8] = {0x23,0x0D,0x20,0x01,0x00,0x00,0x00,0x00};
        struct can_frame frame {};
        frame.can_id  = 0x06000001 | CAN_EFF_FLAG;
        frame.can_dlc = 8;
        std::memcpy(frame.data, data, 8);

        const int sent = write(can_socket_, &frame, sizeof(frame));
        if (sent == (int)sizeof(frame)) {
            RCLCPP_INFO(get_logger(), "Motor enabled.");
        } else {
            RCLCPP_ERROR(get_logger(), "Failed to send motor enable frame.");
        }
    }

    // ── 타이머: ESTOP/AUTO에서 포지션 홀드 및 keep-alive
    void hold_timer_tick() {
        // MANUAL: 운전자 조작 우선 → 전송 안 함 (정책에 따라 조정 가능)
        if (is_manual_) return;

        if (estop_hold_active_) {
            // ESTOP 동안엔 고정된 목표만 지속 송신 (명령 주기 ≤ 1000ms 유지)
            send_position_scaled(hold_scaled_);
            // 통신 품질이 안 좋을 때 대비 1초마다 Enable 보강(선택)
            if (++enable_ping_cnt_ >= 10) { // 100ms * 10 = 1s
                enable_motor();
                enable_ping_cnt_ = 0;
            }
            return;
        }

        // 평상시 keep-alive (AUTO)
        send_position_scaled(last_scaled_);
        enable_ping_cnt_ = 0;
    }

    // ── 차량 상태 콜백(ESTOP/ASMS)
    void throttle_callback(const throttle_msgs::msg::ThrottleData::SharedPtr msg) {
        estop_     = msg->estop;
        is_manual_ = msg->asms;

        // ESTOP 진입 엣지: 그 순간의 setpoint를 얼려서 유지
        if (estop_ && !last_estop_) {
            estop_hold_active_ = true;
            hold_scaled_ = last_scaled_;
            RCLCPP_WARN(get_logger(),
                        "ESTOP detected → FREEZE setpoint at %d (position hold active)", hold_scaled_);
        }

        // ESTOP 해제 엣지: hold 해제 후 재-Enable & 기존 setpoint 재전송
        if (!estop_ && last_estop_) {
            estop_hold_active_ = false;
            RCLCPP_INFO(get_logger(),
                        "ESTOP cleared → re-enabling motor & restoring setpoint %d", last_scaled_);
            enable_motor();
            struct timespec ts {0, 50 * 1000 * 1000}; // 50 ms
            nanosleep(&ts, nullptr);
            send_position_scaled(last_scaled_);
        }

        // MANUAL → AUTO 전환: 재-Enable 후 마지막 명령 재전송 (버퍼 우선)
        if (!is_manual_ && last_is_manual_) {
            if (!estop_) {
                const int32_t to_send = has_pending_ ? pending_scaled_ : last_scaled_;
                RCLCPP_INFO(get_logger(),
                            "ASMS: MANUAL -> AUTO, re-enabling motor and sending (scaled=%d)", to_send);
                enable_motor();
                struct timespec ts {0, 50 * 1000 * 1000};
                nanosleep(&ts, nullptr);
                send_position_scaled(to_send);
                last_scaled_  = to_send;
                has_pending_  = false;
            } else {
                RCLCPP_WARN(get_logger(),
                            "ASMS: MANUAL -> AUTO but ESTOP is active; keeping hold state");
            }
        }

        last_estop_     = estop_;
        last_is_manual_ = is_manual_;
    }

    // ── 바퀴각 명령(좌=-°, 우=+°) → 핸들각 역변환 → 포지션 스케일
    void steering_callback(const std_msgs::msg::Float32::SharedPtr msg) {
        // ESTOP 중엔 어떤 새 명령도 무시 (목표 얼림)
        if (estop_hold_active_ || estop_) {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
                                 "ESTOP active: steer cmds ignored (frozen)");
            return;
        }
        // MANUAL 중에도 전송/갱신하지 않음 (운전자 우선)
        if (is_manual_) {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
                                 "MANUAL active: steering cmds ignored");
            return;
        }

        float y_cmd_in = msg->data;               // 바퀴각 명령 (deg)
        float y_cmd    = clamp_wheel_deg(y_cmd_in);
        if (y_cmd != y_cmd_in) {
            RCLCPP_WARN(get_logger(),
                        "Wheel cmd clamped: requested=%.2f°, used=%.2f° (±%.0f°)",
                        y_cmd_in, y_cmd, WHEEL_LIMIT_DEG);
        }

        // 역함수로 핸들각 계산, 하드웨어 부호 반전(좌=- → 좌=+ 로 전송)
        float x = -wheel_to_handle_deg_from_avg_piecewise(y_cmd);

        // 360도 기준 10000단위 스케일(포지션 모드)
        int32_t scaled_val = static_cast<int32_t>((x / 360.0f) * 10000.0f);

        // AUTO & ESTOP 아님: 즉시 전송 + keep-alive 타이머가 계속 재전송
        last_scaled_  = scaled_val;
        has_pending_  = false;
        send_position_scaled(scaled_val);

        RCLCPP_INFO(get_logger(),
            "Wheel cmd=%.2f° (limit ±%.0f) → Handle x(inv)=%.2f° → Scaled=%d",
            y_cmd, WHEEL_LIMIT_DEG, x, scaled_val);
    }

    // ── 멤버 변수들
    int can_socket_{-1};

    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr subscription_;
    rclcpp::Subscription<throttle_msgs::msg::ThrottleData>::SharedPtr throttle_sub_;
    rclcpp::TimerBase::SharedPtr hold_timer_;

    bool estop_{false}, last_estop_{false};
    bool is_manual_{false}, last_is_manual_{false};

    // 평상시 최신 목표
    int32_t last_scaled_{0};
    // MANUAL 등에서 한시적으로 쌓는 버퍼(옵션)
    int32_t pending_scaled_{0};
    bool    has_pending_{false};

    // ESTOP 동안 목표 고정
    int32_t hold_scaled_{0};
    bool    estop_hold_active_{false};

    int enable_ping_cnt_{0};
};

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    std::string iface = "can1";
    if (argc > 1) iface = argv[1];
    rclcpp::spin(std::make_shared<SteeringControlNode>(iface));
    rclcpp::shutdown();
    return 0;
}
