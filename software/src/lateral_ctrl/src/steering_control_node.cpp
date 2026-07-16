#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/u_int8.hpp"

#include <rcl_interfaces/msg/set_parameters_result.hpp>

#include <linux/can.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <unistd.h>
#include <time.h>
#include <fcntl.h>

#include <cstring>
#include <cerrno>
#include <string>
#include <cmath> // sqrt, fabs
#include <vector>

class SteeringControlNode : public rclcpp::Node
{
public:
    explicit SteeringControlNode(const std::string &can_iface = "can1")
        : Node("steering_control_node")
    {
        RCLCPP_INFO(get_logger(), "Steering Control Node Started");

        subscription_ = create_subscription<std_msgs::msg::Float32>(
            "/cmd/steer", 10,
            std::bind(&SteeringControlNode::steering_callback, this, std::placeholders::_1));

        estop_sub_ = create_subscription<std_msgs::msg::UInt8>(
            "/estop", 10,
            std::bind(&SteeringControlNode::estop_callback, this, std::placeholders::_1));

        mode_sub_ = create_subscription<std_msgs::msg::UInt8>(
            "/mode", 10,
            std::bind(&SteeringControlNode::mode_callback, this, std::placeholders::_1));

        // 바퀴각 클램프 파라미터 (기본 ±30°)
        wheel_limit_deg_ = this->declare_parameter<double>("wheel_limit_deg", 30.0);
        RCLCPP_INFO(get_logger(), "wheel_limit_deg=%.2f°", wheel_limit_deg_);

        // 런타임에서 ros2 param set으로 변경 가능하게 콜백 등록
        param_cb_handle_ = this->add_on_set_parameters_callback(
            [this](const std::vector<rclcpp::Parameter> &params)
            {
                rcl_interfaces::msg::SetParametersResult result;
                result.successful = true;
                result.reason = "ok";
                for (const auto &p : params)
                {
                    if (p.get_name() == "wheel_limit_deg")
                    {
                        double v = p.as_double();
                        if (!std::isfinite(v) || v <= 0.0)
                        {
                            result.successful = false;
                            result.reason = "wheel_limit_deg must be a finite value > 0";
                            return result;
                        }
                        wheel_limit_deg_ = v;
                        RCLCPP_INFO(this->get_logger(),
                                    "wheel_limit_deg updated to %.2f°", wheel_limit_deg_);
                    }
                }
                return result;
            });

        // ── CAN 소켓 열기
        can_socket_ = socket(PF_CAN, SOCK_RAW, CAN_RAW);
        if (can_socket_ < 0)
        {
            RCLCPP_FATAL(get_logger(), "Failed to open CAN socket");
            rclcpp::shutdown();
            return;
        }

        // TX 큐가 가득 차거나 버스가 죽었을 때 write()가 노드를 멈춰세우지 않도록 non-blocking
        {
            int flags = fcntl(can_socket_, F_GETFL, 0);
            fcntl(can_socket_, F_SETFL, flags | O_NONBLOCK);
        }

        struct ifreq ifr{};
        std::strncpy(ifr.ifr_name, can_iface.c_str(), IFNAMSIZ - 1);
        if (ioctl(can_socket_, SIOCGIFINDEX, &ifr) < 0)
        {
            RCLCPP_FATAL(get_logger(), "Failed to get CAN ifindex for %s", can_iface.c_str());
            rclcpp::shutdown();
            return;
        }
        struct sockaddr_can addr{};
        addr.can_family = AF_CAN;
        addr.can_ifindex = ifr.ifr_ifindex;
        if (bind(can_socket_, (struct sockaddr *)&addr, sizeof(addr)) < 0)
        {
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

    ~SteeringControlNode() override
    {
        if (can_socket_ >= 0)
            close(can_socket_);
    }

private:
    // 평균식 y = A x^2 + B x + C (x>=0), x<0는 원점대칭 y = -f(-x)
    static constexpr float A = 0.0002f;
    static constexpr float B = 0.2159f;
    static constexpr float C = 0.1612f;

    // |y|에 대해 f(x_abs)=|y|를 풀고, 부호는 sign(y) 적용 (원점대칭 역함수)
    float wheel_to_handle_deg_from_avg_piecewise(float y_deg) const
    {
        const float sgn = (y_deg >= 0.0f) ? 1.0f : -1.0f;
        float ya = std::fabs(y_deg);
        float D = B * B - 4.0f * A * (C - ya);
        if (D < 0.0f)
        {
            ya = C;
            D = B * B;
        } // 안전 클램프
        float x_abs = (-B + std::sqrt(D)) / (2.0f * A);
        if (x_abs < 0.0f)
            x_abs = 0.0f;
        return sgn * x_abs;
    }

    inline float clamp_wheel_deg(float y) const
    {
        const float L = static_cast<float>(wheel_limit_deg_);
        if (y > L)
            return L;
        if (y < -L)
            return -L;
        return y;
    }

    // ── CAN 전송: 포지션(10000/circle)
    void send_position_scaled(int32_t scaled_val)
    {
        uint8_t data[8];
        data[0] = 0x23;
        data[1] = 0x02;
        data[2] = 0x20;
        data[3] = 0x01;
        data[4] = (scaled_val >> 24) & 0xFF;
        data[5] = (scaled_val >> 16) & 0xFF;
        data[6] = (scaled_val >> 8) & 0xFF;
        data[7] = scaled_val & 0xFF;

        struct can_frame frame{};
        frame.can_id = 0x06000001 | CAN_EFF_FLAG; // Keya: 0x06000000 + node(=1)
        frame.can_dlc = 8;
        std::memcpy(frame.data, data, 8);

        const int sent = write(can_socket_, &frame, sizeof(frame));
        if (sent != (int)sizeof(frame))
        {
            RCLCPP_WARN(get_logger(), "Failed to send CAN frame (scaled=%d): %s",
                        scaled_val, std::strerror(errno));
        }
    }

    // ── CAN 전송: Enable
    void enable_motor()
    {
        uint8_t data[8] = {0x23, 0x0D, 0x20, 0x01, 0x00, 0x00, 0x00, 0x00};
        struct can_frame frame{};
        frame.can_id = 0x06000001 | CAN_EFF_FLAG;
        frame.can_dlc = 8;
        std::memcpy(frame.data, data, 8);

        const int sent = write(can_socket_, &frame, sizeof(frame));
        if (sent == (int)sizeof(frame))
        {
            RCLCPP_INFO(get_logger(), "Motor enabled.");
        }
        else
        {
            RCLCPP_ERROR(get_logger(), "Failed to send motor enable frame: %s", std::strerror(errno));
        }
    }

    // ── CAN 전송: Disable (Keya 데이터시트 4.3.2: subindex 0x0C, Enable은 0x0D)
    void disable_motor()
    {
        uint8_t data[8] = {0x23, 0x0C, 0x20, 0x01, 0x00, 0x00, 0x00, 0x00};
        struct can_frame frame{};
        frame.can_id = 0x06000001 | CAN_EFF_FLAG;
        frame.can_dlc = 8;
        std::memcpy(frame.data, data, 8);

        const int sent = write(can_socket_, &frame, sizeof(frame));
        if (sent == (int)sizeof(frame))
        {
            RCLCPP_WARN(get_logger(), "Motor disabled (torque off).");
        }
        else
        {
            RCLCPP_ERROR(get_logger(), "Failed to send motor disable frame: %s", std::strerror(errno));
        }
    }

    // ── 타이머: ESTOP/AUTO에서 포지션 홀드 및 keep-alive
    void hold_timer_tick()
    {
        // ESTOP 또는 MANUAL이면 아무 것도 하지 않음
        if (estop_ || is_manual_)
            return;

        // AUTO + 유효 setpoint가 있을 때만 주기 전송
        if (have_active_setpoint_)
        {
            send_position_scaled(last_scaled_);
        }

        // 주기적 enable ping (ESTOP 아닐 때만)
        if (++enable_ping_cnt_ >= 10)
        {
            enable_motor();
            enable_ping_cnt_ = 0;
        }
    }

    // ── /estop 콜백: 0=해제, 그 외=활성 (STM_B ESTOP_STATUS 통합값)
    void estop_callback(const std_msgs::msg::UInt8::SharedPtr msg)
    {
        const bool new_estop = (msg->data != 0);

        // ESTOP 상승엣지: 토크 OFF, 모든 setpoint 홀드 해제
        if (new_estop && !estop_)
        {
            have_active_setpoint_ = false;
            disable_motor();
            enable_ping_cnt_ = 0;
            RCLCPP_WARN(get_logger(), "ESTOP detected → torque OFF, stop sending setpoints");
        }

        // ESTOP 하강엣지: 자동 복구/재전송 하지 않음 (다음 명령 대기)
        if (!new_estop && estop_)
        {
            have_active_setpoint_ = false;
            RCLCPP_INFO(get_logger(), "ESTOP cleared → not restoring previous setpoint; waiting for new command");
        }

        estop_ = new_estop;
    }

    // ── /mode 콜백: STM_A 기준 1=AUTO, 0=MANUAL
    void mode_callback(const std_msgs::msg::UInt8::SharedPtr msg)
    {
        const bool new_is_manual = (msg->data == 0);

        // AUTO -> MANUAL 전환 시: 사람이 손으로 조향할 수 있도록 토크 OFF
        if (new_is_manual && !is_manual_)
        {
            have_active_setpoint_ = false;
            disable_motor();
            RCLCPP_WARN(get_logger(), "MODE: AUTO -> MANUAL, motor disabled");
        }

        // MANUAL -> AUTO 전환 시: 모터만 enable, 이전 setpoint는 전송하지 않음
        if (!new_is_manual && is_manual_)
        {
            if (!estop_)
            {
                enable_motor();
                RCLCPP_INFO(get_logger(), "MODE: MANUAL -> AUTO, motor enabled; waiting for new steer cmd");
                have_active_setpoint_ = false;
            }
            else
            {
                RCLCPP_WARN(get_logger(), "MODE: MANUAL -> AUTO but ESTOP active; staying torque-off");
            }
        }

        is_manual_ = new_is_manual;
    }

    // ── 바퀴각 명령(좌=-°, 우=+°) → 핸들각 역변환 → 포지션 스케일
    void steering_callback(const std_msgs::msg::Float32::SharedPtr msg)
    {
        if (estop_)
        {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
                                 "ESTOP active: steer cmds ignored");
            return;
        }
        if (is_manual_)
        {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
                                 "MANUAL active: steering cmds ignored");
            return;
        }

        float y_cmd_in = msg->data; // 바퀴각 명령 (deg)
        float y_cmd = clamp_wheel_deg(y_cmd_in);
        if (y_cmd != y_cmd_in)
        {
            RCLCPP_WARN(get_logger(),
                        "Wheel cmd clamped: requested=%.2f°, used=%.2f° (±%.0f°)",
                        y_cmd_in, y_cmd, wheel_limit_deg_);
        }

        float x = -wheel_to_handle_deg_from_avg_piecewise(y_cmd);
        int32_t scaled_val = static_cast<int32_t>((x / 360.0f) * 10000.0f);

        last_scaled_ = scaled_val;
        have_active_setpoint_ = true; // 유효 setpoint 활성화
        send_position_scaled(scaled_val);

        RCLCPP_INFO(get_logger(),
                    "Wheel cmd=%.2f° (limit ±%.0f) → Handle x(inv)=%.2f° → Scaled=%d",
                    y_cmd, wheel_limit_deg_, x, scaled_val);
    }

    // ── 멤버 변수들
    int can_socket_{-1};

    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr subscription_;
    rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr estop_sub_;
    rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr mode_sub_;
    rclcpp::TimerBase::SharedPtr hold_timer_;

    bool estop_{false};
    bool is_manual_{false};

    int32_t last_scaled_{0};
    int enable_ping_cnt_{0};

    // 현재 세션에서 유효한 setpoint를 갖고 있는지
    bool have_active_setpoint_{false};

    // 파라미터 & 콜백 핸들
    double wheel_limit_deg_{30.0};
    rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_cb_handle_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    std::string iface = "can1";
    if (argc > 1)
        iface = argv[1];
    rclcpp::spin(std::make_shared<SteeringControlNode>(iface));
    rclcpp::shutdown();
    return 0;
}
