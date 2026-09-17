// ============================================================================
// DeadzoneMeasureNode
//   KEYA KY170DJD01005-08G-V1 스티어링 모터의 데드존(정지 마찰 break-away 문턱)
//   측정 전용 노드. position 명령이 아니라 torque(전류) 명령을 계단식으로
//   0부터 증가시키면서, 각 단계마다 일정 시간(step_hold_ms) 유지한 뒤
//   steering_actuator_node가 발행하는 /steering/status의 모터축 각도
//   (motor_angle_deg)가 조금이라도 변했는지 확인한다. 변화가 없으면 다음
//   단계로 올리고, 변화가 생기면 그 순간의 torque cmd를 데드존 문턱값으로
//   판정한다. 드라이버 파라미터 0020(제어모드)은 이미 Torque control(2)로
//   설정돼 있다고 가정한다.
//
//   motor_angle_deg는 legacy 프로토콜의 Encoder count value query 응답
//   (10000 count/turn = 0.036°/count)을 그대로 각도로 옮긴 값이라
//   heartbeat(360 count/turn)보다 약 28배 촘촘하다. 캘리브레이션 다항식과
//   중앙 등가각 환산을 거치지 않은 모터축 원시값이지만 부호는 raw count와 같고,
//   현재 calibration은 +count = +바퀴각 = LEFT다. 따라서 각도 증감 방향은 바퀴
//   각도(actual_wheel_angle_rad)와 같다. 부호를 뒤집었던 옛 /ctrl/steer_handle_hires
//   기록과는 반대로 보인다.
//
// 사용 시나리오
//   1. steering_actuator_node를 읽기 전용(hardware_can_enabled:=true,
//      actuator_output_enabled:=false)으로 먼저 띄운다. 이 상태에서는 조회
//      프레임만 나가므로 이 노드의 torque 명령과 충돌하지 않는다.
//        ros2 launch lateral_ctrl steering_actuator.launch.py
//            can_interface:=can0 actuator_output_enabled:=false
//   2. 이 노드를 실행한다.
//        ros2 run lateral_ctrl deadzone_measure_node can0
//   3. /deadzone/start (std_msgs/UInt8, data=1)를 쏘면 자동으로 계단식
//      토크 램프가 시작된다. 각 단계를 step_hold_ms만큼 유지하며 각도
//      변화를 기다리고, 변화가 감지되면 자동으로 정지 + disable된 뒤
//      로그에 "DEADZONE DETECTED: torque cmd = N" 형태로 결과가 찍힌다.
//   4. 필요하면 /deadzone/stop (data=1)으로 언제든 수동 중단할 수 있다.
//   5. ramp_direction을 +1/-1로 바꿔가며(CW/CCW) 양방향 데드존을
//      반복 측정한다.
//
// 안전
//   - 측정용 torque_limit의 기존 raw 상한 8000은 유지한다. 매뉴얼에는 raw
//     8000/10000에서 설정 전류에 따른 5초 정지 예제가 있으나, 이 상한 자체가
//     실제 전류나 지속 운전 안전성을 보장하지는 않는다. raw→A 관계는 별도 검증한다.
//   - torque 명령은 1000ms 이내에 연속 전송되지 않으면 드라이버가 CAN
//     단선으로 판단해 보호 동작에 들어간다(재-enable 필요). 그래서 램프
//     타이머가 활성/비활성 여부와 무관하게 매 틱(ramp_period_ms)마다
//     send_torque()를 호출해 keep-alive 역할을 겸한다. enable 프레임은
//     시작 시 1회만 보내며 별도로 재전송하지 않는다.
// ============================================================================

#include "rclcpp/rclcpp.hpp"
#include "kaiev26_msgs/msg/steering_actuator_status.hpp"
#include "std_msgs/msg/u_int8.hpp"
#include "lateral_ctrl/keya_can_driver.hpp"

#include <rcl_interfaces/msg/set_parameters_result.hpp>

#include <linux/can.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <unistd.h>
#include <fcntl.h>

#include <cstring>
#include <cerrno>
#include <cstdint>
#include <cstdlib>
#include <string>
#include <cmath>
#include <vector>
#include <algorithm>

class DeadzoneMeasureNode : public rclcpp::Node
{
public:
    explicit DeadzoneMeasureNode(const std::string &can_iface = "can0")
        : Node("deadzone_measure_node")
    {
        RCLCPP_INFO(get_logger(), "Deadzone Measure Node Started");

        // ── 파라미터 선언 (런타임에 ros2 param set으로 조정 가능)
        can_iface_ = this->declare_parameter<std::string>("can_iface", can_iface);
        ramp_step_ = this->declare_parameter<int>("ramp_step", 5);
        ramp_period_ms_ = this->declare_parameter<int>("ramp_period_ms", 20);
        step_hold_ms_ = this->declare_parameter<int>("step_hold_ms", 300);
        ramp_direction_ = normalize_direction(this->declare_parameter<int>("ramp_direction", 1));
        torque_limit_ = clamp_torque_limit(this->declare_parameter<int>("torque_limit", 2000));
        auto_start_ = this->declare_parameter<bool>("auto_start", false);
        // 각도와 전류를 한 메시지에서 같이 받는다. 둘이 같은 시점의 값이라
        // 판정 로그의 전류가 그 각도 변화와 짝이 맞는다.
        status_topic_ = this->declare_parameter<std::string>("status_topic", "/steering/status");
        // sweep_mode=true: 첫 움직임에서 멈추지 않고 torque_limit까지 계단식으로 끝까지 올리며
        // 매 스텝 steady-state (torque cmd, 전류, 각도)를 기록 — torque-전류 상관관계식 도출용.
        sweep_mode_ = this->declare_parameter<bool>("sweep_mode", false);
        // start_cmd: 0이 아니라 이 값(절대값, 방향은 ramp_direction 따라감)부터 램프 시작.
        // 특정 torque 구간(예: 106~109)을 긴 step_hold_ms로 다시 훑고 싶을 때, 0부터 다시
        // 올라가는 시간을 아끼기 위함.
        start_cmd_ = std::abs(this->declare_parameter<int>("start_cmd", 0));
        // max_angle_delta: 램프 시작 시점 각도 대비 |변화량|(deg)이 이 값을 넘으면 자동
        // disable — 실차 sweep 중 바퀴가 조향 엔드스톱에 걸린 채 torque만 계속 올라가는
        // 상황(스톨 전류 급증, 링키지 무리)을 막기 위한 안전 장치.
        // TODO: 실차에서 핸들 lock-to-lock 돌려보고 안전 범위(deg) 실측 후 값 대입할 것.
        //       0.0 = 비활성 (벤치처럼 무한 회전이 정상인 환경용).
        max_angle_delta_ = std::abs(this->declare_parameter<double>("max_angle_delta", 0.0));

        RCLCPP_INFO(get_logger(),
                    "params: can_iface=%s ramp_step=%d ramp_period_ms=%d step_hold_ms=%d "
                    "ramp_direction=%+d torque_limit=%d auto_start=%s status_topic=%s "
                    "sweep_mode=%s start_cmd=%d max_angle_delta=%.3f",
                    can_iface_.c_str(), ramp_step_, ramp_period_ms_, step_hold_ms_,
                    ramp_direction_, torque_limit_, auto_start_ ? "true" : "false",
                    status_topic_.c_str(), sweep_mode_ ? "true" : "false",
                    start_cmd_, max_angle_delta_);

        // ── 램프 제어 토픽
        start_sub_ = create_subscription<std_msgs::msg::UInt8>(
            "/deadzone/start", 10,
            std::bind(&DeadzoneMeasureNode::start_callback, this, std::placeholders::_1));

        stop_sub_ = create_subscription<std_msgs::msg::UInt8>(
            "/deadzone/stop", 10,
            std::bind(&DeadzoneMeasureNode::stop_callback, this, std::placeholders::_1));

        // ── 움직임 감지용 모터축 각도와 실측 전류 구독 (항상 최신값만 저장)
        status_sub_ = create_subscription<kaiev26_msgs::msg::SteeringActuatorStatus>(
            status_topic_, 10,
            std::bind(&DeadzoneMeasureNode::status_callback, this, std::placeholders::_1));

        // ── 런타임 파라미터 변경 콜백
        param_cb_handle_ = this->add_on_set_parameters_callback(
            std::bind(&DeadzoneMeasureNode::on_set_parameters, this, std::placeholders::_1));

        // ── CAN 소켓 열기/바인딩 (기존 SteeringControlNode 방식 그대로, non-blocking)
        can_socket_ = socket(PF_CAN, SOCK_RAW, CAN_RAW);
        if (can_socket_ < 0)
        {
            RCLCPP_FATAL(get_logger(), "Failed to open CAN socket");
            rclcpp::shutdown();
            return;
        }

        {
            int flags = fcntl(can_socket_, F_GETFL, 0);
            fcntl(can_socket_, F_SETFL, flags | O_NONBLOCK);
        }

        struct ifreq ifr{};
        std::strncpy(ifr.ifr_name, can_iface_.c_str(), IFNAMSIZ - 1);
        if (ioctl(can_socket_, SIOCGIFINDEX, &ifr) < 0)
        {
            RCLCPP_FATAL(get_logger(), "Failed to get CAN ifindex for %s", can_iface_.c_str());
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
        RCLCPP_INFO(get_logger(), "CAN socket initialized on %s", can_iface_.c_str());

        // 드라이버 Enable (시작 시 1회만; 이후 재전송 없음 — torque 명령이 watchdog 유지)
        enable_motor();

        // auto_start는 첫 유효 각도를 받은 뒤 시작한다. 생성자 시점에는 각도가 없어
        // max_angle_delta 기준각이 0으로 잡히므로 여기서 바로 시작하면 안 된다.
        auto_start_pending_ = auto_start_;

        // ── 램프 타이머 (keep-alive 겸용)
        ramp_timer_ = create_wall_timer(
            std::chrono::milliseconds(ramp_period_ms_),
            std::bind(&DeadzoneMeasureNode::ramp_timer_tick, this));
    }

    ~DeadzoneMeasureNode() override
    {
        disable_motor();
        if (can_socket_ >= 0)
            close(can_socket_);
    }

private:
    // 기존 오프라인 측정용 raw cap. 전류 A 또는 열 보호 한계로 해석하지 않는다.
    static constexpr int kHardTorqueLimit = 8000;

    static int normalize_direction(int v) { return (v >= 0) ? 1 : -1; }

    static int clamp_torque_limit(int v)
    {
        int av = std::abs(v);
        if (av > kHardTorqueLimit)
            av = kHardTorqueLimit;
        return av;
    }

    static double rated_pct(int32_t cmd)
    {
        return (std::fabs(static_cast<double>(cmd)) / 10000.0) * 100.0;
    }

    // ── CAN 전송: Torque control (Keya 데이터시트 4.3.2, subindex 0x01)
    void send_torque(int32_t torque_cmd)
    {
        const auto data = lateral_ctrl::KeyaCanDriver::torquePayload(
            std::clamp(torque_cmd, -kHardTorqueLimit, kHardTorqueLimit));

        struct can_frame frame{};
        frame.can_id = 0x06000001 | CAN_EFF_FLAG;
        frame.can_dlc = 8;
        std::memcpy(frame.data, data.data(), data.size());

        const int sent = write(can_socket_, &frame, sizeof(frame));
        if (sent != (int)sizeof(frame))
        {
            RCLCPP_WARN(get_logger(), "Failed to send torque CAN frame (cmd=%d): %s",
                        torque_cmd, std::strerror(errno));
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

    // ── CAN 전송: Disable
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

    // ── 새 토크 단계 진입: 지금 각도를 기준선으로 잡고 hold 데드라인 갱신
    void arm_step(const rclcpp::Time &now)
    {
        baseline_angle_ = latest_angle_;
        step_deadline_ = now + rclcpp::Duration(std::chrono::milliseconds(step_hold_ms_));
    }

    void start_ramp()
    {
        // 이전 사이클(detect/limit/stop)에서 disable_motor()로 꺼졌을 수 있으니
        // 재시작(restart) 없이도 반복 측정이 가능하도록 매번 다시 enable한다.
        enable_motor();
        current_cmd_ = start_cmd_ * ramp_direction_;
        ramp_active_ = true;
        ramp_start_angle_ = latest_angle_; // max_angle_delta 판정 기준점
        arm_step(this->now());
        if (!has_angle_)
        {
            RCLCPP_WARN(get_logger(),
                        "No valid angle received yet on %s — is steering_actuator_node running "
                        "in read-only mode?",
                        status_topic_.c_str());
        }
        RCLCPP_INFO(get_logger(),
                    "Ramp START (dir=%+d, step=%d, period=%dms, hold=%dms, limit=%d)",
                    ramp_direction_, ramp_step_, ramp_period_ms_, step_hold_ms_, torque_limit_);
    }

    // ── 램프 타이머: 매 틱마다 torque cmd 재전송(watchdog). ramp_active_면
    //    step_hold_ms 경과 시점에 각도 변화를 확인해 판정/다음 단계 진행.
    void ramp_timer_tick()
    {
        if (ramp_active_)
        {
            const rclcpp::Time now = this->now();

            // 각도 한계 안전 중단: 램프 시작 각도 대비 max_angle_delta(deg) 이상 벗어나면
            // 즉시 disable. hold 만료를 기다리지 않고 매 틱 검사한다(엔드스톱 스톨 방지).
            if (max_angle_delta_ > 0.0 && has_angle_ &&
                std::fabs(latest_angle_ - ramp_start_angle_) >= max_angle_delta_)
            {
                ramp_active_ = false;
                disable_motor();
                RCLCPP_ERROR(get_logger(),
                             ">>> ANGLE LIMIT ABORT: |angle %.3f - start %.3f| >= %.3f deg "
                             "at torque cmd = %d <<<",
                             latest_angle_, ramp_start_angle_, max_angle_delta_, current_cmd_);
                return;
            }

            if (now >= step_deadline_)
            {
                if (sweep_mode_)
                {
                    // 방금 step_hold_ms 동안 유지됐던 steady-state 값을 CSV 한 줄로 기록.
                    // delta = 이 스텝 hold 동안의 각도 변화량(deg) — step_hold_ms로 나누면
                    // 해당 torque에 대한 정상상태 각속도 응답이 됨.
                    RCLCPP_WARN(get_logger(),
                                "SWEEP,dir=%+d,cmd=%d,rated_pct=%.2f,current=%.2fA,angle=%.3f,delta=%.3f",
                                ramp_direction_, current_cmd_, rated_pct(current_cmd_),
                                latest_current_, latest_angle_, latest_angle_ - baseline_angle_);
                }
                // 인코더 카운트는 정수라 실제로 안 움직였다면 각도값이 완전히
                // 동일하게 나온다 — 그래서 threshold 없이 순수 변화 여부만 본다.
                // (sweep_mode에서는 첫 움직임에 멈추지 않고 끝까지 스윕한다)
                else if (has_angle_ && latest_angle_ != baseline_angle_)
                {
                    const int32_t detected_cmd = current_cmd_;
                    const float delta = latest_angle_ - baseline_angle_;

                    ramp_active_ = false;
                    disable_motor();

                    RCLCPP_WARN(get_logger(),
                                ">>> DEADZONE DETECTED: torque cmd = %d (%.1f%% of rated), "
                                "measured_current=%.2fA, dir=%+d, angle %.3f° -> %.3f° (Δ=%.3f°) <<<",
                                detected_cmd, rated_pct(detected_cmd), latest_current_,
                                ramp_direction_, baseline_angle_, latest_angle_, delta);
                    send_torque(current_cmd_);
                    return;
                }

                const int step = ramp_step_ * ramp_direction_;
                current_cmd_ += step;

                if (std::abs(current_cmd_) >= torque_limit_)
                {
                    current_cmd_ = (ramp_direction_ >= 0) ? torque_limit_ : -torque_limit_;
                    send_torque(current_cmd_);
                    ramp_active_ = false;
                    disable_motor();
                    if (sweep_mode_)
                    {
                        RCLCPP_WARN(get_logger(), ">>> SWEEP COMPLETE (torque_limit=%d) <<<", torque_limit_);
                    }
                    else
                    {
                        RCLCPP_WARN(get_logger(),
                                    ">>> LIMIT REACHED: torque cmd = %d (%.1f%% of rated), "
                                    "measured_current=%.2fA, dir=%+d "
                                    "— no movement detected up to limit <<<",
                                    current_cmd_, rated_pct(current_cmd_), latest_current_, ramp_direction_);
                    }
                    return;
                }

                RCLCPP_INFO(get_logger(),
                            "step -> torque cmd = %d (%.1f%% of rated), dir=%+d, holding %dms",
                            current_cmd_, rated_pct(current_cmd_), ramp_direction_, step_hold_ms_);
                arm_step(now);
            }
        }

        // 램프 활성/비활성과 무관하게 전송 = 드라이버 watchdog(1000ms) keep-alive
        send_torque(current_cmd_);
    }

    // ── /deadzone/start: 토크 0으로 리셋 후 계단식 램프 시작
    void start_callback(const std_msgs::msg::UInt8::SharedPtr msg)
    {
        if (msg->data != 1)
            return;
        start_ramp();
    }

    // ── /deadzone/stop: 램프 중지 + disable + 수동 측정 결과 로그
    void stop_callback(const std_msgs::msg::UInt8::SharedPtr msg)
    {
        if (msg->data != 1)
            return;

        ramp_active_ = false;
        disable_motor();
        RCLCPP_WARN(get_logger(), ">>> STOPPED at torque cmd = %d <<<", current_cmd_);
    }

    // ── 조향 status 콜백: 최신값만 저장 (판정은 ramp_timer_tick에서 수행)
    //    feedback_valid가 false인 동안은 각도가 갱신되지 않은 묵은 값이라 받지 않는다.
    void status_callback(const kaiev26_msgs::msg::SteeringActuatorStatus::SharedPtr msg)
    {
        latest_current_ = msg->motor_current_a;
        if (!msg->feedback_valid) return;
        latest_angle_ = msg->motor_angle_deg;
        has_angle_ = true;
        if (auto_start_pending_)
        {
            auto_start_pending_ = false;
            RCLCPP_WARN(get_logger(), "auto_start: first valid angle %.3f deg received, starting ramp",
                        latest_angle_);
            start_ramp();
        }
    }

    // ── 런타임 파라미터 변경 콜백
    rcl_interfaces::msg::SetParametersResult on_set_parameters(
        const std::vector<rclcpp::Parameter> &params)
    {
        rcl_interfaces::msg::SetParametersResult result;
        result.successful = true;
        result.reason = "ok";

        for (const auto &p : params)
        {
            if (p.get_name() == "torque_limit")
            {
                const int requested = p.as_int();
                const int clamped = clamp_torque_limit(requested);
                if (clamped != requested)
                {
                    RCLCPP_WARN(get_logger(),
                                "torque_limit clamped to hard max %d (requested %d)",
                                clamped, requested);
                }
                torque_limit_ = clamped;
            }
            else if (p.get_name() == "ramp_direction")
            {
                const int nd = normalize_direction(p.as_int());
                if (nd != ramp_direction_)
                {
                    ramp_direction_ = nd;
                    current_cmd_ = 0; // 방향 전환 시 안전을 위해 토크값 리셋
                    RCLCPP_WARN(get_logger(),
                                "ramp_direction changed to %+d -> torque cmd reset to 0 (safety)",
                                ramp_direction_);
                }
            }
            else if (p.get_name() == "ramp_step")
            {
                ramp_step_ = p.as_int();
            }
            else if (p.get_name() == "ramp_period_ms")
            {
                int v = p.as_int();
                if (v < 1)
                    v = 1;
                ramp_period_ms_ = v;
                if (ramp_timer_)
                    ramp_timer_->cancel();
                ramp_timer_ = create_wall_timer(
                    std::chrono::milliseconds(ramp_period_ms_),
                    std::bind(&DeadzoneMeasureNode::ramp_timer_tick, this));
                RCLCPP_INFO(get_logger(), "ramp_period_ms updated to %d ms (timer restarted)",
                            ramp_period_ms_);
            }
            else if (p.get_name() == "step_hold_ms")
            {
                int v = p.as_int();
                if (v < 1)
                    v = 1;
                step_hold_ms_ = v;
            }
            else if (p.get_name() == "max_angle_delta")
            {
                max_angle_delta_ = std::abs(p.as_double());
                RCLCPP_INFO(get_logger(), "max_angle_delta updated to %.3f deg%s",
                            max_angle_delta_,
                            max_angle_delta_ > 0.0 ? "" : " (disabled)");
            }
        }
        return result;
    }

    // ── CAN
    int can_socket_{-1};
    std::string can_iface_{"can0"};

    // ── 파라미터
    int ramp_step_{5};
    int ramp_period_ms_{20};
    int step_hold_ms_{300};
    int ramp_direction_{1};
    int torque_limit_{2000};
    bool auto_start_{false};
    bool auto_start_pending_{false};
    std::string status_topic_{"/steering/status"};
    bool sweep_mode_{false};
    int start_cmd_{0};
    double max_angle_delta_{0.0}; // 0.0 = 비활성. 실차 lock-to-lock 실측 후 값 대입.

    // ── 램프 상태
    bool ramp_active_{false};
    int32_t current_cmd_{0};
    rclcpp::Time step_deadline_{0, 0, RCL_ROS_TIME};
    float ramp_start_angle_{0.0f}; // max_angle_delta 판정 기준(램프 시작 시점 각도)

    // ── 각도 피드백 상태
    bool has_angle_{false};
    float latest_angle_{0.0f};
    float baseline_angle_{0.0f};

    // ── 전류 피드백 상태
    float latest_current_{0.0f};

    // ── ROS 인터페이스
    rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr start_sub_;
    rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr stop_sub_;
    rclcpp::Subscription<kaiev26_msgs::msg::SteeringActuatorStatus>::SharedPtr status_sub_;
    rclcpp::TimerBase::SharedPtr ramp_timer_;
    rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_cb_handle_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    std::string iface = "can0";
    if (argc > 1)
        iface = argv[1];
    rclcpp::spin(std::make_shared<DeadzoneMeasureNode>(iface));
    rclcpp::shutdown();
    return 0;
}
