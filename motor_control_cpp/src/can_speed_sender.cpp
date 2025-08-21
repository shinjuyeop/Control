#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/int32.hpp>

#include <linux/can.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <unistd.h>
#include <cstring>

class CanSpeedSender : public rclcpp::Node
{
public:
  CanSpeedSender(const std::string& can_iface)
  : Node("can_speed_sender")//, phase_current_a_(180)
  {
    /* ── SocketCAN init ─────────────────────────────────────────────── */
    sock_ = socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if (sock_ < 0) throw std::runtime_error("CAN socket open failed");

    struct ifreq ifr{};
    std::strncpy(ifr.ifr_name, can_iface.c_str(), IFNAMSIZ - 1);
    if (ioctl(sock_, SIOCGIFINDEX, &ifr) < 0) throw std::runtime_error("ifindex");

    sockaddr_can addr{};
    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;
    if (bind(sock_, reinterpret_cast<sockaddr*>(&addr), sizeof(addr)) < 0)
      throw std::runtime_error("bind");

    /* ── RPM subscriber ─────────────────────────────────────────────── */
    rpm_sub_ = create_subscription<std_msgs::msg::Int32>(
      "target_rpm", 10,
      [this](const std_msgs::msg::Int32::SharedPtr msg){ last_rpm_ = msg->data; });

    /* ── 핸드셰이크 쓰레드 ──────────────────────────────────────────── */
    hs_thread_ = std::thread([this]{ handshake_loop(); });

    /* ── 제어 프레임 타이머 (50 ms) ────────────────────────────────── */
    timer_ = create_wall_timer(
      std::chrono::milliseconds(50),
      std::bind(&CanSpeedSender::send_control_frame, this));
  }
  ~CanSpeedSender() override
  {
    running_ = false;
    if (hs_thread_.joinable()) hs_thread_.join();
    if (sock_ >= 0) close(sock_);
  }

private:
  /* ── MCU의 0x55 프레임을 기다렸다가 0xAA 응답 ────────────────────── */
  void handshake_loop()
  {
    can_frame rx{};
    while (running_ && !handshake_done_)
    {
      ssize_t n = read(sock_, &rx, sizeof(rx));
      if (n == sizeof(rx) && (rx.can_id & CAN_EFF_MASK) == 0x1801D0EF)
      {
        bool all55 = true;
        for (uint8_t b : rx.data) if (b != 0x55) { all55 = false; break; }
        if (all55)
        {
          RCLCPP_INFO(get_logger(), "MCU 0x55 detected → sending 0xAA");
          send_aa_frame();
          handshake_done_ = true;
        }
      }
    }
  }
  void send_aa_frame()
  {
    can_frame hs{};
    hs.can_id  = 0x0C01EFD0 | CAN_EFF_FLAG;
    hs.can_dlc = 8;
    std::fill(std::begin(hs.data), std::end(hs.data), 0xAA);
    write(sock_, &hs, sizeof(hs));
  }

  /* ── 제어 프레임 전송 ─────────────────────────────────────────────── */
  void send_control_frame()
  {
    if (!handshake_done_) return;               // 아직 연결 전이면 전송 안 함

    can_frame tx{};
    tx.can_id  = 0x0C01EFD0 | CAN_EFF_FLAG;
    tx.can_dlc = 8;

    //int16_t phase_val = static_cast<int16_t>((phase_current_a_ - (-3200)) / 0.1);
    //int16_t rpm_val   = static_cast<int16_t>(last_rpm_ - (-32000));

    /* ▼ ① ±180 A로 부호 결정 -------------------------------------- */
    const double phase_current_a = (last_rpm_ >= 0 ?  180.0 : -180.0); /* 전진 & 후진 */

    /* ▼ ② 변환식은 그대로 사용 -------------------------------------- */
    int16_t phase_val = static_cast<int16_t>((phase_current_a - (-3200)) / 0.1);
    int16_t rpm_val   = static_cast<int16_t>(last_rpm_ - (-32000));

    tx.data[0] = phase_val & 0xFF;
    tx.data[1] = phase_val >> 8;
    tx.data[2] = rpm_val & 0xFF;
    tx.data[3] = rpm_val >> 8;
    tx.data[4] = 0x03;          // RUNNING + SPEED mode
    tx.data[5] = tx.data[6] = 0x00;
    tx.data[7] = life_++;

    write(sock_, &tx, sizeof(tx));

    static int32_t prev_logged_rpm = INT32_MIN;
    if (last_rpm_ != prev_logged_rpm) {
      prev_logged_rpm = last_rpm_;
      RCLCPP_INFO(get_logger(), "CAN TX rpm=%d phase_val=%d [data:%02X %02X %02X %02X %02X %02X %02X %02X]",
                  last_rpm_, (int)phase_val,
                  tx.data[0], tx.data[1], tx.data[2], tx.data[3],
                  tx.data[4], tx.data[5], tx.data[6], tx.data[7]);
    }
  }

  /* ── members ─────────────────────────────────────────────────────── */
  int sock_{-1};
  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr rpm_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::thread hs_thread_;
  std::atomic<bool> running_{true};
  std::atomic<bool> handshake_done_{false};

  int32_t last_rpm_{0};
  //const double phase_current_a_;
  uint8_t life_{0};
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  std::string iface = "can0";
  if (argc > 1) iface = argv[1];
  try {
    rclcpp::spin(std::make_shared<CanSpeedSender>(iface));
  } catch (const std::exception & e) {
    RCLCPP_FATAL(rclcpp::get_logger("rclcpp"), "Exception: %s", e.what());
  }
  rclcpp::shutdown();
}

