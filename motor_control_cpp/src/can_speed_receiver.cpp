#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/int32.hpp>          // ⬅️ 추가

#include <linux/can.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <unistd.h>

#include <cstring>
#include <thread>
#include <atomic>
#include <algorithm>

class CanSpeedReceiver : public rclcpp::Node
{
public:
  CanSpeedReceiver() : Node("can_speed_receiver")
  {
    /* ── SocketCAN 초기화 ─────────────────────────────────────────── */
    sock_ = socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if (sock_ < 0)
      throw std::runtime_error("CAN socket open failed");

    struct ifreq ifr{};
    std::strncpy(ifr.ifr_name, "can0", IFNAMSIZ - 1);
    if (ioctl(sock_, SIOCGIFINDEX, &ifr) < 0)
      throw std::runtime_error("ioctl(SIOCGIFINDEX) failed");

    sockaddr_can addr{};
    addr.can_family  = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;
    if (bind(sock_, reinterpret_cast<sockaddr*>(&addr), sizeof(addr)) < 0)
      throw std::runtime_error("bind failed");

    /* ── 퍼블리셔 생성 (/current_speed) ──────────────────────────── */
    speed_pub_ = create_publisher<std_msgs::msg::Int32>("/current_speed", 10);

    /* ── 수신 쓰레드 시작 ─────────────────────────────────────────── */
    rx_thread_ = std::thread([this]{ receive_loop(); });
  }

  ~CanSpeedReceiver() override
  {
    running_ = false;
    if (rx_thread_.joinable()) rx_thread_.join();
    if (sock_ >= 0) close(sock_);
  }

private:
  /* ── CAN 수신 루프 ─────────────────────────────────────────────── */
  void receive_loop()
  {
    can_frame frame{};
    const uint32_t EXPECT_ID = 0x1801D0EF;

    while (running_)
    {
      ssize_t n = read(sock_, &frame, sizeof(frame));
      if (n != sizeof(frame))                 continue;        // 길이 오류
      if ((frame.can_id & CAN_EFF_MASK) != EXPECT_ID) continue; // 다른 ID

      // 0x55 × 8 핸드셰이크 프레임 무시
      bool all55 = std::all_of(std::begin(frame.data), std::end(frame.data),
                               [](uint8_t b){ return b == 0x55; });
      if (all55) continue;

      // 속도 해석 (1 rpm/bit, offset –32000) — 바이트 [6]=low, [7]=high
      int16_t rpm_raw = static_cast<int16_t>((frame.data[7] << 8) | frame.data[6]);
      int16_t rpm     = rpm_raw - 32000;

      // 로그(디버깅용)
      RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 500 /*ms*/,
                           "MCU feedback RPM: %d", rpm);

      // ⬇️ 여기서 퍼블리시
      std_msgs::msg::Int32 msg;
      msg.data = rpm;
      speed_pub_->publish(msg);
    }
  }

  /* ── members ──────────────────────────────────────────────────── */
  int sock_{-1};
  std::thread                               rx_thread_;
  std::atomic<bool>                         running_{true};
  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr speed_pub_;  // ⬅️ 추가
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<CanSpeedReceiver>());
  } catch (const std::exception & e) {
    RCLCPP_FATAL(rclcpp::get_logger("rclcpp"), "Exception: %s", e.what());
  }
  rclcpp::shutdown();
  return 0;
}
