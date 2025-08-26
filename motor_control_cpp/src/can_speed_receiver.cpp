#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/int32.hpp>
#include <std_msgs/msg/float32.hpp>

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
#include <string>
#include <vector>
#include <iostream>
#include <iomanip>

static constexpr canid_t ID_MCU_PART1 = 0x1801D0EF; // BusV / BusI / PhaseI / Speed
static constexpr canid_t ID_MCU_PART2 = 0x1802D0EF; // Temps / Status / Error / Faults
static constexpr int kRPM_OFFSET   = 32000;  // rpm 오프셋(카운트)
static constexpr int kCURR_OFFSET  = 32000;  // 전류 오프셋(카운트, = 300.0 A)

// Arduino와 동일 파라미터(바퀴 직경/감속비)를 사용해 m/s로 변환
static constexpr float kTireDiamCm = 47.0f;   // cm
static constexpr float kGearRatio  = 4.60f;   // motor rpm / gear ratio = wheel rpm

inline int16_t  le16 (uint8_t lo, uint8_t hi){ return int16_t(uint16_t(lo) | (uint16_t(hi)<<8)); }
inline uint16_t le16u(uint8_t lo, uint8_t hi){ return        (uint16_t(lo) | (uint16_t(hi)<<8)); }

static std::string bitsToList(uint8_t v, const std::vector<const char*>& names) {
  std::string s;
  for (int b=0; b<8 && b<(int)names.size(); ++b) {
    if (v & (1u<<b)) {
      if (!s.empty()) s += ", ";
      s += names[b];
    }
  }
  return s.empty() ? "None" : s;
}

class CanSpeedReceiver : public rclcpp::Node
{
public:
  CanSpeedReceiver(const std::string & can_iface) : Node("can_speed_receiver")
  {
    /* ── SocketCAN 초기화 ─────────────────────────────────────────── */
    sock_ = socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if (sock_ < 0) throw std::runtime_error("CAN socket open failed");

    struct ifreq ifr{};
    std::strncpy(ifr.ifr_name, can_iface.c_str(), IFNAMSIZ - 1);
    if (ioctl(sock_, SIOCGIFINDEX, &ifr) < 0) throw std::runtime_error("ioctl(SIOCGIFINDEX) failed");

    sockaddr_can addr{};
    addr.can_family  = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;
    if (bind(sock_, reinterpret_cast<sockaddr*>(&addr), sizeof(addr)) < 0)
      throw std::runtime_error("bind failed");

    /* (옵션) 커널 필터 장착: Part1/Part2만 수신 */
    struct can_filter flt[2];
    flt[0].can_id   = ID_MCU_PART1 | CAN_EFF_FLAG;
    flt[0].can_mask = (CAN_EFF_MASK | CAN_EFF_FLAG);
    flt[1].can_id   = ID_MCU_PART2 | CAN_EFF_FLAG;
    flt[1].can_mask = (CAN_EFF_MASK | CAN_EFF_FLAG);

    setsockopt(sock_, SOL_CAN_RAW, CAN_RAW_FILTER, &flt, sizeof(flt));
    /* ── 퍼블리셔 생성 (/current_speed/control in m/s) ───────────── */
    speed_pub_ = create_publisher<std_msgs::msg::Float32>("/ctrl/speed", 10);

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

    // 에러/결함 비트 이름표 (PDF 명세)
    static const std::vector<const char*> kErrorNames = {
      "Overcurrent","Overload","Overvoltage","Undervoltage",
      "Controller Overheat","Motor Overheat","Motor Stalled","Motor Out of phase"
    };
    static const std::vector<const char*> kFault1Names = {
      "Motor Sensor","Motor AUX Sensor","Encoder Misaligned","Anti-Runaway Engaged",
      "Main Accelerator","AUX Accelerator","Pre-charge","DC Contactor"
    };
    static const std::vector<const char*> kFault2Names = {
      "Power valve","Current Sensor","Auto-tune","RS485",
      "CAN","Software","Reserved6","Reserved7"
    };

    while (running_)
    {
      ssize_t n = read(sock_, &frame, sizeof(frame));
      if (n != sizeof(frame))                 continue;        // 길이 오류

      canid_t id = frame.can_id;
      if (id & CAN_EFF_FLAG) id &= CAN_EFF_MASK;
      
      // 0x55 × 8 핸드셰이크 프레임은 무시 (Part1 ID에서 올 수 있음)
      if (id == ID_MCU_PART1) {
        bool all55 = std::all_of(std::begin(frame.data), std::end(frame.data),
                                 [](uint8_t b){ return b == 0x55; });
        if (all55) continue;
      }
      if (id == ID_MCU_PART1 && frame.can_dlc == 8) {
        // Part I: 0-1:BusV(0.1V/bit,u16), 2-3:BusI(0.1A/bit,i16), 4-5:PhaseI(0.1A/bit,i16), 6-7:Speed(i16,1rpm/bit)
        // 원시 카운트
        uint16_t busV_u16 = le16u(frame.data[0], frame.data[1]);  // 0.1 V/ct, 오프셋 없음
        uint16_t busI_u16 = le16u(frame.data[2], frame.data[3]);  // 0.1 A/ct, +3000 오프셋
        uint16_t phaI_u16 = le16u(frame.data[4], frame.data[5]);  // 0.1 A/ct, +3000 오프셋
        uint16_t rpm_u16  = le16u(frame.data[6], frame.data[7]);  // 1 rpm/ct, +32000 오프셋

        // 오프셋 제거 (부호 고려 위해 int로 변환 후 보정)
        int busI_ct = static_cast<int>(busI_u16) - kCURR_OFFSET;   // => 카운트
        int phaI_ct = static_cast<int>(phaI_u16) - kCURR_OFFSET;   // => 카운트
        int rpm_ct  = static_cast<int>(rpm_u16)  - kRPM_OFFSET;    // => 카운트

        // 물리량으로 변환
        float busV = busV_u16 * 0.1f;             // V
        float busI = static_cast<float>(busI_ct) * 0.1f; // A
        float phaI = static_cast<float>(phaI_ct) * 0.1f; // A
        int   rpm  = rpm_ct;                       // motor rpm

        // m/s로 변환: wheel_rpm = motor_rpm / kGearRatio
        // v [m/s] = wheel_rpm [rev/min] * (pi * D_m) / 60
        const float D_m        = kTireDiamCm * 0.01f;
        const float wheel_rpm  = 2.0f * static_cast<float>(rpm) / kGearRatio;
        const float speed_mps  = wheel_rpm * (3.1415926f * D_m) / 60.0f;

        // Publish m/s
        std_msgs::msg::Float32 msg; msg.data = speed_mps; speed_pub_->publish(msg);
        // 터미널 출력 (쓰로틀링: 500ms)
        RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 500,
          "[PART1] V=%.1fV, I=%.1fA, Ph=%.1fA, RPM=%d, Speed=%.3f m/s",
          busV, busI, phaI, rpm, speed_mps);
      }
      else if (id == ID_MCU_PART2 && frame.can_dlc == 8) {
        // Part II: 0:CtrlTemp, 1:MotorTemp (°C = raw-40), 2:Status(bit0 RUN, bit1 SpeedMode), 3:Error, 4:Fault1, 5:Fault2
        int      ctrlTemp   = int(frame.data[0]) - 40;
        int      motorTemp  = int(frame.data[1]) - 40;
        uint8_t  status     = frame.data[2];
        uint8_t  error_bits = frame.data[3];
        uint8_t  fault1     = frame.data[4];
        uint8_t  fault2     = frame.data[5];

        bool running   = (status & 0x01) != 0;
        bool speedMode = (status & 0x02) != 0;

        // 기본 상태 로그 (쓰로틀링: 500ms)
        RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 500,
          "[PART2] Ctrl=%dC, Motor=%dC, RUN=%s, SpeedMode=%s, ERR=0x%02X, F1=0x%02X, F2=0x%02X",
          ctrlTemp, motorTemp,
          running ? "YES":"NO",
          speedMode ? "YES":"NO",
          error_bits, fault1, fault2);

        // 에러/결함 비트가 있으면 상세 목록 출력 (즉시 1회)
        if (error_bits) {
          auto list = bitsToList(error_bits, kErrorNames);
          RCLCPP_WARN(get_logger(), "  Error:  %s", list.c_str());
        }
        if (fault1) {
          auto list = bitsToList(fault1, kFault1Names);
          RCLCPP_WARN(get_logger(), "  Fault1: %s", list.c_str());
        }
        if (fault2) {
          auto list = bitsToList(fault2, kFault2Names);
          RCLCPP_WARN(get_logger(), "  Fault2: %s", list.c_str());
        }
      }
    }
  }
  /* ── members ──────────────────────────────────────────────────── */
  int sock_{-1};
  std::thread                               rx_thread_;
  std::atomic<bool>                         running_{true};
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr speed_pub_;  // m/s publisher
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  std::string iface = "can0";
  if (argc > 1) iface = argv[1];
  try {
    rclcpp::spin(std::make_shared<CanSpeedReceiver>(iface));
  } catch (const std::exception & e) {
    RCLCPP_FATAL(rclcpp::get_logger("rclcpp"), "Exception: %s", e.what());
  }
  rclcpp::shutdown();
  return 0;
}
