#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/u_int8.hpp"

#include <linux/can.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <unistd.h>
#include <fcntl.h>

#include <cstring>
#include <string>
#include <vector>
#include <sstream>
#include <iomanip>
#include <cmath>

class SteeringFeedbackNode : public rclcpp::Node {
public:
  SteeringFeedbackNode()
  : Node("steering_feedback_node")
  {
    // ---- Parameters ---------------------------------------------------
    can_ifaces_csv_   = this->declare_parameter<std::string>("can_ifaces", "can1,can0");
    // Keya 데이터시트 4.3.2: Heartbeat ID = 0x07000000 + driver address
    feedback_id_i64_  = this->declare_parameter<int64_t>("feedback_id", 0x07000001);
    is_eff_id_        = this->declare_parameter<bool>("extended_id", true);

    value_offset_     = this->declare_parameter<int>("value_offset", 0);     // 시작 바이트
    value_bits_       = this->declare_parameter<int>("value_bits", 16);      // 16 or 32
    value_big_endian_ = this->declare_parameter<bool>("big_endian", true);   // big-endian 여부

    // 한 바퀴당 카운트(heartbeat ELE-H/ELE-L → 360, SDO Position query → 10000)
    counts_per_rev_   = this->declare_parameter<int>("counts_per_rev", 360);

    publish_topic_    = this->declare_parameter<std::string>("publish_topic", "/ctrl/steer");
    timer_ms_         = this->declare_parameter<int>("poll_period_ms", 20);

    // 미세 최적화: 스케일 사전 계산
    scale_deg_per_count_ = (counts_per_rev_ > 0)
      ? (360.0f / static_cast<float>(counts_per_rev_)) : 0.0f;

    feedback_id_ = static_cast<uint32_t>(feedback_id_i64_);
    encoder_pub_ = this->create_publisher<std_msgs::msg::Float32>(publish_topic_, 10);

    // heartbeat 프레임(8B)에 위치 외에 speed/current/fault가 같이 실려오므로 추가 트래픽 없이 퍼블리시
    speed_pub_   = this->create_publisher<std_msgs::msg::Float32>("/ctrl/steer_motor_rpm", 10);
    current_pub_ = this->create_publisher<std_msgs::msg::Float32>("/ctrl/steer_motor_current", 10);
    fault_pub_   = this->create_publisher<std_msgs::msg::UInt8>("/ctrl/steer_fault", 10);

    // ---- Open sockets -------------------------------------------------
    auto ifaces = split_csv(can_ifaces_csv_);
    for (const auto& iface : ifaces) {
      int fd = open_can_socket(iface);
      if (fd >= 0) sockets_.push_back({iface, fd});
      else RCLCPP_WARN(get_logger(), "Skip iface=%s (open failed)", iface.c_str());
    }

    if (sockets_.empty()) {
      RCLCPP_FATAL(get_logger(), "No CAN sockets opened. Check interfaces: %s", can_ifaces_csv_.c_str());
      rclcpp::shutdown();
      return;
    }

    std::ostringstream oss;
    for (size_t i = 0; i < sockets_.size(); ++i) { if (i) oss << ", "; oss << sockets_[i].name; }
    RCLCPP_INFO(get_logger(),
      "Steering Feedback Node (ifaces=[%s], id=0x%08X %s, offset=%d, bits=%d, %s-endian, CPR=%d, period=%dms, scale=%.6f deg/count)",
      oss.str().c_str(), feedback_id_, (is_eff_id_ ? "EFF" : "SFF"),
      value_offset_, value_bits_, (value_big_endian_ ? "big" : "little"),
      counts_per_rev_, timer_ms_, scale_deg_per_count_);

    // ---- Timer --------------------------------------------------------
    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(timer_ms_),
      std::bind(&SteeringFeedbackNode::drain_all, this)
    );
  }

  ~SteeringFeedbackNode() override {
    for (auto& s : sockets_) if (s.fd >= 0) close(s.fd);
  }

private:
  struct Sock { std::string name; int fd; };

  // ── utils ────────────────────────────────────────────────────────────
  static std::vector<std::string> split_csv(const std::string& s) {
    std::vector<std::string> out; std::string cur;
    for (char c : s) {
      if (c == ',') { if (!cur.empty()) { trim(cur); out.push_back(cur); cur.clear(); } }
      else cur.push_back(c);
    }
    if (!cur.empty()) { trim(cur); out.push_back(cur); }
    return out;
  }

  static void trim(std::string& x) {
    size_t a = x.find_first_not_of(" \t");
    size_t b = x.find_last_not_of(" \t");
    x = (a==std::string::npos) ? "" : x.substr(a, b-a+1);
  }

  static std::string bytes_hex(const uint8_t* data, int len) {
    std::ostringstream oss;
    for (int i = 0; i < len; ++i) {
      oss << std::hex << std::uppercase << std::setw(2) << std::setfill('0')
          << static_cast<int>(data[i]);
      if (i + 1 < len) oss << ' ';
    }
    return oss.str();
  }

  int open_can_socket(const std::string& iface) {
    int fd = socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if (fd < 0) {
      RCLCPP_WARN(get_logger(), "[%s] socket() failed: %s", iface.c_str(), std::strerror(errno));
      return -1;
    }

    struct can_filter filter {};
    filter.can_id   = is_eff_id_ ? (feedback_id_ & CAN_EFF_MASK) : (feedback_id_ & CAN_SFF_MASK);
    filter.can_mask = is_eff_id_ ? CAN_EFF_MASK : CAN_SFF_MASK;
    if (setsockopt(fd, SOL_CAN_RAW, CAN_RAW_FILTER, &filter, sizeof(filter)) < 0) {
      RCLCPP_WARN(get_logger(), "[%s] setsockopt(FILTER) failed: %s", iface.c_str(), std::strerror(errno));
    }

    int flags = fcntl(fd, F_GETFL, 0);
    fcntl(fd, F_SETFL, flags | O_NONBLOCK);

    struct ifreq ifr {};
    std::strncpy(ifr.ifr_name, iface.c_str(), IFNAMSIZ - 1);
    if (ioctl(fd, SIOCGIFINDEX, &ifr) < 0) {
      RCLCPP_WARN(get_logger(), "[%s] ioctl(SIOCGIFINDEX) failed: %s", iface.c_str(), std::strerror(errno));
      close(fd); return -1;
    }
    struct sockaddr_can addr {};
    addr.can_family  = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;
    if (bind(fd, reinterpret_cast<struct sockaddr*>(&addr), sizeof(addr)) < 0) {
      RCLCPP_WARN(get_logger(), "[%s] bind() failed: %s", iface.c_str(), std::strerror(errno));
      close(fd); return -1;
    }
    return fd;
  }

  bool parse_value(const can_frame& frame, int32_t& out_counts) {
    if (value_bits_ != 16 && value_bits_ != 32) return false;
    if (value_offset_ < 0 || value_offset_ >= frame.can_dlc) return false;
    if (value_offset_ + (value_bits_ / 8) > frame.can_dlc) return false;

    if (value_bits_ == 16) {
      uint16_t u = 0;
      if (value_big_endian_) {
        u = (static_cast<uint16_t>(frame.data[value_offset_]) << 8)
          |  static_cast<uint16_t>(frame.data[value_offset_ + 1]);
      } else {
        u = (static_cast<uint16_t>(frame.data[value_offset_ + 1]) << 8)
          |  static_cast<uint16_t>(frame.data[value_offset_]);
      }
      out_counts = static_cast<int16_t>(u);
      return true;
    } else {
      uint32_t u = 0;
      if (value_big_endian_) {
        u = (static_cast<uint32_t>(frame.data[value_offset_    ]) << 24)
          | (static_cast<uint32_t>(frame.data[value_offset_ + 1]) << 16)
          | (static_cast<uint32_t>(frame.data[value_offset_ + 2]) << 8)
          |  static_cast<uint32_t>(frame.data[value_offset_ + 3]);
      } else {
        u = (static_cast<uint32_t>(frame.data[value_offset_ + 3]) << 24)
          | (static_cast<uint32_t>(frame.data[value_offset_ + 2]) << 16)
          | (static_cast<uint32_t>(frame.data[value_offset_ + 1]) << 8)
          |  static_cast<uint32_t>(frame.data[value_offset_    ]);
      }
      out_counts = static_cast<int32_t>(u);
      return true;
    }
  }

  // 오른쪽(+x)에서 피팅된 다항식 f(t) = a*t^2 + b*t + c (Horner)
  static inline float poly_right(float t) {
    return (0.0002f * t + 0.2159f) * t + 0.1612f;
  }

  // ---- 핵심 로직: 1) raw 부호 반전 → x, 2) 원점대칭 스케일링, 3) 퍼블리시/로그
  void publish_angle_from_counts(int32_t raw_counts) {
    if (scale_deg_per_count_ == 0.0f) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "counts_per_rev invalid.");
      return;
    }

    // raw 규약: 좌 +, 우 −, 0은 센터(고정)
    const float x_raw = static_cast<float>(raw_counts) * scale_deg_per_count_;

    // (1) 부호 반전: 요구 규약(좌 −, 우 +) 만들기
    const float x = -x_raw;

    // (2) 원점대칭 스케일링: y = sign(x) * f(|x|)
    const float t = std::fabs(x);
    const float f = poly_right(t);
    const float wheel_deg = std::copysign(f, x);

    // (3) 퍼블리시/로그
    std_msgs::msg::Float32 msg; msg.data = wheel_deg; encoder_pub_->publish(msg);

    const char* side_raw = (x_raw > 0.f) ? "RAW:LEFT(+)" : (x_raw < 0.f ? "RAW:RIGHT(-)" : "RAW:CENTER");
    const char* side_out = (x     > 0.f) ? "OUT:RIGHT(+)" : (x     < 0.f ? "OUT:LEFT(-)"  : "OUT:CENTER");
    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 500,
      "raw_counts=%d  raw_handle=%.2f° [%s]  x=%.2f° [%s]  -> wheel=%.2f°",
      raw_counts, x_raw, side_raw, x, side_out, wheel_deg);
  }

  // heartbeat 프레임(8B): [ELE-H ELE-L][speed_H speed_L][DATA_H DATA_L][Err_H Err_L]
  // 위치(ELE-H/L)는 publish_angle_from_counts에서 이미 처리하므로 나머지 3필드만 추출
  //
  // 에러코드(데이터시트 4.2.1 LED 표): 0=Enable/정상 1=Disable 2=과전압 3=HW과전류 4=EEPROM오류
  // 5=저전압 6=예비 7=SW과전류 8=제어모드오류 9=상결선단선 10=스톨 11=온도경고
  // 12=Hall고장 13=전류센서고장 14=예비 15=CAN단선 16=RS232단선
  void publish_heartbeat_telemetry(const can_frame& frame) {
    if (frame.can_dlc < 8) return;

    const int16_t speed_rpm = static_cast<int16_t>(
      (static_cast<uint16_t>(frame.data[2]) << 8) | frame.data[3]);
    const int16_t current_a = static_cast<int16_t>(
      (static_cast<uint16_t>(frame.data[4]) << 8) | frame.data[5]);
    const uint16_t fault_code =
      (static_cast<uint16_t>(frame.data[6]) << 8) | frame.data[7];

    std_msgs::msg::Float32 speed_msg; speed_msg.data = static_cast<float>(speed_rpm);
    speed_pub_->publish(speed_msg);

    std_msgs::msg::Float32 current_msg; current_msg.data = static_cast<float>(current_a);
    current_pub_->publish(current_msg);

    std_msgs::msg::UInt8 fault_msg; fault_msg.data = static_cast<uint8_t>(fault_code & 0xFF);
    fault_pub_->publish(fault_msg);

    // 0=Enable/정상, 1=Disable/정상(비활성) 은 실제 폴트가 아니므로 2 이상만 경고
    if (fault_code >= 2) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
        "Keya fault code=%u (0=enable/OK, 1=disable, 2=over-volt, 3=HW over-current, "
        "5=under-volt, 7=SW over-current, 9=phase fail, 10=stall, 11=over-temp, "
        "12=hall fault, 15=CAN lost — see datasheet 4.2.1)", fault_code);
    }
  }

  void drain_fd(int fd, const char* ifname) {
    while (true) {
      struct can_frame frame;
      int n = read(fd, &frame, sizeof(frame));
      if (n < 0) {
        if (errno == EAGAIN || errno == EWOULDBLOCK) break;
        RCLCPP_WARN(get_logger(), "[%s] CAN read error: %s", ifname, std::strerror(errno));
        break;
      }
      if (n != sizeof(frame)) {
        RCLCPP_WARN(get_logger(), "[%s] Short CAN frame read (%d bytes)", ifname, n);
        continue;
      }

      const bool eff = (frame.can_id & CAN_EFF_FLAG) != 0;
      const uint32_t id = eff ? (frame.can_id & CAN_EFF_MASK) : (frame.can_id & CAN_SFF_MASK);
      if (id != feedback_id_) {
        RCLCPP_DEBUG(get_logger(), "[%s] Other id=0x%08X%s dlc=%d data=[%s]",
                     ifname, id, eff ? " (EFF)" : " (SFF)", frame.can_dlc,
                     bytes_hex(frame.data, frame.can_dlc).c_str());
        continue;
      }

      int32_t counts = 0;
      if (!parse_value(frame, counts)) {
        RCLCPP_WARN(get_logger(), "[%s] Parse fail (id=0x%08X dlc=%d data=[%s])",
                    ifname, id, frame.can_dlc, bytes_hex(frame.data, frame.can_dlc).c_str());
        continue;
      }

      // 시작 디버그: 처음 몇 프레임은 원시 바이트/파싱값을 그대로 보여줌
      if (startup_debug_left_-- > 0) {
        const float deg_raw = static_cast<float>(counts) * scale_deg_per_count_;
        RCLCPP_INFO(get_logger(),
          "[STARTUP] id=0x%08X dlc=%d data=[%s] -> counts=%d (%.2f deg)",
          id, frame.can_dlc, bytes_hex(frame.data, frame.can_dlc).c_str(),
          counts, deg_raw);
      }

      publish_angle_from_counts(counts);
      publish_heartbeat_telemetry(frame);
    }
  }

  void drain_all() {
    for (auto& s : sockets_) drain_fd(s.fd, s.name.c_str());
  }

private:
  // sockets
  std::vector<Sock> sockets_;

  // params / state
  std::string can_ifaces_csv_;
  int64_t  feedback_id_i64_;
  uint32_t feedback_id_;
  bool     is_eff_id_;
  int      value_offset_;
  int      value_bits_;
  bool     value_big_endian_;
  int      counts_per_rev_;
  std::string publish_topic_;
  int      timer_ms_;

  float    scale_deg_per_count_{0.0f};
  int      startup_debug_left_{12}; // 시작 시 12프레임 자세히 덤프

  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr encoder_pub_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr speed_pub_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr current_pub_;
  rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr fault_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SteeringFeedbackNode>());
  rclcpp::shutdown();
  return 0;
}
