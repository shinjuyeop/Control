#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32.hpp"

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
    // 콤마로 구분된 인터페이스 목록. 기본값: can1,can0 둘 다 수신
    can_ifaces_csv_   = this->declare_parameter<std::string>("can_ifaces", "can1,can0");
    feedback_id_i64_  = this->declare_parameter<int64_t>("feedback_id", 0x07000001);
    is_eff_id_        = this->declare_parameter<bool>("extended_id", true);

    // raw 데이터에서 각도 값 추출 위치/형식
    value_offset_     = this->declare_parameter<int>("value_offset", 0);     // 시작 바이트
    value_bits_       = this->declare_parameter<int>("value_bits", 16);      // 16 or 32
    value_big_endian_ = this->declare_parameter<bool>("big_endian", true);   // big-endian 여부

    // 한 바퀴당 카운트(예: heartbeat ELE-H/ELE-L → 360, TPDO/SDO Position → 10000)
    counts_per_rev_   = this->declare_parameter<int>("counts_per_rev", 360);

    publish_topic_    = this->declare_parameter<std::string>("publish_topic", "/ctrl/steer");
    timer_ms_         = this->declare_parameter<int>("poll_period_ms", 20);

    // raw(좌+ / 우−) → 출력(좌− / 우+) 로 바꾸기 위한 단순 부호 반전 스위치
    invert_direction_ = this->declare_parameter<bool>("invert_direction", true);

    feedback_id_ = static_cast<uint32_t>(feedback_id_i64_);
    encoder_pub_ = this->create_publisher<std_msgs::msg::Float32>(publish_topic_, 10);

    // ---- Open sockets for all interfaces ------------------------------
    auto ifaces = split_csv(can_ifaces_csv_);
    for (const auto& iface : ifaces) {
      int fd = open_can_socket(iface);
      if (fd >= 0) {
        sockets_.push_back({iface, fd});
      } else {
        RCLCPP_WARN(get_logger(), "Skip iface=%s (open failed)", iface.c_str());
      }
    }

    if (sockets_.empty()) {
      RCLCPP_FATAL(get_logger(), "No CAN sockets opened. Check interfaces: %s", can_ifaces_csv_.c_str());
      rclcpp::shutdown();
      return;
    }

    std::ostringstream oss;
    for (size_t i = 0; i < sockets_.size(); ++i) {
      if (i) oss << ", ";
      oss << sockets_[i].name;
    }
    RCLCPP_INFO(get_logger(),
      "Steering Feedback Node (ifaces=[%s], id=0x%08X %s, offset=%d, bits=%d, %s-endian, CPR=%d, period=%dms, publish='%s', invert=%s)",
      oss.str().c_str(), feedback_id_, (is_eff_id_ ? "EFF" : "SFF"),
      value_offset_, value_bits_, (value_big_endian_ ? "big" : "little"),
      counts_per_rev_, timer_ms_, publish_topic_.c_str(),
      (invert_direction_ ? "yes" : "no"));

    // ---- Timer: drain all sockets ------------------------------------
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
    std::vector<std::string> out;
    std::string cur;
    for (char c : s) {
      if (c == ',') {
        if (!cur.empty()) { trim(cur); out.push_back(cur); cur.clear(); }
      } else cur.push_back(c);
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

    // filter only desired ID
    struct can_filter filter {};
    filter.can_id   = is_eff_id_ ? (feedback_id_ & CAN_EFF_MASK) : (feedback_id_ & CAN_SFF_MASK);
    filter.can_mask = is_eff_id_ ? CAN_EFF_MASK : CAN_SFF_MASK;
    if (setsockopt(fd, SOL_CAN_RAW, CAN_RAW_FILTER, &filter, sizeof(filter)) < 0) {
      RCLCPP_WARN(get_logger(), "[%s] setsockopt(FILTER) failed: %s", iface.c_str(), std::strerror(errno));
      // 계속 진행
    }

    // non-blocking
    int flags = fcntl(fd, F_GETFL, 0);
    fcntl(fd, F_SETFL, flags | O_NONBLOCK);

    // bind
    struct ifreq ifr {};
    std::strncpy(ifr.ifr_name, iface.c_str(), IFNAMSIZ - 1);
    if (ioctl(fd, SIOCGIFINDEX, &ifr) < 0) {
      RCLCPP_WARN(get_logger(), "[%s] ioctl(SIOCGIFINDEX) failed: %s", iface.c_str(), std::strerror(errno));
      close(fd);
      return -1;
    }
    struct sockaddr_can addr {};
    addr.can_family  = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;
    if (bind(fd, reinterpret_cast<struct sockaddr*>(&addr), sizeof(addr)) < 0) {
      RCLCPP_WARN(get_logger(), "[%s] bind() failed: %s", iface.c_str(), std::strerror(errno));
      close(fd);
      return -1;
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

  // ── 핵심: raw(좌+ / 우−, 0 유지)를 출력(좌− / 우+)로 변환 ───────────────
  void publish_angle_from_counts(int32_t counts) {
    if (counts_per_rev_ == 0) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
        "counts_per_rev is 0; check parameter.");
      return;
    }

    // 1) raw counts → raw 핸들각(deg) : 영점 보정/랩핑 없음 (raw 0은 0으로 유지)
    const float x_raw = (static_cast<float>(counts) / static_cast<float>(counts_per_rev_)) * 360.0f;
    //    raw 규약: 좌 +, 우 −

    // 2) 요구 규약: 좌 −, 우 +  → 부호만 반전 (필요 시 파라미터로 끌 수 있음)
    const float x = invert_direction_ ? -x_raw : x_raw;

    // 3) 오른쪽(+x)에서 피팅된 다항식 f(t) = a*t^2 + b*t + c  (t = |x|)
    const float t = std::fabs(x);
    const float f = 0.0002f * t * t + 0.2159f * t + 0.1612f;

    // 4) 최종 바퀴각(좌는 음수, 우는 양수): 부호는 x를 따름
    const float wheel_deg = std::copysign(f, x);

    // 5) 퍼블리시
    std_msgs::msg::Float32 msg;
    msg.data = wheel_deg;
    encoder_pub_->publish(msg);

    // 6) 로그: 변환 전/후 부호와 방향을 명확히 표기
    const char* side_raw = (x_raw > 0.f) ? "RAW:LEFT(+)" : (x_raw < 0.f ? "RAW:RIGHT(-)" : "RAW:CENTER");
    const char* side_out = (x     > 0.f) ? "OUT:RIGHT(+)" : (x     < 0.f ? "OUT:LEFT(-)"  : "OUT:CENTER");
    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 500,
      "counts=%d  raw_handle=%.2f° [%s]  -> conv_handle=%.2f° [%s]  -> wheel=%.2f°",
      counts, x_raw, side_raw, x, side_out, wheel_deg);
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
      publish_angle_from_counts(counts);
    }
  }

  void drain_all() {
    for (auto& s : sockets_) {
      drain_fd(s.fd, s.name.c_str());
    }
  }

private:
  // multiple sockets
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

  bool     invert_direction_;  // raw 좌+ / 우− → 출력 좌− / 우+ 로 뒤집기

  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr encoder_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SteeringFeedbackNode>());
  rclcpp::shutdown();
  return 0;
}
