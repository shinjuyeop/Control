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
#include <algorithm>  // std::clamp

class SteeringFeedbackNode : public rclcpp::Node {
public:
  SteeringFeedbackNode()
  : Node("steering_feedback_node")
  {
    // ---- Parameters ---------------------------------------------------
    can_ifaces_csv_   = this->declare_parameter<std::string>("can_ifaces", "can1,can0");
    feedback_id_i64_  = this->declare_parameter<int64_t>("feedback_id", 0x07000001);
    is_eff_id_        = this->declare_parameter<bool>("extended_id", true);
    value_offset_     = this->declare_parameter<int>("value_offset", 0);     // 데이터 시작 바이트
    value_bits_       = this->declare_parameter<int>("value_bits", 16);      // 16 or 32
    value_big_endian_ = this->declare_parameter<bool>("big_endian", true);
    counts_per_rev_   = this->declare_parameter<int>("counts_per_rev", 360); // 360 또는 10000 등
    publish_topic_    = this->declare_parameter<std::string>("publish_topic", "/ctrl/steer");
    timer_ms_         = this->declare_parameter<int>("poll_period_ms", 20);

    // 새 파라미터(부호/영점/클램프)
    auto_zero_on_start_ = this->declare_parameter<bool>("auto_zero_on_start", true);
    zero_counts_        = this->declare_parameter<int>("zero_counts", 0);
    invert_direction_   = this->declare_parameter<bool>("invert_direction", true);
    //  ↑ raw가 "좌=양, 우=음"이면 true로 두면 "좌=음, 우=양"으로 뒤집힘
    use_clamp_          = this->declare_parameter<bool>("use_clamp", false);
    left_limit_deg_     = this->declare_parameter<double>("left_limit_deg",  -46.0); // 좌측 끝 (음수)
    right_limit_deg_    = this->declare_parameter<double>("right_limit_deg", +43.0); // 우측 끝 (양수)

    feedback_id_ = static_cast<uint32_t>(feedback_id_i64_);
    encoder_pub_ = this->create_publisher<std_msgs::msg::Float32>(publish_topic_, 10);

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
      "Steering Feedback Node (ifaces=[%s], id=0x%08X %s, offset=%d, bits=%d, %s-endian, CPR=%d, period=%dms, publish='%s', invert=%s, clamp=%s [L=%.1f°, R=%.1f°])",
      oss.str().c_str(), feedback_id_, (is_eff_id_ ? "EFF" : "SFF"),
      value_offset_, value_bits_, (value_big_endian_ ? "big" : "little"),
      counts_per_rev_, timer_ms_, publish_topic_.c_str(),
      (invert_direction_ ? "yes" : "no"),
      (use_clamp_ ? "yes" : "no"), left_limit_deg_, right_limit_deg_);

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

  // ---------- utils ----------
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

  // [-half, +half)로 접기(회전형 카운터 안정화)
  static int signed_wrap_counts(int delta, int mod) {
    if (mod <= 0) return delta;
    int d = delta % mod; if (d < 0) d += mod;
    if (d >= (mod + 1) / 2) d -= mod;
    return d;
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

  // ---------- 핵심: 좌회전은 음수, 우회전은 양수 ----------
  void publish_angle_from_counts(int32_t counts) {
    // 0) 시작 시 자동 영점
    if (auto_zero_on_start_ && !zero_locked_) {
      zero_counts_ = counts;
      zero_locked_ = true;
      RCLCPP_INFO(get_logger(), "Auto-zero captured: zero_counts=%d", zero_counts_);
    }

    // 1) zero 기준 signed delta (회전형이면 wrap)
    const int mod = counts_per_rev_; // 360(heartbeat), 10000(position) 등 실제 프로토콜에 맞추기
    const int delta_counts_raw = counts - zero_counts_;
    const int delta_counts     = (mod > 0) ? signed_wrap_counts(delta_counts_raw, mod)
                                           : delta_counts_raw;

    // 2) 핸들각 x(deg): 원본 규약(좌=양/우=음)을 우선 반영 → 이후 필요 시 전체 반전
    float x = (static_cast<float>(delta_counts) / static_cast<float>(mod)) * 360.0f;

    //   요구사항: 출력은 좌=음, 우=양 이므로 raw 규약이 좌=양/우=음이라면 부호 반전 필요
    if (invert_direction_) x = -x;

    // 3) 오른쪽(+x) 실험식 f(t) = a*t^2 + b*t + c  (t=|x|)
    const float t = std::fabs(x);
    const float f = 0.0002f * t * t + 0.2159f * t + 0.1612f;

    // 4) 부호는 x를 따름 → 바퀴각
    float wheel_deg = std::copysign(f, x);

    // 5) (선택) 안전 클램프
    if (use_clamp_) {
      wheel_deg = std::clamp(wheel_deg, static_cast<float>(left_limit_deg_), static_cast<float>(right_limit_deg_));
    }

    // 6) 퍼블리시
    std_msgs::msg::Float32 msg;
    msg.data = wheel_deg;
    encoder_pub_->publish(msg);

    // 7) 로그: 좌/우 표시 포함
    const char* side = (x > 0.0f) ? "RIGHT(+)" : (x < 0.0f ? "LEFT(-)" : "CENTER");
    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 500,
      "raw=%d  zero=%d  delta=%d  handle=%.2f° [%s]  -> wheel=%.2f°%s",
      counts, zero_counts_, delta_counts, x, side, wheel_deg,
      use_clamp_ ? " (clamped)" : "");
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

  // zero/dir/clamp
  bool     auto_zero_on_start_;
  int      zero_counts_;
  bool     zero_locked_ {false};
  bool     invert_direction_;     // raw 좌=양/우=음 → 출력 좌=음/우=양로 뒤집기
  bool     use_clamp_;
  double   left_limit_deg_;
  double   right_limit_deg_;

  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr encoder_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SteeringFeedbackNode>());
  rclcpp::shutdown();
  return 0;
}

