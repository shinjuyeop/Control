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

class SteeringFeedbackNode : public rclcpp::Node {
public:
  SteeringFeedbackNode()
  : Node("steering_feedback_node")
  {
    // ---- Parameters (Humble: ints -> int64_t) -------------------------
    // 콤마로 구분된 인터페이스 목록. 기본값으로 can1,can0 둘 다 수신
    can_ifaces_csv_  = this->declare_parameter<std::string>("can_ifaces", "can1,can0");
    feedback_id_i64_ = this->declare_parameter<int64_t>("feedback_id", 0x07000001);
    is_eff_id_       = this->declare_parameter<bool>("extended_id", true);
    value_offset_    = this->declare_parameter<int>("value_offset", 0);     // candump 결과: [0:1]
    value_bits_      = this->declare_parameter<int>("value_bits", 16);      // 16-bit
    value_big_endian_= this->declare_parameter<bool>("big_endian", true);   // big-endian
    counts_per_rev_  = this->declare_parameter<int>("counts_per_rev", 360); // (counts/CPR)*360 = handle_deg
    publish_topic_   = this->declare_parameter<std::string>("publish_topic", "/ctrl/steer");
    timer_ms_        = this->declare_parameter<int>("poll_period_ms", 20);

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
      "Steering Feedback Node (ifaces=[%s], id=0x%08X %s, offset=%d, bits=%d, %s-endian, CPR=%d, period=%dms, publish='%s')",
      oss.str().c_str(), feedback_id_, (is_eff_id_ ? "EFF" : "SFF"),
      value_offset_, value_bits_, (value_big_endian_ ? "big" : "little"),
      counts_per_rev_, timer_ms_, publish_topic_.c_str());

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

  // utils
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

  // ---- 핵심 변경: 바퀴각 퍼블리시(우측 다항식 + 좌측 원점대칭) ----------
  void publish_angle_from_counts(int32_t counts) {
    // 1) counts -> 핸들각(raw)
    const float handle_deg =
      (static_cast<float>(counts) / static_cast<float>(counts_per_rev_)) * 360.0f; // x

    // 2) 우측 구간(+) 실험식 y = 0.0002*x^2 + 0.2159*x + 0.1612
    const float poly_y = 0.0002f * handle_deg * handle_deg
                       + 0.2159f * handle_deg
                       + 0.1612f;

    // 3) 좌측(−)은 원점대칭 적용: y(-x) = -y(x)
    const float wheel_deg = (handle_deg >= 0.0f) ? poly_y : -poly_y;

    // 4) 퍼블리시: /ctrl/steer (바퀴 조향각)
    std_msgs::msg::Float32 msg;
    msg.data = wheel_deg;
    encoder_pub_->publish(msg);

    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 500,
      "counts=%d -> handle=%.2f deg -> wheel=%.2f deg (poly right, odd-sym left)",
      counts, handle_deg, wheel_deg);
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

  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr encoder_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SteeringFeedbackNode>());
  rclcpp::shutdown();
  return 0;
}
