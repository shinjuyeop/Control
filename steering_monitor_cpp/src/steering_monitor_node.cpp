// steering_monitor_node.cpp
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32.hpp"
#include <deque>

class SteeringMonitor : public rclcpp::Node {
public:
  SteeringMonitor() : Node("steering_monitor") {
    angle_sub_ = create_subscription<std_msgs::msg::Float32>(
      "/encoder_angle", 10,
      [this](const std_msgs::msg::Float32::SharedPtr msg){
        const double v = msg->data;

        if (!initialized_) { min_ = max_ = v; initialized_ = true; last_time_ = this->now(); }
        if (v < min_) min_ = v;
        if (v > max_) max_ = v;

        // 이동평균
        buf_.push_back(v);
        if ((int)buf_.size() > window_) buf_.pop_front();
        double sum = 0; for (double x: buf_) sum += x;
        const double avg = sum / buf_.size();

        // 주기 추정
        auto t = this->now();
        const double dt = (t - last_time_).seconds();
        last_time_ = t;
        if (dt > 0) hz_ = 0.9*hz_ + 0.1*(1.0/dt);

        // 10샘플마다 출력 (STEADY CLOCK으로 throttle)
        if (print_count_++ % 10 == 0) {
          RCLCPP_INFO_THROTTLE(
            this->get_logger(), steady_clock_, 500,
            "Angle=%.2f deg | Avg=%.2f deg | Min=%.2f Max=%.2f | ~%.1f Hz",
            v, avg, min_, max_, hz_);
        }
      });

    cmd_sub_ = create_subscription<std_msgs::msg::Float32>(
      "/steering_command", 10,
      [this](const std_msgs::msg::Float32::SharedPtr msg){
        last_cmd_ = msg->data;
        have_cmd_ = true;
      });

    RCLCPP_INFO(this->get_logger(), "steering_monitor started");
  }

private:
  // STEADY time clock for *_THROTTLE macros (멤버로 보관)
  rclcpp::Clock steady_clock_{RCL_STEADY_TIME};

  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr angle_sub_;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr cmd_sub_;
  bool initialized_ = false;
  double min_ = 0, max_ = 0, hz_ = 0;
  int print_count_ = 0;
  int window_ = 9;
  std::deque<double> buf_;
  rclcpp::Time last_time_;
  bool have_cmd_ = false;
  double last_cmd_ = 0;
};

int main(int argc, char** argv){
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SteeringMonitor>());
  rclcpp::shutdown();
  return 0;
}
