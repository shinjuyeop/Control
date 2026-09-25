#include "mpc_common/linear_mpc.hpp"
#include <ackermann_msgs/msg/ackermann_drive_stamped.hpp>
#include <algorithm>
#include <chrono>
#include <cmath>
#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <kaiev26_msgs/msg/actuator_command.hpp>
#include <kaiev26_msgs/msg/centerline.hpp>
#include <kaiev26_msgs/msg/vehicle_state.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <yaml-cpp/yaml.h>
#include <visualization_msgs/msg/marker_array.hpp>

class PathMpcNode : public rclcpp::Node {
public:
  PathMpcNode()
      : Node("lateral_path_mpc"),
        qp_(4, 40, 3,
            (Eigen::Vector4d() << startup<double>("cross_track_weight", 20.),
             startup<double>("heading_weight", 10.),
             startup<double>("yaw_rate_weight", .1),
             startup<double>("steering_state_weight", 1.))
                .finished(),
            startup<double>("steering_effort_weight", .1),
            startup<double>("steering_change_weight", 200.), .025),
        reference_(Eigen::MatrixXd::Zero(4, 40)),
        offset_(Eigen::MatrixXd::Zero(4, 40)) {
    const auto model =
        YAML::LoadFile(startup<std::string>("path_model_file", ""));
    wheelbase_ = model["wheelbase_m"].as<double>();
    steer_tau_ = model["steering_time_constant_s"].as<double>();
    yaw_tau_ = model["yaw_time_constant_s"].as<double>();
    yaw_gain_ = model["yaw_gain"].as<double>();
    steer_gain_ = model["steering_gain"].as<double>();
    steer_bias_ = model["steering_bias_rad"].as<double>();
    yaw_bias_ = model["yaw_bias_rad_s"].as<double>();
    angle_limit_ = startup<double>("max_steering_rad", .4363);
    rate_limit_ = startup<double>("steering_rate_limit_radps", 1.0471975512);
    state_angle_limit_ = angle_limit_;
    observer_only_ = startup<bool>("observer_only", false);
    const double reference_limit =
        model["steering_reference_limit_rad"].as<double>();
    const double reference_rate =
        model["steering_reference_rate_rad_s"].as<double>();
    if (!(reference_limit > 0) || !(reference_rate > 0) ||
        !std::isfinite(reference_limit + reference_rate))
      throw std::runtime_error("Path model requires a finite positive "
                               "identified reference envelope");
    if (!observer_only_) {
      angle_limit_ = std::min(angle_limit_, reference_limit);
      rate_limit_ = std::min(rate_limit_, reference_rate);
    }
    if (!(wheelbase_ > 0) || !(steer_tau_ > 0) || !(yaw_tau_ > 0) ||
        !(yaw_gain_ > 0) || !(steer_gain_ > 0) ||
        !std::isfinite(wheelbase_ + steer_tau_ + yaw_tau_ + yaw_gain_ +
                       steer_gain_ + steer_bias_ + yaw_bias_ + angle_limit_ +
                       rate_limit_) ||
        angle_limit_ <= 0 || rate_limit_ <= 0)
      throw std::runtime_error("Invalid lateral response model/limits");
    path_sub_ = create_subscription<kaiev26_msgs::msg::Centerline>(
        "/planning/target_path", 10,
        [this](kaiev26_msgs::msg::Centerline::SharedPtr m) { path_ = m; });
    state_sub_ = create_subscription<kaiev26_msgs::msg::VehicleState>(
        "/vehicle/state", 10,
        [this](kaiev26_msgs::msg::VehicleState::SharedPtr m) { state_ = m; });
    imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
        startup<std::string>("imu_topic", "/imu/data"), rclcpp::SensorDataQoS(),
        [this](sensor_msgs::msg::Imu::SharedPtr m) {
          imu_ = m;
          imu_received_ = now();
        });
    applied_sub_ = create_subscription<kaiev26_msgs::msg::ActuatorCommand>(
        "/vehicle/command", 10,
        [this](kaiev26_msgs::msg::ActuatorCommand::SharedPtr m) {
          if (!observer_only_)
            previous_ = m->steering_target_rad;
        });
    output_ = create_publisher<ackermann_msgs::msg::AckermannDriveStamped>(
        observer_only_ ? "~/target" : "/control/lateral/target", 10);
    diagnostics_ = create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
        observer_only_ ? "~/diagnostics" : "/control/lateral/diagnostics", 10);
    prediction_ = create_publisher<visualization_msgs::msg::MarkerArray>(
        observer_only_ ? "/debug/path_compare/mpc/trajectory"
                       : "/control/lateral/mpc/trajectory",
        10);
    timer_ = create_wall_timer(std::chrono::milliseconds(50),
                               std::bind(&PathMpcNode::tick, this));
  }

private:
  template <class T> T startup(const std::string &name, const T &value) {
    rcl_interfaces::msg::ParameterDescriptor descriptor;
    descriptor.read_only = true;
    return declare_parameter<T>(name, value, descriptor);
  }
  bool fresh(const builtin_interfaces::msg::Time &stamp, double age) {
    if (stamp.sec == 0 && stamp.nanosec == 0)
      return false;
    double a = (now() - rclcpp::Time(stamp)).seconds();
    return a >= -.05 && a <= age;
  }
  void tick() {
    const auto started = std::chrono::steady_clock::now();
    mpc_common::Solution sol;
    double ey = 0, epsi = 0;
    visualization_msgs::msg::Marker predicted;
    predicted.header.stamp = now();
    predicted.header.frame_id = "base_footprint";
    predicted.ns = "path_mpc_optimized_horizon";
    predicted.id = 0;
    predicted.action = visualization_msgs::msg::Marker::DELETE;
    std::string reason = "missing_or_stale_input";
    if (path_ && state_ && imu_ && fresh(path_->header.stamp, .35) &&
        fresh(state_->header.stamp, .2) &&
        (now() - imu_received_).seconds() < .2 &&
        path_->header.frame_id == "base_footprint" && path_->confidence > 0 &&
        path_->points.size() >= 2 && !state_->estop_active &&
        std::isfinite(state_->speed_mps + state_->steering_rad +
                      imu_->angular_velocity.z) &&
        state_->gear == kaiev26_msgs::msg::VehicleState::GEAR_FORWARD) {
      const double v = std::max(0., static_cast<double>(state_->speed_mps));
      std::vector<Eigen::Vector2d> points;
      for (const auto &p : path_->points) {
        if (!std::isfinite(p.x + p.y)) {
          points.clear();
          break;
        }
        Eigen::Vector2d xy(p.x, p.y);
        if (points.empty() || (xy - points.back()).norm() > 1e-6)
          points.push_back(xy);
      }
      if (points.size() >= 2) {
        size_t nearest = 0;
        double distance = 1e30, ratio = 0;
        for (size_t i = 0; i + 1 < points.size(); ++i) {
          auto d = points[i + 1] - points[i];
          double t = std::clamp(-points[i].dot(d) / d.squaredNorm(), 0., 1.);
          double dist = (points[i] + t * d).squaredNorm();
          if (dist < distance) {
            distance = dist;
            nearest = i;
            ratio = t;
          }
        }
        const auto direction =
            (points[nearest + 1] - points[nearest]).normalized();
        const Eigen::Vector2d projection =
            points[nearest] + ratio * (points[nearest + 1] - points[nearest]);
        ey = direction.y() * projection.x() - direction.x() * projection.y();
        epsi = -std::atan2(direction.y(), direction.x());
        if (observer_only_ && !observer_previous_ready_) {
          previous_ = state_->steering_rad;
          observer_previous_ready_ = true;
        }
        if (v < .1 ||
            (!observer_only_ &&
             state_->mode == kaiev26_msgs::msg::VehicleState::MODE_MANUAL)) {
          sol.solved = true;
          sol.input = state_->steering_rad;
        } else {
          Eigen::Matrix4d a = Eigen::Matrix4d::Identity();
          Eigen::Vector4d b = Eigen::Vector4d::Zero();
          const double ay = std::exp(-.05 / yaw_tau_),
                       as = std::exp(-.05 / steer_tau_);
          a(0, 1) = .05 * v;
          a(1, 2) = .05;
          a(2, 2) = ay;
          a(2, 3) = (1 - ay) * yaw_gain_ * v / wheelbase_;
          a(3, 3) = as;
          b[3] = (1 - as) * steer_gain_;
          qp_.dynamics(a, b);
          offset_.setZero();
          reference_.setZero();
          double along = ratio * (points[nearest + 1] - points[nearest]).norm();
          size_t segment = nearest;
          for (int k = 0; k < 40; ++k) {
            along += v * .05;
            while (segment + 2 < points.size() &&
                   along > (points[segment + 1] - points[segment]).norm()) {
              along -= (points[segment + 1] - points[segment]).norm();
              ++segment;
            }
            double curvature = 0;
            if (segment > 0 && segment + 1 < points.size()) {
              const Eigen::Vector2d d0 = points[segment] - points[segment - 1],
                                    d1 = points[segment + 1] - points[segment];
              curvature = 2 * (d0.x() * d1.y() - d0.y() * d1.x()) /
                          (d0.norm() * d1.norm() * (d0 + d1).norm() + 1e-12);
            }
            offset_(1, k) = -v * .05 * curvature;
            offset_(2, k) = (1 - ay) * yaw_bias_;
            offset_(3, k) = (1 - as) * steer_bias_;
            reference_(2, k) = v * curvature;
            reference_(3, k) = std::atan(wheelbase_ * curvature / yaw_gain_);
          }
          sol = qp_.solve(Eigen::Vector4d(ey, epsi, imu_->angular_velocity.z,
                                          state_->steering_rad),
                          reference_, offset_, previous_, angle_limit_,
                          rate_limit_ * .05, -state_angle_limit_,
                          state_angle_limit_);
          if (sol.solved) {
            auto controls = qp_.planned_inputs();
            predicted.action = visualization_msgs::msg::Marker::ADD;
            predicted.type = visualization_msgs::msg::Marker::LINE_STRIP;
            predicted.pose.orientation.w = 1.0;
            predicted.scale.x = 0.10;
            predicted.color.r = 1.0;
            predicted.color.g = 0.55;
            predicted.color.a = 1.0;
            geometry_msgs::msg::Point origin;
            origin.z = 0.10;
            predicted.points.push_back(origin);
            Eigen::Vector4d forecast(ey, epsi, imu_->angular_velocity.z,
                                     state_->steering_rad);
            double px = 0, py = 0, heading = 0;
            for (size_t k = 0; k < controls.size(); ++k) {
              const Eigen::Vector4d next = a * forecast + b * controls[k] +
                                           offset_.col(k);
              const double next_heading = heading + .025 * (forecast[2] + next[2]);
              px += v * .05 * std::cos((heading + next_heading) * .5);
              py += v * .05 * std::sin((heading + next_heading) * .5);
              heading = next_heading;
              geometry_msgs::msg::Point point;
              point.x = px;
              point.y = py;
              point.z = 0.10;
              predicted.points.push_back(point);
              forecast = next;
            }
          }
        }
        reason = sol.solved ? (sol.status == OSQP_SOLVED_INACCURATE
                                   ? "solved_inaccurate"
                                   : "solved")
                            : "solver_failed_" + std::to_string(sol.status);
      } else
        reason = "invalid_path";
    }
    if (sol.solved) {
      if (observer_only_)
        previous_ = sol.input;
      ackermann_msgs::msg::AckermannDriveStamped m;
      m.header.stamp = now();
      m.header.frame_id = "base_footprint";
      m.drive.steering_angle = sol.input;
      output_->publish(m);
    } else if (observer_only_)
      observer_previous_ready_ = false;
    visualization_msgs::msg::MarkerArray markers;
    markers.markers.push_back(predicted);
    prediction_->publish(markers);
    diagnostic_msgs::msg::DiagnosticArray d;
    d.header.stamp = now();
    diagnostic_msgs::msg::DiagnosticStatus s;
    s.name = "lateral_path_mpc";
    s.level = sol.solved ? (sol.status == OSQP_SOLVED_INACCURATE ? 1 : 0) : 2;
    s.message = reason;
    for (const auto &item : std::vector<std::pair<std::string, double>>{
             {"cross_track_error_m", ey},
             {"heading_error_rad", epsi},
             {"solve_ms", sol.solve_ms},
             {"compute_ms", std::chrono::duration<double, std::milli>(
                                std::chrono::steady_clock::now() - started)
                                .count()}}) {
      diagnostic_msgs::msg::KeyValue kv;
      kv.key = item.first;
      kv.value = std::to_string(item.second);
      s.values.push_back(kv);
    }
    d.status.push_back(s);
    diagnostics_->publish(d);
  }
  mpc_common::LinearMpc qp_;
  Eigen::MatrixXd reference_, offset_;
  double wheelbase_, steer_tau_, yaw_tau_, yaw_gain_, steer_gain_, steer_bias_,
      yaw_bias_, angle_limit_, state_angle_limit_, rate_limit_, previous_{0};
  bool observer_only_{false};
  bool observer_previous_ready_{false};
  rclcpp::Time imu_received_{0, 0, RCL_ROS_TIME};
  kaiev26_msgs::msg::Centerline::SharedPtr path_;
  kaiev26_msgs::msg::VehicleState::SharedPtr state_;
  sensor_msgs::msg::Imu::SharedPtr imu_;
  rclcpp::Subscription<kaiev26_msgs::msg::Centerline>::SharedPtr path_sub_;
  rclcpp::Subscription<kaiev26_msgs::msg::VehicleState>::SharedPtr state_sub_;
  rclcpp::Subscription<kaiev26_msgs::msg::ActuatorCommand>::SharedPtr
      applied_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr
      output_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr
      diagnostics_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr prediction_;
  rclcpp::TimerBase::SharedPtr timer_;
};
int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PathMpcNode>());
  rclcpp::shutdown();
}
