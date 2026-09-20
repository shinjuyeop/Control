#include "longitudinal_ctrl/longitudinal_actuator_node.hpp"

#include <algorithm>
#include <cmath>
#include <functional>
#include <stdexcept>

namespace longitudinal_ctrl
{
namespace
{
constexpr double kPi = 3.14159265358979323846;
double seconds(std::chrono::steady_clock::time_point time)
{
  return std::chrono::duration<double>(time.time_since_epoch()).count();
}
double ageMs(std::chrono::steady_clock::time_point now, std::chrono::steady_clock::time_point then)
{
  return std::chrono::duration<double, std::milli>(now - then).count();
}
}  // namespace

LongitudinalActuatorNode::LongitudinalActuatorNode(const rclcpp::NodeOptions & options)
: Node("longitudinal_actuator_node", options),
  controller_(declareControllerParameters())
{
  rcl_interfaces::msg::ParameterDescriptor startup_only;
  startup_only.read_only = true;
  startup_only.description = "Startup only; restart the node to change this parameter.";
  declare_parameter("tuning_preset", std::string("unspecified"), startup_only);
  torque_control_enabled_ = declare_parameter("torque_control_enabled", false);
  speed_source_ = declare_parameter("speed_source", std::string("drive_status"), startup_only);
  if (speed_source_ != "drive_status" && speed_source_ != "gnss") {
    throw std::invalid_argument("speed_source must be drive_status or gnss");
  }
  publish_period_ms_ = static_cast<int>(get_parameter("publish_period_ms").as_int());
  command_timeout_ms_ = static_cast<int>(declare_parameter<int64_t>("command_timeout_ms", 200, startup_only));
  drive_status_timeout_ms_ =
    static_cast<int>(declare_parameter<int64_t>("drive_status_timeout_ms", 200, startup_only));
  if (publish_period_ms_ < 5 || publish_period_ms_ > 100 || command_timeout_ms_ <= 0 ||
    drive_status_timeout_ms_ <= 0)
  {
    throw std::invalid_argument("publish period must be 5..100 ms and timeouts positive");
  }
  tire_diameter_m_ = declare_parameter("tire_diameter_m", 0.47, startup_only);
  gear_ratio_ = declare_parameter("gear_ratio", 4.6, startup_only);
  if (!std::isfinite(tire_diameter_m_) || tire_diameter_m_ <= 0.0 ||
    !std::isfinite(gear_ratio_) || gear_ratio_ <= 0.0)
  {
    throw std::invalid_argument("tire_diameter_m and gear_ratio must be finite and positive");
  }
  speed_limit_margin_mps_ = declare_parameter("speed_limit_margin_mps", 2.0);
  speed_limit_floor_mps_ = declare_parameter("speed_limit_floor_mps", 3.0);
  const auto gnss_topic = declare_parameter("gnss_velocity_topic", std::string("/gnss/fix_velocity"),
      startup_only);

  torque_pub_ = create_publisher<kaiev26_msgs::msg::DriveTorqueCommand>("/drive/torque_command", 10);
  state_pub_ = create_publisher<std_msgs::msg::String>("/longitudinal/state", 10);
  const auto f32 = [this](const char * name) {
      return create_publisher<std_msgs::msg::Float32>(name, 10);
    };
  const auto boolean = [this](const char * name) {
      return create_publisher<std_msgs::msg::Bool>(name, 10);
    };
  target_pub_ = f32("/longitudinal/target_speed");
  shaped_target_pub_ = f32("/longitudinal/shaped_target_speed");
  filtered_target_pub_ = f32("/longitudinal/filtered_target_speed");
  target_accel_pub_ = f32("/longitudinal/target_accel");
  speed_pub_ = f32("/longitudinal/speed");
  speed_raw_pub_ = f32("/longitudinal/speed_raw");
  error_pub_ = f32("/longitudinal/speed_error");
  ff_pub_ = f32("/longitudinal/current_ff");
  p_pub_ = f32("/longitudinal/current_p");
  i_pub_ = f32("/longitudinal/current_i");
  unsaturated_pub_ = f32("/longitudinal/current_unsaturated");
  command_pub_ = f32("/longitudinal/current_command");
  regen_cap_pub_ = f32("/longitudinal/regen_cap");
  rejected_pub_ = f32("/longitudinal/speed_rejected_total");
  saturated_pub_ = boolean("/longitudinal/saturated");
  slew_limited_pub_ = boolean("/longitudinal/slew_limited");
  regen_blocked_pub_ = boolean("/longitudinal/regen_blocked");
  stopped_pub_ = boolean("/longitudinal/stopped");
  active_pub_ = boolean("/longitudinal/torque_active");

  command_sub_ = create_subscription<kaiev26_msgs::msg::ActuatorCommand>(
    "/vehicle/command", 10, std::bind(&LongitudinalActuatorNode::onCommand, this, std::placeholders::_1));
  drive_status_sub_ = create_subscription<kaiev26_msgs::msg::DriveActuatorStatus>(
    "/drive/status", 10,
    std::bind(&LongitudinalActuatorNode::onDriveStatus, this, std::placeholders::_1));
  brake_status_sub_ = create_subscription<kaiev26_msgs::msg::BrakeActuatorStatus>(
    "/brake/status", 10,
    std::bind(&LongitudinalActuatorNode::onBrakeStatus, this, std::placeholders::_1));
  if (speed_source_ == "gnss") {
    gnss_sub_ = create_subscription<geometry_msgs::msg::TwistWithCovarianceStamped>(
      gnss_topic, 10,
      std::bind(&LongitudinalActuatorNode::onGnssVelocity, this, std::placeholders::_1));
  }
  parameter_callback_ = add_on_set_parameters_callback(
    std::bind(&LongitudinalActuatorNode::onParameters, this, std::placeholders::_1));
  control_timer_ = create_wall_timer(
    std::chrono::milliseconds(publish_period_ms_),
    std::bind(&LongitudinalActuatorNode::controlTick, this));

  const auto & c = controller_.config();
  RCLCPP_WARN(get_logger(),
    "longitudinal torque control %s (torque_control_enabled). speed_source=%s, "
    "K=%.5f m/s^2/A, kp=%.0f A/(m/s), ki=%.0f A/m, drive<=%.0f A, regen<=%.0f A, slew %.0f A/s. "
    "STM_A firmware must subscribe /drive/torque_command; older firmware ignores it.",
    torque_control_enabled_ ? "ENABLED" : "disabled", speed_source_.c_str(),
    c.current_gain_mps2_per_a, c.kp_a_per_mps, c.ki_a_per_m, c.max_drive_current_a,
    c.max_regen_current_a, c.current_slew_a_per_s);
}

SpeedTorqueConfig LongitudinalActuatorNode::declareControllerParameters()
{
  // 생성자 초기화 목록에서 호출된다. 제어 주기는 발행 주기와 같으므로 여기서 먼저 읽는다.
  rcl_interfaces::msg::ParameterDescriptor startup_only;
  startup_only.read_only = true;
  startup_only.description = "Startup only; restart the node to change this parameter.";
  SpeedTorqueConfig c;
  c.control_period_sec = declare_parameter<int64_t>("publish_period_ms", 20, startup_only) / 1000.0;
  c.current_gain_mps2_per_a = declare_parameter("current_gain_mps2_per_a", c.current_gain_mps2_per_a);
  c.rolling_resistance_mps2 = declare_parameter("rolling_resistance_mps2", c.rolling_resistance_mps2);
  c.speed_damping_per_s = declare_parameter("speed_damping_per_s", c.speed_damping_per_s);
  c.feedforward_accel_scale = declare_parameter("feedforward_accel_scale", c.feedforward_accel_scale);
  c.feedforward_resistance_scale =
    declare_parameter("feedforward_resistance_scale", c.feedforward_resistance_scale);
  c.kp_a_per_mps = declare_parameter("kp_a_per_mps", c.kp_a_per_mps);
  c.ki_a_per_m = declare_parameter("ki_a_per_m", c.ki_a_per_m);
  c.integrator_limit_a = declare_parameter("integrator_limit_a", c.integrator_limit_a);
  c.accel_limit_mps2 = declare_parameter("accel_limit_mps2", c.accel_limit_mps2);
  c.decel_limit_mps2 = declare_parameter("decel_limit_mps2", c.decel_limit_mps2);
  c.tracking_error_limit_mps = declare_parameter("tracking_error_limit_mps", c.tracking_error_limit_mps);
  c.max_drive_current_a = declare_parameter("max_drive_current_a", c.max_drive_current_a);
  c.max_regen_current_a = declare_parameter("max_regen_current_a", c.max_regen_current_a);
  c.regen_bus_current_limit_a = declare_parameter("regen_bus_current_limit_a", c.regen_bus_current_limit_a);
  c.current_slew_a_per_s = declare_parameter("current_slew_a_per_s", c.current_slew_a_per_s);
  c.regen_min_speed_mps = declare_parameter("regen_min_speed_mps", c.regen_min_speed_mps);
  c.stop_speed_mps = declare_parameter("stop_speed_mps", c.stop_speed_mps);
  c.speed_filter_hz = declare_parameter("speed_filter_hz", c.speed_filter_hz);
  c.speed_jump_reject_mps = declare_parameter("speed_jump_reject_mps", c.speed_jump_reject_mps);
  c.speed_jump_reject_max_samples = static_cast<int>(
    declare_parameter<int64_t>("speed_jump_reject_max_samples", c.speed_jump_reject_max_samples));
  c.feedback_timeout_sec = declare_parameter<int64_t>("feedback_timeout_ms", 300) / 1000.0;
  return c;
}

rcl_interfaces::msg::SetParametersResult LongitudinalActuatorNode::onParameters(
  const std::vector<rclcpp::Parameter> & parameters)
{
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;
  SpeedTorqueConfig candidate = controller_.config();
  bool enabled = torque_control_enabled_;
  double margin = speed_limit_margin_mps_, floor = speed_limit_floor_mps_;
  bool tuning_changed = false;
  try {
    for (const auto & p : parameters) {
      const auto & name = p.get_name();
      if (name == "use_sim_time") continue;
      if (name == "torque_control_enabled") {enabled = p.as_bool(); continue;}
      if (name == "speed_limit_margin_mps") {margin = p.as_double(); continue;}
      if (name == "speed_limit_floor_mps") {floor = p.as_double(); continue;}
      tuning_changed = true;
      if (name == "current_gain_mps2_per_a") candidate.current_gain_mps2_per_a = p.as_double();
      else if (name == "rolling_resistance_mps2") candidate.rolling_resistance_mps2 = p.as_double();
      else if (name == "speed_damping_per_s") candidate.speed_damping_per_s = p.as_double();
      else if (name == "feedforward_accel_scale") candidate.feedforward_accel_scale = p.as_double();
      else if (name == "feedforward_resistance_scale") candidate.feedforward_resistance_scale = p.as_double();
      else if (name == "kp_a_per_mps") candidate.kp_a_per_mps = p.as_double();
      else if (name == "ki_a_per_m") candidate.ki_a_per_m = p.as_double();
      else if (name == "integrator_limit_a") candidate.integrator_limit_a = p.as_double();
      else if (name == "accel_limit_mps2") candidate.accel_limit_mps2 = p.as_double();
      else if (name == "decel_limit_mps2") candidate.decel_limit_mps2 = p.as_double();
      else if (name == "tracking_error_limit_mps") candidate.tracking_error_limit_mps = p.as_double();
      else if (name == "max_drive_current_a") candidate.max_drive_current_a = p.as_double();
      else if (name == "max_regen_current_a") candidate.max_regen_current_a = p.as_double();
      else if (name == "regen_bus_current_limit_a") candidate.regen_bus_current_limit_a = p.as_double();
      else if (name == "current_slew_a_per_s") candidate.current_slew_a_per_s = p.as_double();
      else if (name == "regen_min_speed_mps") candidate.regen_min_speed_mps = p.as_double();
      else if (name == "stop_speed_mps") candidate.stop_speed_mps = p.as_double();
      else if (name == "speed_filter_hz") candidate.speed_filter_hz = p.as_double();
      else if (name == "speed_jump_reject_mps") candidate.speed_jump_reject_mps = p.as_double();
      else if (name == "speed_jump_reject_max_samples") {
        candidate.speed_jump_reject_max_samples = static_cast<int>(p.as_int());
      } else if (name == "feedback_timeout_ms") {
        candidate.feedback_timeout_sec = p.as_int() / 1000.0;
      } else {
        throw std::invalid_argument(name + " is read at startup (restart required)");
      }
    }
    if (!std::isfinite(margin) || !std::isfinite(floor) || floor < 0.0) {
      throw std::invalid_argument("speed limit margin/floor must be finite, floor >= 0");
    }
    candidate.validate();
  } catch (const std::exception & error) {
    result.successful = false;
    result.reason = error.what();
    return result;
  }
  if (tuning_changed) controller_.configure(candidate);
  if (enabled != torque_control_enabled_) {
    RCLCPP_WARN(get_logger(), "torque_control_enabled -> %s", enabled ? "true" : "false");
  }
  torque_control_enabled_ = enabled;
  speed_limit_margin_mps_ = margin;
  speed_limit_floor_mps_ = floor;
  return result;
}

void LongitudinalActuatorNode::onCommand(const kaiev26_msgs::msg::ActuatorCommand::SharedPtr msg)
{
  if (!msg->brake_engage && !std::isfinite(msg->speed_target_mps)) return;
  command_received_ = true;
  target_speed_mps_ = msg->brake_engage ? 0.0f : msg->speed_target_mps;
  brake_engage_ = msg->brake_engage;
  last_command_time_ = Clock::now();
}

void LongitudinalActuatorNode::onDriveStatus(const kaiev26_msgs::msg::DriveActuatorStatus::SharedPtr msg)
{
  const auto now = Clock::now();
  drive_ = *msg;
  drive_received_ = true;
  last_drive_time_ = now;
  if (msg->part1_valid) {
    controller_.observeBusCurrent(msg->bus_current_a);
    if (speed_source_ == "drive_status") controller_.observeSpeed(msg->speed_mps, seconds(now));
  }
}

void LongitudinalActuatorNode::onBrakeStatus(const kaiev26_msgs::msg::BrakeActuatorStatus::SharedPtr msg)
{
  estop_active_ = (msg->stop_reason_bits & kaiev26_msgs::msg::BrakeActuatorStatus::STOP_REASON_ESTOP) != 0;
}

void LongitudinalActuatorNode::onGnssVelocity(
  const geometry_msgs::msg::TwistWithCovarianceStamped::SharedPtr msg)
{
  const double speed = std::hypot(msg->twist.twist.linear.x, msg->twist.twist.linear.y);
  controller_.observeSpeed(speed, seconds(Clock::now()));
}

std::string LongitudinalActuatorNode::blockingReason(Clock::time_point now) const
{
  if (!torque_control_enabled_) return "disabled_by_parameter";
  if (!drive_received_) return "drive_status_not_received";
  if (ageMs(now, last_drive_time_) > drive_status_timeout_ms_) return "drive_status_stale";
  if (drive_.mode != kaiev26_msgs::msg::DriveActuatorStatus::MODE_AUTO) return "mode_not_auto";
  if (estop_active_) return "estop";
  if (drive_.stop_request) return "stop_request";
  if (!drive_.part1_valid) return "part1_invalid";
  if (drive_.fault_bits_valid && drive_.fault_bits != 0) return "motor_fault";
  if (drive_.gear != kaiev26_msgs::msg::DriveActuatorStatus::GEAR_FORWARD) return "gear_reverse";
  if (!command_received_) return "command_not_received";
  if (ageMs(now, last_command_time_) > command_timeout_ms_) return "command_stale";
  if (brake_engage_) return "brake_engage";
  if (target_speed_mps_ < 0.0f) return "target_negative";
  if (!controller_.feedbackFresh(seconds(now))) return "speed_feedback_stale";
  return "";
}

float LongitudinalActuatorNode::motorRpmLimit(double shaped_target_mps) const
{
  if (speed_limit_margin_mps_ <= 0.0) return 0.0f;  // STM_A 기본(전진 4500 rpm)
  const double limit_mps = std::max(shaped_target_mps + speed_limit_margin_mps_, speed_limit_floor_mps_);
  const double rpm = limit_mps * 60.0 / (kPi * tire_diameter_m_) * gear_ratio_;
  return static_cast<float>(rpm);
}

void LongitudinalActuatorNode::controlTick()
{
  const auto now = Clock::now();
  const std::string reason = blockingReason(now);
  kaiev26_msgs::msg::DriveTorqueCommand command;
  command.header.stamp = this->now();
  SpeedTorqueOutput out;
  std::string state;
  if (reason.empty()) {
    if (!controller_.active()) {
      // EZ speed control에서 넘어올 때 실측 상전류에서 시작한다.
      controller_.start(seconds(now), drive_.phase_current_a);
      RCLCPP_INFO(get_logger(), "torque control ACTIVE from %.0f A at %.2f m/s",
        drive_.phase_current_a, controller_.filteredSpeed());
    }
    out = controller_.update(target_speed_mps_, seconds(now));
    command.torque_mode = true;
    command.phase_current_target_a = static_cast<float>(out.command_a);
    command.motor_rpm_limit = motorRpmLimit(out.shaped_target_mps);
    state = "active";
  } else {
    if (controller_.active()) {
      RCLCPP_WARN(get_logger(), "torque control released: %s", reason.c_str());
    }
    controller_.stop();
    out.target_mps = target_speed_mps_;
    out.speed_mps = controller_.filteredSpeed();
    command.torque_mode = false;
    command.phase_current_target_a = 0.0f;
    command.motor_rpm_limit = 0.0f;
    state = "off:" + reason;
  }
  if (state != last_state_) {
    RCLCPP_INFO(get_logger(), "longitudinal state: %s", state.c_str());
    last_state_ = state;
  }
  torque_pub_->publish(command);
  publishDebug(out, state);
}

void LongitudinalActuatorNode::publishDebug(const SpeedTorqueOutput & out, const std::string & state)
{
  std_msgs::msg::String s;
  s.data = state;
  state_pub_->publish(s);
  const auto pub = [](auto & publisher, double value) {
      std_msgs::msg::Float32 m;
      m.data = static_cast<float>(value);
      publisher->publish(m);
    };
  const auto pubb = [](auto & publisher, bool value) {
      std_msgs::msg::Bool m;
      m.data = value;
      publisher->publish(m);
    };
  pub(target_pub_, out.target_mps);
  pub(shaped_target_pub_, out.shaped_target_mps);
  pub(filtered_target_pub_, out.filtered_target_mps);
  pub(target_accel_pub_, out.target_accel_mps2);
  pub(speed_pub_, out.speed_mps);
  pub(speed_raw_pub_, controller_.rawSpeed());
  pub(error_pub_, out.error_mps);
  pub(ff_pub_, out.ff_a);
  pub(p_pub_, out.p_a);
  pub(i_pub_, out.i_a);
  pub(unsaturated_pub_, out.unsaturated_a);
  pub(command_pub_, out.command_a);
  pub(regen_cap_pub_, out.regen_cap_a);
  pub(rejected_pub_, static_cast<double>(controller_.rejectedTotal()));
  pubb(saturated_pub_, out.saturated);
  pubb(slew_limited_pub_, out.slew_limited);
  pubb(regen_blocked_pub_, out.regen_blocked);
  pubb(stopped_pub_, out.stopped);
  pubb(active_pub_, controller_.active());
}
}  // namespace longitudinal_ctrl
