#include "lateral_ctrl/steering_actuator_node.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <functional>
#include <limits>
#include <stdexcept>

namespace lateral_ctrl
{
namespace
{
constexpr double kDegToRad = 3.14159265358979323846 / 180.0;
double seconds(std::chrono::steady_clock::time_point time)
{
  return std::chrono::duration<double>(time.time_since_epoch()).count();
}
}

SteeringActuatorNode::SteeringActuatorNode(
  const rclcpp::NodeOptions & options, std::unique_ptr<KeyaCanDriver> driver)
: Node("steering_actuator_node", options),
  converter_(
    declare_parameter<double>("count_scale"),
    declare_parameter<double>("equivalent_k1_deg"),
    declare_parameter<double>("equivalent_k2_deg"),
    declare_parameter<double>("equivalent_k3_deg"),
    declare_parameter<double>("equivalent_k4_deg"),
    declare_parameter<double>("equivalent_k5_deg"),
    declare_parameter<std::int64_t>("command_count_limit"),
    declare_parameter<double>("counts_per_rev")),
  tracking_controller_(
    declare_parameter("tracking_control_enabled", false),
    declare_parameter("tracking_kp", 0.0),
    declare_parameter("tracking_ki", 0.0),
    declare_parameter("tracking_kd", 0.0),
    declare_parameter("tracking_integral_limit", 1.0),
    declare_parameter("tracking_correction_limit_deg", 0.5))
{
  rcl_interfaces::msg::ParameterDescriptor startup_only;
  startup_only.read_only = true;
  startup_only.description = "Startup only; restart the node to change this parameter.";
  actuation_mode_ = declare_parameter("actuation_mode", std::string("position"), startup_only);
  if (actuation_mode_ != "position" && actuation_mode_ != "torque") {
    throw std::invalid_argument("actuation_mode must be position or torque");
  }
  const auto integer_parameter = [this](const char * name, int initial, int minimum, int maximum,
      const rcl_interfaces::msg::ParameterDescriptor & descriptor =
      rcl_interfaces::msg::ParameterDescriptor{}) {
      const auto value = declare_parameter<int64_t>(name, initial, descriptor);
      if (value < minimum || value > maximum) {
        throw std::invalid_argument(std::string(name) + " is outside its supported range");
      }
      return static_cast<int>(value);
    };
  torque_direction_sign_ = integer_parameter("torque_direction_sign", 1, -1, 1, startup_only);
  keyaTorqueCommand(0, torque_direction_sign_);
  TorqueSteeringConfig torque;
  torque.full_scale_error_rad = declare_parameter("torque_full_scale_error_deg", 0.5) * kDegToRad;
  torque.kd = declare_parameter("torque_kd", 1500.0);
  torque.peak_limit_raw = integer_parameter("torque_peak_limit_raw", 2000, 1, 10000);
  torque.sustained_limit_raw = integer_parameter("torque_sustained_limit_raw", 1000, 1, 10000);
  torque.peak_duration_sec = integer_parameter("torque_peak_duration_ms", 200, 10, 1000) / 1000.0;
  torque.peak_recovery_sec =
    declare_parameter<int64_t>("torque_peak_recovery_ms", 2000) / 1000.0;
  torque.settle_angle_rad = declare_parameter("torque_settle_angle_deg", 0.05) * kDegToRad;
  torque.settle_rate_rad_s = declare_parameter("torque_settle_rate_deg_s", 0.5) * kDegToRad;
  torque.rate_filter_hz = declare_parameter("torque_rate_filter_hz", 10.0);
  torque.soft_limit_margin_rad = declare_parameter("torque_soft_limit_margin_deg", 1.0) * kDegToRad;
  torque.hard_rate_limit_rad_s = declare_parameter("torque_hard_rate_limit_deg_s", 120.0) * kDegToRad;
  torque.feedback_timeout_sec =
    integer_parameter("torque_feedback_timeout_ms", 100, 1, 200, startup_only) / 1000.0;
  const int torque_period_ms = integer_parameter("torque_control_period_ms", 20, 10, 50, startup_only);
  torque.control_period_sec = torque_period_ms / 1000.0;
  torque.min_angle_rad = converter_.countToEquivalentDeg(-converter_.commandCountLimit()) * kDegToRad;
  torque.max_angle_rad = converter_.countToEquivalentDeg(converter_.commandCountLimit()) * kDegToRad;
  torque_controller_.configure(torque);
  RCLCPP_WARN(get_logger(),
    "actuation_mode=%s REQUIRES KEYA parameter 0020=%d, Control Ways=CAN (0019=2). "
    "Host does NOT set or verify 0020. Burn settings before operation. "
    "Torque direction sign=%+d: verify +target produces LEFT and increasing calibrated angle.",
    actuation_mode_.c_str(), torqueMode() ? 2 : 3, torque_direction_sign_);
  tracking_enabled_ = get_parameter("tracking_control_enabled").as_bool();
  control_period_ms_ = declare_parameter("control_period_ms", 20);
  if (torqueMode()) control_period_ms_ = torque_period_ms;
  can_timeout_ms_ = declare_parameter("can_timeout_ms", 500);
  feedback_timeout_ms_ = declare_parameter("feedback_timeout_ms", 500);
  mode_timeout_ms_ = declare_parameter("mode_timeout_ms", 1500);
  drive_status_timeout_ms_ = declare_parameter("drive_status_timeout_ms", 500);
  steer_command_timeout_ms_ = declare_parameter("steer_command_timeout_ms", 200);
  max_steering_rate_deg_s_ = declare_parameter("max_steering_rate_deg_s", 60.0);
  const double minimum_steering_speed_kph =
    declare_parameter("minimum_steering_speed_kph", 1.0);
  minimum_steering_speed_mps_ = minimum_steering_speed_kph / 3.6;
  if (control_period_ms_ <= 0 || can_timeout_ms_ <= 0 || feedback_timeout_ms_ <= 0 ||
    mode_timeout_ms_ <= 0 || drive_status_timeout_ms_ <= 0 || steer_command_timeout_ms_ <= 0)
  {
    throw std::invalid_argument("steering periods and timeouts must be positive");
  }
  if (!std::isfinite(max_steering_rate_deg_s_) || max_steering_rate_deg_s_ <= 0.0) {
    throw std::invalid_argument("max_steering_rate_deg_s must be finite and positive");
  }
  if (!std::isfinite(minimum_steering_speed_kph) || minimum_steering_speed_kph < 0.0) {
    throw std::invalid_argument("minimum_steering_speed_kph must be finite and nonnegative");
  }
  hardware_can_enabled_ = declare_parameter("hardware_can_enabled", false);
  actuator_output_enabled_ = declare_parameter("actuator_output_enabled", false);
  steering_enabled_ = declare_parameter("steering_enabled", true);
  test_input_injection_enabled_ = declare_parameter("test_input_injection_enabled", false);
  const auto can_interface = declare_parameter("can_interface", std::string("can0"));
  const auto node_id = declare_parameter<std::int64_t>("node_id", 1);
  if (node_id < 0 || node_id > std::numeric_limits<std::uint8_t>::max()) {
    throw std::out_of_range("node_id must be in [0, 255]");
  }

  // canonical 통합 인터페이스: 명령은 /vehicle/command 하나,
  // 안전·모드 상태는 STM이 발행하는 status에서 읽는다.
  const auto command_topic =
    declare_parameter("command_topic", std::string("/vehicle/command"));
  const auto drive_status_topic =
    declare_parameter("drive_status_topic", std::string("/drive/status"));
  const auto brake_status_topic =
    declare_parameter("brake_status_topic", std::string("/brake/status"));
  const auto test_estop_topic =
    declare_parameter("test_estop_topic", std::string("/steering/test/estop"));
  const auto angle_topic = declare_parameter("angle_topic", std::string("/steering/angle"));
  const auto target_motor_count_topic = declare_parameter(
    "target_motor_count_topic", std::string("/steering/target_motor_position_count"));
  const auto status_topic = declare_parameter("status_topic", std::string("/steering/status"));
  const auto enable_commanded_topic =
    declare_parameter("enable_commanded_topic", std::string("/steering/enable_commanded"));
  const auto enabled_topic = declare_parameter("enabled_topic", std::string("/steering/enabled"));

  command_sub_ = create_subscription<kaiev26_msgs::msg::ActuatorCommand>(
    command_topic, 10,
    std::bind(&SteeringActuatorNode::onCommand, this, std::placeholders::_1));
  drive_status_sub_ = create_subscription<kaiev26_msgs::msg::DriveActuatorStatus>(
    drive_status_topic, 10,
    std::bind(&SteeringActuatorNode::onDriveStatus, this, std::placeholders::_1));
  brake_status_sub_ = create_subscription<kaiev26_msgs::msg::BrakeActuatorStatus>(
    brake_status_topic, 10,
    std::bind(&SteeringActuatorNode::onBrakeStatus, this, std::placeholders::_1));
  if (test_input_injection_enabled_) {
    test_estop_sub_ = create_subscription<std_msgs::msg::UInt8>(
      test_estop_topic, 10,
      [this](std_msgs::msg::UInt8::SharedPtr message) {setEstop(message->data != 0, true);});
  }
  angle_pub_ = create_publisher<std_msgs::msg::Float32>(angle_topic, 10);
  target_motor_count_pub_ =
    create_publisher<std_msgs::msg::Int32>(target_motor_count_topic, 10);
  status_pub_ = create_publisher<kaiev26_msgs::msg::SteeringActuatorStatus>(status_topic, 10);
  enable_commanded_pub_ = create_publisher<std_msgs::msg::Bool>(enable_commanded_topic, 10);
  enabled_pub_ = create_publisher<std_msgs::msg::Bool>(enabled_topic, 10);
  actuation_mode_pub_ = create_publisher<std_msgs::msg::String>(
    "/steering/actuation_mode", rclcpp::QoS(1).transient_local());
  if (torqueMode()) {
    torque_command_pub_ = create_publisher<std_msgs::msg::Int32>("/steering/torque_command_raw", 10);
    torque_limit_pub_ = create_publisher<std_msgs::msg::Int32>("/steering/torque_limit_raw", 10);
    requested_angle_pub_ = create_publisher<std_msgs::msg::Float32>("/steering/requested_angle", 10);
    angle_error_pub_ = create_publisher<std_msgs::msg::Float32>("/steering/angle_error", 10);
    rate_pub_ = create_publisher<std_msgs::msg::Float32>("/steering/rate", 10);
    raw_rate_pub_ = create_publisher<std_msgs::msg::Float32>("/steering/unfiltered_rate", 10);
    torque_p_pub_ = create_publisher<std_msgs::msg::Float32>("/steering/torque_p_raw", 10);
    torque_d_pub_ = create_publisher<std_msgs::msg::Float32>("/steering/torque_d_raw", 10);
    torque_unsaturated_pub_ = create_publisher<std_msgs::msg::Float32>("/steering/torque_unsaturated_raw", 10);
    torque_saturated_pub_ = create_publisher<std_msgs::msg::Bool>("/steering/torque_saturated", 10);
    torque_settled_pub_ = create_publisher<std_msgs::msg::Bool>("/steering/torque_settled", 10);
    torque_peak_limited_pub_ = create_publisher<std_msgs::msg::Bool>("/steering/torque_peak_limited", 10);
    torque_rate_ready_pub_ = create_publisher<std_msgs::msg::Bool>("/steering/torque_rate_ready", 10);
    torque_safety_pub_ = create_publisher<std_msgs::msg::String>("/steering/torque_safety", 10);
  }

  can_driver_ = driver ? std::move(driver) :
    std::make_unique<KeyaCanDriver>(can_interface, static_cast<uint8_t>(node_id));
  if (!canLinkEnabled()) {
    RCLCPP_WARN(
      get_logger(),
      "CAN link closed. Set hardware_can_enabled=true to observe the motor, and "
      "actuator_output_enabled=true as well to permit CAN actuation.");
  } else if (!can_driver_->open()) {
    RCLCPP_ERROR(get_logger(), "Failed to open SocketCAN interface '%s'", can_interface.c_str());
  } else if (outputArmed()) {
    stopMotor();
    RCLCPP_WARN(
      get_logger(),
      "HARDWARE OUTPUT ARMED: hardware_can_enabled and actuator_output_enabled are both true");
  } else {
    RCLCPP_WARN(
      get_logger(),
      "READ-ONLY: polling the motor and publishing status only. No enable, disable or "
      "position/torque frame is sent. Set actuator_output_enabled=true to permit CAN actuation.");
  }
  parameter_callback_ = add_on_set_parameters_callback(
    std::bind(&SteeringActuatorNode::onParameters, this, std::placeholders::_1));
  previous_tick_ = std::chrono::steady_clock::now();
  control_timer_ = create_wall_timer(
    std::chrono::milliseconds(std::max(1, control_period_ms_)),
    std::bind(&SteeringActuatorNode::controlTick, this));
}

SteeringActuatorNode::~SteeringActuatorNode()
{
  stopMotor();
  discardAllSteeringCommands();
}

void SteeringActuatorNode::stopMotor()
{
  if (!outputArmed() || !can_driver_ || !can_driver_->isOpen()) return;
  // Always attempt Disable, even if the preceding zero frame fails.
  const bool zero_ok = !torqueMode() || can_driver_->sendTorqueCommand(0);
  const bool disable_ok = can_driver_->sendDisable();
  sent_torque_raw_ = 0;
  if (torqueMode()) {
    stop_pending_ = !zero_ok || !disable_ok;
    torque_controller_.reset(seconds(std::chrono::steady_clock::now()));
    torque_output_ = {};
  }
}

void SteeringActuatorNode::torqueFault(TorqueSafetyFault fault)
{
  if (torque_fault_latched_ == TorqueSafetyFault::NONE) {
    RCLCPP_ERROR(get_logger(),
      "Torque safety stop: %s. Inspect cause, set steering_enabled=false then true to reset.",
      torqueSafetyFaultName(fault));
  }
  torque_fault_latched_ = fault;
  requestImmediateDisable(SteeringDisableReason::MOTOR_FAULT);
  disable_reason_ = SteeringDisableReason::MOTOR_FAULT;
  state_ = SteeringState::FAULT;
}

rcl_interfaces::msg::SetParametersResult SteeringActuatorNode::onParameters(
  const std::vector<rclcpp::Parameter> & parameters)
{
  rcl_interfaces::msg::SetParametersResult result;
  auto candidate = torque_controller_.config();
  bool enabled = steering_enabled_;
  bool tuning_changed = false;
  try {
    for (const auto & p : parameters) {
      const auto & name = p.get_name();
      // These were historically read once. Reject misleading runtime writes.
      if (name == "steering_enabled") {
        enabled = p.as_bool();
        continue;
      }
      if (name == "use_sim_time") continue;
      if (!torqueMode()) throw std::invalid_argument(name + " is startup-only in position mode");
      if (name == "torque_full_scale_error_deg") candidate.full_scale_error_rad = p.as_double() * kDegToRad;
      else if (name == "torque_kd") candidate.kd = p.as_double();
      else if (name == "torque_peak_limit_raw" || name == "torque_sustained_limit_raw") {
        const auto value = p.as_int();
        if (value < 1 || value > 10000) throw std::invalid_argument("torque raw limit must be 1..10000");
        if (name == "torque_peak_limit_raw") candidate.peak_limit_raw = static_cast<int32_t>(value);
        else candidate.sustained_limit_raw = static_cast<int32_t>(value);
      } else if (name == "torque_peak_duration_ms") candidate.peak_duration_sec = p.as_int() / 1000.0;
      else if (name == "torque_peak_recovery_ms") candidate.peak_recovery_sec = p.as_int() / 1000.0;
      else if (name == "torque_settle_angle_deg") candidate.settle_angle_rad = p.as_double() * kDegToRad;
      else if (name == "torque_settle_rate_deg_s") candidate.settle_rate_rad_s = p.as_double() * kDegToRad;
      else if (name == "torque_rate_filter_hz") candidate.rate_filter_hz = p.as_double();
      else if (name == "torque_soft_limit_margin_deg") candidate.soft_limit_margin_rad = p.as_double() * kDegToRad;
      else if (name == "torque_hard_rate_limit_deg_s") candidate.hard_rate_limit_rad_s = p.as_double() * kDegToRad;
      else throw std::invalid_argument(name + " is startup-only");
      tuning_changed = true;
    }
    candidate.validate();
  } catch (const std::exception & error) {
    result.successful = false;
    result.reason = error.what();
    return result;
  }
  // Validate the entire batch before any output/state mutation.
  if (tuning_changed || enabled != steering_enabled_) {
    stopMotor();
    if (tuning_changed) torque_controller_.configure(candidate);
    steering_enabled_ = enabled;
    if (!enabled) torque_fault_latched_ = TorqueSafetyFault::NONE;
    enterWaitCommand();
  }
  result.successful = true;
  return result;
}

void SteeringActuatorNode::onCommand(const kaiev26_msgs::msg::ActuatorCommand::SharedPtr input)
{
  const auto stamp = rclcpp::Time(input->header.stamp);
  const bool stamp_present = stamp.nanoseconds() > 0;
  const double stamp_age_ms = stamp_present ?
    (now() - stamp).seconds() * 1000.0 :
    std::numeric_limits<double>::infinity();
  const bool stamp_fresh =
    std::isfinite(stamp_age_ms) &&
    (!torqueMode() || stamp_age_ms >= 0.0) &&
    stamp_age_ms <= static_cast<double>(steer_command_timeout_ms_);
  const bool brake_activated = !safety_.command_brake && input->brake_engage;
  safety_.command_brake = nextCommandBrakeState(
    safety_.command_brake, input->brake_engage, stamp_fresh);
  if (brake_activated) {
    requestImmediateDisable(SteeringDisableReason::COMMAND_BRAKE);
  }
  if (!steering_enabled_) return;

  if (!std::isfinite(input->steering_target_rad) ||
    !stamp_fresh)
  {
    requestImmediateDisable(
      safety_.command_brake ?
      SteeringDisableReason::COMMAND_BRAKE :
      SteeringDisableReason::STEER_COMMAND_TIMEOUT);
    return;
  }
  // 내부와 외부 모두 중앙 등가 조향각·좌회전 양수다. 여기서는 단위만 바꾼다.
  constexpr double kCanonicalRadToDeg = 180.0 / 3.14159265358979323846;
  pending_target_wheel_angle_deg_ =
    converter_.clampEquivalentDeg(input->steering_target_rad * kCanonicalRadToDeg);
  pending_command_valid_ = true;
  last_steer_command_time_ = std::chrono::steady_clock::now();
  torque_command_deadline_ = last_steer_command_time_ +
    std::chrono::duration_cast<std::chrono::steady_clock::duration>(
    std::chrono::duration<double, std::milli>(steer_command_timeout_ms_ - stamp_age_ms));
  pending_command_generation_ = ++steer_command_generation_;
}

void SteeringActuatorNode::applyMode(uint8_t mode_value)
{
  const DriveMode previous_mode = drive_mode_;
  const bool was_received = mode_received_;
  mode_received_ = true;
  last_mode_time_ = std::chrono::steady_clock::now();
  if (mode_value == static_cast<uint8_t>(DriveMode::MANUAL)) drive_mode_ = DriveMode::MANUAL;
  else if (mode_value == static_cast<uint8_t>(DriveMode::AUTO)) drive_mode_ = DriveMode::AUTO;
  else drive_mode_ = DriveMode::UNKNOWN;
  if (drive_mode_ != DriveMode::AUTO &&
    (!was_received || previous_mode != drive_mode_))
  {
    requestImmediateDisable(
      drive_mode_ == DriveMode::MANUAL ? SteeringDisableReason::MANUAL_MODE :
      SteeringDisableReason::INVALID_MODE);
  }
}

void SteeringActuatorNode::onDriveStatus(
  const kaiev26_msgs::msg::DriveActuatorStatus::SharedPtr input)
{
  // 물리 모드 스위치 상태는 STM_A의 drive status로 전달된다.
  applyMode(input->mode);
  drive_speed_permits_steering_ =
    driveSpeedPermitsSteering(
    input->speed_mps, input->part1_valid,
    static_cast<float>(minimum_steering_speed_mps_));
  if (!drive_speed_permits_steering_) {
    // Drop the old target and disable immediately, before the next CAN control tick.
    enterWaitCommand();
  }
}

void SteeringActuatorNode::onBrakeStatus(
  const kaiev26_msgs::msg::BrakeActuatorStatus::SharedPtr input)
{
  // 물리 E-stop 상태는 STM_B의 brake status 원인 bit로 전달된다.
  setEstop(
    (input->stop_reason_bits &
     kaiev26_msgs::msg::BrakeActuatorStatus::STOP_REASON_ESTOP) != 0,
    false);
}

void SteeringActuatorNode::setEstop(bool active, bool test_input)
{
  const bool was_active = safety_.effectiveEstop();
  if (test_input) safety_.test_estop = active;
  else safety_.real_estop = active;
  if (!was_active && safety_.effectiveEstop()) {
    requestImmediateDisable(SteeringDisableReason::ESTOP);
  }
}

void SteeringActuatorNode::requestImmediateDisable(SteeringDisableReason reason)
{
  if (enabled_commanded_ || torqueMode()) stopMotor();
  discardAllSteeringCommands();
  enabled_commanded_ = false;
  requested_disable_reason_ = reason;
}

void SteeringActuatorNode::discardAllSteeringCommands()
{
  target_wheel_angle_deg_ = actual_wheel_angle_deg_;
  commanded_wheel_angle_deg_ = actual_wheel_angle_deg_;
  corrected_wheel_angle_deg_ = actual_wheel_angle_deg_;
  movement_target_valid_ = false;
  pending_command_valid_ = false;
  pending_target_wheel_angle_deg_ = actual_wheel_angle_deg_;
  pending_command_generation_ = 0;
  minimum_accepted_command_generation_ = steer_command_generation_ + 1;
  last_steer_command_time_ = {};
  tracking_controller_.reset();
  if (torqueMode()) {
    torque_controller_.reset(seconds(std::chrono::steady_clock::now()));
    torque_output_ = {};
  }
}

void SteeringActuatorNode::enterWaitCommand(SteeringDisableReason reason)
{
  if (enabled_commanded_) stopMotor();
  discardAllSteeringCommands();
  enabled_commanded_ = false;
  disable_reason_ = reason;
  state_ = SteeringState::WAIT_CMD;
}

bool SteeringActuatorNode::tryEnterActiveFromPendingCommand(
  std::chrono::steady_clock::time_point now)
{
  const bool fresh = last_steer_command_time_.time_since_epoch().count() != 0 &&
    std::chrono::duration<double, std::milli>(now - last_steer_command_time_).count() <=
    steer_command_timeout_ms_ && (!torqueMode() || now <= torque_command_deadline_);
  if (!pendingCommandIsEligible(
      pending_command_valid_, pending_command_generation_,
      minimum_accepted_command_generation_, fresh))
  {
    return false;
  }
  if (!outputArmed() || stop_pending_) {
    // 읽기 전용에서는 ACTIVE로 올라가지 않는다. 받아들일 만한 명령이 와 있는데도
    // 출력이 잠겨 있다는 사실만 알리고, 명령은 pending으로 남겨 status에 드러낸다.
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 5000,
      "Steering command held: motor output is not armed (read-only)");
    return false;
  }
  commanded_wheel_angle_deg_ = actual_wheel_angle_deg_;
  corrected_wheel_angle_deg_ = actual_wheel_angle_deg_;
  target_wheel_angle_deg_ = pending_target_wheel_angle_deg_;
  tracking_controller_.reset();
  if (!can_driver_->isOpen()) {
    discardAllSteeringCommands();
    return false;
  }
  const int32_t actual_count = can_driver_->motorState().motor_position_count;
  if (torqueMode()) {
    torque_controller_.reset(seconds(now));
    torque_controller_.observeFeedback(actual_wheel_angle_deg_ * kDegToRad,
      seconds(can_driver_->motorState().last_position_feedback));
    torque_output_ = {};
    sent_torque_raw_ = 0;
  }
  if (!(torqueMode() ? can_driver_->sendTorqueCommand(0) :
    can_driver_->sendPositionCount(actual_count)))
  {
    if (torqueMode()) stopMotor();
    discardAllSteeringCommands();
    return false;
  }
  if (!can_driver_->sendEnable()) {
    if (torqueMode()) stopMotor();
    discardAllSteeringCommands();
    return false;
  }
  enabled_commanded_ = true;
  movement_target_valid_ = true;
  pending_command_valid_ = false;
  state_ = SteeringState::ACTIVE;
  return true;
}

void SteeringActuatorNode::transitionTo(SteeringState next)
{
  if (next == state_) return;
  if (!shouldMotorBeEnabled(next)) {
    if (enabled_commanded_ || torqueMode()) stopMotor();
    if (torqueMode()) discardAllSteeringCommands();
    target_wheel_angle_deg_ = actual_wheel_angle_deg_;
    commanded_wheel_angle_deg_ = actual_wheel_angle_deg_;
    corrected_wheel_angle_deg_ = actual_wheel_angle_deg_;
    movement_target_valid_ = false;
    tracking_controller_.reset();
    enabled_commanded_ = false;
  }
  state_ = next;
}

void SteeringActuatorNode::controlTick()
{
  const auto now = std::chrono::steady_clock::now();
  double dt = std::chrono::duration<double>(now - previous_tick_).count();
  previous_tick_ = now;
  if (!std::isfinite(dt) || dt <= 0.0) dt = control_period_ms_ / 1000.0;
  if (torqueMode() && stop_pending_) stopMotor();

  if (can_driver_->isOpen()) {
    can_driver_->receiveAvailableFrames();
    can_driver_->requestEncoderPosition();
    if (tick_count_ % 5 == 0) can_driver_->requestFault();
    if (tick_count_ % 10 == 1) can_driver_->requestMotorSpeed();
    if (tick_count_ % 10 == 3) can_driver_->requestMotorCurrent();
    if (tick_count_ % 25 == 5) can_driver_->requestBusVoltage();
    if (tick_count_ % 25 == 15) can_driver_->requestDriverTemperature();
  }
  ++tick_count_;

  // receiveAvailableFrames()가 수신 프레임에 찍은 시각보다 뒤에서 age를 계산한다.
  // tick 시작 시각인 now는 제어 dt 계산 기준으로 계속 사용한다.
  const auto age_now = std::chrono::steady_clock::now();
  const auto & motor = can_driver_->motorState();
  double feedback_age_ms = std::numeric_limits<double>::infinity();
  double position_age_ms = std::numeric_limits<double>::infinity();
  double mode_age_ms = std::numeric_limits<double>::infinity();
  double steer_age_ms = std::numeric_limits<double>::infinity();
  if (motor.last_feedback.time_since_epoch().count() != 0) {
    feedback_age_ms =
      std::chrono::duration<double, std::milli>(age_now - motor.last_feedback).count();
  }
  if (motor.last_position_feedback.time_since_epoch().count() != 0) {
    position_age_ms =
      std::chrono::duration<double, std::milli>(
      age_now - motor.last_position_feedback).count();
  }
  can_timeout_ = !can_driver_->isOpen() || feedback_age_ms > can_timeout_ms_;
  if (last_mode_time_.time_since_epoch().count() != 0) {
    mode_age_ms =
      std::chrono::duration<double, std::milli>(age_now - last_mode_time_).count();
  }
  if (last_steer_command_time_.time_since_epoch().count() != 0) {
    steer_age_ms =
      std::chrono::duration<double, std::milli>(
      age_now - last_steer_command_time_).count();
  }
  mode_timeout_ = mode_received_ && mode_age_ms > mode_timeout_ms_;
  const bool drive_speed_allowed = drive_speed_permits_steering_ &&
    mode_age_ms <= drive_status_timeout_ms_;
  if (motor.position_valid) {
    actual_wheel_angle_deg_ = converter_.countToEquivalentDeg(motor.motor_position_count);
  }
  const double position_timeout_ms = torqueMode() ? std::min(
    static_cast<double>(feedback_timeout_ms_),
    torque_controller_.config().feedback_timeout_sec * 1000.0) : feedback_timeout_ms_;
  feedback_timeout_ = !motor.position_valid || position_age_ms > position_timeout_ms ||
    !std::isfinite(actual_wheel_angle_deg_);
  const bool feedback_valid = !feedback_timeout_ && !can_timeout_;
  if (torqueMode() && feedback_valid) {
    const bool count_out_of_range =
      motor.motor_position_count < -converter_.commandCountLimit() ||
      motor.motor_position_count > converter_.commandCountLimit();
    const auto fault = count_out_of_range ? TorqueSafetyFault::ANGLE_LIMIT :
      torque_controller_.observeFeedback(
      actual_wheel_angle_deg_ * kDegToRad, seconds(motor.last_position_feedback));
    if (fault != TorqueSafetyFault::NONE && steering_enabled_) torqueFault(fault);
  }
  // Even freshly queued feedback must not conceal an outer-loop scheduling stall.
  if (torqueMode() && state_ == SteeringState::ACTIVE &&
    dt > torque_controller_.config().feedback_timeout_sec)
  {
    torqueFault(TorqueSafetyFault::FEEDBACK_TIMEOUT);
  }
  const bool motor_fault = (motor.fault_bits & static_cast<uint16_t>(~uint16_t{1})) != 0 ||
    (torqueMode() && torque_fault_latched_ != TorqueSafetyFault::NONE);
  steer_command_timeout_ = state_ == SteeringState::ACTIVE &&
    (!std::isfinite(steer_age_ms) || steer_age_ms > steer_command_timeout_ms_ ||
    (torqueMode() && age_now > torque_command_deadline_));
  disable_reason_ = selectDisableReason(
    mode_received_, mode_timeout_, drive_mode_, safety_, motor_fault, can_timeout_,
    feedback_valid, steer_command_timeout_);
  if (requested_disable_reason_ != SteeringDisableReason::NONE) {
    disable_reason_ = requested_disable_reason_;
    requested_disable_reason_ = SteeringDisableReason::NONE;
  }

  bool entered_active_this_tick = false;
  if (!drive_speed_allowed) {
    enterWaitCommand(disable_reason_);
    if (steering_enabled_ && outputArmed()) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "Steering disabled: requires valid drive speed >= %.3f km/h and fresh /drive/status",
        minimum_steering_speed_mps_ * 3.6);
    }
  }
  if (disable_reason_ != SteeringDisableReason::NONE) {
    if (disable_reason_ == SteeringDisableReason::STEER_COMMAND_TIMEOUT) {
      enterWaitCommand(disable_reason_);
    } else {
      transitionTo(stateForDisableReason(disable_reason_));
    }
  } else if (!steering_enabled_ || !drive_speed_allowed) {
    enterWaitCommand();
  } else {
    if (state_ != SteeringState::WAIT_CMD && state_ != SteeringState::ACTIVE) {
      disable_reason_ = SteeringDisableReason::NONE;
      state_ = SteeringState::WAIT_CMD;
    } else if (state_ == SteeringState::WAIT_CMD) {
      disable_reason_ = SteeringDisableReason::NONE;
      entered_active_this_tick = tryEnterActiveFromPendingCommand(now);
    } else if (pending_command_valid_) {
      target_wheel_angle_deg_ = pending_target_wheel_angle_deg_;
      pending_command_valid_ = false;
    }
  }

  if (state_ == SteeringState::ACTIVE && movement_target_valid_ && !entered_active_this_tick) {
    if (torqueMode()) {
      // Direct target step; the 60 deg/s position shaper does not sit in this path.
      commanded_wheel_angle_deg_ = target_wheel_angle_deg_;
      corrected_wheel_angle_deg_ = target_wheel_angle_deg_;
      torque_output_ = torque_controller_.update(target_wheel_angle_deg_ * kDegToRad,
        0.0, seconds(age_now));  // No target derivative kick on a step.
      if (torque_output_.fault != TorqueSafetyFault::NONE) {
        torqueFault(torque_output_.fault);
      } else if (enabled_commanded_) {
        const auto raw = keyaTorqueCommand(torque_output_.command_raw, torque_direction_sign_);
        if (can_driver_->sendTorqueCommand(raw)) {
          sent_torque_raw_ = raw;
          corrected_wheel_angle_deg_ = torque_output_.target_rad / kDegToRad;
        } else {
          requestImmediateDisable(SteeringDisableReason::CAN_TIMEOUT);
          disable_reason_ = SteeringDisableReason::CAN_TIMEOUT;
          state_ = SteeringState::CAN_LOST;
        }
      }
    } else {
      commanded_wheel_angle_deg_ = rateLimit(
        commanded_wheel_angle_deg_, target_wheel_angle_deg_, max_steering_rate_deg_s_, dt);
      corrected_wheel_angle_deg_ = converter_.clampEquivalentDeg(
        tracking_controller_.update(
          commanded_wheel_angle_deg_, actual_wheel_angle_deg_, dt));
      if (enabled_commanded_) {
        can_driver_->sendPositionCount(
          converter_.equivalentDegToCount(corrected_wheel_angle_deg_));
      }
    }
  } else {
    tracking_controller_.reset();
    if (!torqueMode() && steering_enabled_ && drive_speed_allowed && outputArmed() && feedback_valid &&
      can_driver_->isOpen())
    {
      can_driver_->sendPositionCount(motor.motor_position_count);
    }
  }
  if (torqueMode() && state_ != SteeringState::ACTIVE) {
    torque_controller_.idle(seconds(age_now));
    torque_output_ = {};
  }
  if (state_ != SteeringState::ACTIVE) enabled_commanded_ = false;
  publishState(position_age_ms, mode_age_ms, steer_age_ms);
}

void SteeringActuatorNode::publishState(
  double feedback_age_ms, double mode_age_ms, double steer_age_ms)
{
  const auto & motor = can_driver_->motorState();
  if (motor.position_valid) {
    std_msgs::msg::Float32 angle;
    angle.data = static_cast<float>(actual_wheel_angle_deg_);
    angle_pub_->publish(angle);
  }
  std_msgs::msg::Int32 target_motor_count;
  // Reuse the position-command conversion so dashboard actual and target share
  // the KEYA encoder axis, zero, sign and full count representation.
  target_motor_count.data = converter_.equivalentDegToCount(target_wheel_angle_deg_);
  target_motor_count_pub_->publish(target_motor_count);
  std_msgs::msg::Bool commanded;
  commanded.data = enabled_commanded_;
  enable_commanded_pub_->publish(commanded);
  std_msgs::msg::Bool enabled;
  enabled.data = motor.enabled_feedback_valid && motor.enabled_feedback;
  enabled_pub_->publish(enabled);

  kaiev26_msgs::msg::SteeringActuatorStatus status;
  status.header.stamp = now();
  status.state = static_cast<uint8_t>(state_);
  status.disable_reason = static_cast<uint8_t>(disable_reason_);
  status.drive_mode = static_cast<uint8_t>(drive_mode_);
  status.mode_received = mode_received_;
  status.mode_timeout = mode_timeout_;
  status.steer_command_timeout = steer_command_timeout_;
  status.feedback_timeout = feedback_timeout_;
  status.estop_active = safety_.effectiveEstop();
  status.command_brake_active = safety_.command_brake;
  status.communication_ok = motor.communication_ok && !can_timeout_;
  status.can_timeout = can_timeout_;
  status.feedback_valid = !feedback_timeout_ && !can_timeout_;
  status.enabled_commanded = enabled_commanded_;
  status.enabled_feedback = motor.enabled_feedback;
  status.enabled_feedback_valid = motor.enabled_feedback_valid;
  status.enable_mismatch =
    motor.enabled_feedback_valid && enabled_commanded_ != motor.enabled_feedback;
  status.tracking_control_enabled = tracking_enabled_;
  // Tracking error remains telemetry only; keep the status fields for consumers.
  status.tracking_error_warning = false;
  status.tracking_error_fault = false;
  constexpr double kDegToCanonicalRad = 3.14159265358979323846 / 180.0;
  status.target_wheel_angle_rad =
    static_cast<float>(target_wheel_angle_deg_ * kDegToCanonicalRad);
  status.commanded_wheel_angle_rad =
    static_cast<float>(commanded_wheel_angle_deg_ * kDegToCanonicalRad);
  status.corrected_wheel_angle_rad =
    static_cast<float>(corrected_wheel_angle_deg_ * kDegToCanonicalRad);
  status.actual_wheel_angle_rad =
    static_cast<float>(actual_wheel_angle_deg_ * kDegToCanonicalRad);
  status.tracking_error_rad =
    static_cast<float>(torqueMode() ? torque_output_.error_rad :
    tracking_controller_.errorDeg() * kDegToCanonicalRad);
  status.tracking_correction_rad =
    static_cast<float>(tracking_controller_.correctionDeg() * kDegToCanonicalRad);
  status.motor_position_count = motor.motor_position_count;
  // 모터축 원시 진단값 — 로드휠 부호 변환을 적용하지 않는다
  status.motor_angle_deg =
    static_cast<float>(converter_.countToMotorDeg(motor.motor_position_count));
  status.motor_speed_rpm = motor.motor_speed_rpm;
  status.motor_current_a = motor.motor_current_a;
  status.bus_voltage_v = motor.bus_voltage_v;
  status.driver_temperature_c = motor.driver_temperature_c;
  status.fault_bits = motor.fault_bits;
  status.feedback_age_ms =
    std::isfinite(feedback_age_ms) ? static_cast<float>(feedback_age_ms) :
    std::numeric_limits<float>::infinity();
  status.mode_age_ms = std::isfinite(mode_age_ms) ? static_cast<float>(mode_age_ms) :
    std::numeric_limits<float>::infinity();
  status.steer_command_age_ms = std::isfinite(steer_age_ms) ? static_cast<float>(steer_age_ms) :
    std::numeric_limits<float>::infinity();
  status.steer_command_received = steer_command_generation_ != 0;
  status.pending_command_valid = pending_command_valid_;
  status.movement_target_valid = movement_target_valid_;
  status_pub_->publish(status);
  std_msgs::msg::String mode;
  mode.data = actuation_mode_;
  actuation_mode_pub_->publish(mode);
  if (torqueMode()) publishTorqueState();
}

void SteeringActuatorNode::publishTorqueState()
{
  const auto publish_float = [](const auto & publisher, double value) {
      std_msgs::msg::Float32 msg;
      msg.data = static_cast<float>(value);
      publisher->publish(msg);
    };
  const auto publish_bool = [](const auto & publisher, bool value) {
      std_msgs::msg::Bool msg;
      msg.data = value;
      publisher->publish(msg);
    };
  std_msgs::msg::Int32 raw;
  raw.data = sent_torque_raw_;
  torque_command_pub_->publish(raw);
  raw.data = torque_output_.limit_raw;
  torque_limit_pub_->publish(raw);
  publish_float(requested_angle_pub_, target_wheel_angle_deg_ * kDegToRad);
  publish_float(angle_error_pub_, torque_output_.error_rad);
  publish_float(rate_pub_, torque_output_.rate_rad_s);
  publish_float(raw_rate_pub_, torque_output_.unfiltered_rate_rad_s);
  publish_float(torque_p_pub_, torque_output_.p_raw);
  publish_float(torque_d_pub_, torque_output_.d_raw);
  publish_float(torque_unsaturated_pub_, torque_output_.unsaturated_raw);
  publish_bool(torque_saturated_pub_, torque_output_.saturated);
  publish_bool(torque_settled_pub_, torque_output_.settled);
  publish_bool(torque_peak_limited_pub_, torque_output_.peak_limited);
  publish_bool(torque_rate_ready_pub_, torque_output_.rate_ready);
  std_msgs::msg::String safety;
  safety.data = torqueSafetyFaultName(torque_fault_latched_);
  torque_safety_pub_->publish(safety);
}
}  // namespace lateral_ctrl
