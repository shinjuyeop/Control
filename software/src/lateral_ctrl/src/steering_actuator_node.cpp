#include "lateral_ctrl/steering_actuator_node.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <functional>
#include <limits>

namespace lateral_ctrl
{
SteeringActuatorNode::SteeringActuatorNode(const rclcpp::NodeOptions & options)
: Node("steering_actuator_node", options),
  converter_(
    declare_parameter("calibration_a", 0.0002),
    declare_parameter("calibration_b", 0.2159),
    declare_parameter("calibration_c", 0.1612),
    declare_parameter("counts_per_rev", 10000.0),
    declare_parameter("direction_sign", -1.0),
    declare_parameter("wheel_limit_deg", 30.0)),
  tracking_controller_(
    declare_parameter("tracking_control_enabled", false),
    declare_parameter("tracking_kp", 0.0),
    declare_parameter("tracking_ki", 0.0),
    declare_parameter("tracking_kd", 0.0),
    declare_parameter("tracking_integral_limit", 1.0),
    declare_parameter("tracking_correction_limit_deg", 0.5))
{
  tracking_enabled_ = get_parameter("tracking_control_enabled").as_bool();
  control_period_ms_ = declare_parameter("control_period_ms", 20);
  can_timeout_ms_ = declare_parameter("can_timeout_ms", 500);
  feedback_timeout_ms_ = declare_parameter("feedback_timeout_ms", 500);
  mode_timeout_ms_ = declare_parameter("mode_timeout_ms", 1500);
  steer_command_timeout_ms_ = declare_parameter("steer_command_timeout_ms", 200);
  max_steering_rate_deg_s_ = declare_parameter("max_steering_rate_deg_s", 60.0);
  tracking_error_warning_deg_ = declare_parameter("tracking_error_warning_deg", 1.0);
  tracking_error_fault_deg_ = declare_parameter("tracking_error_fault_deg", 3.0);
  tracking_error_fault_duration_ms_ =
    declare_parameter("tracking_error_fault_duration_ms", 500);
  hardware_can_enabled_ = declare_parameter("hardware_can_enabled", false);
  actuator_output_enabled_ = declare_parameter("actuator_output_enabled", false);
  test_input_injection_enabled_ = declare_parameter("test_input_injection_enabled", false);
  const auto can_interface = declare_parameter("can_interface", std::string("can0"));
  const auto node_id = declare_parameter("node_id", 1);

  // canonical 통합 인터페이스: 명령은 /vehicle/actuator_command 하나,
  // 안전·모드 상태는 STM이 발행하는 status에서 읽는다.
  const auto command_topic =
    declare_parameter("command_topic", std::string("/vehicle/actuator_command"));
  const auto drive_status_topic =
    declare_parameter("drive_status_topic", std::string("/drive/status"));
  const auto brake_status_topic =
    declare_parameter("brake_status_topic", std::string("/brake/status"));
  const auto aeb_state_topic =
    declare_parameter("aeb_state_topic", std::string("/safety/aeb_state"));
  const auto test_estop_topic =
    declare_parameter("test_estop_topic", std::string("/steering/test/estop"));
  const auto test_aeb_topic =
    declare_parameter("test_aeb_topic", std::string("/steering/test/aeb"));
  const auto angle_topic = declare_parameter("angle_topic", std::string("/steering/angle"));
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
  aeb_state_sub_ = create_subscription<kaiev26_msgs::msg::AEBState>(
    aeb_state_topic, 10,
    std::bind(&SteeringActuatorNode::onAebState, this, std::placeholders::_1));
  if (test_input_injection_enabled_) {
    test_estop_sub_ = create_subscription<std_msgs::msg::UInt8>(
      test_estop_topic, 10,
      [this](std_msgs::msg::UInt8::SharedPtr message) {setEstop(message->data != 0, true);});
    test_aeb_sub_ = create_subscription<std_msgs::msg::UInt8>(
      test_aeb_topic, 10,
      [this](std_msgs::msg::UInt8::SharedPtr message) {setAeb(message->data != 0, true);});
  }
  angle_pub_ = create_publisher<std_msgs::msg::Float32>(angle_topic, 10);
  status_pub_ = create_publisher<kaiev26_msgs::msg::SteeringActuatorStatus>(status_topic, 10);
  enable_commanded_pub_ = create_publisher<std_msgs::msg::Bool>(enable_commanded_topic, 10);
  enabled_pub_ = create_publisher<std_msgs::msg::Bool>(enabled_topic, 10);

  can_driver_ = std::make_unique<KeyaCanDriver>(can_interface, static_cast<uint8_t>(node_id));
  if (outputArmed()) {
    if (!can_driver_->open()) {
      RCLCPP_ERROR(get_logger(), "Failed to open SocketCAN interface '%s'", can_interface.c_str());
    } else {
      can_driver_->sendDisable();
      RCLCPP_WARN(
        get_logger(),
        "HARDWARE OUTPUT ARMED: hardware_can_enabled and actuator_output_enabled are both true");
    }
  } else {
    RCLCPP_WARN(
      get_logger(),
      "Motor output disarmed. Set BOTH hardware_can_enabled=true and "
      "actuator_output_enabled=true to permit CAN actuation.");
  }
  previous_tick_ = std::chrono::steady_clock::now();
  control_timer_ = create_wall_timer(
    std::chrono::milliseconds(std::max(1, control_period_ms_)),
    std::bind(&SteeringActuatorNode::controlTick, this));
}

SteeringActuatorNode::~SteeringActuatorNode()
{
  discardAllSteeringCommands();
  if (can_driver_ && can_driver_->isOpen()) can_driver_->sendDisable();
}

void SteeringActuatorNode::onCommand(const kaiev26_msgs::msg::ActuatorCommand::SharedPtr input)
{
  if (!std::isfinite(input->steering_target_rad)) return;
  // canonical 계약 단위 변환: rad·좌회전 양수 → 내부 deg·좌회전 음수.
  // 이 변환은 명령 수신 경계에서만 수행한다.
  constexpr double kCanonicalRadToDeg = -180.0 / 3.14159265358979323846;
  pending_target_wheel_angle_deg_ =
    converter_.clampWheelDeg(input->steering_target_rad * kCanonicalRadToDeg);
  pending_command_valid_ = true;
  last_steer_command_time_ = std::chrono::steady_clock::now();
  pending_command_generation_ = ++steer_command_generation_;
}

void SteeringActuatorNode::applyMode(uint8_t mode_value)
{
  mode_received_ = true;
  last_mode_time_ = std::chrono::steady_clock::now();
  if (mode_value == static_cast<uint8_t>(DriveMode::MANUAL)) drive_mode_ = DriveMode::MANUAL;
  else if (mode_value == static_cast<uint8_t>(DriveMode::AUTO)) drive_mode_ = DriveMode::AUTO;
  else drive_mode_ = DriveMode::UNKNOWN;
  if (drive_mode_ != DriveMode::AUTO) {
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

void SteeringActuatorNode::onAebState(const kaiev26_msgs::msg::AEBState::SharedPtr input)
{
  setAeb(input->active, false);
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

void SteeringActuatorNode::setAeb(bool active, bool test_input)
{
  const bool was_active = safety_.effectiveAeb();
  if (test_input) safety_.test_aeb = active;
  else safety_.real_aeb = active;
  if (!was_active && safety_.effectiveAeb()) {
    requestImmediateDisable(SteeringDisableReason::AEB);
  }
}

void SteeringActuatorNode::requestImmediateDisable(SteeringDisableReason reason)
{
  discardAllSteeringCommands();
  if (enabled_commanded_ && can_driver_->isOpen()) can_driver_->sendDisable();
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
}

void SteeringActuatorNode::enterWaitCommand(SteeringDisableReason reason)
{
  discardAllSteeringCommands();
  if (enabled_commanded_ && can_driver_->isOpen()) can_driver_->sendDisable();
  enabled_commanded_ = false;
  disable_reason_ = reason;
  state_ = SteeringState::WAIT_CMD;
}

bool SteeringActuatorNode::tryEnterActiveFromPendingCommand(
  std::chrono::steady_clock::time_point now)
{
  const bool fresh = last_steer_command_time_.time_since_epoch().count() != 0 &&
    std::chrono::duration<double, std::milli>(now - last_steer_command_time_).count() <=
    steer_command_timeout_ms_;
  if (!pendingCommandIsEligible(
      pending_command_valid_, pending_command_generation_,
      minimum_accepted_command_generation_, fresh))
  {
    return false;
  }
  commanded_wheel_angle_deg_ = actual_wheel_angle_deg_;
  corrected_wheel_angle_deg_ = actual_wheel_angle_deg_;
  target_wheel_angle_deg_ = pending_target_wheel_angle_deg_;
  tracking_controller_.reset();
  if (!can_driver_->isOpen() || !can_driver_->sendEnable()) {
    discardAllSteeringCommands();
    return false;
  }
  enabled_commanded_ = true;
  const int32_t actual_count =
    converter_.motorDegToCount(converter_.wheelToMotorDeg(actual_wheel_angle_deg_));
  if (!can_driver_->sendPositionCount(actual_count)) {
    can_driver_->sendDisable();
    enabled_commanded_ = false;
    discardAllSteeringCommands();
    return false;
  }
  movement_target_valid_ = true;
  pending_command_valid_ = false;
  state_ = SteeringState::ACTIVE;
  return true;
}

void SteeringActuatorNode::transitionTo(SteeringState next)
{
  if (next == state_) return;
  if (!shouldMotorBeEnabled(next)) {
    discardAllSteeringCommands();
    if (enabled_commanded_ && can_driver_->isOpen()) can_driver_->sendDisable();
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

  const auto & motor = can_driver_->motorState();
  double feedback_age_ms = std::numeric_limits<double>::infinity();
  double position_age_ms = std::numeric_limits<double>::infinity();
  double mode_age_ms = std::numeric_limits<double>::infinity();
  double steer_age_ms = std::numeric_limits<double>::infinity();
  if (motor.last_feedback.time_since_epoch().count() != 0) {
    feedback_age_ms =
      std::chrono::duration<double, std::milli>(now - motor.last_feedback).count();
  }
  if (motor.last_position_feedback.time_since_epoch().count() != 0) {
    position_age_ms =
      std::chrono::duration<double, std::milli>(now - motor.last_position_feedback).count();
  }
  can_timeout_ = !can_driver_->isOpen() || feedback_age_ms > can_timeout_ms_;
  if (last_mode_time_.time_since_epoch().count() != 0) {
    mode_age_ms = std::chrono::duration<double, std::milli>(now - last_mode_time_).count();
  }
  if (last_steer_command_time_.time_since_epoch().count() != 0) {
    steer_age_ms =
      std::chrono::duration<double, std::milli>(now - last_steer_command_time_).count();
  }
  mode_timeout_ = mode_received_ && mode_age_ms > mode_timeout_ms_;
  if (motor.position_valid) {
    actual_wheel_angle_deg_ =
      converter_.motorToWheelDeg(converter_.countToMotorDeg(motor.motor_position_count));
  }
  feedback_timeout_ = !motor.position_valid || position_age_ms > feedback_timeout_ms_;
  const bool feedback_valid = !feedback_timeout_ && !can_timeout_;
  const bool motor_fault = (motor.fault_bits & static_cast<uint16_t>(~uint16_t{1})) != 0;
  steer_command_timeout_ = state_ == SteeringState::ACTIVE &&
    (!std::isfinite(steer_age_ms) || steer_age_ms > steer_command_timeout_ms_);
  disable_reason_ = selectDisableReason(
    mode_received_, mode_timeout_, drive_mode_, safety_, motor_fault, can_timeout_,
    feedback_valid, steer_command_timeout_);
  if (requested_disable_reason_ != SteeringDisableReason::NONE) {
    disable_reason_ = requested_disable_reason_;
    requested_disable_reason_ = SteeringDisableReason::NONE;
  }

  bool entered_active_this_tick = false;
  if (disable_reason_ != SteeringDisableReason::NONE) {
    if (disable_reason_ == SteeringDisableReason::STEER_COMMAND_TIMEOUT) {
      enterWaitCommand(disable_reason_);
    } else {
      transitionTo(stateForDisableReason(disable_reason_));
    }
  } else {
    if (state_ != SteeringState::WAIT_CMD && state_ != SteeringState::ACTIVE) {
      enterWaitCommand();
    } else if (state_ == SteeringState::WAIT_CMD) {
      disable_reason_ = SteeringDisableReason::NONE;
      entered_active_this_tick = tryEnterActiveFromPendingCommand(now);
    } else if (pending_command_valid_) {
      target_wheel_angle_deg_ = pending_target_wheel_angle_deg_;
      pending_command_valid_ = false;
    }
  }

  steering_rate_deg_s_ = 0.0;
  if (state_ == SteeringState::ACTIVE && movement_target_valid_ && !entered_active_this_tick) {
    const double previous = commanded_wheel_angle_deg_;
    commanded_wheel_angle_deg_ = rateLimit(
      previous, target_wheel_angle_deg_, max_steering_rate_deg_s_, dt);
    steering_rate_deg_s_ = (commanded_wheel_angle_deg_ - previous) / dt;
    corrected_wheel_angle_deg_ = converter_.clampWheelDeg(
      tracking_controller_.update(
        commanded_wheel_angle_deg_, actual_wheel_angle_deg_, dt));
    if (enabled_commanded_) {
      can_driver_->sendPositionCount(
        converter_.motorDegToCount(converter_.wheelToMotorDeg(corrected_wheel_angle_deg_)));
    }
  } else {
    tracking_controller_.reset();
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
  status.aeb_active = safety_.effectiveAeb();
  status.communication_ok = motor.communication_ok && !can_timeout_;
  status.can_timeout = can_timeout_;
  status.feedback_valid =
    motor.position_valid &&
    motor.last_position_feedback.time_since_epoch().count() != 0 &&
    std::chrono::duration<double, std::milli>(
      std::chrono::steady_clock::now() - motor.last_position_feedback).count() <= can_timeout_ms_ &&
    !can_timeout_;
  status.enabled_commanded = enabled_commanded_;
  status.enabled_feedback = motor.enabled_feedback;
  status.enabled_feedback_valid = motor.enabled_feedback_valid;
  status.enable_mismatch =
    motor.enabled_feedback_valid && enabled_commanded_ != motor.enabled_feedback;
  status.tracking_control_enabled = tracking_enabled_;
  status.tracking_error_warning =
    std::abs(tracking_controller_.errorDeg()) >= tracking_error_warning_deg_;
  const bool fault_error_now =
    std::abs(tracking_controller_.errorDeg()) >= tracking_error_fault_deg_;
  if (!fault_error_now) {
    tracking_fault_since_ = {};
    status.tracking_error_fault = false;
  } else {
    const auto steady_now = std::chrono::steady_clock::now();
    if (tracking_fault_since_.time_since_epoch().count() == 0) {
      tracking_fault_since_ = steady_now;
    }
    status.tracking_error_fault =
      std::chrono::duration_cast<std::chrono::milliseconds>(
        steady_now - tracking_fault_since_).count() >= tracking_error_fault_duration_ms_;
  }
  // canonical 계약 단위 변환: 내부 deg·좌회전 음수 → rad·좌회전 양수.
  // 이 변환은 status 발행 경계에서만 수행한다.
  constexpr double kDegToCanonicalRad = -3.14159265358979323846 / 180.0;
  status.target_wheel_angle_rad =
    static_cast<float>(target_wheel_angle_deg_ * kDegToCanonicalRad);
  status.commanded_wheel_angle_rad =
    static_cast<float>(commanded_wheel_angle_deg_ * kDegToCanonicalRad);
  status.corrected_wheel_angle_rad =
    static_cast<float>(corrected_wheel_angle_deg_ * kDegToCanonicalRad);
  status.actual_wheel_angle_rad =
    static_cast<float>(actual_wheel_angle_deg_ * kDegToCanonicalRad);
  status.tracking_error_rad =
    static_cast<float>(tracking_controller_.errorDeg() * kDegToCanonicalRad);
  status.tracking_correction_rad =
    static_cast<float>(tracking_controller_.correctionDeg() * kDegToCanonicalRad);
  status.steering_rate_radps =
    static_cast<float>(steering_rate_deg_s_ * kDegToCanonicalRad);
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
}
}  // namespace lateral_ctrl

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<lateral_ctrl::SteeringActuatorNode>());
  rclcpp::shutdown();
  return 0;
}
