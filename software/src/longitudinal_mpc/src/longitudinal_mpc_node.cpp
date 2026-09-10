#include "longitudinal_mpc/osqp_mpc.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <limits>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>
#include <vector>

#include <kaiev26_msgs/msg/actuator_command.hpp>
#include <kaiev26_msgs/msg/drive_actuator_status.hpp>
#include <kaiev26_msgs/msg/drive_current_command.hpp>
#include <kaiev26_msgs/msg/vehicle_state.hpp>
#include <rclcpp/rclcpp.hpp>

namespace longitudinal_mpc
{
namespace
{

using ActuatorCommand = kaiev26_msgs::msg::ActuatorCommand;
using DriveActuatorStatus = kaiev26_msgs::msg::DriveActuatorStatus;
using DriveCurrentCommand = kaiev26_msgs::msg::DriveCurrentCommand;
using VehicleState = kaiev26_msgs::msg::VehicleState;
using SteadyClock = std::chrono::steady_clock;
constexpr std::size_t kTimingWindowCapacity = 2048U;

struct TimingSummary
{
  std::size_t totalCount{0U};
  std::size_t windowCount{0U};
  double medianMs{0.0};
  double p95Ms{0.0};
  double maximumAllMs{0.0};
};

class BoundedTiming
{
public:
  void Add(const double sampleMs)
  {
    ++totalCount_;
    maximumAllMs_ = std::max(maximumAllMs_, sampleMs);
    if (window_.size() < kTimingWindowCapacity) {
      window_.push_back(sampleMs);
      return;
    }
    window_[next_] = sampleMs;
    next_ = (next_ + 1U) % kTimingWindowCapacity;
  }

  TimingSummary Summary() const
  {
    TimingSummary summary;
    summary.totalCount = totalCount_;
    summary.windowCount = window_.size();
    summary.maximumAllMs = maximumAllMs_;
    if (window_.empty()) {
      return summary;
    }
    std::vector<double> sorted = window_;
    std::sort(sorted.begin(), sorted.end());
    const auto quantile = [&sorted](const double q) {
        const double position = q * static_cast<double>(sorted.size() - 1U);
        const std::size_t lower = static_cast<std::size_t>(std::floor(position));
        const std::size_t upper = static_cast<std::size_t>(std::ceil(position));
        const double fraction = position - static_cast<double>(lower);
        return sorted[lower] + fraction * (sorted[upper] - sorted[lower]);
      };
    summary.medianMs = quantile(0.5);
    summary.p95Ms = quantile(0.95);
    return summary;
  }

private:
  std::vector<double> window_;
  std::size_t next_{0U};
  std::size_t totalCount_{0U};
  double maximumAllMs_{0.0};
};

double ElapsedSeconds(
  const SteadyClock::time_point now,
  const std::optional<SteadyClock::time_point> & then)
{
  if (!then.has_value()) {
    return std::numeric_limits<double>::infinity();
  }
  return std::chrono::duration<double>(now - *then).count();
}

}  // namespace

class LongitudinalMpcNode final : public rclcpp::Node
{
public:
  LongitudinalMpcNode()
  : Node("longitudinal_mpc")
  {
    const auto commandTopic = declare_parameter<std::string>(
      "command_topic", "/vehicle/command");
    const auto vehicleStateTopic = declare_parameter<std::string>(
      "vehicle_state_topic", "/vehicle/state");
    const auto driveStatusTopic = declare_parameter<std::string>(
      "drive_status_topic", "/drive/status");
    const auto outputTopic = declare_parameter<std::string>(
      "output_topic", "/vehicle/drive_current_command");
    const double rateHz = declare_parameter<double>("rate_hz", 20.0);

    MpcParameters parameters;
    parameters.model.dtSec = 1.0 / rateHz;
    parameters.horizonSec = declare_parameter<double>("horizon_s", 2.0);
    const auto controlBlockSteps = declare_parameter<std::int64_t>(
      "control_block_steps", 4);
    parameters.maximumCurrentA = declare_parameter<double>(
      "maximum_current_a", 300.0);
    parameters.currentSlewAps = declare_parameter<double>(
      "current_slew_a_s", 300.0);
    commandTimeoutSec_ = declare_parameter<double>("command_timeout_sec", 0.2);
    feedbackTimeoutSec_ = declare_parameter<double>("feedback_timeout_sec", 0.2);
    parameters.model.vehicleMassKg = declare_parameter<double>(
      "vehicle_mass_kg", 285.0);
    parameters.model.motorTorqueConstantNmPerA = declare_parameter<double>(
      "motor_torque_constant_nm_per_a", 0.11);
    parameters.model.motorFinalDriveRatio = declare_parameter<double>(
      "motor_final_drive_ratio", 4.6);
    parameters.model.drivetrainEfficiency = declare_parameter<double>(
      "drivetrain_efficiency", 0.95);
    parameters.model.tireRadiusM = declare_parameter<double>(
      "tire_radius_m", 0.2374765591);
    parameters.model.effectiveTorqueScale = declare_parameter<double>(
      "effective_torque_scale", 0.6839881619421034);
    parameters.model.speedDampingPerSec = declare_parameter<double>(
      "speed_damping_per_s", 0.04283179368466928);
    parameters.model.resistanceAccelMps2 = declare_parameter<double>(
      "resistance_accel_mps2", 0.35551481265086643);
    parameters.model.currentResponseAlpha = declare_parameter<double>(
      "current_response_alpha", 0.04078012055497491);
    parameters.model.currentCommandGain = declare_parameter<double>(
      "current_command_gain", 1.0055195705281386);
    parameters.inputWeight = declare_parameter<double>("input_weight", 0.002);
    parameters.slewWeight = declare_parameter<double>("slew_weight", 0.05);
    parameters.solverTimeLimitSec = declare_parameter<double>(
      "solver_time_limit_sec", 0.02);

    if (!std::isfinite(rateHz) || rateHz <= 0.0 ||
      controlBlockSteps <= 0 ||
      !std::isfinite(commandTimeoutSec_) || commandTimeoutSec_ <= 0.0 ||
      !std::isfinite(feedbackTimeoutSec_) || feedbackTimeoutSec_ <= 0.0)
    {
      throw std::invalid_argument("invalid MPC rate or timeout");
    }
    parameters.controlBlockSteps = static_cast<std::size_t>(controlBlockSteps);
    mpc_ = std::make_unique<OsqpMpc>(parameters);

    publisher_ = create_publisher<DriveCurrentCommand>(outputTopic, 10);
    commandSubscription_ = create_subscription<ActuatorCommand>(
      commandTopic, 10,
      [this](const ActuatorCommand::SharedPtr message) {OnCommand(*message);});
    stateSubscription_ = create_subscription<VehicleState>(
      vehicleStateTopic, 10,
      [this](const VehicleState::SharedPtr message) {OnVehicleState(*message);});
    driveSubscription_ = create_subscription<DriveActuatorStatus>(
      driveStatusTopic, 10,
      [this](const DriveActuatorStatus::SharedPtr message) {OnDriveStatus(*message);});
    const auto period = std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::duration<double>(1.0 / rateHz));
    timer_ = create_wall_timer(period, [this]() {OnTimer();});

    const auto & model = mpc_->Model();
    RCLCPP_INFO(
      get_logger(),
      "C++ OSQP %s MPC: N=%zu M=%zu rate=%.1f Hz, model "
      "A=[[%.12f,%.12f],[0,%.12f]] B=[0,%.12f] c=[%.12f,0]",
      OSQP_VERSION, mpc_->PredictionSteps(), mpc_->ControlBlocks(), rateHz,
      model.speedDecay, model.currentToSpeed, model.currentDecay,
      model.commandToCurrent, model.speedOffset);
    RCLCPP_INFO(
      get_logger(),
      "physical model: mass=%.3f kg Kt=%.6f Nm/A G=%.6f eta=%.6f "
      "radius=%.10f m scale=%.12f nominal_gain=%.12f effective_gain=%.12f",
      parameters.model.vehicleMassKg,
      parameters.model.motorTorqueConstantNmPerA,
      parameters.model.motorFinalDriveRatio,
      parameters.model.drivetrainEfficiency,
      parameters.model.tireRadiusM,
      parameters.model.effectiveTorqueScale,
      model.physicalAccelPerAmp, model.effectiveAccelPerAmp);
  }

  void LogTimingSummary() const
  {
    const auto solver = solveTiming_.Summary();
    const auto callback = callbackTiming_.Summary();
    RCLCPP_INFO(
      get_logger(),
      "MPC_TIMING solver_total_samples=%zu solver_window_samples=%zu "
      "median_ms=%.6f p95_ms=%.6f max_all_ms=%.6f "
      "callback_total_samples=%zu callback_window_samples=%zu "
      "median_ms=%.6f p95_ms=%.6f max_all_ms=%.6f",
      solver.totalCount, solver.windowCount, solver.medianMs, solver.p95Ms,
      solver.maximumAllMs, callback.totalCount, callback.windowCount,
      callback.medianMs, callback.p95Ms, callback.maximumAllMs);
    RCLCPP_INFO(
      get_logger(),
      "MPC_RESULT successful_solves=%zu solver_failures=%zu "
      "maximum_constraint_violation=%.12g published_commands=%zu",
      successfulSolves_, solverFailures_, maximumConstraintViolation_,
      publishedCommands_);
  }

private:
  void OnCommand(const ActuatorCommand & message)
  {
    const auto now = SteadyClock::now();
    if (!std::isfinite(message.speed_target_mps)) {
      targetSpeedMps_.reset();
      commandReceivedAt_.reset();
      return;
    }
    targetSpeedMps_ = std::max(0.0, static_cast<double>(message.speed_target_mps));
    brakeEngaged_ = message.brake_engage;
    if (brakeEngaged_) {
      mpc_->Reset();
    }
    commandReceivedAt_ = now;
  }

  void OnVehicleState(const VehicleState & message)
  {
    mode_ = message.mode;
    estopActive_ = message.estop_active;
    if (estopActive_ || mode_ != VehicleState::MODE_AUTO) {
      mpc_->Reset();
    }
    if (!std::isfinite(message.speed_mps)) {
      speedMps_.reset();
      stateReceivedAt_.reset();
      return;
    }
    speedMps_ = std::max(0.0, static_cast<double>(message.speed_mps));
    stateReceivedAt_ = SteadyClock::now();
  }

  void OnDriveStatus(const DriveActuatorStatus & message)
  {
    driveStopRequested_ = message.stop_request;
    driveFaultActive_ = message.fault_bits_valid && message.fault_bits != 0U;
    if (driveStopRequested_ || driveFaultActive_) {
      mpc_->Reset();
    }
    if (!message.part1_valid || !std::isfinite(message.phase_current_a)) {
      phaseCurrentA_.reset();
      currentReceivedAt_.reset();
      return;
    }
    phaseCurrentA_ = message.phase_current_a;
    currentReceivedAt_ = SteadyClock::now();
  }

  std::optional<std::string> ReleaseReason(
    const SteadyClock::time_point now) const
  {
    if (estopActive_) {
      return "vehicle E-stop active";
    }
    if (driveStopRequested_) {
      return "drive stop requested";
    }
    if (driveFaultActive_) {
      return "drive fault active";
    }
    if (ElapsedSeconds(now, commandReceivedAt_) > commandTimeoutSec_) {
      return "command stale";
    }
    if (brakeEngaged_) {
      return "brake engaged";
    }
    if (ElapsedSeconds(now, stateReceivedAt_) > feedbackTimeoutSec_ ||
      ElapsedSeconds(now, currentReceivedAt_) > feedbackTimeoutSec_)
    {
      return "feedback stale or invalid";
    }
    if (mode_ != VehicleState::MODE_AUTO) {
      return "vehicle is not in AUTO";
    }
    if (!targetSpeedMps_.has_value() || !speedMps_.has_value() ||
      !phaseCurrentA_.has_value())
    {
      return "target or feedback is invalid";
    }
    return std::nullopt;
  }

  void Release(const std::string & reason)
  {
    mpc_->Reset();
    PublishCurrent(0.0);
    if (reason != lastReleaseReason_) {
      RCLCPP_WARN(get_logger(), "traction released: %s", reason.c_str());
      lastReleaseReason_ = reason;
    }
  }

  void PublishCurrent(const double currentA)
  {
    DriveCurrentCommand message;
    message.header.stamp = now();
    message.phase_current_a = static_cast<float>(currentA);
    publisher_->publish(message);
    ++publishedCommands_;
  }

  void OnTimer()
  {
    const auto started = SteadyClock::now();
    OnTimerImpl();
    callbackTiming_.Add(
      std::chrono::duration<double, std::milli>(
        SteadyClock::now() - started).count());
  }

  void OnTimerImpl()
  {
    const auto releaseReason = ReleaseReason(SteadyClock::now());
    if (releaseReason.has_value()) {
      Release(*releaseReason);
      return;
    }
    if (*targetSpeedMps_ <= 0.0) {
      PublishCurrent(mpc_->ReleaseWithSlew());
      lastReleaseReason_.clear();
      return;
    }

    const auto solveStarted = SteadyClock::now();
    const SolveResult result = mpc_->Solve(
      *speedMps_, *phaseCurrentA_, *targetSpeedMps_);
    solveTiming_.Add(
      std::chrono::duration<double, std::milli>(
        SteadyClock::now() - solveStarted).count());
    maximumConstraintViolation_ = std::max(
      maximumConstraintViolation_, result.maximumConstraintViolation);
    if (!result.success) {
      ++solverFailures_;
      Release("solver failure: " + result.status);
      return;
    }

    const auto postSolveReleaseReason = ReleaseReason(SteadyClock::now());
    if (postSolveReleaseReason.has_value()) {
      Release(*postSolveReleaseReason);
      return;
    }
    ++successfulSolves_;
    PublishCurrent(result.currentA);
    lastReleaseReason_.clear();
  }

  double commandTimeoutSec_{0.2};
  double feedbackTimeoutSec_{0.2};
  std::unique_ptr<OsqpMpc> mpc_;

  std::optional<double> targetSpeedMps_;
  bool brakeEngaged_{false};
  std::optional<SteadyClock::time_point> commandReceivedAt_;
  std::optional<double> speedMps_;
  std::uint8_t mode_{VehicleState::MODE_MANUAL};
  bool estopActive_{false};
  std::optional<SteadyClock::time_point> stateReceivedAt_;
  std::optional<double> phaseCurrentA_;
  bool driveStopRequested_{false};
  bool driveFaultActive_{false};
  std::optional<SteadyClock::time_point> currentReceivedAt_;
  std::string lastReleaseReason_;

  BoundedTiming solveTiming_;
  BoundedTiming callbackTiming_;
  std::size_t successfulSolves_{0U};
  std::size_t solverFailures_{0U};
  std::size_t publishedCommands_{0U};
  double maximumConstraintViolation_{0.0};

  rclcpp::Publisher<DriveCurrentCommand>::SharedPtr publisher_;
  rclcpp::Subscription<ActuatorCommand>::SharedPtr commandSubscription_;
  rclcpp::Subscription<VehicleState>::SharedPtr stateSubscription_;
  rclcpp::Subscription<DriveActuatorStatus>::SharedPtr driveSubscription_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace longitudinal_mpc

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    auto node = std::make_shared<longitudinal_mpc::LongitudinalMpcNode>();
    rclcpp::spin(node);
    node->LogTimingSummary();
    node.reset();
  } catch (const std::exception & error) {
    RCLCPP_FATAL(rclcpp::get_logger("longitudinal_mpc"), "%s", error.what());
    if (rclcpp::ok()) {
      rclcpp::shutdown();
    }
    return 1;
  }
  if (rclcpp::ok()) {
    rclcpp::shutdown();
  }
  return 0;
}
