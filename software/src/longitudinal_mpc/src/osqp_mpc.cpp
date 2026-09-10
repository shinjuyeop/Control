#include "longitudinal_mpc/osqp_mpc.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <utility>

namespace longitudinal_mpc
{
namespace
{

constexpr double kCurrentScaleA = 100.0;
constexpr double kConstraintTolerance = 1e-4;

bool PositiveFinite(const double value)
{
  return std::isfinite(value) && value > 0.0;
}

}  // namespace

DiscreteModel BuildDiscreteModel(const PhysicalModelParameters & parameters)
{
  if (!PositiveFinite(parameters.dtSec) ||
    !PositiveFinite(parameters.vehicleMassKg) ||
    !PositiveFinite(parameters.motorTorqueConstantNmPerA) ||
    !PositiveFinite(parameters.motorFinalDriveRatio) ||
    !PositiveFinite(parameters.drivetrainEfficiency) ||
    parameters.drivetrainEfficiency > 1.0 ||
    !PositiveFinite(parameters.tireRadiusM) ||
    !PositiveFinite(parameters.effectiveTorqueScale) ||
    !std::isfinite(parameters.speedDampingPerSec) ||
    parameters.speedDampingPerSec < 0.0 ||
    !std::isfinite(parameters.resistanceAccelMps2) ||
    parameters.resistanceAccelMps2 < 0.0 ||
    !std::isfinite(parameters.currentResponseAlpha) ||
    parameters.currentResponseAlpha < 0.0 ||
    parameters.currentResponseAlpha >= 1.0 ||
    !PositiveFinite(parameters.currentCommandGain))
  {
    throw std::invalid_argument("invalid physical model parameters");
  }

  DiscreteModel model;
  model.physicalAccelPerAmp =
    parameters.drivetrainEfficiency * parameters.motorFinalDriveRatio *
    parameters.motorTorqueConstantNmPerA /
    (parameters.vehicleMassKg * parameters.tireRadiusM);
  model.effectiveAccelPerAmp =
    parameters.effectiveTorqueScale * model.physicalAccelPerAmp;
  model.speedDecay = 1.0 -
    parameters.dtSec * parameters.speedDampingPerSec;
  model.currentToSpeed =
    parameters.dtSec * kCurrentScaleA * model.effectiveAccelPerAmp;
  model.currentDecay = parameters.currentResponseAlpha;
  model.commandToCurrent =
    (1.0 - parameters.currentResponseAlpha) *
    parameters.currentCommandGain;
  model.speedOffset =
    -parameters.dtSec * parameters.resistanceAccelMps2;
  return model;
}

OsqpMpc::OsqpMpc(const MpcParameters & parameters)
: parameters_(parameters), model_(BuildDiscreteModel(parameters.model))
{
  if (!PositiveFinite(parameters_.horizonSec) ||
    parameters_.controlBlockSteps == 0U ||
    !PositiveFinite(parameters_.maximumCurrentA) ||
    !PositiveFinite(parameters_.currentSlewAps) ||
    !std::isfinite(parameters_.inputWeight) ||
    parameters_.inputWeight < 0.0 ||
    !std::isfinite(parameters_.slewWeight) ||
    parameters_.slewWeight < 0.0 ||
    !PositiveFinite(parameters_.solverTimeLimitSec))
  {
    throw std::invalid_argument("invalid MPC parameters");
  }
  predictionSteps_ = static_cast<std::size_t>(
    std::llround(parameters_.horizonSec / parameters_.model.dtSec));
  if (predictionSteps_ == 0U) {
    throw std::invalid_argument("MPC horizon has no prediction steps");
  }
  controlBlocks_ =
    (predictionSteps_ + parameters_.controlBlockSteps - 1U) /
    parameters_.controlBlockSteps;
  maximumInput_ = parameters_.maximumCurrentA / kCurrentScaleA;
  stepLimit_ =
    parameters_.currentSlewAps * parameters_.model.dtSec / kCurrentScaleA;

  BuildPrediction();
  BuildProblemMatrices();
  SetupSolver();
  Reset();
}

OsqpMpc::~OsqpMpc()
{
  if (workspace_ != nullptr) {
    osqp_cleanup(workspace_);
  }
  if (pMatrix_ != nullptr) {
    c_free(pMatrix_);
  }
  if (aMatrix_ != nullptr) {
    c_free(aMatrix_);
  }
}

void OsqpMpc::BuildPrediction()
{
  blockIndex_.resize(predictionSteps_);
  response_.assign(predictionSteps_ * controlBlocks_, 0.0);
  std::vector<double> speedResponse(controlBlocks_, 0.0);
  std::vector<double> currentResponse(controlBlocks_, 0.0);
  for (std::size_t step = 0U; step < predictionSteps_; ++step) {
    blockIndex_[step] = std::min(
      step / parameters_.controlBlockSteps, controlBlocks_ - 1U);
    std::vector<double> nextSpeed(controlBlocks_, 0.0);
    std::vector<double> nextCurrent(controlBlocks_, 0.0);
    for (std::size_t block = 0U; block < controlBlocks_; ++block) {
      nextSpeed[block] =
        model_.speedDecay * speedResponse[block] +
        model_.currentToSpeed * currentResponse[block];
      nextCurrent[block] = model_.currentDecay * currentResponse[block];
    }
    nextCurrent[blockIndex_[step]] += model_.commandToCurrent;
    speedResponse = std::move(nextSpeed);
    currentResponse = std::move(nextCurrent);
    for (std::size_t block = 0U; block < controlBlocks_; ++block) {
      response_[step * controlBlocks_ + block] = speedResponse[block];
    }
  }

  difference_.assign(controlBlocks_ * controlBlocks_, 0.0);
  for (std::size_t row = 0U; row < controlBlocks_; ++row) {
    difference_[row * controlBlocks_ + row] = 1.0;
    if (row > 0U) {
      difference_[row * controlBlocks_ + row - 1U] = -1.0;
    }
  }

  hessian_.assign(controlBlocks_ * controlBlocks_, 0.0);
  for (std::size_t row = 0U; row < controlBlocks_; ++row) {
    for (std::size_t column = 0U; column < controlBlocks_; ++column) {
      double value = row == column ? parameters_.inputWeight : 0.0;
      for (std::size_t step = 0U; step < predictionSteps_; ++step) {
        value += response_[step * controlBlocks_ + row] *
          response_[step * controlBlocks_ + column];
      }
      for (std::size_t move = 0U; move < controlBlocks_; ++move) {
        value += parameters_.slewWeight *
          difference_[move * controlBlocks_ + row] *
          difference_[move * controlBlocks_ + column];
      }
      hessian_[row * controlBlocks_ + column] = value;
    }
  }
}

void OsqpMpc::BuildProblemMatrices()
{
  pColumns_.reserve(controlBlocks_ + 1U);
  pColumns_.push_back(0);
  for (std::size_t column = 0U; column < controlBlocks_; ++column) {
    for (std::size_t row = 0U; row <= column; ++row) {
      pRows_.push_back(static_cast<c_int>(row));
      pValues_.push_back(
        static_cast<c_float>(
          2.0 * hessian_[row * controlBlocks_ + column]));
    }
    pColumns_.push_back(static_cast<c_int>(pValues_.size()));
  }

  aColumns_.reserve(controlBlocks_ + 1U);
  aColumns_.push_back(0);
  for (std::size_t column = 0U; column < controlBlocks_; ++column) {
    aRows_.push_back(static_cast<c_int>(column));
    aValues_.push_back(1.0);
    aRows_.push_back(static_cast<c_int>(controlBlocks_ + column));
    aValues_.push_back(1.0);
    if (column + 1U < controlBlocks_) {
      aRows_.push_back(static_cast<c_int>(controlBlocks_ + column + 1U));
      aValues_.push_back(-1.0);
    }
    aColumns_.push_back(static_cast<c_int>(aValues_.size()));
  }

  linearCost_.assign(controlBlocks_, 0.0);
  lowerBounds_.assign(2U * controlBlocks_, 0.0);
  upperBounds_.assign(2U * controlBlocks_, 0.0);
  warmStart_.assign(controlBlocks_, 0.0);
  zeroDual_.assign(2U * controlBlocks_, 0.0);
}

void OsqpMpc::SetupSolver()
{
  pMatrix_ = csc_matrix(
    static_cast<c_int>(controlBlocks_),
    static_cast<c_int>(controlBlocks_),
    static_cast<c_int>(pValues_.size()), pValues_.data(), pRows_.data(),
    pColumns_.data());
  aMatrix_ = csc_matrix(
    static_cast<c_int>(2U * controlBlocks_),
    static_cast<c_int>(controlBlocks_),
    static_cast<c_int>(aValues_.size()), aValues_.data(), aRows_.data(),
    aColumns_.data());
  if (pMatrix_ == nullptr || aMatrix_ == nullptr) {
    if (pMatrix_ != nullptr) {
      c_free(pMatrix_);
    }
    if (aMatrix_ != nullptr) {
      c_free(aMatrix_);
    }
    pMatrix_ = nullptr;
    aMatrix_ = nullptr;
    throw std::runtime_error("OSQP CSC allocation failed");
  }

  data_.n = static_cast<c_int>(controlBlocks_);
  data_.m = static_cast<c_int>(2U * controlBlocks_);
  data_.P = pMatrix_;
  data_.A = aMatrix_;
  data_.q = linearCost_.data();
  data_.l = lowerBounds_.data();
  data_.u = upperBounds_.data();

  osqp_set_default_settings(&settings_);
  settings_.verbose = 0;
  settings_.warm_start = 1;
  settings_.polish = 1;
  settings_.max_iter = 4000;
  settings_.eps_abs = 1e-6;
  settings_.eps_rel = 1e-6;
  settings_.check_termination = 1;
  settings_.time_limit = parameters_.solverTimeLimitSec;
  const c_int setupStatus = osqp_setup(&workspace_, &data_, &settings_);
  if (setupStatus != 0 || workspace_ == nullptr) {
    c_free(pMatrix_);
    c_free(aMatrix_);
    pMatrix_ = nullptr;
    aMatrix_ = nullptr;
    throw std::runtime_error(
            "OSQP setup failed with status " + std::to_string(setupStatus));
  }
}

SolveResult OsqpMpc::Solve(
  const double speedMps, const double phaseCurrentA,
  const double targetSpeedMps)
{
  SolveResult result;
  if (!std::isfinite(speedMps) || !std::isfinite(phaseCurrentA) ||
    !std::isfinite(targetSpeedMps))
  {
    result.status = "nonfinite state or target";
    return result;
  }

  std::vector<double> base(predictionSteps_, 0.0);
  double speed = speedMps;
  double current = phaseCurrentA / kCurrentScaleA;
  for (std::size_t step = 0U; step < predictionSteps_; ++step) {
    const double nextSpeed =
      model_.speedDecay * speed + model_.currentToSpeed * current +
      model_.speedOffset;
    current = model_.currentDecay * current;
    speed = nextSpeed;
    base[step] = speed;
  }

  std::vector<double> previous(controlBlocks_, 0.0);
  previous[0] = lastInput_;
  for (std::size_t block = 0U; block < controlBlocks_; ++block) {
    double linear = 0.0;
    for (std::size_t step = 0U; step < predictionSteps_; ++step) {
      linear += response_[step * controlBlocks_ + block] *
        (base[step] - targetSpeedMps);
    }
    double differenceTransposePrevious = 0.0;
    for (std::size_t row = 0U; row < controlBlocks_; ++row) {
      differenceTransposePrevious +=
        difference_[row * controlBlocks_ + block] * previous[row];
    }
    linear -= parameters_.slewWeight * differenceTransposePrevious;
    linearCost_[block] = static_cast<c_float>(2.0 * linear);
    lowerBounds_[block] = 0.0;
    upperBounds_[block] = static_cast<c_float>(maximumInput_);
  }
  for (std::size_t move = 0U; move < controlBlocks_; ++move) {
    const double limit = move == 0U ? stepLimit_ :
      parameters_.controlBlockSteps * stepLimit_;
    lowerBounds_[controlBlocks_ + move] = static_cast<c_float>(
      previous[move] - limit);
    upperBounds_[controlBlocks_ + move] = static_cast<c_float>(
      previous[move] + limit);
  }

  if (osqp_update_lin_cost(workspace_, linearCost_.data()) != 0 ||
    osqp_update_bounds(
      workspace_, lowerBounds_.data(), upperBounds_.data()) != 0 ||
    osqp_warm_start_x(workspace_, warmStart_.data()) != 0)
  {
    result.status = "OSQP update or warm start failed";
    return result;
  }
  const c_int solveCallStatus = osqp_solve(workspace_);
  result.statusValue = workspace_->info->status_val;
  result.status = workspace_->info->status;
  result.iterations = static_cast<int>(workspace_->info->iter);
  if (solveCallStatus != 0 ||
    (result.statusValue != OSQP_SOLVED &&
    result.statusValue != OSQP_SOLVED_INACCURATE) ||
    workspace_->solution == nullptr || workspace_->solution->x == nullptr)
  {
    return result;
  }

  result.solution.resize(controlBlocks_);
  bool finite = true;
  for (std::size_t block = 0U; block < controlBlocks_; ++block) {
    result.solution[block] = workspace_->solution->x[block];
    finite = finite && std::isfinite(result.solution[block]);
  }
  result.maximumConstraintViolation =
    ConstraintViolation(result.solution, lastInput_);
  result.objective = Objective(result.solution, linearCost_);
  if (!finite || !std::isfinite(result.objective) ||
    result.maximumConstraintViolation > kConstraintTolerance)
  {
    result.status = "OSQP solution failed finite or constraint check";
    return result;
  }

  for (std::size_t block = 0U; block < controlBlocks_; ++block) {
    warmStart_[block] = static_cast<c_float>(result.solution[block]);
  }
  const double firstLower = std::max(0.0, lastInput_ - stepLimit_);
  const double firstUpper = std::min(maximumInput_, lastInput_ + stepLimit_);
  lastInput_ = std::clamp(
    result.solution.front(), firstLower, firstUpper);
  warmStart_.front() = static_cast<c_float>(lastInput_);
  result.currentA = kCurrentScaleA * lastInput_;
  result.success = true;
  return result;
}

double OsqpMpc::ConstraintViolation(
  const std::vector<double> & solution, const double previousInput) const
{
  double violation = 0.0;
  for (const double value : solution) {
    violation = std::max(violation, -value);
    violation = std::max(violation, value - maximumInput_);
  }
  for (std::size_t move = 0U; move < controlBlocks_; ++move) {
    const double difference = solution[move] -
      (move == 0U ? previousInput : solution[move - 1U]);
    const double limit = move == 0U ? stepLimit_ :
      parameters_.controlBlockSteps * stepLimit_;
    violation = std::max(violation, std::abs(difference) - limit);
  }
  return std::max(0.0, violation);
}

double OsqpMpc::Objective(
  const std::vector<double> & solution,
  const std::vector<c_float> & linearCost) const
{
  double objective = 0.0;
  for (std::size_t row = 0U; row < controlBlocks_; ++row) {
    objective += static_cast<double>(linearCost[row]) * solution[row];
    for (std::size_t column = 0U; column < controlBlocks_; ++column) {
      objective += solution[row] *
        hessian_[row * controlBlocks_ + column] * solution[column];
    }
  }
  return objective;
}

double OsqpMpc::ReleaseWithSlew()
{
  lastInput_ = std::max(0.0, lastInput_ - stepLimit_);
  std::fill(warmStart_.begin(), warmStart_.end(), lastInput_);
  return LastCurrentA();
}

void OsqpMpc::Reset()
{
  lastInput_ = 0.0;
  std::fill(warmStart_.begin(), warmStart_.end(), 0.0);
  std::fill(zeroDual_.begin(), zeroDual_.end(), 0.0);
  if (workspace_ != nullptr) {
    (void)osqp_warm_start(workspace_, warmStart_.data(), zeroDual_.data());
  }
}

double OsqpMpc::LastCurrentA() const
{
  return kCurrentScaleA * lastInput_;
}

const DiscreteModel & OsqpMpc::Model() const
{
  return model_;
}

std::size_t OsqpMpc::PredictionSteps() const
{
  return predictionSteps_;
}

std::size_t OsqpMpc::ControlBlocks() const
{
  return controlBlocks_;
}

}  // namespace longitudinal_mpc
