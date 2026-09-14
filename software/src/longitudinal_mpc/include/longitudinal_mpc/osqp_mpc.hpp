#pragma once

#include <cstddef>
#include <string>
#include <vector>

#include <osqp.h>

namespace longitudinal_mpc
{

struct PhysicalModelParameters
{
  double dtSec{0.05};
  double vehicleMassKg{285.0};
  double motorTorqueConstantNmPerA{0.11};
  double motorFinalDriveRatio{4.6};
  double drivetrainEfficiency{0.95};
  double tireRadiusM{0.2374765591};
  double effectiveTorqueScale{0.6839881619421034};
  double speedDampingPerSec{0.04283179368466928};
  double resistanceAccelMps2{0.35551481265086643};
  double currentResponseAlpha{0.04078012055497491};
  double currentCommandGain{1.0055195705281386};
};

struct DiscreteModel
{
  double speedDecay{0.0};
  double currentToSpeed{0.0};
  double currentDecay{0.0};
  double commandToCurrent{0.0};
  double speedOffset{0.0};
  double physicalAccelPerAmp{0.0};
  double effectiveAccelPerAmp{0.0};
};

DiscreteModel BuildDiscreteModel(const PhysicalModelParameters & parameters);

struct MpcParameters
{
  PhysicalModelParameters model;
  double horizonSec{2.0};
  std::size_t controlBlockSteps{4U};
  double maximumCurrentA{300.0};
  double currentSlewAps{300.0};
  double inputWeight{0.002};
  double slewWeight{0.05};
  double solverTimeLimitSec{0.02};
};

struct SolveResult
{
  bool success{false};
  double currentA{0.0};
  double objective{0.0};
  double maximumConstraintViolation{0.0};
  int iterations{0};
  int statusValue{0};
  std::string status;
  std::vector<double> solution;
};

class OsqpMpc
{
public:
  explicit OsqpMpc(const MpcParameters & parameters);
  ~OsqpMpc();

  OsqpMpc(const OsqpMpc &) = delete;
  OsqpMpc & operator=(const OsqpMpc &) = delete;

  SolveResult Solve(
    double speedMps, double phaseCurrentA, double targetSpeedMps);
  double ReleaseWithSlew();
  void Reset();

  double LastCurrentA() const;
  const DiscreteModel & Model() const;
  std::size_t PredictionSteps() const;
  std::size_t ControlBlocks() const;

private:
  void BuildPrediction();
  void BuildProblemMatrices();
  void SetupSolver();
  double ConstraintViolation(
    const std::vector<double> & solution, double previousInput) const;
  double Objective(
    const std::vector<double> & solution,
    const std::vector<c_float> & linearCost) const;

  MpcParameters parameters_;
  DiscreteModel model_;
  std::size_t predictionSteps_{0U};
  std::size_t controlBlocks_{0U};
  double maximumInput_{0.0};
  double stepLimit_{0.0};

  std::vector<std::size_t> blockIndex_;
  std::vector<double> response_;
  std::vector<double> difference_;
  std::vector<double> hessian_;

  std::vector<c_float> pValues_;
  std::vector<c_int> pRows_;
  std::vector<c_int> pColumns_;
  std::vector<c_float> aValues_;
  std::vector<c_int> aRows_;
  std::vector<c_int> aColumns_;
  std::vector<c_float> linearCost_;
  std::vector<c_float> lowerBounds_;
  std::vector<c_float> upperBounds_;
  std::vector<c_float> warmStart_;
  std::vector<c_float> zeroDual_;

  csc * pMatrix_{nullptr};
  csc * aMatrix_{nullptr};
  OSQPData data_{};
  OSQPSettings settings_{};
  OSQPWorkspace * workspace_{nullptr};
  double lastInput_{0.0};
};

}  // namespace longitudinal_mpc
