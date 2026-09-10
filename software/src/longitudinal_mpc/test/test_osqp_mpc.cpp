#include <array>
#include <cmath>

#include <gtest/gtest.h>

#include "longitudinal_mpc/osqp_mpc.hpp"

namespace
{

using longitudinal_mpc::BuildDiscreteModel;
using longitudinal_mpc::MpcParameters;
using longitudinal_mpc::OsqpMpc;
using longitudinal_mpc::PhysicalModelParameters;

TEST(PhysicalModel, PreservesFittedReferenceAndUsesPhysicalParameters)
{
  PhysicalModelParameters reference;
  const auto model = BuildDiscreteModel(reference);
  EXPECT_NEAR(model.physicalAccelPerAmp, 0.007102455387844914, 1e-15);
  EXPECT_NEAR(model.effectiveAccelPerAmp, 0.004857995406007832, 1e-15);
  EXPECT_NEAR(model.speedDecay, 0.9978584103157665, 1e-15);
  EXPECT_NEAR(model.currentToSpeed, 0.024289977030039163, 1e-15);
  EXPECT_NEAR(model.currentDecay, 0.04078012055497491, 1e-15);
  EXPECT_NEAR(model.commandToCurrent, 0.9645143612216145, 1e-15);
  EXPECT_NEAR(model.speedOffset, -0.01777574063254332, 1e-15);

  auto changed = reference;
  changed.vehicleMassKg *= 2.0;
  EXPECT_NEAR(
    BuildDiscreteModel(changed).currentToSpeed,
    0.5 * model.currentToSpeed, 1e-15);
  changed = reference;
  changed.motorTorqueConstantNmPerA *= 2.0;
  EXPECT_NEAR(
    BuildDiscreteModel(changed).currentToSpeed,
    2.0 * model.currentToSpeed, 1e-15);
  changed = reference;
  changed.motorFinalDriveRatio *= 2.0;
  EXPECT_NEAR(
    BuildDiscreteModel(changed).currentToSpeed,
    2.0 * model.currentToSpeed, 1e-15);
  changed = reference;
  changed.tireRadiusM *= 2.0;
  EXPECT_NEAR(
    BuildDiscreteModel(changed).currentToSpeed,
    0.5 * model.currentToSpeed, 1e-15);
  changed = reference;
  changed.drivetrainEfficiency *= 0.5;
  EXPECT_NEAR(
    BuildDiscreteModel(changed).currentToSpeed,
    0.5 * model.currentToSpeed, 1e-15);
}

TEST(OsqpMpc, MatchesSequentialSlsqpReferenceCases)
{
  OsqpMpc mpc(MpcParameters{});
  struct Reference
  {
    double speedMps;
    double current100A;
    double targetMps;
    double firstCurrentA;
    double objective;
  };
  const std::array<Reference, 4> references{{
    {0.0, 0.0, 1.0, 15.0, -52.342317448786744},
    {0.1, 0.15, 1.0, 30.0, -48.30078812727995},
    {0.8, 0.3, 2.0, 45.0, -72.10819962336666},
    {1.5, 1.0, 1.5, 60.0, -8.370172663857852},
  }};

  for (const auto & reference : references) {
    const auto result = mpc.Solve(
      reference.speedMps, 100.0 * reference.current100A,
      reference.targetMps);
    ASSERT_TRUE(result.success) << result.status;
    EXPECT_NEAR(result.currentA, reference.firstCurrentA, 1e-3);
    EXPECT_NEAR(result.objective, reference.objective, 1e-3);
    EXPECT_LE(result.maximumConstraintViolation, 1e-6);
  }
}

TEST(OsqpMpc, ResetRestartsTheFifteenAmpSlewBoundary)
{
  OsqpMpc mpc(MpcParameters{});
  auto result = mpc.Solve(0.0, 0.0, 1.0);
  ASSERT_TRUE(result.success) << result.status;
  EXPECT_NEAR(result.currentA, 15.0, 1e-3);

  result = mpc.Solve(0.1, 15.0, 1.0);
  ASSERT_TRUE(result.success) << result.status;
  EXPECT_NEAR(result.currentA, 30.0, 1e-3);

  mpc.Reset();
  EXPECT_DOUBLE_EQ(mpc.LastCurrentA(), 0.0);
  result = mpc.Solve(0.0, 0.0, 1.0);
  ASSERT_TRUE(result.success) << result.status;
  EXPECT_NEAR(result.currentA, 15.0, 1e-3);
}

TEST(OsqpMpc, ProjectsThePublishedInputAtTheZeroAmpBoundary)
{
  OsqpMpc mpc(MpcParameters{});
  const auto zero = mpc.Solve(10.0, 0.0, 0.0);
  ASSERT_TRUE(zero.success) << zero.status;
  EXPECT_DOUBLE_EQ(zero.currentA, 0.0);
  EXPECT_DOUBLE_EQ(mpc.LastCurrentA(), 0.0);

  const auto restart = mpc.Solve(0.0, 0.0, 1.0);
  ASSERT_TRUE(restart.success) << restart.status;
  EXPECT_NEAR(restart.currentA, 15.0, 1e-3);
}

}  // namespace
