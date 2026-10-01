#include "steering_mpc/steering_mpc.hpp"
#include <cmath>
#include <stdexcept>
#include <vector>
#include <yaml-cpp/yaml.h>

namespace steering_mpc {
SteeringModel SteeringModel::load(const std::string &path) {
  const auto d = YAML::LoadFile(path);
  SteeringModel m;
  if (d["input"].as<std::string>() != "/steering/torque_command_raw" ||
      d["states"].as<std::vector<std::string>>() !=
          std::vector<std::string>{"steering_rad", "steering_rate_rad_s"})
    throw std::runtime_error(
        "Steering model must map KEYA torque raw to wheel angle/rate");
  m.dt = d["dt"].as<double>();
  m.delay = d["delay_steps"].as<int>();
  m.input_scale = d["input_scale"].as<double>();
  if (d["A"].size() != 2 || d["B"].size() != 2 || d["c"].size() != 2)
    throw std::runtime_error("Expected two steering states");
  for (int i = 0; i < 2; ++i) {
    if (d["A"][i].size() != 2)
      throw std::runtime_error("Expected 2x2 steering A");
    for (int j = 0; j < 2; ++j)
      m.A(i, j) = d["A"][i][j].as<double>();
    m.B[i] = d["B"][i].as<double>();
    m.c[i] = d["c"][i].as<double>();
  }
  if (!m.A.allFinite() || !m.B.allFinite() || !m.c.allFinite() ||
      !std::isfinite(m.dt) || m.dt <= 0 || !std::isfinite(m.input_scale) ||
      m.input_scale <= 0 || m.delay < 0 || m.delay > 50)
    throw std::runtime_error(
        "Invalid steering model dynamics, period, scale or delay");
  return m;
}
SteeringMpc::SteeringMpc(const SteeringModel &m, int n, double angle_weight,
                         double rate_weight, double change_weight)
    : model_(m), qp_(2 + m.delay, n, 0,
                     (Eigen::VectorXd(2 + m.delay) << angle_weight, rate_weight,
                      Eigen::VectorXd::Zero(m.delay))
                         .finished(),
                     .01, change_weight, .016),
      state_(Eigen::VectorXd::Zero(2 + m.delay)),
      reference_(Eigen::MatrixXd::Zero(2 + m.delay, n)),
      offset_(Eigen::MatrixXd::Zero(2 + m.delay, n)) {
  Eigen::MatrixXd a = Eigen::MatrixXd::Zero(state_.size(), state_.size());
  a.topLeftCorner(2, 2) = m.A;
  Eigen::VectorXd b = Eigen::VectorXd::Zero(state_.size());
  if (m.delay) {
    a.block(0, 2, 2, 1) = m.B;
    for (int i = 2; i < state_.size() - 1; ++i)
      a(i, i + 1) = 1;
    b[state_.size() - 1] = 1;
  } else
    b.head(2) = m.B;
  offset_.topRows(2) = m.c.replicate(1, n);
  qp_.dynamics(a, b);
}
mpc_common::Solution SteeringMpc::solve(double angle, double rate,
                                        double target, double previous,
                                        double limit, double amin,
                                        double amax) {
  state_[0] = angle;
  state_[1] = rate;
  if (model_.delay) {
    for (int i = 2; i < state_.size() - 1; ++i)
      state_[i] = state_[i + 1];
    state_[state_.size() - 1] = previous / model_.input_scale;
  }
  reference_.row(0).setConstant(target);
  auto s = qp_.solve(state_, reference_, offset_, previous / model_.input_scale,
                     limit / model_.input_scale, 2., amin, amax);
  s.input *= model_.input_scale;
  return s;
}
void SteeringMpc::reset() { state_.setZero(); }
} // namespace steering_mpc
