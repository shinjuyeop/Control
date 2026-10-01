#pragma once
#include "mpc_common/linear_mpc.hpp"
#include <Eigen/Core>
#include <string>

namespace steering_mpc {
struct SteeringModel {
  double dt{}, input_scale{};
  int delay{};
  Eigen::Matrix2d A;
  Eigen::Vector2d B, c;
  static SteeringModel load(const std::string &path);
};
class SteeringMpc {
public:
  explicit SteeringMpc(const SteeringModel &model, int horizon = 50,
                       double angle_weight = 4000., double rate_weight = .1,
                       double change_weight = .02);
  mpc_common::Solution solve(double angle, double rate, double target,
                             double previous_raw, double raw_limit,
                             double angle_min, double angle_max);
  void reset();
  double dt() const { return model_.dt; }

private:
  SteeringModel model_;
  mpc_common::LinearMpc qp_;
  Eigen::VectorXd state_;
  Eigen::MatrixXd reference_, offset_;
};
} // namespace steering_mpc
