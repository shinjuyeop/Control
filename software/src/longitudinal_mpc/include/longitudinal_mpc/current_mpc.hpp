#pragma once
#include <Eigen/Core>
#include <osqp.h>
#include <memory>
#include <string>
#include <vector>

namespace longitudinal_mpc {
struct Model {
  double dt{}, speed_max{};
  Eigen::Matrix2d A;
  Eigen::Vector2d B, c;
  static Model Load(const std::string &path);
};
struct Limits { double current{300}, up{2400}, down{4800}; };
struct Result { double current{0}, solve_ms{0}; long iterations{0}; bool solved{false}; };
class CurrentMPC {
public:
  CurrentMPC(const Model &model, int horizon, double speed_weight,
             double delta_current_weight, double current_weight, Limits limits);
  ~CurrentMPC();
  CurrentMPC(const CurrentMPC &) = delete;
  CurrentMPC &operator=(const CurrentMPC &) = delete;
  Result Solve(const Eigen::Vector2d &state, double target, double previous_current);
private:
  Model model_;
  Limits limits_;
  int n_;
  double qv_, rd_, ru_, equilibrium_gain_, equilibrium_offset_;
  std::vector<double> F_, G_, g_;
  std::vector<c_float> px_, ax_, q_, lower_, upper_;
  std::vector<c_int> pi_, pp_, ai_, ap_;
  csc *P_{nullptr}, *A_{nullptr};
  OSQPWorkspace *work_{nullptr};
};
}
