#pragma once
#include <Eigen/Core>
#include <osqp.h>
#include <vector>

namespace mpc_common {
struct Solution {
  bool solved{false};
  double input{0}, solve_ms{0};
  int iterations{0}, status{0};
};
// Scalar-input condensed QP, used by the actuator and path controllers.
class LinearMpc {
public:
  LinearMpc(int states, int horizon, int constrained_state,
            const Eigen::VectorXd &weights, double effort_weight,
            double change_weight, double time_limit);
  ~LinearMpc();
  LinearMpc(const LinearMpc &) = delete;
  LinearMpc &operator=(const LinearMpc &) = delete;
  void dynamics(const Eigen::MatrixXd &a, const Eigen::VectorXd &b);
  Solution solve(const Eigen::VectorXd &state, const Eigen::MatrixXd &reference,
                 const Eigen::MatrixXd &offset, double previous,
                 double input_limit, double input_step, double state_min,
                 double state_max);
  std::vector<double> planned_inputs() const;

private:
  int nx_, n_, state_index_;
  double ru_, rd_, time_limit_;
  Eigen::VectorXd weights_, free_, gradient_;
  Eigen::MatrixXd a_, b_, F_, G_, H_, D_, weighted_, state_rows_;
  std::vector<c_float> px_, ax_, q_, lo_, hi_;
  std::vector<c_int> pi_, pp_, ai_, ap_;
  csc *P_{nullptr}, *C_{nullptr};
  OSQPWorkspace *work_{nullptr};
};
} // namespace mpc_common
