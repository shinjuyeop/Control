#include "mpc_common/linear_mpc.hpp"
#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace mpc_common {
LinearMpc::LinearMpc(int nx, int n, int si, const Eigen::VectorXd &weights,
                     double ru, double rd, double limit)
    : nx_(nx), n_(n), state_index_(si), ru_(ru), rd_(rd), time_limit_(limit),
      weights_(weights), free_(nx * n), gradient_(n), a_(nx, nx), b_(nx, 1),
      F_(nx * n, nx), G_(nx * n, n), H_(n, n), D_(n, n), weighted_(nx * n, n),
      state_rows_(n, n) {
  if (nx < 1 || n < 1 || n > 200 || si < 0 || si >= nx ||
      weights.size() != nx || !weights.allFinite() || weights.minCoeff() < 0 ||
      !(ru > 0) || !(rd >= 0) || !(limit > 0))
    throw std::invalid_argument("Invalid MPC dimensions or weights");
  D_.setIdentity();
  for (int i = 1; i < n; ++i)
    D_(i, i - 1) = -1;
  q_.resize(n);
  lo_.resize(3 * n);
  hi_.resize(3 * n);
  pp_.push_back(0);
  ap_.push_back(0);
  for (int j = 0; j < n; ++j) {
    for (int i = 0; i <= j; ++i) {
      pi_.push_back(i);
      px_.push_back(0);
    }
    pp_.push_back(px_.size());
    ai_.push_back(j);
    ax_.push_back(1);
    ai_.push_back(n + j);
    ax_.push_back(1);
    if (j + 1 < n) {
      ai_.push_back(n + j + 1);
      ax_.push_back(-1);
    }
    for (int k = 0; k < n; ++k) {
      ai_.push_back(2 * n + k);
      ax_.push_back(0);
    }
    ap_.push_back(ax_.size());
  }
}
LinearMpc::~LinearMpc() {
  if (work_)
    osqp_cleanup(work_);
  if (P_)
    c_free(P_);
  if (C_)
    c_free(C_);
}
void LinearMpc::dynamics(const Eigen::MatrixXd &a, const Eigen::VectorXd &b) {
  a_ = a;
  b_ = b;
  Eigen::MatrixXd f = Eigen::MatrixXd::Identity(nx_, nx_);
  Eigen::MatrixXd g = Eigen::MatrixXd::Zero(nx_, n_);
  for (int k = 0; k < n_; ++k) {
    f = (a * f).eval();
    g = (a * g).eval();
    g.col(k) += b;
    F_.middleRows(k * nx_, nx_) = f;
    G_.middleRows(k * nx_, nx_) = g;
    state_rows_.row(k) = g.row(state_index_);
    for (int i = 0; i < nx_; ++i)
      weighted_.row(k * nx_ + i) = weights_[i] * g.row(i);
  }
  H_.noalias() = 2 * (G_.transpose() * weighted_ + rd_ * D_.transpose() * D_);
  H_.diagonal().array() += 2 * ru_;
  int p = 0;
  for (int j = 0; j < n_; ++j)
    for (int i = 0; i <= j; ++i)
      px_[p++] = H_(i, j);
  for (int j = 0; j < n_; ++j) {
    int start = ap_[j] + 2 + (j + 1 < n_ ? 1 : 0);
    for (int k = 0; k < n_; ++k)
      ax_[start + k] = state_rows_(k, j);
  }
  if (work_) {
    if (osqp_update_P(work_, px_.data(), nullptr, px_.size()) ||
        osqp_update_A(work_, ax_.data(), nullptr, ax_.size()))
      throw std::runtime_error("OSQP matrix update failed");
  }
}
Solution LinearMpc::solve(const Eigen::VectorXd &x, const Eigen::MatrixXd &ref,
                          const Eigen::MatrixXd &offset, double previous,
                          double limit, double step, double smin, double smax) {
  Eigen::VectorXd drift = Eigen::VectorXd::Zero(nx_);
  free_.noalias() = F_ * x;
  for (int k = 0; k < n_; ++k) {
    drift = a_ * drift + offset.col(k);
    free_.segment(k * nx_, nx_) += drift;
    lo_[k] = -limit;
    hi_[k] = limit;
    lo_[n_ + k] = -step;
    hi_[n_ + k] = step;
    lo_[2 * n_ + k] = smin - free_[k * nx_ + state_index_];
    hi_[2 * n_ + k] = smax - free_[k * nx_ + state_index_];
    free_.segment(k * nx_, nx_) -= ref.col(k);
  }
  gradient_.noalias() = 2 * weighted_.transpose() * free_;
  gradient_[0] -= 2 * rd_ * previous;
  for (int k = 0; k < n_; ++k)
    q_[k] = gradient_[k];
  lo_[n_] += previous;
  hi_[n_] += previous;
  if (!work_) {
    P_ = csc_matrix(n_, n_, px_.size(), px_.data(), pi_.data(), pp_.data());
    C_ = csc_matrix(3 * n_, n_, ax_.size(), ax_.data(), ai_.data(), ap_.data());
    OSQPData data{};
    data.n = n_;
    data.m = 3 * n_;
    data.P = P_;
    data.A = C_;
    data.q = q_.data();
    data.l = lo_.data();
    data.u = hi_.data();
    OSQPSettings settings;
    osqp_set_default_settings(&settings);
    settings.verbose = 0;
    settings.warm_start = 1;
    settings.eps_abs = 1e-4;
    settings.eps_rel = 1e-3;
    settings.max_iter = 2000;
    settings.time_limit = time_limit_;
    settings.adaptive_rho_interval = 25;
    if (osqp_setup(&work_, &data, &settings))
      return {};
  } else if (osqp_update_lin_cost(work_, q_.data()) ||
             osqp_update_bounds(work_, lo_.data(), hi_.data()))
    return {};
  osqp_solve(work_);
  Solution s;
  s.solve_ms = work_->info->solve_time * 1000;
  s.iterations = work_->info->iter;
  s.status = work_->info->status_val;
  const double low = std::max(-limit, previous - step);
  const double high = std::min(limit, previous + step);
  s.solved = (s.status == OSQP_SOLVED || s.status == OSQP_SOLVED_INACCURATE) &&
             std::isfinite(work_->solution->x[0]) && low <= high;
  if (s.solved)
    s.input = std::clamp(static_cast<double>(work_->solution->x[0]), low, high);
  return s;
}
std::vector<double> LinearMpc::planned_inputs() const {
  if (!work_ || !work_->solution || !work_->solution->x)
    return {};
  return std::vector<double>(work_->solution->x, work_->solution->x + n_);
}
} // namespace mpc_common
