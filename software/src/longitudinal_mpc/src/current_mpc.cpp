#include "longitudinal_mpc/current_mpc.hpp"
#include <Eigen/LU>
#include <yaml-cpp/yaml.h>
#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace longitudinal_mpc {
Model Model::Load(const std::string &path) {
  const auto d = YAML::LoadFile(path);
  if (!d["input"] || d["input"].as<std::string>() != "/vehicle/drive_current_command.phase_current_a" ||
      !d["states"] || d["states"].size()!=2 ||
      d["states"][0].as<std::string>()!="speed_mps" || d["states"][1].as<std::string>()!="phase_current_a")
    throw std::runtime_error("Model must map PC phase-current demand to [speed_mps, phase_current_a]");
  Model m;
  m.dt = d["dt"].as<double>(); m.speed_max = d["speed_max_mps"].as<double>();
  if (d["A"].size()!=2 || d["B"].size()!=2 || d["c"].size()!=2)
    throw std::runtime_error("Model states must be [speed_mps, phase_current_a]");
  for (int i=0;i<2;++i) {
    if (d["A"][i].size()!=2) throw std::runtime_error("Model A must be 2x2");
    m.B[i]=d["B"][i].as<double>(); m.c[i]=d["c"][i].as<double>();
    for (int j=0;j<2;++j) m.A(i,j)=d["A"][i][j].as<double>();
  }
  if (!m.A.allFinite() || !m.B.allFinite() || !m.c.allFinite() ||
      !std::isfinite(m.dt) || std::abs(m.dt-.05)>1e-9 ||
      !std::isfinite(m.speed_max) || m.speed_max<=0)
    throw std::runtime_error("Model must have finite dynamics, positive speed range and dt=0.05 s");
  return m;
}

CurrentMPC::CurrentMPC(const Model &model, int n, double qv, double rd, double ru, Limits limits)
    :model_(model),limits_(limits),n_(n),qv_(qv),rd_(rd),ru_(ru) {
  if (n<1 || qv<=0 || rd<0 || ru<=0 || !std::isfinite(qv+rd+ru))
    throw std::invalid_argument("MPC needs a positive horizon/tracking/input weight and nonnegative slew weight");
  F_.resize(2*n); G_.resize(n*n); g_.resize(n);
  Eigen::Matrix2d fx=Eigen::Matrix2d::Identity();
  Eigen::Matrix<double,2,Eigen::Dynamic> gu=Eigen::Matrix<double,2,Eigen::Dynamic>::Zero(2,n);
  Eigen::Vector2d offset=Eigen::Vector2d::Zero();
  for (int k=0;k<n;++k) {
    fx=(model.A*fx).eval(); gu=(model.A*gu).eval(); gu.col(k)+=model.B;
    offset=model.A*offset+model.c;
    F_[2*k]=fx(0,0); F_[2*k+1]=fx(0,1); g_[k]=offset[0];
    for(int j=0;j<n;++j) G_[k*n+j]=gu(0,j);
  }
  Eigen::Matrix2d steady;
  steady.col(0)=model.A.col(1)-Eigen::Vector2d(0,1); steady.col(1)=model.B;
  const auto lu=steady.fullPivLu();
  if (!lu.isInvertible()) throw std::runtime_error("Identified model has no unique steady current input");
  equilibrium_gain_=lu.solve(Eigen::Vector2d(1,0)-model.A.col(0))[1];
  equilibrium_offset_=lu.solve(-model.c)[1];
  q_.resize(n); lower_.resize(2*n); upper_.resize(2*n);
  pp_.push_back(0); ap_.push_back(0);
  for (int j=0;j<n;++j) {
    for (int i=0;i<=j;++i) {
      double value=0;
      for (int k=0;k<n;++k) value+=qv*G_[k*n+i]*G_[k*n+j];
      if(i==j) value+=rd*(j==n-1?1:2)+ru;
      if(j==i+1) value-=rd;
      px_.push_back(2*value); pi_.push_back(i);
    }
    pp_.push_back(px_.size());
    ax_.push_back(1); ai_.push_back(j);
    ax_.push_back(1); ai_.push_back(n+j);
    if(j+1<n) { ax_.push_back(-1); ai_.push_back(n+j+1); }
    ap_.push_back(ax_.size());
    lower_[j]=0; upper_[j]=limits.current;
    lower_[n+j]=-limits.down*model.dt; upper_[n+j]=limits.up*model.dt;
  }
  P_=csc_matrix(n,n,px_.size(),px_.data(),pi_.data(),pp_.data());
  A_=csc_matrix(2*n,n,ax_.size(),ax_.data(),ai_.data(),ap_.data());
  OSQPData data{}; data.n=n; data.m=2*n; data.P=P_; data.A=A_;
  data.q=q_.data(); data.l=lower_.data(); data.u=upper_.data();
  OSQPSettings settings; osqp_set_default_settings(&settings);
  settings.verbose=0; settings.warm_start=1; settings.eps_abs=1e-4; settings.eps_rel=1e-5;
  settings.max_iter=4000; settings.time_limit=.035; settings.adaptive_rho_interval=25;
  if(osqp_setup(&work_,&data,&settings)!=0) {
    if(work_) osqp_cleanup(work_);
    c_free(P_); c_free(A_);
    throw std::runtime_error("OSQP setup failed");
  }
}
CurrentMPC::~CurrentMPC() { if(work_) osqp_cleanup(work_); c_free(P_); c_free(A_); }
Result CurrentMPC::Solve(const Eigen::Vector2d &x, double target, double previous) {
  target=std::clamp(target,0.,model_.speed_max);
  const double equilibrium=std::clamp(equilibrium_gain_*target+equilibrium_offset_,0.,limits_.current);
  for(int j=0;j<n_;++j) {
    double cost=-ru_*equilibrium;
    for(int k=0;k<n_;++k)
      cost+=qv_*G_[k*n_+j]*(F_[2*k]*x[0]+F_[2*k+1]*x[1]+g_[k]-target);
    if(j==0) cost-=rd_*previous;
    q_[j]=2*cost;
  }
  lower_[n_]=previous-limits_.down*model_.dt;
  upper_[n_]=previous+limits_.up*model_.dt;
  if(osqp_update_lin_cost(work_,q_.data()) || osqp_update_bounds(work_,lower_.data(),upper_.data())) return {};
  osqp_solve(work_);
  Result r; r.solve_ms=1000*work_->info->solve_time; r.iterations=work_->info->iter;
  r.solved=work_->info->status_val==OSQP_SOLVED && std::isfinite(work_->solution->x[0]);
  if(r.solved) r.current=std::clamp(static_cast<double>(work_->solution->x[0]),
      std::max(0.,static_cast<double>(lower_[n_])), std::min(limits_.current,static_cast<double>(upper_[n_])));
  return r;
}
}
