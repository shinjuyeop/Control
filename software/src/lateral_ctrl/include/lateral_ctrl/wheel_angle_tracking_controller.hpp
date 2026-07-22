#pragma once

namespace lateral_ctrl
{
class WheelAngleTrackingController
{
public:
  WheelAngleTrackingController(
    bool enabled = false, double kp = 0.0, double ki = 0.0, double kd = 0.0,
    double integral_limit = 1.0, double correction_limit_deg = 0.5);
  double update(double target_deg, double actual_deg, double dt_sec);
  void reset();
  void setEnabled(bool enabled);
  double errorDeg() const { return error_deg_; }
  double correctionDeg() const { return correction_deg_; }
  double integralValue() const { return integral_; }

private:
  bool enabled_;
  bool have_previous_{false};
  double kp_, ki_, kd_, integral_limit_, correction_limit_deg_;
  double error_deg_{0.0}, previous_error_{0.0}, correction_deg_{0.0}, integral_{0.0};
};
}  // namespace lateral_ctrl
