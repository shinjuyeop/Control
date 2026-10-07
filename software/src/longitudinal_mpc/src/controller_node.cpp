#include "longitudinal_mpc/current_mpc.hpp"
extern "C" {
#include <longitudinal_controller.h>
}
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/create_timer.hpp>
#include <kaiev26_msgs/msg/actuator_command.hpp>
#include <kaiev26_msgs/msg/drive_current_command.hpp>
#include <kaiev26_msgs/msg/drive_actuator_status.hpp>
#include <kaiev26_msgs/msg/vehicle_state.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <std_msgs/msg/string.hpp>
#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>

namespace longitudinal_mpc {
using Steady=std::chrono::steady_clock;
using Command=kaiev26_msgs::msg::ActuatorCommand;
using Drive=kaiev26_msgs::msg::DriveActuatorStatus;
using State=kaiev26_msgs::msg::VehicleState;
using Current=kaiev26_msgs::msg::DriveCurrentCommand;
class ControllerNode: public rclcpp::Node {
public:
  ControllerNode():Node("longitudinal_controller") {
    const auto controller=StartupParameter<std::string>("controller","mpc");
    if(controller!="mpc" && controller!="pi") throw std::invalid_argument("controller must be mpc or pi");
    is_mpc_=controller=="mpc";
    limits_.current=StartupParameter<double>("current_max_a",300.);
    limits_.up=StartupParameter<double>("current_slew_up_a_s",2400.);
    limits_.down=StartupParameter<double>("current_slew_down_a_s",4800.);
    maximum_speed_=StartupParameter<double>("max_speed_mps",30./3.6);
    if(!std::isfinite(limits_.current+limits_.up+limits_.down+maximum_speed_) ||
        limits_.current<=0 || limits_.current>480 || limits_.up<=0 || limits_.down<=0 || maximum_speed_<=0)
      throw std::invalid_argument("Invalid current/speed operating envelope");
    const auto file=StartupParameter<std::string>("model_file","");
    const auto horizon=StartupParameter<int>("horizon",40);
    const double qv=StartupParameter<double>("speed_weight",10.);
    const double rd=StartupParameter<double>("delta_current_weight",.002);
    const double ru=StartupParameter<double>("current_weight",.00001);
    if(is_mpc_) {
      const auto model=Model::Load(file);
      maximum_speed_=std::min(maximum_speed_,model.speed_max);
      StartupParameter<std::vector<double>>("model_A", {model.A(0,0),model.A(0,1),model.A(1,0),model.A(1,1)}, true);
      StartupParameter<std::vector<double>>("model_B", {model.B[0],model.B[1]}, true);
      StartupParameter<std::vector<double>>("model_c", {model.c[0],model.c[1]}, true);
      StartupParameter<double>("model_dt", model.dt, true);
      mpc_=std::make_unique<CurrentMPC>(model,horizon,qv,rd,ru,limits_);
    }
    pi_config_=Long_DefaultConfig();
    if(!is_mpc_) {
      pi_config_.kp=StartupParameter<double>("speed_kp_a_per_mps",pi_config_.kp);
      pi_config_.ki=StartupParameter<double>("speed_ki_a_per_mps_s",pi_config_.ki);
      pi_config_.integrator_limit=StartupParameter<double>("integrator_limit_a",pi_config_.integrator_limit);
      pi_config_.deadband=StartupParameter<double>("speed_deadband_mps",pi_config_.deadband);
      pi_config_.hysteresis=StartupParameter<double>("speed_hysteresis_mps",pi_config_.hysteresis);
      pi_config_.reapply_current=StartupParameter<double>("reapply_current_a",pi_config_.reapply_current);
      pi_config_.reapply_slew=StartupParameter<double>("reapply_slew_a_s",pi_config_.reapply_slew);
    }
    pi_config_.max_current=limits_.current;
    pi_config_.slew_up=limits_.up; pi_config_.slew_down=limits_.down;
    if(!is_mpc_) {
      float wire[LONG_REQUEST_COUNT]={LONG_VERSION,1,0};
      Long_EncodeConfig(&pi_config_,wire+3);
      LongRequest request;
      if(!Long_DecodeRequest(wire,LONG_REQUEST_COUNT,&request))
        throw std::invalid_argument("Invalid PI configuration");
    }
    current_pub_=create_publisher<Current>("/vehicle/drive_current_command",rclcpp::QoS(1));
    diag_pub_=create_publisher<std_msgs::msg::String>("/control/longitudinal/diagnostics",10);
    diagnostics_.data.reserve(768);
    command_sub_=create_subscription<Command>("/vehicle/command",rclcpp::QoS(1),
      [this](Command::ConstSharedPtr m){
        if(!std::isfinite(m->speed_target_mps) || m->header.stamp.sec<0 || m->header.stamp.nanosec>=1000000000u) {
          command_.reset(); return;
        }
        command_=m; command_time_=Steady::now();
      });
    drive_sub_=create_subscription<Drive>("/drive/status",rclcpp::SensorDataQoS(),
      [this](Drive::ConstSharedPtr m){
        if(!std::isfinite(m->phase_current_a) || !std::isfinite(m->commanded_phase_current_a)) {
          drive_.reset(); return;
        }
        drive_=m; drive_time_=Steady::now();
      });
    state_sub_=create_subscription<State>("/vehicle/state",rclcpp::SensorDataQoS(),
      [this](State::ConstSharedPtr m){
        if(!std::isfinite(m->speed_mps)) { state_.reset(); return; }
        state_=m; state_time_=Steady::now();
      });
    config_sub_=create_subscription<std_msgs::msg::Float32MultiArray>("/drive/longitudinal_status",rclcpp::SensorDataQoS(),
      [this](std_msgs::msg::Float32MultiArray::ConstSharedPtr m){
        const auto &d=m->data;
        external_mode_=d.size()==LONG_STATUS_COUNT && d[0]==LONG_VERSION && d[5]==LONG_EXTERNAL_CURRENT;
        applied_limit_=external_mode_ && std::isfinite(d[8]) ? d[8]:0;
        config_time_=Steady::now();
      });
    timer_=rclcpp::create_timer(get_node_base_interface(),get_node_timers_interface(),get_clock(),
                              std::chrono::milliseconds(50),std::bind(&ControllerNode::Tick,this));
    RCLCPP_INFO(get_logger(),"%s: 20 Hz, %.0f A, external_current input, OSQP %s",controller.c_str(),limits_.current,osqp_version());
  }
private:
  template<class T> T StartupParameter(const std::string &name, const T &value, bool readback=false) {
    rcl_interfaces::msg::ParameterDescriptor descriptor;
    descriptor.read_only=true;
    return declare_parameter<T>(name,value,descriptor,readback);
  }
  const char *Inhibit() const {
    const auto now=Steady::now();
    auto age=[&](Steady::time_point t){return std::chrono::duration<double>(now-t).count();};
    if(!command_ || age(command_time_)>=.2) return "stale target";
    if(!drive_ || age(drive_time_)>=.2 || !state_ || age(state_time_)>=.2) return "stale feedback";
    if(!external_mode_ || age(config_time_)>=.7) return "external_current mode required";
    if(limits_.current>applied_limit_) return "controller limit exceeds MCU limit";
    const double stamp_age=(this->now()-rclcpp::Time(command_->header.stamp)).seconds();
    if(stamp_age<0 || stamp_age>=.2) return "stale target stamp";
    if(drive_->mode!=Drive::MODE_AUTO || drive_->stop_request || command_->brake_engage ||
       !drive_->part1_valid || !drive_->part2_valid ||
       drive_->part1_age_ms>=200 || drive_->part2_age_ms>=200 ||
       (drive_->fault_bits_valid && drive_->fault_bits!=0)) return "drive inhibited";
    if(drive_->controller_control_mode!=0) return "drive mode mismatch";
    if(command_->speed_target_mps<=0 || state_->speed_mps<-.05) return "zero target or reverse";
    if(count_publishers("/vehicle/drive_current_command")!=1) return "multiple current publishers";
    return nullptr;
  }
  void Tick() {
    const auto start=Steady::now();
    const double interval_ms=last_tick_==Steady::time_point{}?0:
      std::chrono::duration<double,std::milli>(start-last_tick_).count();
    last_tick_=start;
    const char *reason=Inhibit();
    Result result;
    double target=0,speed=0;
    if(!reason) {
      target=std::min(static_cast<double>(command_->speed_target_mps),maximum_speed_);
      speed=state_->speed_mps;
      if(is_mpc_) {
        result=mpc_->Solve(Eigen::Vector2d(speed,drive_->phase_current_a),target,
            last_current_);
        if(!result.solved) reason="solver failed";
      } else {
        // External-current mode: STM_A applies the rolling floor to this value itself.
        result.current=Long_Update(&pi_state_,&pi_config_,target,speed,.05,0);
        result.solved=true;
      }
      if(const char *late=Inhibit()) reason=late;
    }
    if(reason) { result.current=0; Long_Reset(&pi_state_); }
    Current message;
    message.header.stamp=now(); message.phase_current_a=result.current;
    current_pub_->publish(message);
    last_current_=result.current;
    const double cycle_ms=std::chrono::duration<double,std::milli>(Steady::now()-start).count();
    if(cycle_ms>=50.) ++deadline_misses_;
    char buffer[768];
    std::snprintf(buffer,sizeof(buffer),
      "{\"controller\":\"%s\",\"status\":\"%s\",\"current_a\":%.6f,\"speed_mps\":%.6f,\"target_mps\":%.6f,"
      "\"solve_ms\":%.6f,\"cycle_ms\":%.6f,\"interval_ms\":%.6f,\"deadline_misses\":%llu,\"iterations\":%ld}",
      is_mpc_?"MPC":"PI",reason?reason:"solved",result.current,speed,target,
      result.solve_ms,cycle_ms,interval_ms,static_cast<unsigned long long>(deadline_misses_),result.iterations);
    diagnostics_.data.assign(buffer); diag_pub_->publish(diagnostics_);
  }
  bool is_mpc_{false},external_mode_{false};
  double maximum_speed_{0},applied_limit_{0},last_current_{0};
  Limits limits_;
  std::unique_ptr<CurrentMPC> mpc_;
  LongConfig pi_config_{}; LongController pi_state_{};
  Command::ConstSharedPtr command_; Drive::ConstSharedPtr drive_; State::ConstSharedPtr state_;
  Steady::time_point command_time_{},drive_time_{},state_time_{},config_time_{},last_tick_{};
  uint64_t deadline_misses_{0};
  std_msgs::msg::String diagnostics_;
  rclcpp::Subscription<Command>::SharedPtr command_sub_;
  rclcpp::Subscription<Drive>::SharedPtr drive_sub_;
  rclcpp::Subscription<State>::SharedPtr state_sub_;
  rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr config_sub_;
  rclcpp::Publisher<Current>::SharedPtr current_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr diag_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};
}
int main(int argc,char **argv) {
  rclcpp::init(argc,argv);
  try {rclcpp::spin(std::make_shared<longitudinal_mpc::ControllerNode>());}
  catch(const std::exception &e) {RCLCPP_FATAL(rclcpp::get_logger("longitudinal_controller"),"%s",e.what());rclcpp::shutdown();return 1;}
  rclcpp::shutdown();return 0;
}
