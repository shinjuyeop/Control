// file: can_speed_sender.cpp
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/int32.hpp>
#include <std_msgs/msg/float32.hpp>  // [NEW] Float32 피드백
#include <rcl_interfaces/msg/set_parameters_result.hpp>

#include <linux/can.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <unistd.h>
#include <poll.h>

#include <atomic>
#include <thread>
#include <cstring>
#include <stdexcept>
#include <algorithm>
#include <cmath>
#include <limits>
#include <cstdint>

class CanSpeedSender : public rclcpp::Node
{
public:
  explicit CanSpeedSender(const std::string &can_iface)
      : Node("can_speed_sender")
  {
    // ===== Parameters (tune here) =====
    motoring_limit_a_ = declare_parameter<double>("motoring_limit_a", 500.0); // 가속 상한(phase)
    regen_limit_a_    = declare_parameter<double>("regen_limit_a", 0.0);      // 회생 상한(phase, 기본 0)

    rpm_fwd_max_ = declare_parameter<int>("rpm_fwd_max", 4500);
    rpm_rev_max_ = declare_parameter<int>("rpm_rev_max", 1500);

    // ★ 가속/감속 분리 램프 스텝
    accel_step_rpm_ = declare_parameter<int>("accel_step_rpm", 50); // 가속 시 한 틱(50ms)당 rpm
    decel_step_rpm_ = declare_parameter<int>("decel_step_rpm", 10); // 감속 시 한 틱(50ms)당 rpm

    brake_hyst_rpm_            = declare_parameter<int>("brake_hyst_rpm", 150);
    ctrl_temp_derate_start_    = declare_parameter<double>("ctrl_temp_derate_start_c", 58.0);
    ctrl_temp_derate_cut_      = declare_parameter<double>("ctrl_temp_derate_cut_c", 70.0);
    motor_temp_derate_start_   = declare_parameter<double>("motor_temp_derate_start_c", 100.0);
    motor_temp_derate_cut_     = declare_parameter<double>("motor_temp_derate_cut_c", 140.0);
    vdc_regen_derate_start_    = declare_parameter<double>("vdc_regen_derate_start_v", 58.0);
    vdc_regen_derate_cut_      = declare_parameter<double>("vdc_regen_derate_cut_v", 60.5);

    // Zero-hold(저속 채터링 방지)
    zero_hold_up_rpm_ = declare_parameter<double>("zero_hold_up_rpm", 120.0);
    zero_hold_dn_rpm_ = declare_parameter<double>("zero_hold_dn_rpm", 40.0);

    // Blend & Slew(코스팅→유지 경계 연속화)
    brake_blend_rpm_ = declare_parameter<double>("brake_blend_rpm", 180.0);
    i_limit_tau_ms_  = declare_parameter<double>("i_limit_tau_ms", 300.0);

    // ==== PID front-end (parameters) ====
    pid_enable_        = declare_parameter<bool>("pid_enable", true);
    pid_kp_            = declare_parameter<double>("pid_kp", 1.0);
    pid_ki_            = declare_parameter<double>("pid_ki", 0.0);
    pid_kd_            = declare_parameter<double>("pid_kd", 0.0);
    pid_out_limit_rpm_ = declare_parameter<double>("pid_out_limit_rpm", 800.0);  // |Δrpm| 상한
    pid_i_limit_       = declare_parameter<double>("pid_i_limit", 2000.0);       // ∫e dt 제한
    pid_d_tau_ms_      = declare_parameter<double>("pid_d_tau_ms", 100.0);       // D 저역통과 상수

    // ===== SocketCAN init =====
    sock_ = socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if (sock_ < 0) throw std::runtime_error("CAN socket open failed");

    struct ifreq ifr{};
    std::strncpy(ifr.ifr_name, can_iface.c_str(), IFNAMSIZ - 1);
    if (ioctl(sock_, SIOCGIFINDEX, &ifr) < 0) throw std::runtime_error("ifindex");

    sockaddr_can addr{};
    addr.can_family  = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;
    if (bind(sock_, reinterpret_cast<sockaddr *>(&addr), sizeof(addr)) < 0)
      throw std::runtime_error("bind");

    // ===== ROS I/O =====
    // 목표 rpm(판단/플래닝에서 옴)
    rpm_sub_ = create_subscription<std_msgs::msg::Int32>(
        "target_rpm", 10,
        [this](const std_msgs::msg::Int32::SharedPtr m) { target_rpm_.store(m->data); });

    // 현재 속도 피드백: 우리가 퍼블리시하는 /ctrl/speed (Float32)
    rpm_fb_sub_ = create_subscription<std_msgs::msg::Float32>(
        "/ctrl/speed", 10,
        [this](const std_msgs::msg::Float32::SharedPtr m) { ctrl_speed_rpm_.store(static_cast<double>(m->data)); });

    // RX thread
    rx_thread_ = std::thread([this]{ rx_loop(); });

    // Periodic TX (50 ms)
    tx_timer_ = create_wall_timer(std::chrono::milliseconds(50),
                                  std::bind(&CanSpeedSender::tick_50ms, this));

    // Live parameter update
    param_cb_ = this->add_on_set_parameters_callback(
        [this](const std::vector<rclcpp::Parameter> &params)
        {
          rcl_interfaces::msg::SetParametersResult out;
          out.successful = true;
          for (const auto &p : params)
          {
            if (p.get_name() == "motoring_limit_a")        motoring_limit_a_ = p.as_double();
            else if (p.get_name() == "regen_limit_a")      regen_limit_a_ = p.as_double();
            else if (p.get_name() == "rpm_fwd_max")        rpm_fwd_max_ = p.as_int();
            else if (p.get_name() == "rpm_rev_max")        rpm_rev_max_ = p.as_int();
            else if (p.get_name() == "accel_step_rpm")     accel_step_rpm_ = p.as_int();
            else if (p.get_name() == "decel_step_rpm")     decel_step_rpm_ = p.as_int();
            else if (p.get_name() == "brake_hyst_rpm")     brake_hyst_rpm_ = p.as_int();
            else if (p.get_name() == "zero_hold_up_rpm")   zero_hold_up_rpm_ = p.as_double();
            else if (p.get_name() == "zero_hold_dn_rpm")   zero_hold_dn_rpm_ = p.as_double();
            else if (p.get_name() == "brake_blend_rpm")    brake_blend_rpm_ = p.as_double();
            else if (p.get_name() == "i_limit_tau_ms")     i_limit_tau_ms_ = p.as_double();
            // [PID]
            else if (p.get_name() == "pid_enable")         pid_enable_ = p.as_bool();
            else if (p.get_name() == "pid_kp")             pid_kp_ = p.as_double();
            else if (p.get_name() == "pid_ki")             pid_ki_ = p.as_double();
            else if (p.get_name() == "pid_kd")             pid_kd_ = p.as_double();
            else if (p.get_name() == "pid_out_limit_rpm")  pid_out_limit_rpm_ = p.as_double();
            else if (p.get_name() == "pid_i_limit")        pid_i_limit_ = p.as_double();
            else if (p.get_name() == "pid_d_tau_ms")       pid_d_tau_ms_ = p.as_double();
          }
          return out;
        });
  }

  ~CanSpeedSender() override
  {
    running_ = false;
    if (rx_thread_.joinable()) rx_thread_.join();
    if (sock_ >= 0) close(sock_);
  }

private:
  // ===== EZkontrol CAN (J1939 ext) =====
  static constexpr canid_t ID_VCU_CMD   = 0x0C01EFD0;
  static constexpr canid_t ID_MCU_STAT1 = 0x1801D0EF;
  static constexpr canid_t ID_MCU_STAT2 = 0x1802D0EF;

  // Control bits (Byte4)
  static constexpr uint8_t CTRL_RUN   = 1u << 0;
  static constexpr uint8_t CTRL_SPEED = 1u << 1;

  // Protocol ranges/offsets
  static constexpr int RPM_MIN = -32000, RPM_MAX = 32000;
  static constexpr double IA_MIN_A = -3200.0, IA_MAX_A = 3200.0;
  static constexpr int TEMP_OFF_C = -40;

  // ===== State =====
  int sock_{-1};
  std::thread rx_thread_;
  rclcpp::TimerBase::SharedPtr tx_timer_;
  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr   rpm_sub_;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr rpm_fb_sub_; // [PID] feedback Float32

  std::atomic<bool> running_{true};
  std::atomic<bool> handshake_done_{false};

  // ROS input
  std::atomic<int32_t> target_rpm_{0};     // 계획(판단)에서 오는 목표
  std::atomic<double>  ctrl_speed_rpm_{0}; // /ctrl/speed Float32 피드백

  // Command smoothing
  int16_t vref_now_{0};
  uint8_t life_{0};

  // Feedback (from MCU CAN)
  std::atomic<double> vdc_v_{0.0};
  std::atomic<double> bus_current_a_{0.0};
  std::atomic<double> phase_current_a_{0.0};
  std::atomic<int32_t> speed_rpm_{0};
  std::atomic<double> ctrl_temp_c_{25.0};
  std::atomic<double> motor_temp_c_{25.0};
  std::atomic<uint32_t> err_group_{0};

  // Params
  double motoring_limit_a_{500.0};
  double regen_limit_a_{0.0};
  int rpm_fwd_max_{4500};
  int rpm_rev_max_{1500};
  int accel_step_rpm_{50};
  int decel_step_rpm_{10};
  int brake_hyst_rpm_{150};

  double ctrl_temp_derate_start_{58.0}, ctrl_temp_derate_cut_{70.0};
  double motor_temp_derate_start_{100.0}, motor_temp_derate_cut_{140.0};
  double vdc_regen_derate_start_{58.0}, vdc_regen_derate_cut_{60.5};

  // Zero-hold
  double zero_hold_up_rpm_{120.0};
  double zero_hold_dn_rpm_{40.0};
  bool   zero_hold_{true};

  // Blend & Slew
  double i_limit_tau_ms_{300.0};
  double brake_blend_rpm_{180.0};
  double i_max_smooth_{0.0};

  // ==== PID front-end ====
  bool   pid_enable_{true};
  double pid_kp_{1.0}, pid_ki_{0.0}, pid_kd_{0.0};
  double pid_out_limit_rpm_{800.0};
  double pid_i_limit_{2000.0};
  double pid_d_tau_ms_{100.0};

  double pid_i_state_{0.0};
  double pid_prev_err_{0.0};
  double pid_d_state_{0.0};

  // Param callback handle
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_cb_;

  // ===== Utilities =====
  template <typename T>
  static inline T clamp(T v, T lo, T hi)
  {
    return (v < lo) ? lo : ((v > hi) ? hi : v);
  }
  static inline int16_t ramp_to(int16_t tgt, int16_t cur, int16_t step)
  {
    if (tgt > cur) return static_cast<int16_t>(std::min<int>(cur + step, tgt));
    if (tgt < cur) return static_cast<int16_t>(std::max<int>(cur - step, tgt));
    return cur;
  }
  static inline double lin_derate(double x, double start, double cut)
  {
    if (cut <= start) return 1.0;
    if (x <= start)   return 1.0;
    if (x >= cut)     return 0.0;
    return (cut - x) / (cut - start);
  }

  // ===== RX: handshake + status =====
  void rx_loop()
  {
    can_frame fr{};
    while (running_)
    {
      struct pollfd pfd{};
      pfd.fd = sock_;
      pfd.events = POLLIN;
      pfd.revents = 0;
      int r = poll(&pfd, 1, 200);
      if (r <= 0) continue;
      if (!(pfd.revents & POLLIN)) continue;

      ssize_t n = read(sock_, &fr, sizeof(fr));
      if (n != sizeof(fr)) continue;

      const canid_t rxid = (fr.can_id & CAN_EFF_MASK);

      // (Re)Handshake: 0x55×8 → 0xAA
      if (rxid == ID_MCU_STAT1 && fr.can_dlc == 8)
      {
        bool all55 = true;
        for (int i = 0; i < fr.can_dlc; ++i)
          if (fr.data[i] != 0x55) { all55 = false; break; }
        if (all55)
        {
          send_handshake();
          handshake_done_ = true;
          RCLCPP_INFO(get_logger(), "Handshake (re)established (0x55→0xAA)");
          continue;
        }
      }

      // Status decode
      if (rxid == ID_MCU_STAT1 && fr.can_dlc == 8)
      {
        uint16_t vdc_raw = static_cast<uint16_t>(fr.data[0] | (fr.data[1] << 8));
        vdc_v_.store(vdc_raw * 0.1);

        uint16_t ibus_raw = static_cast<uint16_t>(fr.data[2] | (fr.data[3] << 8));
        bus_current_a_.store(ibus_raw * 0.1 + IA_MIN_A);

        uint16_t iph_raw = static_cast<uint16_t>(fr.data[4] | (fr.data[5] << 8));
        phase_current_a_.store(iph_raw * 0.1 + IA_MIN_A);

        uint16_t sp_raw = static_cast<uint16_t>(fr.data[6] | (fr.data[7] << 8));
        speed_rpm_.store(static_cast<int32_t>(sp_raw) + RPM_MIN);
      }
      else if (rxid == ID_MCU_STAT2 && fr.can_dlc == 8)
      {
        ctrl_temp_c_.store(static_cast<int>(fr.data[0]) + TEMP_OFF_C);
        motor_temp_c_.store(static_cast<int>(fr.data[1]) + TEMP_OFF_C);

        uint32_t g = 0;
        g |= (static_cast<uint32_t>(fr.data[3]));
        g |= (static_cast<uint32_t>(fr.data[4]) << 8);
        g |= (static_cast<uint32_t>(fr.data[5]) << 16);
        err_group_.store(g);
      }
    }
  }

  void send_handshake()
  {
    can_frame tx{};
    tx.can_id = ID_VCU_CMD | CAN_EFF_FLAG;
    tx.can_dlc = 8;
    std::fill(std::begin(tx.data), std::end(tx.data), 0xAA);
    (void)write(sock_, &tx, sizeof(tx));
  }

  // ===== 50ms tick =====
  void tick_50ms()
  {
    if (!handshake_done_) return;

    // ---- [0] PID front-end (target 보정) ----
    const double dt = 0.050; // 50 ms
    int32_t plan_tgt = target_rpm_.load();     // 판단/플래닝 목표
    double  meas     = ctrl_speed_rpm_.load(); // /ctrl/speed Float32

    double e = static_cast<double>(plan_tgt) - meas;
    double delta = 0.0;

    if (pid_enable_)
    {
      // I 적분(anti-windup)
      pid_i_state_ += e * dt;
      pid_i_state_  = clamp(pid_i_state_, -pid_i_limit_, pid_i_limit_);

      // D(저역통과)
      double d_raw  = (e - pid_prev_err_) / dt;
      double alpha  = (pid_d_tau_ms_ <= 0.0) ? 1.0 : (dt / (pid_d_tau_ms_ / 1000.0 + dt));
      pid_d_state_ += alpha * (d_raw - pid_d_state_);
      pid_prev_err_ = e;

      delta = pid_kp_ * e + pid_ki_ * pid_i_state_ + pid_kd_ * pid_d_state_;
      delta = clamp(delta, -pid_out_limit_rpm_, +pid_out_limit_rpm_);
    }

    // PID 보정된 목표 rpm
    int32_t tgt = plan_tgt + static_cast<int32_t>(std::lround(delta));

    // ---- [1] Clamp target (기존 로직) ----
    int fwd_max = clamp(rpm_fwd_max_, 0, RPM_MAX);
    int rev_max = clamp(rpm_rev_max_, 0, RPM_MAX);
    if (tgt >= 0) tgt = clamp<int32_t>(tgt, 0, fwd_max);
    else          tgt = -clamp<int32_t>(-tgt, 0, rev_max);

    // 1.5) 가속/감속 판정 → 스텝 선택 (기존)
    bool same_sign_tv = (tgt == 0 || vref_now_ == 0) ? (tgt >= 0 ? vref_now_ >= 0 : vref_now_ <= 0)
                                                     : ((tgt > 0) == (vref_now_ > 0));
    bool accel_phase   = same_sign_tv && (std::abs(tgt) > std::abs(static_cast<int32_t>(vref_now_)));

    int step_cap_for_decel = static_cast<int>(std::floor(0.2 * std::max(1.0, brake_blend_rpm_)));
    step_cap_for_decel = std::max(step_cap_for_decel, 1);

    int step_use = accel_phase ? std::max(accel_step_rpm_, 1)
                               : std::max(1, std::min(decel_step_rpm_, step_cap_for_decel));

    // [2] Ramp toward tgt (기존)
    vref_now_ = ramp_to(static_cast<int16_t>(tgt), vref_now_, static_cast<int16_t>(step_use));

    // [3] Zero-hold (기존)
    {
      if (zero_hold_up_rpm_ < zero_hold_dn_rpm_ + 1.0)
        zero_hold_up_rpm_ = zero_hold_dn_rpm_ + 1.0;

      double abs_tgt = std::abs(static_cast<double>(tgt));
      if (zero_hold_)
      {
        if (abs_tgt >= zero_hold_up_rpm_) zero_hold_ = false;
      }
      else
      {
        if (abs_tgt <= zero_hold_dn_rpm_) zero_hold_ = true;
      }
      if (zero_hold_) vref_now_ = 0;
    }

    // [4] Derate (기존)
    double f_temp      = std::min(
        lin_derate(ctrl_temp_c_.load(),  ctrl_temp_derate_start_,  ctrl_temp_derate_cut_),
        lin_derate(motor_temp_c_.load(), motor_temp_derate_start_, motor_temp_derate_cut_));
    double f_regen_vdc = lin_derate(vdc_v_.load(), vdc_regen_derate_start_, vdc_regen_derate_cut_);
    bool hard_fault    = (err_group_.load() != 0);
    if (hard_fault) f_temp = 0.0;

    // [5] 회생/가속 판단 (기존)
    int spd = speed_rpm_.load();
    bool same_sign_sv = (spd != 0 && vref_now_ != 0) ? ((spd > 0) == (vref_now_ > 0)) : false;
    bool braking      = same_sign_sv && (std::abs(spd) > std::abs(vref_now_) + brake_hyst_rpm_);

    double mot_lim = motoring_limit_a_ * clamp(f_temp, 0.0, 1.0);
    double reg_lim = regen_limit_a_   * clamp(std::min(f_temp, f_regen_vdc), 0.0, 1.0); // 기본 0

    // [6] 브레이크 블렌딩 (기존)
    double gap = std::max(0.0, std::abs(static_cast<double>(spd)) - std::abs(static_cast<double>(vref_now_)));
    double H = static_cast<double>(brake_hyst_rpm_);
    double B = std::max(1.0, brake_blend_rpm_);

    double k = 0.0;
    if      (gap <= H)       k = 0.0;
    else if (gap >= H + B)   k = 1.0;
    else                     k = (gap - H) / B;

    double i_cmd = (1.0 - k) * mot_lim + k * reg_lim;

    // [7] 전류 상한 슬루(저역통과) (기존)
    {
      const double tau = std::max(0.010, i_limit_tau_ms_ / 1000.0);
      const double a   = dt / (tau + dt);
      i_max_smooth_   += a * (i_cmd - i_max_smooth_);
    }
    double i_max = i_max_smooth_;

    // [8] Phase current limit sign (기존)
    double phase_limit_a = 0.0;
    if      (vref_now_ > 0) phase_limit_a = +i_max;
    else if (vref_now_ < 0) phase_limit_a = -i_max;

    // [9] Encode & TX (기존)
    auto enc_current = [](double amp_a) -> uint16_t
    {
      const double raw = (amp_a - IA_MIN_A) / 0.1;
      return static_cast<uint16_t>(std::clamp<long>(lround(raw), 0, 65535));
    };
    auto enc_speed = [](int16_t rpm) -> uint16_t
    {
      const long raw = static_cast<long>(rpm) - (-32000);
      return static_cast<uint16_t>(std::clamp<long>(raw, 0, 65535));
    };

    const uint16_t current_raw = enc_current(phase_limit_a);
    const uint16_t speed_raw   = enc_speed(vref_now_);

    can_frame tx{};
    tx.can_id  = ID_VCU_CMD | CAN_EFF_FLAG;
    tx.can_dlc = 8;
    tx.data[0] = static_cast<uint8_t>(current_raw & 0xFF);
    tx.data[1] = static_cast<uint8_t>((current_raw >> 8) & 0xFF);
    tx.data[2] = static_cast<uint8_t>(speed_raw & 0xFF);
    tx.data[3] = static_cast<uint8_t>((speed_raw >> 8) & 0xFF);
    uint8_t ctrl = 0;
    ctrl |= CTRL_RUN;
    ctrl |= CTRL_SPEED;
    tx.data[4] = ctrl;
    tx.data[5] = 0x00;
    tx.data[6] = 0x00;
    tx.data[7] = life_++;
    (void)write(sock_, &tx, sizeof(tx));

    // Log when tgt changes
    static int32_t prev_tgt = std::numeric_limits<int32_t>::min();
    if (tgt != prev_tgt)
    {
      prev_tgt = tgt;
      RCLCPP_INFO(get_logger(),
        "TX vref=%d tgt=%d(step=%d accel=%s)  iMax=%.0fA(%s) k=%.2f  vdc=%.1fV Tctrl=%.0fC Tmot=%.0fC err=0x%06X  [PID plan=%d meas=%.1f e=%.1f dRPM=%.1f]",
        vref_now_, tgt, step_use, accel_phase ? "Y" : "N",
        i_max, braking ? "regen" : "mot", k,
        vdc_v_.load(), ctrl_temp_c_.load(), motor_temp_c_.load(), err_group_.load(),
        plan_tgt, meas, e, delta);
    }
  }
};

// ---- main: 사용자 인자(--kp/--ki/--kd/--pid-enable/--pid-disable/--iface) 파싱 지원 ----
int main(int argc, char **argv)
{
  // ROS 인자 제거
  rclcpp::init(argc, argv);

  // 남은 사용자 인자 파싱
  std::string iface = "can0";
  bool set_pid_enable = false, set_pid_disable = false;
  bool has_kp=false, has_ki=false, has_kd=false;
  double kp=0, ki=0, kd=0;

  auto eat_val = [&](int &i, double &dst){
    std::string s = argv[i];
    auto eq = s.find('=');
    if (eq != std::string::npos) { dst = std::stod(s.substr(eq+1)); return; }
    if (i+1 < argc) { dst = std::stod(argv[++i]); }
  };

  for (int i=1; i<argc; ++i)
  {
    std::string a = argv[i];
    if (a == "--iface") {
      if (i+1 < argc) iface = argv[++i];
    } else if (a.rfind("--iface=",0)==0) {
      iface = a.substr(8);
    } else if (a == "--kp" || a.rfind("--kp=",0)==0) {
      eat_val(i, kp); has_kp=true;
    } else if (a == "--ki" || a.rfind("--ki=",0)==0) {
      eat_val(i, ki); has_ki=true;
    } else if (a == "--kd" || a.rfind("--kd=",0)==0) {
      eat_val(i, kd); has_kd=true;
    } else if (a == "--pid-enable") {
      set_pid_enable = true;
    } else if (a == "--pid-disable") {
      set_pid_disable = true;
    } else if (i==1 && !a.empty() && a[0] != '-') {
      // 첫 번째 위치 인자로 iface 허용 (호환)
      iface = a;
    }
  }

  auto node = std::make_shared<CanSpeedSender>(iface);

  // 인자로 들어온 PID 값 즉시 반영 (ROS 파라미터 override와 병행 가능)
  std::vector<rclcpp::Parameter> ps;
  if (has_kp) ps.emplace_back("pid_kp", kp);
  if (has_ki) ps.emplace_back("pid_ki", ki);
  if (has_kd) ps.emplace_back("pid_kd", kd);
  if (set_pid_enable)  ps.emplace_back("pid_enable", true);
  if (set_pid_disable) ps.emplace_back("pid_enable", false);
  if (!ps.empty()) node->set_parameters(ps);

  try {
    rclcpp::spin(node);
  } catch (const std::exception &e) {
    RCLCPP_FATAL(rclcpp::get_logger("rclcpp"), "Exception: %s", e.what());
  }
  rclcpp::shutdown();
  return 0;
}
