// file: can_speed_sender.cpp
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/int32.hpp>
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
    regen_limit_a_ = declare_parameter<double>("regen_limit_a", 0.0);         // 회생 상한(phase, 기본 0)

    rpm_fwd_max_ = declare_parameter<int>("rpm_fwd_max", 4500);
    rpm_rev_max_ = declare_parameter<int>("rpm_rev_max", 1500);

    // ★ 가속/감속 분리 램프 스텝
    accel_step_rpm_ = declare_parameter<int>("accel_step_rpm", 100); // 가속 시 한 틱(50ms)당 rpm
    decel_step_rpm_ = declare_parameter<int>("decel_step_rpm", 10); // 감속 시 한 틱(50ms)당 rpm

    brake_hyst_rpm_ = declare_parameter<int>("brake_hyst_rpm", 150); // 경계 히스테리시스(권장 상향)
    ctrl_temp_derate_start_ = declare_parameter<double>("ctrl_temp_derate_start_c", 58.0);
    ctrl_temp_derate_cut_ = declare_parameter<double>("ctrl_temp_derate_cut_c", 70.0);
    motor_temp_derate_start_ = declare_parameter<double>("motor_temp_derate_start_c", 100.0);
    motor_temp_derate_cut_ = declare_parameter<double>("motor_temp_derate_cut_c", 140.0);
    vdc_regen_derate_start_ = declare_parameter<double>("vdc_regen_derate_start_v", 58.0);
    vdc_regen_derate_cut_ = declare_parameter<double>("vdc_regen_derate_cut_v", 60.5);

    // Zero-hold(저속 채터링 방지)
    zero_hold_up_rpm_ = declare_parameter<double>("zero_hold_up_rpm", 120.0);
    zero_hold_dn_rpm_ = declare_parameter<double>("zero_hold_dn_rpm", 40.0);

    // Blend & Slew(코스팅→유지 경계 연속화)
    brake_blend_rpm_ = declare_parameter<double>("brake_blend_rpm", 180.0);
    i_limit_tau_ms_  = declare_parameter<double>("i_limit_tau_ms", 200.0);

    // ==== PID front-end (parameters) ====
    pid_enable_        = declare_parameter<bool>("pid_enable", true);
    pid_kp_            = declare_parameter<double>("pid_kp", 0.0);
    pid_ki_            = declare_parameter<double>("pid_ki", 0.0);
    pid_kd_            = declare_parameter<double>("pid_kd", 0.0);
    pid_out_limit_rpm_ = declare_parameter<double>("pid_out_limit_rpm", 800.0);  // |Δrpm| 상한
    pid_i_limit_       = declare_parameter<double>("pid_i_limit", 2000.0);       // ∫e dt 제한
    pid_d_tau_ms_      = declare_parameter<double>("pid_d_tau_ms", 100.0);       // D 저역통과 상수

    // ===== SocketCAN init =====
    sock_ = socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if (sock_ < 0)
      throw std::runtime_error("CAN socket open failed");

    struct ifreq ifr{};
    std::strncpy(ifr.ifr_name, can_iface.c_str(), IFNAMSIZ - 1);
    if (ioctl(sock_, SIOCGIFINDEX, &ifr) < 0)
      throw std::runtime_error("ifindex");

    sockaddr_can addr{};
    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;
    if (bind(sock_, reinterpret_cast<sockaddr *>(&addr), sizeof(addr)) < 0)
      throw std::runtime_error("bind");

    // ===== ROS I/O =====
    rpm_sub_ = create_subscription<std_msgs::msg::Int32>(
        "target_rpm", 10,
        [this](const std_msgs::msg::Int32::SharedPtr m)
        { target_rpm_.store(m->data); });

    // RX thread
    rx_thread_ = std::thread([this]
                             { rx_loop(); });

    // Periodic TX (50 ms)
    tx_timer_ = create_wall_timer(
        std::chrono::milliseconds(50),
        std::bind(&CanSpeedSender::tick_50ms, this));

    // Live parameter update
    param_cb_ = this->add_on_set_parameters_callback(
        [this](const std::vector<rclcpp::Parameter> &params)
        {
          rcl_interfaces::msg::SetParametersResult out;
          out.successful = true;
          for (const auto &p : params)
          {
            if (p.get_name() == "motoring_limit_a")
              motoring_limit_a_ = p.as_double();
            else if (p.get_name() == "regen_limit_a")
              regen_limit_a_ = p.as_double();
            else if (p.get_name() == "rpm_fwd_max")
              rpm_fwd_max_ = p.as_int();
            else if (p.get_name() == "rpm_rev_max")
              rpm_rev_max_ = p.as_int();
            else if (p.get_name() == "accel_step_rpm")
              accel_step_rpm_ = p.as_int();
            else if (p.get_name() == "decel_step_rpm")
              decel_step_rpm_ = p.as_int();
            else if (p.get_name() == "brake_hyst_rpm")
              brake_hyst_rpm_ = p.as_int();
            else if (p.get_name() == "zero_hold_up_rpm")
              zero_hold_up_rpm_ = p.as_double();
            else if (p.get_name() == "zero_hold_dn_rpm")
              zero_hold_dn_rpm_ = p.as_double();
            else if (p.get_name() == "brake_blend_rpm")
              brake_blend_rpm_ = p.as_double();
            else if (p.get_name() == "i_limit_tau_ms")
              i_limit_tau_ms_ = p.as_double();
          }
          return out;
        });
  }

  ~CanSpeedSender() override
  {
    running_ = false;
    if (rx_thread_.joinable())
      rx_thread_.join();
    if (sock_ >= 0)
      close(sock_);
  }

private:
  // ===== EZkontrol CAN (J1939 ext) =====
  static constexpr canid_t ID_VCU_CMD = 0x0C01EFD0;
  static constexpr canid_t ID_MCU_STAT1 = 0x1801D0EF;
  static constexpr canid_t ID_MCU_STAT2 = 0x1802D0EF;

  // Control bits (Byte4)
  static constexpr uint8_t CTRL_RUN = 1u << 0;
  static constexpr uint8_t CTRL_SPEED = 1u << 1;

  // Protocol ranges/offsets
  static constexpr int RPM_MIN = -32000, RPM_MAX = 32000;
  static constexpr double IA_MIN_A = -3200.0, IA_MAX_A = 3200.0;
  static constexpr int TEMP_OFF_C = -40;

  // ===== State =====
  int sock_{-1};
  std::thread rx_thread_;
  rclcpp::TimerBase::SharedPtr tx_timer_;
  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr rpm_sub_;

  std::atomic<bool> running_{true};
  std::atomic<bool> handshake_done_{false};

  // ROS input
  std::atomic<int32_t> target_rpm_{0};

  // Command smoothing
  int16_t vref_now_{0};
  uint8_t life_{0};

  // Feedback
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
  int accel_step_rpm_{100}; // 가속 시
  int decel_step_rpm_{10}; // 감속 시
  int brake_hyst_rpm_{150};

  double ctrl_temp_derate_start_{58.0}, ctrl_temp_derate_cut_{70.0};
  double motor_temp_derate_start_{100.0}, motor_temp_derate_cut_{140.0};
  double vdc_regen_derate_start_{58.0}, vdc_regen_derate_cut_{60.5};

  // Zero-hold
  double zero_hold_up_rpm_{120.0};
  double zero_hold_dn_rpm_{40.0};
  bool zero_hold_{true};

  // Blend & Slew
  double i_limit_tau_ms_{200.0};
  double brake_blend_rpm_{180.0};
  double i_limit_tau_ms_{300.0};
  double i_max_smooth_{0.0};

  // ==== PID front-end ====
  bool   pid_enable_{true};
  double pid_kp_{0.0}, pid_ki_{0.0}, pid_kd_{0.0};
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
    if (tgt > cur)
      return static_cast<int16_t>(std::min<int>(cur + step, tgt));
    if (tgt < cur)
      return static_cast<int16_t>(std::max<int>(cur - step, tgt));
    return cur;
  }
  static inline double lin_derate(double x, double start, double cut)
  {
    if (cut <= start)
      return 1.0;
    if (x <= start)
      return 1.0;
    if (x >= cut)
      return 0.0;
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
      if (r <= 0)
        continue;
      if (!(pfd.revents & POLLIN))
        continue;

      ssize_t n = read(sock_, &fr, sizeof(fr));
      if (n != sizeof(fr))
        continue;

      const canid_t rxid = (fr.can_id & CAN_EFF_MASK);

      // (Re)Handshake: 0x55×8 → 0xAA
      if (rxid == ID_MCU_STAT1 && fr.can_dlc == 8)
      {
        bool all55 = true;
        for (int i = 0; i < fr.can_dlc; ++i)
          if (fr.data[i] != 0x55)
          {
            all55 = false;
            break;
          }
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
    if (!handshake_done_)
      return;

    // 1) Clamp target
    int32_t tgt = target_rpm_.load();
    int fwd_max = clamp(rpm_fwd_max_, 0, RPM_MAX);
    int rev_max = clamp(rpm_rev_max_, 0, RPM_MAX);
    if (tgt >= 0)
      tgt = clamp<int32_t>(tgt, 0, fwd_max);
    else
      tgt = -clamp<int32_t>(-tgt, 0, rev_max);

    // 1.5) 가속/감속 판정 → 스텝 선택
    // 가속: |tgt|가 |vref_now_|보다 크고 부호가 같을 때
    bool same_sign_tv = (tgt == 0 || vref_now_ == 0) ? (tgt >= 0 ? vref_now_ >= 0 : vref_now_ <= 0)
                                                     : ((tgt > 0) == (vref_now_ > 0));
    bool accel_phase = same_sign_tv && (std::abs(tgt) > std::abs(static_cast<int32_t>(vref_now_)));

    int step_cap_for_decel = static_cast<int>(std::floor(0.2 * std::max(1.0, brake_blend_rpm_))); // Δk≤0.2
    step_cap_for_decel = std::max(step_cap_for_decel, 1);

    int step_use = accel_phase
                       ? std::max(accel_step_rpm_, 1)                                // 가속: 캡 없음(경계 영향 적음)
                       : std::max(1, std::min(decel_step_rpm_, step_cap_for_decel)); // 감속: 블렌드 기반 캡 적용

    // 2) Ramp toward tgt
    vref_now_ = ramp_to(static_cast<int16_t>(tgt), vref_now_, static_cast<int16_t>(step_use));

    // 3) Zero-hold Schmitt trigger
    {
      if (zero_hold_up_rpm_ < zero_hold_dn_rpm_ + 1.0)
        zero_hold_up_rpm_ = zero_hold_dn_rpm_ + 1.0;

      double abs_tgt = std::abs(static_cast<double>(tgt));
      if (zero_hold_)
      {
        if (abs_tgt >= zero_hold_up_rpm_)
          zero_hold_ = false;
      }
      else
      {
        if (abs_tgt <= zero_hold_dn_rpm_)
          zero_hold_ = true;
      }
      if (zero_hold_)
        vref_now_ = 0;
    }

    // 4) Derate calculation
    double f_temp = std::min(
        lin_derate(ctrl_temp_c_.load(), ctrl_temp_derate_start_, ctrl_temp_derate_cut_),
        lin_derate(motor_temp_c_.load(), motor_temp_derate_start_, motor_temp_derate_cut_));
    double f_regen_vdc = lin_derate(vdc_v_.load(), vdc_regen_derate_start_, vdc_regen_derate_cut_);
    bool hard_fault = (err_group_.load() != 0);
    if (hard_fault)
      f_temp = 0.0;

    // 5) 회생/가속 판단 (현재 vref 기준)
    int spd = speed_rpm_.load();
    bool same_sign_sv = (spd != 0 && vref_now_ != 0) ? ((spd > 0) == (vref_now_ > 0)) : false;
    bool braking = same_sign_sv && (std::abs(spd) > std::abs(vref_now_) + brake_hyst_rpm_);

    double mot_lim = motoring_limit_a_ * clamp(f_temp, 0.0, 1.0);
    double reg_lim = regen_limit_a_ * clamp(std::min(f_temp, f_regen_vdc), 0.0, 1.0); // 기본 0

    // 6) 브레이크 블렌딩
    double gap = std::max(0.0,
                          std::abs(static_cast<double>(spd)) - std::abs(static_cast<double>(vref_now_)));
    double H = static_cast<double>(brake_hyst_rpm_);
    double B = std::max(1.0, brake_blend_rpm_);

    double k = 0.0;
    if (gap <= H)
      k = 0.0;
    else if (gap >= H + B)
      k = 1.0;
    else
      k = (gap - H) / B;

    double i_cmd = (1.0 - k) * mot_lim + k * reg_lim; // 회생=0이면 모터링→0으로 연속 전이

    // 7) 전류 상한 슬루(저역통과)
    const double dt = 0.050;
    const double tau = std::max(0.010, i_limit_tau_ms_ / 1000.0);
    const double a = dt / (tau + dt);
    i_max_smooth_ += a * (i_cmd - i_max_smooth_);
    double i_max = i_max_smooth_;

    // 8) Phase current limit sign
    double phase_limit_a = 0.0;
    if (vref_now_ > 0)
      phase_limit_a = +i_max;
    else if (vref_now_ < 0)
      phase_limit_a = -i_max;
    else
      phase_limit_a = 0.0;

    // 9) Encode & TX
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
    const uint16_t speed_raw = enc_speed(vref_now_);

    can_frame tx{};
    tx.can_id = ID_VCU_CMD | CAN_EFF_FLAG;
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
                  "TX vref=%d tgt=%d step=%d accel=%s  iMax=%.0fA(%s) k=%.2f  vdc=%.1fV Tctrl=%.0fC Tmot=%.0fC err=0x%06X",
                  vref_now_, tgt, step_use, accel_phase ? "Y" : "N",
                  i_max, braking ? "regen" : "mot", k,
                  vdc_v_.load(), ctrl_temp_c_.load(), motor_temp_c_.load(), err_group_.load());
    }
  }
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  std::string iface = (argc > 1) ? argv[1] : "can0";
  try
  {
    rclcpp::spin(std::make_shared<CanSpeedSender>(iface));
  }
  catch (const std::exception &e)
  {
    RCLCPP_FATAL(rclcpp::get_logger("rclcpp"), "Exception: %s", e.what());
  }
  rclcpp::shutdown();
  return 0;
}
