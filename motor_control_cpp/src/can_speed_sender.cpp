// file: can_speed_sender.cpp
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/int32.hpp>

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

class CanSpeedSender : public rclcpp::Node {
public:
  explicit CanSpeedSender(const std::string& can_iface)
  : Node("can_speed_sender")
  {
    // ===== Parameters (tune here) =====
    // HPM-10kW @48V: Max phase 500A(10s) / rated bus ~245A.
    // EZkontrol C481200: Rated phase 480A / Max phase 1200A / Max bus 600A.
    motoring_limit_a_  = declare_parameter<double>("motoring_limit_a", 450.0); // 가속 상한(phase)
    regen_limit_a_     = declare_parameter<double>("regen_limit_a",    180.0); // 회생 상한(phase)

    rpm_fwd_max_       = declare_parameter<int>("rpm_fwd_max", 4500);  // 전진 상한
    rpm_rev_max_       = declare_parameter<int>("rpm_rev_max", 1500);  // 후진 상한(~30%)
    ramp_step_rpm_     = declare_parameter<int>("ramp_step_rpm", 10);  // 50 ms당 10 rpm → 200 rpm/s
    brake_hyst_rpm_    = declare_parameter<int>("brake_hyst_rpm", 50); // 브레이크 판정 히스테리시스

    // Derate thresholds (선형 스케일): factor = (cut - x)/(cut - start)
    // 컨트롤러 과열 기준 70℃ 근처 → 55~60에서 감쇠 시작, 70에서 컷
    ctrl_temp_derate_start_ = declare_parameter<double>("ctrl_temp_derate_start_c", 58.0);
    ctrl_temp_derate_cut_   = declare_parameter<double>("ctrl_temp_derate_cut_c",   70.0);

    motor_temp_derate_start_= declare_parameter<double>("motor_temp_derate_start_c",100.0);
    motor_temp_derate_cut_  = declare_parameter<double>("motor_temp_derate_cut_c", 140.0);

    // 48V계 팩: 과충전/과전압 방지용 회생 감쇠
    vdc_regen_derate_start_ = declare_parameter<double>("vdc_regen_derate_start_v", 58.0);
    vdc_regen_derate_cut_   = declare_parameter<double>("vdc_regen_derate_cut_v",   60.5);

    // ===== SocketCAN init =====
    sock_ = socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if (sock_ < 0) throw std::runtime_error("CAN socket open failed");

    struct ifreq ifr{};
    std::strncpy(ifr.ifr_name, can_iface.c_str(), IFNAMSIZ - 1);
    if (ioctl(sock_, SIOCGIFINDEX, &ifr) < 0) throw std::runtime_error("ifindex");

    sockaddr_can addr{};
    addr.can_family  = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;
    if (bind(sock_, reinterpret_cast<sockaddr*>(&addr), sizeof(addr)) < 0)
      throw std::runtime_error("bind");

    // ===== ROS I/O =====
    rpm_sub_ = create_subscription<std_msgs::msg::Int32>(
      "target_rpm", 10,
      [this](const std_msgs::msg::Int32::SharedPtr m){ target_rpm_.store(m->data); });

    // ===== RX thread (handshake + status receive) =====
    rx_thread_ = std::thread([this]{ rx_loop(); });

    // ===== Periodic TX (50 ms) =====
    tx_timer_ = create_wall_timer(
      std::chrono::milliseconds(50),
      std::bind(&CanSpeedSender::tick_50ms, this));
  }

  ~CanSpeedSender() override {
    running_ = false;
    if (rx_thread_.joinable()) rx_thread_.join();
    if (sock_ >= 0) close(sock_);
  }

private:
  // ===== EZkontrol CAN (J1939 ext) =====
  static constexpr canid_t ID_VCU_CMD   = 0x0C01EFD0; // VCU→MCU
  static constexpr canid_t ID_MCU_STAT1 = 0x1801D0EF; // MCU→VCU (bus/phase/speed)
  static constexpr canid_t ID_MCU_STAT2 = 0x1802D0EF; // MCU→VCU (temps/status)

  // Control bits (Byte4)
  static constexpr uint8_t CTRL_RUN   = 1u << 0;     // RUNNING
  static constexpr uint8_t CTRL_SPEED = 1u << 1;     // SPEED mode (표시용)

  // Protocol ranges/offsets
  static constexpr int     RPM_MIN  = -32000, RPM_MAX = 32000; // 1 rpm/bit, offset -32000
  static constexpr double  IA_MIN_A = -3200.0, IA_MAX_A = 3200.0; // 0.1 A/bit, offset -3200
  static constexpr int     TEMP_OFF_C = -40; // 1 C/bit, offset -40 (STAT2)

  // ===== State =====
  int sock_{-1};
  std::thread rx_thread_;
  rclcpp::TimerBase::SharedPtr tx_timer_;
  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr rpm_sub_;

  std::atomic<bool> running_{true};
  std::atomic<bool> handshake_done_{false};

  // ROS inputs
  std::atomic<int32_t> target_rpm_{0};

  // Command smoothing
  int16_t vref_now_{0};         // current commanded rpm (signed)
  uint8_t life_{0};

  // Latest feedback (atomic for simplicity)
  std::atomic<double> vdc_v_{0.0};        // bus voltage (V)
  std::atomic<double> bus_current_a_{0.0};
  std::atomic<double> phase_current_a_{0.0};
  std::atomic<int32_t> speed_rpm_{0};     // 실제 속도 (rpm, 부호 있음)
  std::atomic<double> ctrl_temp_c_{25.0};
  std::atomic<double> motor_temp_c_{25.0};
  std::atomic<uint32_t> err_group_{0};    // OR of error groups (STAT2 bytes 3,4,5)

  // Params
  double motoring_limit_a_{450.0};
  double regen_limit_a_{180.0};
  int    rpm_fwd_max_{4500};
  int    rpm_rev_max_{1500};
  int    ramp_step_rpm_{10};
  int    brake_hyst_rpm_{50};

  double ctrl_temp_derate_start_{58.0}, ctrl_temp_derate_cut_{70.0};
  double motor_temp_derate_start_{100.0}, motor_temp_derate_cut_{140.0};
  double vdc_regen_derate_start_{58.0},   vdc_regen_derate_cut_{60.5};

  // ===== Utilities =====
  template<typename T>
  static inline T clamp(T v, T lo, T hi) {
    return (v < lo) ? lo : ((v > hi) ? hi : v);
  }
  static inline int16_t ramp_to(int16_t tgt, int16_t cur, int16_t step) {
    if (tgt > cur) return static_cast<int16_t>(std::min<int>(cur + step, tgt));
    if (tgt < cur) return static_cast<int16_t>(std::max<int>(cur - step, tgt));
    return cur;
  }
  static inline double lin_derate(double x, double start, double cut) {
    // x >= cut → 0,  x <= start → 1,  선형 감소
    if (cut <= start) return 1.0;
    if (x <= start) return 1.0;
    if (x >= cut)   return 0.0;
    return (cut - x) / (cut - start);
  }

  // ===== RX: handshake + status =====
  void rx_loop() {
    can_frame fr{};
    while (running_) {
      struct pollfd pfd{}; // C++17-compatible init
      pfd.fd = sock_;
      pfd.events = POLLIN;
      pfd.revents = 0;
      int r = poll(&pfd, 1, 200);
      if (r <= 0) continue;
      if (!(pfd.revents & POLLIN)) continue;

      ssize_t n = read(sock_, &fr, sizeof(fr));
      if (n != sizeof(fr)) continue;

      const canid_t rxid = (fr.can_id & CAN_EFF_MASK);

      // --- (Re)Handshake: 언제든 0x55×8 받으면 0xAA 회신 ---
      if (rxid == ID_MCU_STAT1 && fr.can_dlc == 8) {
        bool all55 = true;
        for (int i = 0; i < fr.can_dlc; ++i) if (fr.data[i] != 0x55) { all55 = false; break; }
        if (all55) {
          send_handshake();
          handshake_done_ = true;
          RCLCPP_INFO(get_logger(), "Handshake (re)established (0x55→0xAA)");
          continue;
        }
      }

      // --- Status decode ---
      if (rxid == ID_MCU_STAT1 && fr.can_dlc == 8) {
        // Byte0-1: Bus Voltage (0.1 V/bit, no offset)
        uint16_t vdc_raw = static_cast<uint16_t>(fr.data[0] | (fr.data[1] << 8));
        vdc_v_.store(vdc_raw * 0.1);

        // Byte2-3: Bus Current (0.1 A, offset -3200)
        int16_t ibus_raw = static_cast<int16_t>(fr.data[2] | (fr.data[3] << 8));
        bus_current_a_.store((ibus_raw * 0.1) + IA_MIN_A);

        // Byte4-5: Phase Current (0.1 A, offset -3200)
        int16_t iph_raw = static_cast<int16_t>(fr.data[4] | (fr.data[5] << 8));
        phase_current_a_.store((iph_raw * 0.1) + IA_MIN_A);

        // Byte6-7: Speed (1 rpm, offset -32000)
        int16_t sp_raw = static_cast<int16_t>(fr.data[6] | (fr.data[7] << 8));
        speed_rpm_.store(static_cast<int32_t>(sp_raw - (-32000)));
      }
      else if (rxid == ID_MCU_STAT2 && fr.can_dlc == 8) {
        // *** FIX: 부호 없는 값으로 산술 후 -40 오프셋 적용 ***
        ctrl_temp_c_.store(static_cast<int>(fr.data[0]) + TEMP_OFF_C);
        motor_temp_c_.store(static_cast<int>(fr.data[1]) + TEMP_OFF_C);

        // Byte3,4,5: ERROR groups
        uint32_t g = 0;
        g |= (static_cast<uint32_t>(fr.data[3])      ); // group0: Over/Under/Overheat/Stall...
        g |= (static_cast<uint32_t>(fr.data[4]) << 8 ); // group1: sensors/contactor...
        g |= (static_cast<uint32_t>(fr.data[5]) << 16); // group2: power valve/current sensor/etc.
        err_group_.store(g);
      }
    }
  }

  void send_handshake() {
    can_frame tx{};
    tx.can_id  = ID_VCU_CMD | CAN_EFF_FLAG;
    tx.can_dlc = 8;
    std::fill(std::begin(tx.data), std::end(tx.data), 0xAA);
    (void)write(sock_, &tx, sizeof(tx));
  }

  // ===== 50ms tick: compute derate & send command =====
  void tick_50ms() {
    if (!handshake_done_) return;

    // 1) Clamp target rpm and ramp
    int32_t tgt = target_rpm_.load();
    int fwd_max = clamp(rpm_fwd_max_, 0, RPM_MAX);
    int rev_max = clamp(rpm_rev_max_, 0, RPM_MAX);

    if (tgt >= 0) tgt = clamp<int32_t>(tgt, 0, fwd_max);
    else          tgt = -clamp<int32_t>(-tgt, 0, rev_max);

    vref_now_ = ramp_to(static_cast<int16_t>(tgt), vref_now_, static_cast<int16_t>(ramp_step_rpm_));

    // 2) Derate calculation
    double f_temp = std::min(
      lin_derate(ctrl_temp_c_.load(),  ctrl_temp_derate_start_,  ctrl_temp_derate_cut_),
      lin_derate(motor_temp_c_.load(), motor_temp_derate_start_, motor_temp_derate_cut_)
    );

    double f_regen_vdc = lin_derate(vdc_v_.load(), vdc_regen_derate_start_, vdc_regen_derate_cut_);

    // 에러가 발생(예: 과전압/과열 등)하면 즉시 0으로
    bool hard_fault = (err_group_.load() != 0);
    if (hard_fault) f_temp = 0.0;

    // 3) 실제 속도 기반으로 회생/가속 판단 → 한도 선택
    int spd     = speed_rpm_.load();
    bool same_sign = (spd != 0 && vref_now_ != 0) ? ((spd > 0) == (vref_now_ > 0)) : false;
    bool braking    = same_sign && (std::abs(spd) > std::abs(vref_now_) + brake_hyst_rpm_);

    double mot_lim = motoring_limit_a_ * clamp(f_temp, 0.0, 1.0);
    double reg_lim = regen_limit_a_    * clamp(std::min(f_temp, f_regen_vdc), 0.0, 1.0);

    double i_max = braking ? reg_lim : mot_lim;

    // 4) Phase current limit sign (스펙: 주행 시 속도와 동일 부호 유지)
    double phase_limit_a = 0.0;
    if (vref_now_ > 0)      phase_limit_a = +i_max;
    else if (vref_now_ < 0) phase_limit_a = -i_max; // 후진 가속/제동도 대칭
    else                    phase_limit_a = 0.0;    // 정지목표는 0

    // 5) Offset-binary encode (LE)
    //   current_raw = (A - (-3200)) / 0.1
    //   speed_raw   = (rpm - (-32000))
    auto enc_current = [](double amp_a)->uint16_t{
      const double raw = (amp_a - IA_MIN_A) / 0.1; // IA_MIN_A = -3200
      return static_cast<uint16_t>(std::clamp<long>(lround(raw), 0, 65535));
    };
    auto enc_speed = [](int16_t rpm)->uint16_t{
      const long raw = static_cast<long>(rpm) - (-32000);
      return static_cast<uint16_t>(std::clamp<long>(raw, 0, 65535));
    };

    const uint16_t current_raw = enc_current(phase_limit_a);
    const uint16_t speed_raw   = enc_speed(vref_now_);

    // 6) Compose frame
    can_frame tx{};
    tx.can_id  = ID_VCU_CMD | CAN_EFF_FLAG;
    tx.can_dlc = 8;

    tx.data[0] = static_cast<uint8_t>(current_raw & 0xFF);
    tx.data[1] = static_cast<uint8_t>((current_raw >> 8) & 0xFF);
    tx.data[2] = static_cast<uint8_t>(speed_raw & 0xFF);
    tx.data[3] = static_cast<uint8_t>((speed_raw >> 8) & 0xFF);

    uint8_t ctrl = 0;
    ctrl |= CTRL_RUN;        // RUNNING
    ctrl |= CTRL_SPEED;      // SPEED (표시/가독용; setpoint 조합이 실질)
    tx.data[4] = ctrl;

    tx.data[5] = 0x00;       // reserved
    tx.data[6] = 0x00;       // reserved
    tx.data[7] = life_++;

    (void)write(sock_, &tx, sizeof(tx));

    // Optional logging (변화시)
    static int32_t prev_tgt = INT32_MIN;
    if (tgt != prev_tgt) {
      prev_tgt = tgt;
      RCLCPP_INFO(get_logger(),
        "TX rpm=%d  iMax=%.0fA(%s)  vdc=%.1fV  Tctrl=%.0fC  Tmot=%.0fC  err=0x%06X  ibus=%.1fA iph=%.1fA",
        vref_now_, i_max, braking ? "regen" : "mot",
        vdc_v_.load(), ctrl_temp_c_.load(), motor_temp_c_.load(),
        err_group_.load(), bus_current_a_.load(), phase_current_a_.load());
    }
  }
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  std::string iface = (argc > 1) ? argv[1] : "can0";
  try {
    rclcpp::spin(std::make_shared<CanSpeedSender>(iface));
  } catch (const std::exception& e) {
    RCLCPP_FATAL(rclcpp::get_logger("rclcpp"), "Exception: %s", e.what());
  }
  rclcpp::shutdown();
  return 0;
}
