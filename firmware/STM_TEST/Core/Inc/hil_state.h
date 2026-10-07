#ifndef HIL_STATE_H
#define HIL_STATE_H

#include <stdbool.h>
#include <stdint.h>

typedef enum {
  HIL_BRAKE_UNKNOWN = 0,
  HIL_BRAKE_RELEASED,
  HIL_BRAKE_ENGAGED
} HilBrakeState_t;

typedef struct {
  bool auto_mode;
  bool reverse;

  bool estop_dashboard;
  bool estop_side;
  bool estop_rf;

  float pedal_percent;

  float vehicle_speed_kph;
  float motor_rpm;
  float motor_voltage_v;
  float motor_current_a;
  float motor_temperature_c;

  /* KEYA motor-shaft position/target in degrees (10000 count/rev). */
  float steering_position;
  float steering_target;

  uint32_t brake_period_us;
  uint32_t brake_pulse_us;
  bool brake_engaged;
  bool brake_pwm_valid;
  bool brake_pwm_lost;
  HilBrakeState_t brake_state;

  bool ez_can_enabled;
  bool keya_can_enabled;

  bool ez_drop_tx;
  bool keya_drop_tx;

  bool ez_fault;
  bool keya_fault;

  bool ez_can_healthy;
  bool keya_can_healthy;
  bool ez_running;
  bool ez_speed_control;

  /* FDCAN protocol status per bus. ez_can_healthy/keya_can_healthy only report
   * peripheral init and RX overflow, so they stay 1 while the controller sits
   * in error-passive or bus-off. These carry the actual bus error state and
   * are the fields to judge a stalled TX by. */
  bool ez_bus_off;
  bool ez_error_passive;
  bool ez_warning;
  uint8_t ez_tec;
  uint8_t ez_rec;
  bool keya_bus_off;
  bool keya_error_passive;
  bool keya_warning;
  uint8_t keya_tec;
  uint8_t keya_rec;

  /* Last controller commands observed on the two CAN inputs. RX age uses
   * UINT32_MAX until a valid frame has been received. */
  float ez_command_rpm;
  float ez_command_current_a;
  uint32_t ez_rx_age_ms;
  bool keya_motor_enabled;
  uint32_t keya_rx_age_ms;

  /* Closed-loop plant: derive motor_rpm from the received STM_A command
   * instead of holding whatever SET SPEED/SET RPM last injected. */
  bool ez_plant_enabled;
  float ez_target_rpm;

  /* Optional square-wave defect model (SET EZ_SQUARE). Part I only: the
   * plant RPM is untouched, the reported current/RPM are distorted. */
  bool ez_square_fault;

  uint32_t ez_rx_count;
  uint32_t ez_tx_count;
  uint32_t keya_rx_count;
  uint32_t keya_tx_count;
} HilState_t;

void HilState_Init(void);
HilState_t *HilState_Lock(uint32_t timeout_ticks);
void HilState_Unlock(void);
bool HilState_GetSnapshot(HilState_t *snapshot, uint32_t timeout_ticks);

#endif /* HIL_STATE_H */
