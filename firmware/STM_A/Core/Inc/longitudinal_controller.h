#ifndef LONGITUDINAL_CONTROLLER_H
#define LONGITUDINAL_CONTROLLER_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

/* Float32MultiArray ABI v2; see docs/INTERFACES.md. No generated interface change. */
#define LONG_VERSION 2u
#define LONG_CONFIG_COUNT 16u
#define LONG_REQUEST_COUNT (3u + LONG_CONFIG_COUNT)
#define LONG_STATUS_COUNT (5u + LONG_CONFIG_COUNT + 11u)
#define LONG_ID_MAX 16777215u

enum { LONG_SPEED = 0, LONG_TORQUE = 1 };
enum { LONG_DEFAULT = 0, LONG_HOST_OVERRIDE = 1 };
enum { LONG_IDLE = 0, LONG_APPLIED = 1, LONG_INVALID = 2,
       LONG_UNSAFE = 3, LONG_CONFLICT = 4, LONG_BUSY = 5 };
enum { LONG_OK = 0, LONG_DISABLED = 1, LONG_FEEDBACK = 2,
       LONG_WAIT_COMMAND = 3, LONG_REVERSE = 4, LONG_STOP = 5 };

typedef struct {
  uint8_t mode;
  float kp, ki, max_current, integrator_limit, slew_up, slew_down;
  float deadband, hysteresis, reapply_current, reapply_slew, rpm_limit;
  /* Identified plant a = ff_gain * I - ff_rolling - ff_damping * v, inverted as
   * feedforward. ff_gain 0 disables all feedforward; ff_accel_limit bounds the
   * term taken from the commanded target rate. */
  float ff_gain, ff_rolling, ff_damping, ff_accel_limit;
} LongConfig;

typedef struct {
  float error, p, i, ff, unsaturated, current;
  bool saturated, anti_windup, coasting, reapplying;
  float target_previous;
  bool have_target;
} LongController;

typedef struct {
  uint32_t id, expected_revision;
  LongConfig config;
} LongRequest;

typedef struct {
  LongConfig config;
  uint32_t revision, request_id;
  uint8_t result, source;
} LongCalibration;

void Long_ApplyConfiguration(LongCalibration *state, const LongRequest *request, bool stopped);

/* RAM default is the existing speed path; gains are driving first-test candidates. */
LongConfig Long_DefaultConfig(void);
typedef struct { int32_t rpm; float current; } LongMotorCommand;
/* AUTO Speed uses the same configured current/RPM envelope as AUTO Torque.
 * The CAN sender still applies the separate physical forward/reverse limits. */
LongMotorCommand Long_SpeedCommand(const LongConfig *config, int32_t target_rpm);
/* MANUAL pedal demand through the configured torque envelope and slew rates.
 * Pure function: the pedal path keeps no PI history, so AUTO safety resets
 * never touch it and a MANUAL driver keeps direct torque authority. */
float Long_PedalCommand(const LongConfig *config, float pedal_percent,
                        float previous_current, float dt);
bool Long_DecodeRequest(const float *data, size_t count, LongRequest *request);
void Long_EncodeConfig(const LongConfig *config, float *data);
bool Long_CanConfigure(bool manual, bool feedback_valid, float speed,
                       uint8_t pedal, float commanded_current, float actual_current);
typedef struct { bool feedback_blocked; } LongGate;
uint8_t Long_CheckInhibit(LongController *controller, LongGate *gate,
                         bool torque_auto, bool feedback_valid, bool command_fresh,
                         bool stop, float target, float actual, bool *discard_command);
void Long_Reset(LongController *controller);
float Long_Update(LongController *controller, const LongConfig *config,
                  float target, float actual, float dt);
#endif
