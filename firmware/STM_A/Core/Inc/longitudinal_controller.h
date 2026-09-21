#ifndef LONGITUDINAL_CONTROLLER_H
#define LONGITUDINAL_CONTROLLER_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

/* Float32MultiArray ABI v1; see docs/INTERFACES.md. No generated interface change. */
#define LONG_VERSION 1u
#define LONG_CONFIG_COUNT 12u
#define LONG_REQUEST_COUNT (3u + LONG_CONFIG_COUNT)
#define LONG_STATUS_COUNT (5u + LONG_CONFIG_COUNT + 10u)
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
} LongConfig;

typedef struct {
  float error, p, i, unsaturated, current;
  bool saturated, anti_windup, coasting, reapplying;
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
