#ifndef LONGITUDINAL_CONTROLLER_H
#define LONGITUDINAL_CONTROLLER_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

/* ABI v3 appends the Torque coast current floor; v1/v2 requests remain valid. */
#define LONG_VERSION 3u
#define LONG_CONFIG_COUNT 13u
#define LONG_REQUEST_COUNT (3u + LONG_CONFIG_COUNT)
#define LONG_STATUS_COUNT (5u + LONG_CONFIG_COUNT + 10u)
#define LONG_ID_MAX 16777215u

enum { LONG_SPEED = 0, LONG_TORQUE = 1, LONG_EXTERNAL_CURRENT = 2 };
enum { LONG_DEFAULT = 0, LONG_HOST_OVERRIDE = 1 };
enum { LONG_IDLE = 0, LONG_APPLIED = 1, LONG_INVALID = 2,
       LONG_UNSAFE = 3, LONG_CONFLICT = 4, LONG_BUSY = 5 };
enum { LONG_OK = 0, LONG_DISABLED = 1, LONG_FEEDBACK = 2,
       LONG_WAIT_COMMAND = 3, LONG_REVERSE = 4, LONG_STOP = 5 };

typedef struct {
  uint8_t mode;
  float kp, ki, max_current, integrator_limit, slew_up, slew_down;
  /* hysteresis is retained for ABI compatibility; continuous PI does not use it. */
  float deadband, hysteresis, reapply_current, reapply_slew, rpm_limit;
  float coast_current_floor;
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

/* RAM boot default is Torque; native Speed is selectable at a safe stop. */
LongConfig Long_DefaultConfig(void);
typedef struct { int32_t rpm; float current; } LongMotorCommand;
/* AUTO Speed uses the same configured current/RPM envelope as AUTO Torque.
 * The CAN sender still applies the separate physical forward/reverse limits. */
LongMotorCommand Long_SpeedCommand(const LongConfig *config, int32_t target_rpm);
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

/* Shared by the MCU receiver and SITL. Receipt time owns the watchdog;
 * source stamps reject queued/reordered commands across a stop or mode change. */
typedef struct {
  float current;
  uint32_t received_tick;
  int64_t last_stamp_ns;
  bool received;
} LongExternalInput;
void Long_ClearExternal(LongExternalInput *input, int64_t stamp_floor_ns);
bool Long_ReceiveExternal(LongExternalInput *input, float current,
                          int64_t stamp_ns, uint32_t tick);
bool Long_ExternalFresh(const LongExternalInput *input, uint32_t tick,
                       uint32_t timeout_ms);
/* AUTO Torque regen request: nonpositive phase current, sent with target RPM 0. */
bool Long_ReceiveRegen(LongExternalInput *input, float current,
                       int64_t stamp_ns, uint32_t tick);
#endif
