#ifndef LONGITUDINAL_CONTROLLER_H
#define LONGITUDINAL_CONTROLLER_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

/* ABI v4 appends drive-protection and feedforward fields. Slot 12 (v3
 * coast_current_floor) is now the rolling minimum drive current applied to the
 * final forward command of every drive path. v1/v2/v3 requests remain valid and
 * take the v4 defaults for the appended fields. */
#define LONG_VERSION 4u
#define LONG_CONFIG_COUNT 24u
#define LONG_V3_CONFIG_COUNT 13u
#define LONG_REQUEST_COUNT (3u + LONG_CONFIG_COUNT)
#define LONG_STATE_COUNT 28u
#define LONG_STATUS_COUNT (5u + LONG_CONFIG_COUNT + LONG_STATE_COUNT)
#define LONG_ID_MAX 16777215u
/* MANUAL pedal full scale outside a Torque/External calibration. */
#define LONG_MANUAL_CURRENT_LIMIT_A 480.0f
/* Validation cap for the rolling minimum current: a floor is a dead-zone
 * workaround, never a traction demand. */
#define LONG_MIN_CURRENT_CAP_A 30.0f

/* Collapse detector (P0-6). Bounds come from the 2026-09-29/30 airborne
 * square-wave MCAPs: 49 collapses vs 33,901 normal RUN frames. Below 400 rpm
 * Part I reports benign 0-dropouts; an 80 A airborne launch changes at most
 * 147 rpm per 50 ms frame. Normal |phase-command| stayed below 33 A and the
 * bus current never exceeded the command by more than 21 A. */
#define LONG_COLLAPSE_RPM_MIN 400.0f
#define LONG_COLLAPSE_RPM_STEP 200.0f
#define LONG_COLLAPSE_PHASE_MARGIN_A 60.0f
#define LONG_COLLAPSE_PHASE_NEGATIVE_A 30.0f
#define LONG_COLLAPSE_BUS_MARGIN_A 60.0f
/* Clean Part I frames (50 ms) before the next anomaly counts as a new event. */
#define LONG_COLLAPSE_REARM_FRAMES 20u

enum { LONG_SPEED = 0, LONG_TORQUE = 1, LONG_EXTERNAL_CURRENT = 2 };
/* Operator-declared EZkontrol commutation. STM_A only echoes it; the host
 * resolves waveform-specific values before sending a request. */
enum { LONG_SQUARE = 0, LONG_SINE = 1 };
enum { LONG_DEFAULT = 0, LONG_HOST_OVERRIDE = 1 };
enum { LONG_IDLE = 0, LONG_APPLIED = 1, LONG_INVALID = 2,
       LONG_UNSAFE = 3, LONG_CONFLICT = 4, LONG_BUSY = 5 };
/* LONG_REVERSE_PENDING keeps RUN at 0 A while a negative speed is debounced. */
enum { LONG_OK = 0, LONG_DISABLED = 1, LONG_FEEDBACK = 2,
       LONG_WAIT_COMMAND = 3, LONG_REVERSE = 4, LONG_STOP = 5,
       LONG_REVERSE_PENDING = 6 };
/* Motor command source, published with every status and collapse snapshot. */
enum { LONG_PATH_HALT = 0, LONG_PATH_MANUAL = 1, LONG_PATH_SPEED = 2,
       LONG_PATH_TORQUE = 3, LONG_PATH_EXTERNAL = 4, LONG_PATH_REGEN = 5,
       LONG_PATH_ZERO = 6 };
enum { LONG_COLLAPSE_RPM = 1u, LONG_COLLAPSE_PHASE_HIGH = 2u,
       LONG_COLLAPSE_PHASE_LOW = 4u, LONG_COLLAPSE_BUS = 8u };

typedef struct {
  uint8_t mode;
  float kp, ki, max_current, integrator_limit, slew_up, slew_down;
  /* hysteresis is retained for ABI compatibility; continuous PI does not use it. */
  float deadband, hysteresis, reapply_current, reapply_slew, rpm_limit;
  /* Minimum forward current while rolling; 0 disables the rolling hold, the
   * start step and the dead-zone guard (Sine/FOC default). */
  float min_current;
  uint8_t waveform;
  /* Below the hold, a positive demand smaller than min_current steps to it. */
  bool start_step;
  float hold_on_speed, hold_off_speed;
  /* Measured reverse speed must persist for reverse_frames before HALTED. */
  float reverse_speed;
  uint8_t reverse_frames;
  /* AUTO frames that hold the floor and freeze PI after a collapse; 0 observes. */
  uint8_t collapse_hold_frames;
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

/* RAM boot default is Torque on a Sine (FOC) controller: no floor, no FF. */
LongConfig Long_DefaultConfig(void);
typedef struct { int32_t rpm; float current; } LongMotorCommand;
/* AUTO Speed uses the same configured current/RPM envelope as AUTO Torque.
 * The CAN sender still applies the separate physical forward/reverse limits. */
LongMotorCommand Long_SpeedCommand(const LongConfig *config, int32_t target_rpm);
bool Long_DecodeRequest(const float *data, size_t count, LongRequest *request);
void Long_EncodeConfig(const LongConfig *config, float *data);
bool Long_CanConfigure(bool manual, bool feedback_valid, float speed,
                       uint8_t pedal, float commanded_current, float actual_current);
typedef struct { bool feedback_blocked; uint8_t reverse_frames; } LongGate;
uint8_t Long_CheckInhibit(LongController *controller, LongGate *gate,
                         const LongConfig *config, bool torque_auto,
                         bool feedback_valid, bool command_fresh, bool stop,
                         float target, float actual, bool *discard_command);
void Long_Reset(LongController *controller);
/* floor > 0 while the rolling hold is active: the slew state is seeded at the
 * floor and the output never falls below it. Pass 0 everywhere else. */
float Long_Update(LongController *controller, const LongConfig *config,
                  float target, float actual, float dt, float floor);

/* Rolling hold and the final forward current, shared by every drive path.
 * floor_applied: the hold floor decided the output (request <= min_current). */
typedef struct {
  bool hold, floor_applied, start_stepped;
} LongDrive;
/* eligible: a forward drive path with valid feedback, not HALTED, regen or reverse. */
bool Long_UpdateHold(LongDrive *drive, const LongConfig *config, bool eligible, float speed);
float Long_HoldFloor(const LongDrive *drive, const LongConfig *config);
/* Applied once to the forward current just before it is sent. */
float Long_DriveFloor(LongDrive *drive, const LongConfig *config, float current);
/* MANUAL pedal through the active envelope. envelope=true shares the Torque
 * calibration current limit and slew rates; false keeps 0..480 A, no slew.
 * While the hold is active the pedal spans [min_current, limit]. Pure: the
 * pedal keeps no PI history, so AUTO resets never touch MANUAL authority. */
float Long_PedalCommand(const LongConfig *config, float pedal_percent, bool envelope,
                        bool hold, float previous_current, float dt);

/* Collapse detector input for one Part I frame. */
typedef struct {
  float command, phase, bus;
  int32_t rpm, previous_rpm;
  bool running, torque_command;
} LongCollapseInput;
typedef struct {
  uint32_t events;
  uint8_t quiet, flags, path;
  bool hold;
  LongCollapseInput snapshot;
} LongCollapse;
uint8_t Long_CollapseFlags(const LongCollapseInput *input);
/* Returns true on the first frame of a new event; path/hold describe the
 * command that was in effect for the snapshot. */
bool Long_CollapseUpdate(LongCollapse *collapse, const LongCollapseInput *input,
                         uint8_t path, bool hold);

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
