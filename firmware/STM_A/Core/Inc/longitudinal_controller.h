#ifndef LONGITUDINAL_CONTROLLER_H
#define LONGITUDINAL_CONTROLLER_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

/* ABI v5 appends regenerative braking and the waveform-specific collapse
 * threshold to v4. Regeneration is off unless regen_max_current > 0. v4 added
 * drive protection and feedforward; slot 12 (v3 coast_current_floor) is the
 * rolling minimum drive current. v1..v4 requests remain valid and take the v5
 * defaults for the appended fields. */
#define LONG_VERSION 5u
#define LONG_CONFIG_COUNT 37u
#define LONG_V4_CONFIG_COUNT 24u
#define LONG_V3_CONFIG_COUNT 13u
#define LONG_REQUEST_COUNT (3u + LONG_CONFIG_COUNT)
#define LONG_STATE_COUNT 36u
#define LONG_STATUS_COUNT (5u + LONG_CONFIG_COUNT + LONG_STATE_COUNT)
#define LONG_ID_MAX 16777215u
/* MANUAL pedal full scale outside a Torque/External calibration. */
#define LONG_MANUAL_CURRENT_LIMIT_A 480.0f
/* Validation cap for the rolling minimum current: a floor is a dead-zone
 * workaround, never a traction demand. */
#define LONG_MIN_CURRENT_CAP_A 30.0f
/* Regeneration ceilings: 14S pack (58.8 V full), 39 A continuous charge. */
#define LONG_REGEN_CURRENT_CAP_A 60.0f
#define LONG_REGEN_VOLTAGE_CAP_V 60.0f
#define LONG_REGEN_BUS_CURRENT_CAP_A 39.0f
/* Bus guard: each frame above the charge limit cuts the regen limit by this
 * fraction; it recovers at regen_slew. */
#define LONG_REGEN_BUS_CUT_PER_FRAME 0.2f

/* Collapse detector. Bounds come from the 2026-09-29/30 airborne square-wave
 * MCAPs (49 collapses vs 33,901 normal RUN frames) and the 2026-10-01 sine
 * runs. Below 400 rpm Part I reports benign 0-dropouts. A sine 350 A airborne
 * launch moved up to 600 rpm per 50 ms frame (about 1.7 rpm per amp), so the
 * RPM allowance grows with the commanded current. Currents compare in
 * magnitude against the latest or previous command (the frame may lag one);
 * the negative-phase threshold is configured per waveform (collapse_phase_low). */
#define LONG_COLLAPSE_RPM_MIN 400.0f
#define LONG_COLLAPSE_RPM_STEP 200.0f
#define LONG_COLLAPSE_RPM_PER_A 3.0f
#define LONG_COLLAPSE_PHASE_MARGIN_A 60.0f
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
  /* Regenerative braking (v5). regen_max_current 0 disables every regen path.
   * The limit fades to 0 from regen_fade_speed down to regen_cutoff_speed and
   * from regen_voltage_soft up to regen_voltage_hard, and shrinks while the
   * measured charge current exceeds regen_bus_current_limit. */
  float regen_max_current, regen_slew, regen_fade_speed, regen_cutoff_speed;
  float regen_voltage_soft, regen_voltage_hard, regen_bus_current_limit;
  /* RPM field sent with regen: 0 (as before) or rpm_limit as a positive ceiling. */
  uint8_t regen_rpm_field;
  /* Speed PI may request regen: enter below -regen_enter_error once the drive
   * output is 0, leave above -regen_exit_error, P gain on the overspeed beyond
   * the exit band. */
  bool pi_regen;
  float regen_enter_error, regen_exit_error, regen_kp;
  /* Phase current (A, negative) below which a non-regen torque frame is a
   * collapse. The host sends the value for the declared waveform. */
  float collapse_phase_low;
} LongConfig;

typedef struct {
  float error, p, i, ff, unsaturated, current;
  bool saturated, anti_windup, coasting, reapplying;
  float target_previous;
  bool have_target;
  bool regen_mode;   /* speed PI holds a regen request (pi_regen) */
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

/* Collapse detector input for one Part I frame. command/previous_command are
 * the latest and the previous transmitted phase currents. */
typedef struct {
  float command, previous_command, phase, bus, phase_low;
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

/* Regenerative braking for one AUTO current-path frame (Torque or external). */
typedef struct {
  float target, speed;             /* m/s: commanded target, measured forward speed */
  float bus_voltage, bus_current;  /* Part I; negative bus current is charging */
  float external_request;          /* LT/external request magnitude, A >= 0; 0 when stale */
  bool part1_fresh;                /* bus voltage/current known */
  bool allowed;                    /* AUTO current path, OK inhibit, fresh feedback, no fault, EZ torque mode */
  bool pi;                         /* AUTO Torque speed PI path: pi_regen may add a request */
} LongRegenInput;
typedef struct {
  float request, limit, speed_factor, voltage_factor, bus_cut, current;
  bool active;
} LongRegen;
/* Speed-PI regen request (A >= 0) with its enter/exit hysteresis. */
float Long_PiRegen(LongController *controller, const LongConfig *config,
                   float target, float actual);
/* Returns the regen current (<= 0). The limit (speed fade/cutoff, voltage,
 * bus guard, stale Part I) applies at once; the request ramps at regen_slew,
 * also back to 0 when allowed drops. While regen->active the caller sends the
 * regen current instead of the drive PI; that includes the frame that returns
 * to 0 A. The drive output then restarts from 0 A. */
float Long_RegenFrame(LongRegen *regen, LongController *controller,
                      const LongConfig *config, const LongRegenInput *input, float dt);
/* Safety stops, MANUAL and Speed mode: no regen, no ramp. */
void Long_RegenReset(LongRegen *regen);

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
