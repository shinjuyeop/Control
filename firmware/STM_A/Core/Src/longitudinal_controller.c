#include "longitudinal_controller.h"
#include <math.h>
#include <string.h>

static float clamp(float x, float lo, float hi) { return fminf(hi, fmaxf(lo, x)); }
static bool whole(float x, float lo, float hi) { return x >= lo && x <= hi && floorf(x) == x; }

LongConfig Long_DefaultConfig(void)
{
  /* Boot in Torque on a Sine (FOC) controller: no rolling floor and no
   * feedforward. Square-wave workarounds arrive with a host preset. */
  return (LongConfig){
    .mode = LONG_TORQUE, .kp = 180.0f, .ki = 12.0f, .max_current = 480.0f,
    .integrator_limit = 120.0f, .slew_up = 2400.0f, .slew_down = 4800.0f,
    .deadband = 0.05f, .hysteresis = 0.12f, .reapply_current = 20.0f,
    .reapply_slew = 200.0f, .rpm_limit = 4500.0f, .min_current = 0.0f,
    .waveform = LONG_SINE, .start_step = true,
    .hold_on_speed = 0.35f, .hold_off_speed = 0.20f,
    .reverse_speed = 0.30f, .reverse_frames = 3u, .collapse_hold_frames = 0u};
}

LongMotorCommand Long_SpeedCommand(const LongConfig *c, int32_t target_rpm)
{
  return (LongMotorCommand){
    (int32_t)clamp((float)target_rpm, -c->rpm_limit, c->rpm_limit), c->max_current};
}

bool Long_DecodeRequest(const float *d, size_t count, LongRequest *r)
{
  if (count < 3u) return false;
  for (size_t n = 0; n < count; ++n) if (!isfinite(d[n])) return false;
  const size_t fields = count - 3u;
  const bool v4 = d[0] == LONG_VERSION && fields == LONG_CONFIG_COUNT;
  const bool v3 = d[0] == 3 && fields == LONG_V3_CONFIG_COUNT;
  const bool v12 = (d[0] == 1 || d[0] == 2) && fields == LONG_V3_CONFIG_COUNT - 1u;
  if (!(v4 || v3 || v12) || !whole(d[1], 1, LONG_ID_MAX) || !whole(d[2], 0, LONG_ID_MAX) ||
      (d[3] != LONG_SPEED && d[3] != LONG_TORQUE &&
       !(d[0] >= 2 && d[3] == LONG_EXTERNAL_CURRENT))) return false;
  /* Older requests keep the v4 defaults for every appended field. */
  LongConfig c = Long_DefaultConfig();
  c.mode = (uint8_t)d[3]; c.kp = d[4]; c.ki = d[5]; c.max_current = d[6];
  c.integrator_limit = d[7]; c.slew_up = d[8]; c.slew_down = d[9];
  c.deadband = d[10]; c.hysteresis = d[11]; c.reapply_current = d[12];
  c.reapply_slew = d[13]; c.rpm_limit = d[14];
  c.min_current = v12 ? 0.0f : d[15];
  if (v4) {
    if (!whole(d[16], 0, 1) || !whole(d[17], 0, 1) ||
        !whole(d[21], 1, 10) || !whole(d[22], 0, 40)) return false;
    c.waveform = (uint8_t)d[16]; c.start_step = d[17] != 0;
    c.hold_on_speed = d[18]; c.hold_off_speed = d[19]; c.reverse_speed = d[20];
    c.reverse_frames = (uint8_t)d[21]; c.collapse_hold_frames = (uint8_t)d[22];
    c.ff_gain = d[23]; c.ff_rolling = d[24]; c.ff_damping = d[25]; c.ff_accel_limit = d[26];
  }
  /* Bounds protect current/RPM capability and 20 Hz numerical authority, not a
   * claim of vehicle-validated tuning. Integral/slew must fit the output range. */
  if (c.max_current <= 0 || c.max_current > 480 ||
      c.rpm_limit < 100 || c.rpm_limit > 4500) return false;
  if (c.mode == LONG_TORQUE &&
      (c.kp <= 0 || c.kp > 1000 || c.ki < 0 || c.ki > 200 || c.integrator_limit < 0 ||
      c.integrator_limit > c.max_current || c.slew_up <= 0 || c.slew_up > 9600 ||
      c.slew_down <= 0 || c.slew_down > 9600 || c.deadband < 0 || c.deadband > .2f ||
      c.hysteresis <= c.deadband || c.hysteresis > .5f || c.reapply_current < 0 ||
      c.reapply_current > c.max_current || c.reapply_slew <= 0 ||
      c.reapply_slew > c.slew_up)) return false;
  /* The rolling floor and the reverse debounce also act in MANUAL, whatever the
   * calibration mode. The hold needs a real hysteresis band. */
  if (c.min_current < 0 || c.min_current > fminf(c.max_current, LONG_MIN_CURRENT_CAP_A) ||
      c.hold_on_speed < .1f || c.hold_on_speed > 2 || c.hold_off_speed < 0 ||
      c.hold_off_speed >= c.hold_on_speed || c.reverse_speed < .05f ||
      c.reverse_speed > 1) return false;
  /* Feedforward is optional: gain 0 disables it. A nonzero gain stays inside the
   * identifiable range so a near-zero divisor cannot turn into a huge command. */
  if (!(c.ff_gain == 0 || (c.ff_gain >= .0005f && c.ff_gain <= .2f)) ||
      c.ff_rolling < 0 || c.ff_rolling > 2 || c.ff_damping < 0 || c.ff_damping > 1 ||
      c.ff_accel_limit < 0 || c.ff_accel_limit > 5) return false;
  *r = (LongRequest){(uint32_t)d[1], (uint32_t)d[2], c};
  return true;
}

void Long_EncodeConfig(const LongConfig *c, float *d)
{
  const float values[LONG_CONFIG_COUNT] = {c->mode, c->kp, c->ki, c->max_current,
    c->integrator_limit, c->slew_up, c->slew_down, c->deadband, c->hysteresis,
    c->reapply_current, c->reapply_slew, c->rpm_limit, c->min_current, c->waveform,
    c->start_step, c->hold_on_speed, c->hold_off_speed, c->reverse_speed,
    c->reverse_frames, c->collapse_hold_frames, c->ff_gain, c->ff_rolling,
    c->ff_damping, c->ff_accel_limit};
  memcpy(d, values, sizeof(values));
}

bool Long_CanConfigure(bool manual, bool feedback_valid, float speed,
                       uint8_t pedal, float commanded_current, float actual_current)
{
  /* MANUAL sends RUN even at pedal=0: RUN alone is not traction demand. */
  return manual && feedback_valid && isfinite(speed) && fabsf(speed) <= .05f &&
         pedal == 0 && isfinite(commanded_current) && fabsf(commanded_current) < .1f &&
         isfinite(actual_current) && fabsf(actual_current) <= 1.0f;
}

void Long_Reset(LongController *s) { memset(s, 0, sizeof(*s)); }

void Long_ClearExternal(LongExternalInput *s, int64_t floor)
{
  s->current = 0;
  s->received = false;
  if (floor > s->last_stamp_ns) s->last_stamp_ns = floor;
}

bool Long_ReceiveExternal(LongExternalInput *s, float current, int64_t stamp, uint32_t tick)
{
  if (!isfinite(current) || current < 0 || current > 480 ||
      stamp <= s->last_stamp_ns) return false;
  *s = (LongExternalInput){current, tick, stamp, true};
  return true;
}

bool Long_ReceiveRegen(LongExternalInput *s, float current, int64_t stamp, uint32_t tick)
{
  if (!isfinite(current) || current > 0 || current < -480 ||
      stamp <= s->last_stamp_ns) return false;
  *s = (LongExternalInput){current, tick, stamp, true};
  return true;
}

bool Long_ExternalFresh(const LongExternalInput *s, uint32_t tick, uint32_t timeout)
{
  return s->received && (uint32_t)(tick - s->received_tick) < timeout;
}

/* Invert the identified plant for the steady-state and target-acceleration
 * current, so the integrator only carries load, grade, surface and SOC error.
 * The acceleration term comes from the commanded target rate and is bounded, so
 * an upstream target step cannot become an unbounded current spike. */
static float feedforward(LongController *s, const LongConfig *c,
                         float target, float actual, float dt)
{
  float accel = 0;
  if (s->have_target && dt > 0) {
    accel = clamp((target - s->target_previous) / dt,
                  -c->ff_accel_limit, c->ff_accel_limit);
  }
  s->target_previous = target;
  s->have_target = true;
  if (c->ff_gain <= 0) return 0;
  return clamp((accel + c->ff_rolling + c->ff_damping * actual) / c->ff_gain,
               0, c->max_current);
}

/* Lowest current a falling output may take this frame. Inside the hold it is
 * the floor or the downward slew. Outside it, when a minimum current is
 * configured, a fall that would enter (0, min_current) is allowed straight to
 * the requested value: the square-wave EZ is stable when cut fast and unstable
 * when it crawls through its dead zone at speed. */
static float lowest(const LongConfig *c, float floor, float previous, float dt)
{
  float lower = fmaxf(floor, previous - c->slew_down * dt);
  if (floor <= 0 && c->min_current > 0 && lower < c->min_current) lower = 0;
  return fmaxf(0, lower);
}

float Long_Update(LongController *s, const LongConfig *c, float target, float actual,
                  float dt, float floor)
{
  /* Inputs/config were validated at the external boundary. Caller handles
   * stale feedback, mode, direction and stop gates before invoking this loop. */
  if (target <= 0) { Long_Reset(s); return 0; }
  floor = clamp(floor, 0, c->max_current);
  /* A held output restarts from the floor, never from 0 through the reapply band. */
  if (floor > 0) s->current = fmaxf(s->current, floor);
  s->error = target - actual;
  s->p = c->kp * s->error;
  s->ff = feedforward(s, c, target, actual, dt);
  s->anti_windup = false;
  const float lower = lowest(c, floor, s->current, dt);
  float error_i = fabsf(s->error) <= c->deadband ? 0 : s->error;
  float candidate = clamp(s->i + c->ki * error_i * dt, 0, c->integrator_limit);
  float proposed = s->ff + s->p + candidate;
  float up = s->reapplying && s->current < c->reapply_current ? c->reapply_slew : c->slew_up;
  /* Keep the learned positive load current through a downward speed step.
   * Integrate negative error only while the output can decrease: at the floor
   * or the downward slew limit, freezing I prevents an artificial current debt.
   * P + I can then become positive before speed falls below the target. */
  if ((error_i > 0 && (proposed > c->max_current || proposed > s->current + up * dt)) ||
      (error_i < 0 && proposed < lower))
    s->anti_windup = true;
  else s->i = candidate;
  s->unsaturated = s->ff + s->p + s->i;
  float desired = clamp(s->unsaturated, floor, c->max_current);
  /* Coast is now an output state, not a latch that overrides PI. */
  s->coasting = s->unsaturated <= floor && s->error < 0;
  if (s->coasting) s->reapplying = true;
  s->saturated = s->unsaturated < floor || s->unsaturated > c->max_current;
  up = s->reapplying && s->current < c->reapply_current ? c->reapply_slew : c->slew_up;
  s->current = clamp(desired, lower, s->current + up * dt);
  if (s->current >= c->reapply_current) s->reapplying = false;
  return s->current;
}

bool Long_UpdateHold(LongDrive *d, const LongConfig *c, bool eligible, float speed)
{
  if (!eligible || !(c->min_current > 0) || !isfinite(speed)) d->hold = false;
  else if (speed >= c->hold_on_speed) d->hold = true;
  else if (speed <= c->hold_off_speed) d->hold = false;
  return d->hold;
}

float Long_HoldFloor(const LongDrive *d, const LongConfig *c)
{
  return d->hold ? c->min_current : 0.0f;
}

float Long_DriveFloor(LongDrive *d, const LongConfig *c, float current)
{
  d->floor_applied = d->start_stepped = false;
  if (!(c->min_current > 0)) return current;
  if (d->hold) {
    /* Binding floor: the path asked for no more than the minimum current. The
     * Torque PI already returns the floor itself, so this flags it too. */
    d->floor_applied = !(current > c->min_current);
    return fmaxf(current, c->min_current);
  }
  /* Standing start: step over the dead zone instead of ramping through it. */
  if (c->start_step && current > 0 && current < c->min_current) {
    d->start_stepped = true;
    return c->min_current;
  }
  return current;
}

float Long_PedalCommand(const LongConfig *c, float pedal_percent, bool envelope,
                        bool hold, float previous_current, float dt)
{
  const float limit = envelope ? c->max_current : LONG_MANUAL_CURRENT_LIMIT_A;
  const float floor = hold && c->min_current > 0 ? fminf(c->min_current, limit) : 0;
  const float pedal = clamp(isfinite(pedal_percent) ? pedal_percent : 0, 0, 100) / 100.0f;
  /* 0 % keeps the floor while rolling; 1 % already adds torque above it. */
  const float demand = floor + pedal * (limit - floor);
  if (!envelope) return demand;
  float previous = clamp(isfinite(previous_current) ? previous_current : 0, 0, limit);
  if (floor > 0) previous = fmaxf(previous, floor);
  return clamp(demand, lowest(c, floor, previous, dt), previous + c->slew_up * dt);
}

uint8_t Long_CollapseFlags(const LongCollapseInput *in)
{
  uint8_t flags = 0;
  if (!in->running) return 0;
  const float previous = (float)in->previous_rpm, rpm = (float)in->rpm;
  /* A reported speed that collapses to zero, flips sign or jumps beyond any
   * physical acceleration. Low-speed 0-dropouts are benign Part I artefacts. */
  if (fabsf(previous) >= LONG_COLLAPSE_RPM_MIN &&
      (rpm * previous <= 0 || fabsf(rpm - previous) > LONG_COLLAPSE_RPM_STEP))
    flags |= LONG_COLLAPSE_RPM;
  /* Current rules need a known forward torque demand; regen and reverse
   * sign conventions are not characterised, so only the RPM rule applies. */
  if (in->torque_command && in->command >= 0) {
    if (in->phase > in->command + LONG_COLLAPSE_PHASE_MARGIN_A) flags |= LONG_COLLAPSE_PHASE_HIGH;
    if (in->phase < -LONG_COLLAPSE_PHASE_NEGATIVE_A) flags |= LONG_COLLAPSE_PHASE_LOW;
    if (in->bus > in->command + LONG_COLLAPSE_BUS_MARGIN_A) flags |= LONG_COLLAPSE_BUS;
  }
  return flags;
}

bool Long_CollapseUpdate(LongCollapse *s, const LongCollapseInput *in, uint8_t path, bool hold)
{
  const uint8_t flags = Long_CollapseFlags(in);
  if (!flags) {
    if (s->quiet < UINT8_MAX) s->quiet++;
    return false;
  }
  const bool event = s->events == 0u || s->quiet >= LONG_COLLAPSE_REARM_FRAMES;
  s->quiet = 0u;
  if (!event) {
    s->flags |= flags;
    return false;
  }
  s->events++;
  s->flags = flags;
  s->path = path;
  s->hold = hold;
  s->snapshot = *in;
  return true;
}

/* Caller serializes with the producer/reader critical section. All fields commit
 * together; expected revision makes concurrent PC writers conflict, not merge. */
void Long_ApplyConfiguration(LongCalibration *s, const LongRequest *r, bool stopped)
{
  if (s->request_id == r->id && s->result == LONG_APPLIED) return;
  s->request_id = r->id;
  if (r->expected_revision != s->revision || s->revision == LONG_ID_MAX) {
    s->result = LONG_CONFLICT;
  } else if (!stopped) {
    s->result = LONG_UNSAFE;
  } else {
    *s = (LongCalibration){r->config, s->revision + 1u, r->id, LONG_APPLIED, LONG_HOST_OVERRIDE};
  }
}


uint8_t Long_CheckInhibit(LongController *s, LongGate *gate, const LongConfig *c,
                         bool torque_auto, bool feedback_valid, bool command_fresh,
                         bool stop, float target, float actual, bool *discard_command)
{
  uint8_t reason = LONG_OK;
  bool counted = false;
  *discard_command = false;
  if (!torque_auto) {
    reason = LONG_DISABLED;
    gate->feedback_blocked = false;
  } else if (!feedback_valid) {
    reason = LONG_FEEDBACK;
    gate->feedback_blocked = true;
  } else if (gate->feedback_blocked) {
    gate->feedback_blocked = false;
    *discard_command = true;
    reason = LONG_WAIT_COMMAND;
  } else if (stop) reason = LONG_STOP;
  else if (!command_fresh) reason = LONG_WAIT_COMMAND;
  else if (target < 0) reason = LONG_REVERSE;
  else if (actual < -c->reverse_speed) {
    /* A real reverse roll persists; a despiked sign glitch lasts one frame. */
    counted = true;
    if (gate->reverse_frames < UINT8_MAX) gate->reverse_frames++;
    reason = gate->reverse_frames >= c->reverse_frames ? LONG_REVERSE : LONG_REVERSE_PENDING;
  }
  if (!counted) gate->reverse_frames = 0;
  /* Pending keeps the PI history: the caller sends 0 A with RUN and resumes. */
  if (reason != LONG_OK && reason != LONG_REVERSE_PENDING) Long_Reset(s);
  return reason;
}
