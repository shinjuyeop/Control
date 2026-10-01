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
    .reverse_speed = 0.30f, .reverse_frames = 3u, .collapse_hold_frames = 0u,
    /* Regeneration stays off (max 0) until a preset enables it. */
    .regen_max_current = 0.0f, .regen_slew = 100.0f, .regen_fade_speed = 1.5f,
    .regen_cutoff_speed = 0.8f, .regen_voltage_soft = 57.0f, .regen_voltage_hard = 58.0f,
    .regen_bus_current_limit = 30.0f, .regen_rpm_field = 0u, .pi_regen = false,
    .regen_enter_error = 0.3f, .regen_exit_error = 0.1f, .regen_kp = 0.0f,
    .collapse_phase_low = -60.0f};
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
  const bool v5 = d[0] == LONG_VERSION && fields == LONG_CONFIG_COUNT;
  /* v4 is true for every request that carries the v4 fields (v4 and v5). */
  const bool v4 = v5 || (d[0] == 4 && fields == LONG_V4_CONFIG_COUNT);
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
  if (v5) {
    if (!whole(d[34], 0, 1) || !whole(d[35], 0, 1)) return false;
    c.regen_max_current = d[27]; c.regen_slew = d[28]; c.regen_fade_speed = d[29];
    c.regen_cutoff_speed = d[30]; c.regen_voltage_soft = d[31]; c.regen_voltage_hard = d[32];
    c.regen_bus_current_limit = d[33]; c.regen_rpm_field = (uint8_t)d[34];
    c.pi_regen = d[35] != 0; c.regen_enter_error = d[36]; c.regen_exit_error = d[37];
    c.regen_kp = d[38]; c.collapse_phase_low = d[39];
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
  /* Regeneration stays within the battery (58.8 V, 39 A charge) and keeps its
   * fade bands non-degenerate. Checked in every mode; only AUTO uses it. */
  if (c.regen_max_current < 0 || c.regen_max_current > LONG_REGEN_CURRENT_CAP_A ||
      c.regen_slew < 10 || c.regen_slew > 1000 ||
      c.regen_fade_speed < .3f || c.regen_fade_speed > 5 ||
      c.regen_cutoff_speed < .1f || c.regen_cutoff_speed >= c.regen_fade_speed ||
      c.regen_voltage_soft < 40 || c.regen_voltage_hard <= c.regen_voltage_soft ||
      c.regen_voltage_hard > LONG_REGEN_VOLTAGE_CAP_V ||
      c.regen_bus_current_limit < 1 || c.regen_bus_current_limit > LONG_REGEN_BUS_CURRENT_CAP_A ||
      c.regen_enter_error < .05f || c.regen_enter_error > 2 || c.regen_exit_error < 0 ||
      c.regen_exit_error >= c.regen_enter_error || c.regen_kp < 0 || c.regen_kp > 500 ||
      c.collapse_phase_low < -200 || c.collapse_phase_low > -10) return false;
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
    c->ff_damping, c->ff_accel_limit, c->regen_max_current, c->regen_slew,
    c->regen_fade_speed, c->regen_cutoff_speed, c->regen_voltage_soft,
    c->regen_voltage_hard, c->regen_bus_current_limit, c->regen_rpm_field, c->pi_regen,
    c->regen_enter_error, c->regen_exit_error, c->regen_kp, c->collapse_phase_low};
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
  /* The frame may still reflect the previous command, so currents compare in
   * magnitude with the larger of the two. */
  const float reference = fmaxf(fabsf(in->command), fabsf(in->previous_command));
  /* A reported speed that collapses to zero, flips sign or jumps beyond the
   * acceleration the commanded current can make. Low-speed 0-dropouts are
   * benign Part I artefacts. */
  const float step = fmaxf(LONG_COLLAPSE_RPM_STEP, LONG_COLLAPSE_RPM_PER_A * reference);
  if (fabsf(previous) >= LONG_COLLAPSE_RPM_MIN &&
      (rpm * previous <= 0 || fabsf(rpm - previous) > step))
    flags |= LONG_COLLAPSE_RPM;
  if (in->torque_command) {
    if (fabsf(in->phase) > reference + LONG_COLLAPSE_PHASE_MARGIN_A) flags |= LONG_COLLAPSE_PHASE_HIGH;
    /* Negative phase current is the normal state under regen, and the frame
     * after regen returns to 0 A may still show it. */
    if (in->command >= 0 && in->previous_command >= 0 && in->phase < in->phase_low)
      flags |= LONG_COLLAPSE_PHASE_LOW;
    if (fabsf(in->bus) > reference + LONG_COLLAPSE_BUS_MARGIN_A) flags |= LONG_COLLAPSE_BUS;
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

float Long_PiRegen(LongController *s, const LongConfig *c, float target, float actual)
{
  if (!c->pi_regen || !(c->regen_max_current > 0) || !(target > 0)) {
    s->regen_mode = false;
    return 0;
  }
  const float error = target - actual;
  /* Enter only once the drive output is down at 0 A, so the current crosses 0
   * on the slews and never flips sign in one frame. Between the two thresholds
   * the drive PI coasts at 0 A. */
  if (!s->regen_mode && error < -c->regen_enter_error && !(s->current > 0)) s->regen_mode = true;
  else if (s->regen_mode && error > -c->regen_exit_error) s->regen_mode = false;
  return s->regen_mode ? fmaxf(0, c->regen_kp * (-error - c->regen_exit_error)) : 0;
}

void Long_RegenReset(LongRegen *r) { memset(r, 0, sizeof(*r)); }

float Long_RegenFrame(LongRegen *r, LongController *s, const LongConfig *c,
                      const LongRegenInput *in, float dt)
{
  float pi_request = 0;
  if (in->pi && in->allowed) pi_request = Long_PiRegen(s, c, in->target, in->speed);
  else s->regen_mode = false;
  r->request = fmaxf(isfinite(in->external_request) ? fmaxf(in->external_request, 0) : 0,
                     pi_request);
  const bool known = in->part1_fresh && isfinite(in->speed) && isfinite(in->bus_voltage);
  /* Fade to 0 toward the cutoff: the brake must never reverse a stopping wheel. */
  r->speed_factor = !known || in->speed <= c->regen_cutoff_speed ? 0 :
      in->speed >= c->regen_fade_speed ? 1 :
      (in->speed - c->regen_cutoff_speed) / (c->regen_fade_speed - c->regen_cutoff_speed);
  r->voltage_factor = !known || in->bus_voltage >= c->regen_voltage_hard ? 0 :
      in->bus_voltage <= c->regen_voltage_soft ? 1 :
      (c->regen_voltage_hard - in->bus_voltage) / (c->regen_voltage_hard - c->regen_voltage_soft);
  /* Bus guard: measured charge beyond the limit cuts the limit each frame;
   * it recovers at the regen slew once the charge current is back inside. */
  if (known && isfinite(in->bus_current) && in->bus_current < -c->regen_bus_current_limit)
    r->bus_cut = 1 - (1 - r->bus_cut) * (1 - LONG_REGEN_BUS_CUT_PER_FRAME);
  else if (c->regen_max_current > 0)
    r->bus_cut = fmaxf(0, r->bus_cut - c->regen_slew * dt / c->regen_max_current);
  r->limit = c->regen_max_current > 0 ?
      c->regen_max_current * r->speed_factor * r->voltage_factor * (1 - r->bus_cut) : 0;
  /* The request ramps (also back to 0 when the gate drops); the limit applies
   * at once so the cutoff, the voltage ceiling and stale Part I never wait. */
  const float target = in->allowed ? -fminf(r->request, r->limit) : 0;
  const float step = c->regen_slew * dt;
  const bool was_negative = r->current < 0;
  r->current = clamp(clamp(target, r->current - step, r->current + step), -r->limit, 0);
  /* The frame that brings regen back to 0 A still belongs to it: the motor
   * sees one 0 A frame before the drive PI may apply positive current. */
  r->active = r->current < 0 || was_negative;
  if (r->active) {
    /* Regen owns the motor: the drive output restarts from 0 A through the
     * reapply band. The integrator and the PI regen mode are kept. */
    s->current = 0;
    s->reapplying = true;
    s->have_target = false;
  }
  return r->current;
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
