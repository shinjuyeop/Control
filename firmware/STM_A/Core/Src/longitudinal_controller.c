#include "longitudinal_controller.h"
#include <math.h>
#include <string.h>

static float clamp(float x, float lo, float hi) { return fminf(hi, fmaxf(lo, x)); }

LongConfig Long_DefaultConfig(void)
{
  /* Kp/Ki are the omega_n=1 rad/s, zeta=1 pair for the identified gain; the
   * feedforward constants are the 2026-09-20 fit. Mode still boots as speed. */
  return (LongConfig){LONG_SPEED, 380.0f, 190.0f, 480.0f, 120.0f,
                     2400.0f, 4800.0f, 0.05f, 0.12f, 20.0f, 200.0f, 4500.0f,
                     0.00512f, 0.228f, 0.0288f, 1.0f};
}

float Long_PedalCommand(const LongConfig *c, float pedal_percent,
                        float previous_current, float dt)
{
  const float demand = clamp(pedal_percent, 0, 100) / 100.0f * c->max_current;
  const float previous = clamp(previous_current, 0, c->max_current);
  return clamp(demand, fmaxf(0, previous - c->slew_down * dt),
               previous + c->slew_up * dt);
}

LongMotorCommand Long_SpeedCommand(const LongConfig *c, int32_t target_rpm)
{
  return (LongMotorCommand){
    (int32_t)clamp((float)target_rpm, -c->rpm_limit, c->rpm_limit), c->max_current};
}

bool Long_DecodeRequest(const float *d, size_t count, LongRequest *r)
{
  if (count != LONG_REQUEST_COUNT) return false;
  for (size_t n = 0; n < count; ++n) if (!isfinite(d[n])) return false;
  if (d[0] != LONG_VERSION || d[1] < 1 || d[1] > LONG_ID_MAX ||
      floorf(d[1]) != d[1] || d[2] < 0 || d[2] > LONG_ID_MAX ||
      floorf(d[2]) != d[2] || (d[3] != LONG_SPEED && d[3] != LONG_TORQUE)) return false;
  LongConfig c = {(uint8_t)d[3], d[4], d[5], d[6], d[7], d[8], d[9],
                  d[10], d[11], d[12], d[13], d[14], d[15], d[16], d[17], d[18]};
  /* Bounds protect current/RPM capability and 20 Hz numerical authority, not a
   * claim of vehicle-validated tuning. Integral/slew must fit the output range. */
  if (c.kp <= 0 || c.kp > 1000 || c.ki < 0 || c.ki > 200 ||
      c.max_current <= 0 || c.max_current > 480 || c.integrator_limit < 0 ||
      c.integrator_limit > c.max_current || c.slew_up <= 0 || c.slew_up > 9600 ||
      c.slew_down <= 0 || c.slew_down > 9600 || c.deadband < 0 || c.deadband > .2f ||
      c.hysteresis <= c.deadband || c.hysteresis > .5f || c.reapply_current < 0 ||
      c.reapply_current > c.max_current || c.reapply_slew <= 0 ||
      c.reapply_slew > c.slew_up || c.rpm_limit < 100 || c.rpm_limit > 4500) return false;
  /* Feedforward is optional: gain 0 disables it. A nonzero gain stays inside the
   * identifiable range so a near-zero divisor cannot turn into a huge command. */
  if (!(c.ff_gain == 0 || (c.ff_gain >= .0005f && c.ff_gain <= .05f)) ||
      c.ff_rolling < 0 || c.ff_rolling > 2 || c.ff_damping < 0 || c.ff_damping > 1 ||
      c.ff_accel_limit < 0 || c.ff_accel_limit > 5) return false;
  *r = (LongRequest){(uint32_t)d[1], (uint32_t)d[2], c};
  return true;
}

void Long_EncodeConfig(const LongConfig *c, float *d)
{
  const float values[LONG_CONFIG_COUNT] = {c->mode, c->kp, c->ki, c->max_current,
    c->integrator_limit, c->slew_up, c->slew_down, c->deadband, c->hysteresis,
    c->reapply_current, c->reapply_slew, c->rpm_limit, c->ff_gain, c->ff_rolling,
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

float Long_Update(LongController *s, const LongConfig *c, float target, float actual, float dt)
{
  /* Inputs/config were validated at the external boundary. Caller handles
   * stale feedback, mode, direction and stop gates before invoking this loop. */
  if (target <= 0) { Long_Reset(s); return 0; }
  s->error = target - actual;
  s->p = c->kp * s->error;
  s->ff = feedforward(s, c, target, actual, dt);
  s->anti_windup = false;
  if (s->error < -c->deadband) s->coasting = true;
  if (s->coasting && s->error > c->hysteresis) {
    s->coasting = false;
    s->reapplying = true;
  }
  float desired;
  if (s->coasting) {
    /* No negative integral debt, no stored drive torque after overspeed, and no
     * feedforward drive while the vehicle is above the commanded speed. */
    s->i = 0;
    s->ff = 0;
    s->anti_windup = true;
    s->unsaturated = s->p;
    desired = 0;
  } else {
    float error_i = fabsf(s->error) <= c->deadband ? 0 : s->error;
    float candidate = clamp(s->i + c->ki * error_i * dt, 0, c->integrator_limit);
    float proposed = s->ff + s->p + candidate;
    float up = s->reapplying && s->current < c->reapply_current ? c->reapply_slew : c->slew_up;
    /* Conditional integration accounts for both output and upward slew saturation.
     * Negative error may unwind a positive integral; it can never store debt. */
    if ((error_i > 0 && (proposed > c->max_current || proposed > s->current + up * dt)) ||
        (error_i < 0 && proposed < 0)) s->anti_windup = true;
    else s->i = candidate;
    s->unsaturated = s->ff + s->p + s->i;
    desired = clamp(s->unsaturated, 0, c->max_current);
  }
  s->saturated = s->unsaturated < 0 || s->unsaturated > c->max_current;
  float up = s->reapplying && s->current < c->reapply_current ? c->reapply_slew : c->slew_up;
  s->current = clamp(desired, fmaxf(0, s->current - c->slew_down * dt), s->current + up * dt);
  if (s->current >= c->reapply_current) s->reapplying = false;
  return s->current;
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


uint8_t Long_CheckInhibit(LongController *s, LongGate *gate,
                         bool torque_auto, bool feedback_valid, bool command_fresh,
                         bool stop, float target, float actual, bool *discard_command)
{
  uint8_t reason = LONG_OK;
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
  else if (target < 0 || actual < -.05f) reason = LONG_REVERSE;
  if (reason != LONG_OK) Long_Reset(s);
  return reason;
}
