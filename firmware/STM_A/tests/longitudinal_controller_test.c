#include "longitudinal_controller.h"
#include "../../common/ezkontrol_command.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

static void near(float a, float b) { assert(fabsf(a-b) < .001f); }

/* Airborne square-wave preset shape from 2026-09-30 (floor 12 A, max 80 A). */
static LongConfig square_airborne(void)
{
  LongConfig c = Long_DefaultConfig();
  c.kp = 15; c.ki = 10; c.max_current = 80; c.integrator_limit = 6;
  c.slew_up = 120; c.slew_down = 400; c.reapply_current = 5; c.reapply_slew = 20;
  c.waveform = LONG_SQUARE; c.min_current = 12;
  return c;
}

static void test_speed_envelope(void)
{
  const LongConfig driving = Long_DefaultConfig();
  LongConfig airborne = driving;
  airborne.max_current = 20; airborne.rpm_limit = 600;
  const int32_t targets[] = {0, 1, 600, 2000, 4500, 6000, -1, -600, -1500, -4500, -6000};
  for (size_t i=0; i<sizeof(targets)/sizeof(targets[0]); ++i) {
    int32_t target = targets[i];
    LongMotorCommand command = Long_SpeedCommand(&driving, target);
    int32_t actual = EZkontrol_ClampRpm(command.rpm, 4500, 1500);
    int32_t old_rpm = target > 4500 ? 4500 : target < -1500 ? -1500 : target;
    uint8_t previous[8], current[8];
    EZkontrol_EncodeCommand(old_rpm, 480, 1, 1, 42, previous);
    EZkontrol_EncodeCommand(actual, command.current, 1, 1, 42, current);
    assert(memcmp(previous, current, sizeof(current)) == 0);
    near(command.current,480);
    assert(actual >= -1500 && actual <= 4500 && current[4] == 3);

    command = Long_SpeedCommand(&airborne, target);
    actual = EZkontrol_ClampRpm(command.rpm, 4500, 1500);
    assert(actual == (target > 600 ? 600 : target < -600 ? -600 : target));
    near(command.current,20);
    EZkontrol_EncodeCommand(actual,command.current,1,1,42,current);
    assert(current[4] == 3 && ((unsigned)current[1]*256+current[0]) == 32200);
  }
  // Final physical clamp also serves MANUAL, independently of the AUTO preset.
  assert(EZkontrol_ClampRpm(4500,4500,1500) == 4500);
  assert(EZkontrol_ClampRpm(-4500,4500,1500) == -1500);
  uint8_t manual[8];
  EZkontrol_EncodeCommand(EZkontrol_ClampRpm(4500,4500,1500),480,1,0,0,manual);
  assert(manual[4] == 1 && ((unsigned)manual[1]*256+manual[0]) == 36800);
}

static void test_external_inputs(void)
{
  LongExternalInput external = {0};
  assert(!Long_ReceiveExternal(&external, -1, 1, 10));
  assert(!Long_ReceiveExternal(&external, NAN, 1, 10));
  assert(!Long_ReceiveExternal(&external, 481, 1, 10));
  assert(Long_ReceiveExternal(&external, 120, 100, 10));
  assert(Long_ExternalFresh(&external, 509, 500));
  assert(!Long_ExternalFresh(&external, 510, 500));
  Long_ClearExternal(&external, 200);
  assert(!Long_ExternalFresh(&external, 100, 500));
  assert(!Long_ReceiveExternal(&external, 120, 150, 100));
  assert(Long_ReceiveExternal(&external, 80, 201, UINT32_MAX-10));
  assert(Long_ExternalFresh(&external, 10, 500));
  LongExternalInput regen = {0};
  assert(!Long_ReceiveRegen(&regen, 1, 1, 10));
  assert(!Long_ReceiveRegen(&regen, NAN, 1, 10));
  assert(!Long_ReceiveRegen(&regen, -481, 1, 10));
  assert(Long_ReceiveRegen(&regen, -10, 100, 10));
  assert(Long_ExternalFresh(&regen, 509, 500) && regen.current == -10);
  assert(!Long_ReceiveRegen(&regen, -20, 100, 20));
  assert(Long_ReceiveRegen(&regen, 0, 101, 20));
  float external_wire[LONG_REQUEST_COUNT] = {LONG_VERSION, 1, 0};
  LongConfig external_config=Long_DefaultConfig();
  external_config.mode=LONG_EXTERNAL_CURRENT;
  Long_EncodeConfig(&external_config, external_wire+3);
  LongRequest external_request;
  assert(Long_DecodeRequest(external_wire, LONG_REQUEST_COUNT, &external_request));
  external_wire[0]=1;
  assert(!Long_DecodeRequest(external_wire, LONG_REQUEST_COUNT, &external_request));
}

/* The v3 continuous PI is unchanged when no floor is configured (Sine default). */
static void test_pi(void)
{
  LongConfig c = Long_DefaultConfig();
  LongController s = {0};
  assert(c.mode == LONG_TORQUE && c.max_current == 480);
  assert(c.waveform == LONG_SINE && c.min_current == 0 && c.ff_gain == 0);
  near(Long_Update(&s, &c, 2, 2, .05f, 0), 0);
  near(Long_Update(&s, &c, 6, 0, .05f, 0), 120);
  assert(s.anti_windup && s.saturated); near(s.i, 0);
  for (int n=0;n<200;n++) Long_Update(&s,&c,6,0,.05f,0);
  near(s.current,480); near(s.i,0);
  Long_Update(&s,&c,6,7,.05f,0); near(s.current,240);
  Long_Update(&s,&c,6,7,.05f,0); near(s.current,0);
  for (int n=0;n<200;n++) Long_Update(&s,&c,6,7,.05f,0);
  near(s.i,0); assert(s.coasting && s.anti_windup);
  Long_Update(&s,&c,6,5.95f,.05f,0); assert(s.current>0 && s.current<=10 && !s.coasting);
  float reapply_before=s.current;
  Long_Update(&s,&c,6,5.8f,.05f,0); near(s.current,reapply_before+10);
  Long_Update(&s,&c,6,5.8f,.05f,0); assert(s.current>20);
  // Integral builds after output catches up and survives ordinary target steps.
  Long_Reset(&s);
  for (int n=0;n<100;n++) Long_Update(&s,&c,6,5.9f,.05f,0);
  assert(s.i>0); float before=s.i;
  Long_Update(&s,&c,6.02f,6,.05f,0); near(s.i,before);
  Long_Update(&s,&c,5,6,.05f,0); near(s.i,before); // zero-output saturation preserves load current
  Long_Update(&s,&c,0,4,.05f,0); near(s.current,0); near(s.i,0); assert(!s.coasting);
  // Stop/reset does not slew: all command/controller history clears immediately.
  Long_Update(&s,&c,8,0,.05f,0); Long_Reset(&s);
  near(s.current,0); near(s.p,0); near(s.i,0);

  // Airborne downward step held at a floor: no regeneration, no I erasure,
  // positive torque resumes above target and obeys the reapply current slew.
  LongConfig air=c;
  air.kp=25; air.ki=2; air.max_current=40; air.integrator_limit=6;
  air.slew_up=120; air.slew_down=400; air.reapply_current=5; air.reapply_slew=20;
  air.min_current=1;
  s.i=6; s.current=6;
  for (int n=0;n<40;n++) {
    near(Long_Update(&s,&air,10.0f/3.6f,20.0f/3.6f,.05f,1),1);
    near(s.i,6); assert(s.anti_windup);
  }
  near(Long_Update(&s,&air,10.0f/3.6f,10.0f/3.6f+.1f,.05f,1),2);
  assert(!s.coasting && s.i>5.9f);
  float integral=s.i;
  Long_Update(&s,&air,3,3.1f,.05f,1);
  assert(s.i<integral && s.current>0); // unsaturated negative error reduces I
  Long_Update(&s,&air,0,3,.05f,1); near(s.i,0); near(s.current,0);
}

/* P0-2: a held output restarts from the floor, so re-acceleration leaves the
 * floor at slew_up in the first frame instead of re-ramping from 0. */
static void test_seeding(void)
{
  const LongConfig c = square_airborne();
  LongController s = {0};
  near(Long_Update(&s,&c,5.56f,5.56f,.05f,12),12); // fresh controller starts at the floor
  for (int n=0;n<40;n++) {
    near(Long_Update(&s,&c,5.56f,7.0f,.05f,12),12);  // overspeed holds the floor
    assert(s.coasting && s.anti_windup);
  }
  float integral = s.i;
  near(Long_Update(&s,&c,5.56f,4.0f,.05f,12),18);   // 12 + 120 A/s * 50 ms
  assert(s.i >= integral);
  // Without the floor the same overspeed coasts at 0 A and climbs back through
  // the 20 A/s reapply band: the 0.3~0.5 s flat start seen before the rework.
  LongController cold = {0};
  for (int n=0;n<5;n++) Long_Update(&cold,&c,5.56f,7.0f,.05f,0);
  near(cold.current,0);
  near(Long_Update(&cold,&c,5.56f,4.0f,.05f,0),1);
  // A floor below the reapply band still ramps slowly only above the floor.
  LongConfig low = c; low.min_current = 2;
  LongController t = {0};
  for (int n=0;n<5;n++) Long_Update(&t,&low,5.56f,7.0f,.05f,2);
  near(t.current,2);
  near(Long_Update(&t,&low,5.56f,4.0f,.05f,2),3);
}

/* P0-5: inside the hold the output slews down to the floor; once the hold is
 * released it never crawls through (0, min_current). Sine keeps the plain slew. */
static void test_release(void)
{
  LongConfig c = square_airborne();
  c.slew_down = 50; // the value that crawled through the dead zone on 2026-09-30
  LongController s = {0};
  for (int n=0;n<20;n++) Long_Update(&s,&c,5.56f,3.0f,.05f,12);
  assert(s.current > 20);
  float previous = s.current;
  near(Long_Update(&s,&c,5.56f,9.0f,.05f,12), previous - 2.5f); // above the floor: slew
  for (int n=0;n<40;n++) assert(Long_Update(&s,&c,5.56f,9.0f,.05f,12) >= 12);
  near(s.current,12);
  near(Long_Update(&s,&c,5.56f,9.0f,.05f,0),0); // hold released: one frame to 0
  // Released while still high: slew down to the floor band, then cut.
  LongController h = {0};
  for (int n=0;n<20;n++) Long_Update(&h,&c,5.56f,3.0f,.05f,12);
  assert(h.current > 20);
  float out = h.current;
  while (out > 12.5f) {
    const float next = Long_Update(&h,&c,5.56f,9.0f,.05f,0);
    assert(next == 0 || next >= 12);
    assert(next >= out - 2.5f - .001f || next == 0);
    out = next;
    if (out == 0) break;
  }
  near(Long_Update(&h,&c,5.56f,9.0f,.05f,0),0);
  // Sine (min_current 0): the ordinary downward slew is untouched.
  LongConfig sine = c; sine.min_current = 0; sine.waveform = LONG_SINE;
  LongController f = {0};
  f.current = 12;
  near(Long_Update(&f,&sine,5.56f,9.0f,.05f,0),9.5f);
}

static void test_hold_and_floor(void)
{
  LongConfig c = square_airborne();
  LongDrive d = {0};
  // Hysteresis 0.35 on / 0.20 off; only eligible paths hold.
  assert(!Long_UpdateHold(&d,&c,true,.30f));
  assert(Long_UpdateHold(&d,&c,true,.35f));
  assert(Long_UpdateHold(&d,&c,true,.25f));
  assert(!Long_UpdateHold(&d,&c,true,.20f));
  assert(!Long_UpdateHold(&d,&c,true,.30f));
  assert(Long_UpdateHold(&d,&c,true,5));
  assert(!Long_UpdateHold(&d,&c,false,5)); // HALTED/regen/reverse/stale feedback
  assert(Long_UpdateHold(&d,&c,true,5));
  assert(!Long_UpdateHold(&d,&c,true,NAN));
  near(Long_HoldFloor(&d,&c),0);
  assert(Long_UpdateHold(&d,&c,true,5));
  near(Long_HoldFloor(&d,&c),12);
  // Final output stage: held paths never send less than the floor.
  near(Long_DriveFloor(&d,&c,0),12); assert(d.floor_applied && !d.start_stepped);
  near(Long_DriveFloor(&d,&c,11.9f),12); assert(d.floor_applied);
  near(Long_DriveFloor(&d,&c,12),12); assert(d.floor_applied); // PI already at the floor
  near(Long_DriveFloor(&d,&c,12.1f),12.1f); assert(!d.floor_applied);
  near(Long_DriveFloor(&d,&c,30),30); assert(!d.floor_applied);
  // Standing start steps over the dead zone; zero stays zero.
  assert(!Long_UpdateHold(&d,&c,true,0));
  near(Long_DriveFloor(&d,&c,0),0); assert(!d.floor_applied && !d.start_stepped);
  near(Long_DriveFloor(&d,&c,.1f),12); assert(d.start_stepped);
  near(Long_DriveFloor(&d,&c,11.9f),12); assert(d.start_stepped);
  near(Long_DriveFloor(&d,&c,12.5f),12.5f); assert(!d.start_stepped);
  c.start_step = false;
  near(Long_DriveFloor(&d,&c,5),5); assert(!d.start_stepped);
  // Sine/FOC: min_current 0 disables the hold, the step and the floor.
  LongConfig sine = Long_DefaultConfig();
  assert(!Long_UpdateHold(&d,&sine,true,5));
  near(Long_DriveFloor(&d,&sine,0),0); near(Long_DriveFloor(&d,&sine,3),3);
  assert(!d.floor_applied && !d.start_stepped);
}

/* P0-1 property: through coasting, reapply and target steps on an airborne
 * plant, a held output is never below the floor, and with the start step no
 * forward command ever sits inside (0, min_current). */
static void test_floor_all_paths(void)
{
  const LongConfig c = square_airborne();
  LongController s = {0};
  LongDrive d = {0};
  float v = 0;
  const float targets[] = {20/3.6f, 40/3.6f, 20/3.6f, 10/3.6f, 40/3.6f, 0, 20/3.6f};
  int held = 0;
  for (size_t k=0; k<sizeof(targets)/sizeof(targets[0]); ++k) {
    for (int n=0;n<200;n++) {
      const float target = targets[k];
      const bool eligible = target > 0;
      Long_UpdateHold(&d,&c,eligible,v);
      float current = eligible ? Long_Update(&s,&c,target,v,.05f,Long_HoldFloor(&d,&c)) : 0;
      if (!eligible) Long_Reset(&s);
      current = Long_DriveFloor(&d,&c,current);
      if (d.hold) { assert(current >= 12 && d.floor_applied == (current == 12)); held++; }
      assert(current == 0 || current >= 12);
      // 2026-09-29/30 airborne fit: a = 0.07 I - 0.8 - 0.09 v (coast: no current).
      const float a = current > 0 ? .07f*current - .8f - .09f*v : -.8f - .09f*v;
      v = fmaxf(0, v + a*.05f);
    }
  }
  assert(held > 500);
  // External current path: the host value passes through the same stage.
  Long_UpdateHold(&d,&c,true,3);
  const float external[] = {0, 1, 4, 11.9f, 12, 30};
  for (size_t i=0;i<sizeof(external)/sizeof(external[0]);++i)
    assert(Long_DriveFloor(&d,&c,external[i]) >= 12);
}

/* P0-3 and the MANUAL envelope (pedal is torque demand, no PI history). */
static void test_pedal(void)
{
  LongConfig c = Long_DefaultConfig();
  near(Long_PedalCommand(&c,0,true,false,0,.05f),0);
  near(Long_PedalCommand(&c,100,true,false,0,.05f),120);   // slew_up 2400 A/s
  near(Long_PedalCommand(&c,100,true,false,400,.05f),480); // stops at the configured limit
  near(Long_PedalCommand(&c,0,true,false,480,.05f),240);   // slew_down 4800 A/s
  near(Long_PedalCommand(&c,50,true,false,240,.05f),240);
  LongConfig airborne = c;
  airborne.max_current = 20; airborne.slew_up = 120; airborne.slew_down = 400;
  near(Long_PedalCommand(&airborne,100,true,false,0,.05f),6);
  near(Long_PedalCommand(&airborne,100,true,false,20,.05f),20);
  near(Long_PedalCommand(&airborne,50,true,false,0,1),10);
  // Out-of-range pedal or stale history cannot exceed the limit or go negative.
  near(Long_PedalCommand(&airborne,150,true,false,0,1),20);
  near(Long_PedalCommand(&airborne,-10,true,false,10,1),0);
  near(Long_PedalCommand(&airborne,100,true,false,480,.05f),20);
  near(Long_PedalCommand(&airborne,NAN,true,false,NAN,1),0);
  // Speed calibration keeps the direct 0..480 A map without slew.
  near(Long_PedalCommand(&airborne,50,false,false,0,.05f),240);
  near(Long_PedalCommand(&airborne,0,false,false,480,.05f),0);
  // Rolling hold: 0 % keeps the floor, 1 % already adds torque above it.
  LongConfig sq = square_airborne();
  sq.slew_up = sq.slew_down = 9600;
  near(Long_PedalCommand(&sq,0,true,true,0,.05f),12);
  near(Long_PedalCommand(&sq,1,true,true,12,.05f),12 + .01f*(80-12));
  near(Long_PedalCommand(&sq,100,true,true,12,.05f),80);
  near(Long_PedalCommand(&sq,0,false,true,0,.05f),12);
  near(Long_PedalCommand(&sq,1,false,true,0,.05f),12 + .01f*(480-12));
  near(Long_PedalCommand(&sq,100,false,true,0,.05f),480);
  near(Long_PedalCommand(&sq,50,true,false,0,.05f),40); // stopped: existing map
  // Release inside the hold slews only down to the floor; after the hold it cuts.
  sq = square_airborne(); sq.slew_down = 50;
  near(Long_PedalCommand(&sq,0,true,true,40,.05f),37.5f);
  near(Long_PedalCommand(&sq,0,true,true,13,.05f),12);
  near(Long_PedalCommand(&sq,0,true,false,12,.05f),0);
  near(Long_PedalCommand(&sq,0,true,false,40,.05f),37.5f);
}

/* P0-4: one or two negative-speed frames keep RUN; the third halts. */
static void test_reverse_debounce(void)
{
  const LongConfig c = Long_DefaultConfig();
  LongController s = {0};
  LongGate gate = {0}; bool discard = false;
  Long_Update(&s,&c,8,0,.05f,0);
  assert(Long_CheckInhibit(&s,&gate,&c,true,false,true,false,8,0,&discard)==LONG_FEEDBACK);
  near(s.current,0); near(s.i,0);
  assert(Long_CheckInhibit(&s,&gate,&c,true,true,true,false,8,0,&discard)==LONG_WAIT_COMMAND);
  assert(discard); // even a recent pre-recovery command must be discarded
  assert(Long_CheckInhibit(&s,&gate,&c,true,true,false,false,8,0,&discard)==LONG_WAIT_COMMAND);
  assert(Long_CheckInhibit(&s,&gate,&c,true,true,true,false,8,0,&discard)==LONG_OK);
  assert(Long_CheckInhibit(&s,&gate,&c,true,true,true,true,8,0,&discard)==LONG_STOP);
  assert(Long_CheckInhibit(&s,&gate,&c,false,true,true,false,8,0,&discard)==LONG_DISABLED);
  assert(Long_CheckInhibit(&s,&gate,&c,true,true,false,false,8,0,&discard)==LONG_WAIT_COMMAND);
  // A negative target is still refused at once.
  assert(Long_CheckInhibit(&s,&gate,&c,true,true,true,false,-1,0,&discard)==LONG_REVERSE);
  // -0.05 m/s used to halt; inside the 0.30 m/s band it is now ordinary noise.
  assert(Long_CheckInhibit(&s,&gate,&c,true,true,true,false,1,-.13f,&discard)==LONG_OK);
  assert(Long_CheckInhibit(&s,&gate,&c,true,true,true,false,1,-.3f,&discard)==LONG_OK);
  // The logged one-frame -0.39 m/s glitch keeps RUN and the PI history.
  for (int n=0;n<20;n++) Long_Update(&s,&c,1,.8f,.05f,0);
  const float integral = s.i;
  assert(integral > 0);
  assert(Long_CheckInhibit(&s,&gate,&c,true,true,true,false,1,-.39f,&discard)==LONG_REVERSE_PENDING);
  assert(gate.reverse_frames==1); near(s.i,integral);
  assert(Long_CheckInhibit(&s,&gate,&c,true,true,true,false,1,.4f,&discard)==LONG_OK);
  assert(gate.reverse_frames==0); near(s.i,integral);
  // Two frames still pending, the third is a real reverse roll: HALTED path.
  assert(Long_CheckInhibit(&s,&gate,&c,true,true,true,false,1,-.5f,&discard)==LONG_REVERSE_PENDING);
  assert(Long_CheckInhibit(&s,&gate,&c,true,true,true,false,1,-.5f,&discard)==LONG_REVERSE_PENDING);
  assert(Long_CheckInhibit(&s,&gate,&c,true,true,true,false,1,-.5f,&discard)==LONG_REVERSE);
  near(s.i,0); near(s.current,0);
  assert(Long_CheckInhibit(&s,&gate,&c,true,true,true,false,1,-.5f,&discard)==LONG_REVERSE);
  // Any other inhibit clears the count; recovery needs three new frames again.
  assert(Long_CheckInhibit(&s,&gate,&c,true,false,true,false,1,-.5f,&discard)==LONG_FEEDBACK);
  assert(gate.reverse_frames==0);
  assert(Long_CheckInhibit(&s,&gate,&c,true,true,true,false,1,-.5f,&discard)==LONG_WAIT_COMMAND);
  assert(Long_CheckInhibit(&s,&gate,&c,true,true,true,false,1,-.5f,&discard)==LONG_REVERSE_PENDING);
  LongConfig one = c; one.reverse_frames = 1;
  LongGate g1 = {0};
  assert(Long_CheckInhibit(&s,&g1,&one,true,true,true,false,1,-.5f,&discard)==LONG_REVERSE);
}

static LongCollapseInput frame(float command, float previous, float phase, float bus,
                               int32_t rpm, int32_t before, bool torque, float phase_low)
{
  return (LongCollapseInput){.command = command, .previous_command = previous, .phase = phase,
    .bus = bus, .phase_low = phase_low, .rpm = rpm, .previous_rpm = before, .running = true,
    .torque_command = torque};
}

/* P0-6/B3: logged collapses are flagged; normal launches, FOC 0 A noise and
 * regen are not. Square frames use -30 A, sine frames -60 A. */
static void test_collapse(void)
{
  LongCollapseInput in = frame(12, 12, 15.4f, 6.1f, 1094, 1113, true, -30);
  assert(Long_CollapseFlags(&in) == 0);
  // 2026-09-30 00:11 t=7.95: +257.6 A phase, +171.7 A bus at a 12 A command.
  in = frame(12, 12, 257.6f, 171.7f, 802, 948, true, -30);
  assert(Long_CollapseFlags(&in) == (LONG_COLLAPSE_PHASE_HIGH | LONG_COLLAPSE_BUS));
  // t=9.75: -230.4 A phase while slewing down through 11.1 A.
  in = frame(11.1f, 13.6f, -230.4f, 176.2f, 842, 1003, true, -30);
  assert(Long_CollapseFlags(&in) ==
         (LONG_COLLAPSE_PHASE_HIGH | LONG_COLLAPSE_PHASE_LOW | LONG_COLLAPSE_BUS));
  // 00:07 t=21.1: speed estimate jumps at a constant 12 A command.
  in = frame(12, 12, 6.5f, -16.9f, 1633, 2515, true, -30);
  assert(Long_CollapseFlags(&in) == LONG_COLLAPSE_RPM);
  in = frame(1, 0, 6, -.9f, -380, 483, true, -30);
  assert(Long_CollapseFlags(&in) == LONG_COLLAPSE_RPM);
  // Normal 80 A airborne launch (+98 rpm/frame) and benign low-speed dropout.
  in = frame(80, 74.4f, 92.2f, 40, 574, 476, true, -30);
  assert(Long_CollapseFlags(&in) == 0);
  in = frame(3.5f, 0, 6.2f, 8.1f, 0, 216, true, -30);
  assert(Long_CollapseFlags(&in) == 0);
  // Native Speed: EZ commands its own negative current; only the RPM rule counts.
  in = frame(80, 80, -78, -26, 465, 472, false, -30);
  assert(Long_CollapseFlags(&in) == 0);
  in.rpm = 0; assert(Long_CollapseFlags(&in) == LONG_COLLAPSE_RPM);
  // 2026-10-01 sine: FOC current noise at 0 A and 2350 rpm passes the sine
  // threshold; the square threshold would have flagged it.
  in = frame(0, 0, -51.3f, 2.6f, 2354, 2318, true, -60);
  assert(Long_CollapseFlags(&in) == 0);
  in.phase_low = -30; assert(Long_CollapseFlags(&in) == LONG_COLLAPSE_PHASE_LOW);
  // A 350 A launch moves 381 rpm in a frame: inside 3 rpm per commanded amp.
  in = frame(353.5f, 426.8f, 341.4f, 77.2f, 959, 578, true, -60);
  assert(Long_CollapseFlags(&in) == 0);
  in = frame(0, 0, 20, 5, 959, 578, true, -60);
  assert(Long_CollapseFlags(&in) == LONG_COLLAPSE_RPM); // the same jump at 0 A is not physical
  // Sine 23:11 run t=27.8: RPM 0 with a 166 A spike at a 20~60 A command.
  in = frame(59.7f, 20, 166, 8.4f, 0, 728, true, -60);
  assert(Long_CollapseFlags(&in) == (LONG_COLLAPSE_RPM | LONG_COLLAPSE_PHASE_HIGH));
  // A fast cut: the frame still shows the previous 240 A command.
  in = frame(0, 240, 230, 50, 1500, 1480, true, -60);
  assert(Long_CollapseFlags(&in) == 0);
  // Regen: negative phase is normal; magnitude rules still catch a spike.
  in = frame(-20, -15, -22, -12, 1200, 1190, true, -60);
  assert(Long_CollapseFlags(&in) == 0);
  in = frame(-20, -20, -150, -30, 1200, 1190, true, -60);
  assert(Long_CollapseFlags(&in) == LONG_COLLAPSE_PHASE_HIGH);
  in = frame(-20, -20, -25, -95, 1200, 1190, true, -60);
  assert(Long_CollapseFlags(&in) == LONG_COLLAPSE_BUS);
  // The cutoff drops -40 A regen to 0 A at once; the next frame still shows it.
  in = frame(0, -40, -38, -12, 260, 300, true, -30);
  assert(Long_CollapseFlags(&in) == 0);
  in.previous_command = 0; assert(Long_CollapseFlags(&in) == LONG_COLLAPSE_PHASE_LOW);
  // HALTED: nothing is commanded, nothing is attributed.
  in = frame(0, 0, -100, 200, 0, 2000, false, -60);
  in.running = false;
  assert(Long_CollapseFlags(&in) == 0);

  // One event per episode; a new event needs 20 clean frames (1 s).
  LongCollapse s = {0};
  LongCollapseInput clean = frame(12, 12, 15, 6, 1100, 1100, true, -30);
  LongCollapseInput spike = frame(12, 12, 257.6f, 171.7f, 802, 948, true, -30);
  LongCollapseInput drop = frame(12, 12, 3, 2, 0, 802, true, -30);
  assert(!Long_CollapseUpdate(&s,&clean,LONG_PATH_TORQUE,true) && s.events == 0);
  assert(Long_CollapseUpdate(&s,&spike,LONG_PATH_TORQUE,true) && s.events == 1);
  assert(!Long_CollapseUpdate(&s,&drop,LONG_PATH_TORQUE,false) && s.events == 1);
  assert(s.flags == (LONG_COLLAPSE_PHASE_HIGH | LONG_COLLAPSE_BUS | LONG_COLLAPSE_RPM));
  near(s.snapshot.phase,257.6f); assert(s.snapshot.rpm == 802 && s.snapshot.previous_rpm == 948);
  assert(s.path == LONG_PATH_TORQUE && s.hold);
  for (int n=0;n<19;n++) assert(!Long_CollapseUpdate(&s,&clean,LONG_PATH_TORQUE,true));
  assert(!Long_CollapseUpdate(&s,&spike,LONG_PATH_TORQUE,true) && s.events == 1);
  for (int n=0;n<20;n++) assert(!Long_CollapseUpdate(&s,&clean,LONG_PATH_TORQUE,true));
  assert(Long_CollapseUpdate(&s,&drop,LONG_PATH_MANUAL,false) && s.events == 2);
  assert(s.flags == LONG_COLLAPSE_RPM && s.path == LONG_PATH_MANUAL && !s.hold);
}

static LongRegenInput regen_input(float speed, float voltage, float bus, float request)
{
  return (LongRegenInput){.target = 2, .speed = speed, .bus_voltage = voltage,
    .bus_current = bus, .external_request = request, .part1_fresh = true,
    .allowed = true, .pi = false};
}

/* B1: limit, fade, cutoff, voltage, bus guard and slew of the regen current. */
static void test_regen(void)
{
  LongConfig c = Long_DefaultConfig();
  LongController s = {0};
  LongRegen r = {0};
  // Off by default: no request reaches the motor.
  LongRegenInput in = regen_input(5, 52, 0, 20);
  for (int n=0;n<10;n++) near(Long_RegenFrame(&r,&s,&c,&in,.05f),0);
  assert(!r.active && r.limit == 0);
  c.regen_max_current = 20; // regen_slew 100 A/s = 5 A per frame
  near(Long_RegenFrame(&r,&s,&c,&in,.05f),-5); assert(r.active);
  near(Long_RegenFrame(&r,&s,&c,&in,.05f),-10);
  near(Long_RegenFrame(&r,&s,&c,&in,.05f),-15);
  near(Long_RegenFrame(&r,&s,&c,&in,.05f),-20);
  near(Long_RegenFrame(&r,&s,&c,&in,.05f),-20); near(r.limit,20);
  // The request is capped by the configured maximum.
  in.external_request = 60; near(Long_RegenFrame(&r,&s,&c,&in,.05f),-20);
  // Speed fade: half way between cutoff 0.8 and fade 1.5 m/s halves the limit at once.
  in.speed = 1.15f; near(Long_RegenFrame(&r,&s,&c,&in,.05f),-10); near(r.speed_factor,.5f);
  // Cutoff: 0 in the same frame, never negative at or below it. That 0 A
  // frame still belongs to regen; the drive may resume on the next one.
  in.speed = .8f; near(Long_RegenFrame(&r,&s,&c,&in,.05f),0); assert(r.active);
  near(Long_RegenFrame(&r,&s,&c,&in,.05f),0); assert(!r.active);
  for (float v = .8f; v > -1.0f; v -= .05f) {
    LongRegen fresh = {.current = -20};
    in.speed = v;
    near(Long_RegenFrame(&fresh,&s,&c,&in,.05f),0);
    near(Long_RegenFrame(&r,&s,&c,&in,.05f),0);
  }
  // Voltage: linear from soft 57 V to hard 58 V, immediate 0 at the hard limit.
  Long_RegenReset(&r);
  in = regen_input(5, 52, 0, 20);
  for (int n=0;n<4;n++) Long_RegenFrame(&r,&s,&c,&in,.05f);
  near(r.current,-20);
  in.bus_voltage = 57.5f; near(Long_RegenFrame(&r,&s,&c,&in,.05f),-10); near(r.voltage_factor,.5f);
  in.bus_voltage = 58; near(Long_RegenFrame(&r,&s,&c,&in,.05f),0);
  // Bus guard: charge beyond 30 A cuts the limit 20 % per frame, then recovers at the slew.
  Long_RegenReset(&r);
  in = regen_input(5, 52, 0, 20);
  for (int n=0;n<4;n++) Long_RegenFrame(&r,&s,&c,&in,.05f);
  in.bus_current = -35;
  near(Long_RegenFrame(&r,&s,&c,&in,.05f),-16); near(r.limit,16);
  near(Long_RegenFrame(&r,&s,&c,&in,.05f),-12.8f);
  in.bus_current = -20;
  near(Long_RegenFrame(&r,&s,&c,&in,.05f),-17.8f); // recovers 5 A per frame
  near(Long_RegenFrame(&r,&s,&c,&in,.05f),-20); near(r.bus_cut,0);
  // Stale Part I: voltage and charge unknown, limit 0 at once.
  in.part1_fresh = false; near(Long_RegenFrame(&r,&s,&c,&in,.05f),0);
  // A dropped gate ramps back to 0 at the slew instead of cutting.
  Long_RegenReset(&r);
  in = regen_input(5, 52, 0, 20);
  for (int n=0;n<4;n++) Long_RegenFrame(&r,&s,&c,&in,.05f);
  in.allowed = false;
  near(Long_RegenFrame(&r,&s,&c,&in,.05f),-15);
  near(Long_RegenFrame(&r,&s,&c,&in,.05f),-10);
  // Safety stops reset at once.
  Long_RegenReset(&r); near(r.current,0); assert(!r.active);
  // While regen owns the motor the drive output restarts from 0 and keeps I.
  LongController drive = {.current = 30, .i = 6};
  in = regen_input(5, 52, 0, 20);
  Long_RegenFrame(&r,&drive,&c,&in,.05f);
  near(drive.current,0); near(drive.i,6); assert(drive.reapplying);
  in.external_request = NAN;
  LongRegen idle = {0};
  near(Long_RegenFrame(&idle,&drive,&c,&in,.05f),0);
}

/* B2: speed-PI regen hysteresis, zero crossing, frozen integral, and a
 * disabled configuration that leaves the drive output bit-identical. */
static void test_pi_regen(void)
{
  LongConfig c = Long_DefaultConfig();
  c.kp = 15; c.ki = 10; c.max_current = 80; c.integrator_limit = 25;
  c.slew_up = 120; c.slew_down = 400; c.reapply_current = 5; c.reapply_slew = 20;
  LongController s = {0};
  near(Long_PiRegen(&s,&c,5,6),0); assert(!s.regen_mode); // off by default
  c.regen_max_current = 20; c.pi_regen = true; c.regen_kp = 50;
  s.current = 10;
  near(Long_PiRegen(&s,&c,5,5.5f),0); assert(!s.regen_mode); // drive still above 0 A
  s.current = 0;
  near(Long_PiRegen(&s,&c,5,5.2f),0); assert(!s.regen_mode); // inside the coast band
  near(Long_PiRegen(&s,&c,5,5.5f),20); assert(s.regen_mode); // 50 * (0.5 - 0.1)
  near(Long_PiRegen(&s,&c,5,5.2f),5); assert(s.regen_mode);  // stays until the exit band
  near(Long_PiRegen(&s,&c,5,5.05f),0); assert(!s.regen_mode);
  near(Long_PiRegen(&s,&c,0,9),0); assert(!s.regen_mode);    // target 0 never regenerates

  // Closed loop on an airborne-like plant: 30 -> 10 km/h step.
  LongRegen r = {0};
  float v = 30 / 3.6f, previous = 0, frozen = -1;
  int regen_frames = 0;
  for (int n=0;n<400;n++) {
    const float target = n < 40 ? 30 / 3.6f : 10 / 3.6f;
    LongRegenInput in = {.target = target, .speed = v, .bus_voltage = 52, .bus_current = 0,
                         .part1_fresh = true, .allowed = true, .pi = true};
    const float i_before = s.i;
    const float regen = Long_RegenFrame(&r,&s,&c,&in,.05f);
    // Same composition as StartControl: regen owns the frame while active.
    const float current = r.active ? regen : Long_Update(&s,&c,target,v,.05f,0);
    if (r.active) { regen_frames++; near(s.i,i_before); frozen = s.i; }
    assert(!(previous > 0 && current < 0) && !(previous < 0 && current > 0));
    previous = current;
    const float a = current > 0 ? .07f*current - .8f - .09f*v : .07f*current - .5f;
    v = fmaxf(0, v + a*.05f);
  }
  assert(regen_frames > 10 && frozen >= 0);
  assert(fabsf(v - 10 / 3.6f) < .5f);

  // Disabled: the drive output matches plain Long_Update bit for bit.
  LongConfig off = c; off.pi_regen = false; off.regen_max_current = 0;
  LongController a = {0}, b = {0};
  LongRegen ra = {0};
  float va = 0, vb = 0;
  for (int n=0;n<300;n++) {
    const float target = n < 150 ? 8 : 3;
    LongRegenInput in = {.target = target, .speed = va, .bus_voltage = 52,
                         .part1_fresh = true, .allowed = true, .pi = true};
    const float regen = Long_RegenFrame(&ra,&a,&off,&in,.05f);
    const float ca = ra.active ? regen : Long_Update(&a,&off,target,va,.05f,0);
    const float cb = Long_Update(&b,&off,target,vb,.05f,0);
    assert(memcmp(&ca,&cb,sizeof(ca)) == 0);
    va = fmaxf(0, va + (.07f*ca - .8f - .09f*va)*.05f);
    vb = fmaxf(0, vb + (.07f*cb - .8f - .09f*vb)*.05f);
  }
}

/* P1-1: identified-plant feedforward on top of the continuous PI (gain 0 = off). */
static void test_feedforward(void)
{
  LongConfig c = Long_DefaultConfig();
  c.ff_gain = .00512f; c.ff_rolling = .228f; c.ff_damping = .0288f; c.ff_accel_limit = 1;
  c.slew_up = c.slew_down = 9600;
  LongController s = {0};
  // Steady state: the identified resistance current appears without any error.
  const float resistance = (c.ff_rolling + c.ff_damping * 2) / c.ff_gain;
  near(Long_Update(&s,&c,2,2,.05f,0), resistance);
  near(s.ff, resistance); near(s.p, 0); near(s.i, 0);
  near(s.unsaturated, resistance);
  // A commanded ramp adds the bounded acceleration term.
  near(Long_Update(&s,&c,2.025f,2,.05f,0), (.5f + c.ff_rolling + c.ff_damping*2)/c.ff_gain
       + c.kp * .025f);
  // A target step is clamped to ff_accel_limit, not amplified by 1/dt.
  Long_Reset(&s);
  Long_Update(&s,&c,2,2,.05f,0);
  Long_Update(&s,&c,3,2,.05f,0);
  near(s.ff, (c.ff_accel_limit + c.ff_rolling + c.ff_damping*2)/c.ff_gain);
  // Feedforward never exceeds the configured drive limit and never goes negative.
  LongConfig small = c; small.max_current = 30; small.integrator_limit = 30;
  Long_Reset(&s);
  Long_Update(&s,&small,2,2,.05f,0); near(s.ff,30);
  Long_Reset(&s);
  Long_Update(&s,&c,1,-10,.05f,0); near(s.ff,0); // rolling resistance is cancelled
  // Overspeed: P outweighs FF, the output coasts at the floor and I is frozen.
  Long_Reset(&s);
  for (int n=0;n<20;n++) Long_Update(&s,&c,2,1.9f,.05f,0);
  const float integral = s.i;
  assert(s.ff > 0 && integral > 0);
  near(Long_Update(&s,&c,2,3,.05f,0),0);
  assert(s.coasting && s.anti_windup); near(s.i,integral);
  // Gain 0 disables feedforward and restores the plain PI command.
  LongConfig off = c; off.ff_gain = 0;
  Long_Reset(&s);
  near(Long_Update(&s,&off,2,2,.05f,0),0); near(s.ff,0);
  near(Long_Update(&s,&off,2.5f,2,.05f,0), off.kp * .5f + off.ki * .5f * .05f);
}

static void test_decode(void)
{
  LongConfig c = Long_DefaultConfig();
  float wire[LONG_REQUEST_COUNT]={LONG_VERSION,123,0};
  Long_EncodeConfig(&c,wire+3);
  LongRequest r;
  assert(Long_DecodeRequest(wire,LONG_REQUEST_COUNT,&r));
  assert(memcmp(&r.config,&c,sizeof(c)) == 0);
  for (size_t n=0;n<LONG_REQUEST_COUNT;n++) {
    float old=wire[n];wire[n]=NAN;assert(!Long_DecodeRequest(wire,LONG_REQUEST_COUNT,&r));wire[n]=old;
  }
  wire[6]=481;assert(!Long_DecodeRequest(wire,LONG_REQUEST_COUNT,&r));wire[6]=480;
  wire[0]=1;wire[3]=2;assert(!Long_DecodeRequest(wire,LONG_V3_CONFIG_COUNT+2,&r));
  wire[0]=LONG_VERSION;wire[3]=1;
  wire[3]=LONG_SPEED;assert(Long_DecodeRequest(wire,LONG_REQUEST_COUNT,&r));
  LongCalibration speed_cal={.config=Long_DefaultConfig()};
  Long_ApplyConfiguration(&speed_cal,&r,true);
  assert(speed_cal.result==LONG_APPLIED && speed_cal.config.mode==LONG_SPEED);
  // Inactive PI values must not prevent lowering the Speed current limit.
  wire[6]=10;assert(Long_DecodeRequest(wire,LONG_REQUEST_COUNT,&r));
  near(Long_SpeedCommand(&r.config,2000).current,10);
  wire[3]=LONG_TORQUE;assert(!Long_DecodeRequest(wire,LONG_REQUEST_COUNT,&r));
  wire[6]=480;
  // Older layouts stay accepted and take the v4 defaults for appended fields.
  float v3[3+LONG_V3_CONFIG_COUNT];
  memcpy(v3,wire,sizeof(v3));
  v3[0]=3;v3[15]=12;
  assert(Long_DecodeRequest(v3,3+LONG_V3_CONFIG_COUNT,&r));
  near(r.config.min_current,12); assert(r.config.waveform==LONG_SINE && r.config.reverse_frames==3);
  near(r.config.ff_gain,0); near(r.config.regen_max_current,0); near(r.config.collapse_phase_low,-60);
  v3[0]=LONG_VERSION;assert(!Long_DecodeRequest(v3,3+LONG_V3_CONFIG_COUNT,&r));
  v3[0]=2;assert(Long_DecodeRequest(v3,2+LONG_V3_CONFIG_COUNT,&r));
  near(r.config.min_current,0);
  v3[0]=3;assert(!Long_DecodeRequest(v3,2+LONG_V3_CONFIG_COUNT,&r));
  assert(!Long_DecodeRequest(wire,LONG_REQUEST_COUNT-1,&r));
  assert(!Long_DecodeRequest(wire,2,&r));
  // v4 hosts keep working: regeneration stays off.
  wire[0]=4;assert(Long_DecodeRequest(wire,3+LONG_V4_CONFIG_COUNT,&r));
  near(r.config.regen_max_current,0); assert(!r.config.pi_regen);
  assert(!Long_DecodeRequest(wire,LONG_REQUEST_COUNT,&r));
  wire[0]=LONG_VERSION;assert(!Long_DecodeRequest(wire,3+LONG_V4_CONFIG_COUNT,&r));
  // v4 field bounds: integer flags/counts, floor cap, a real hysteresis band.
  const struct { size_t index; float bad, good; } bounds[] = {
    {15,-1,0}, {15,30.5f,30}, {16,2,1}, {16,.5f,0}, {17,.5f,1}, {18,.05f,.25f},
    {18,2.5f,2}, {19,-.1f,0}, {19,.35f,.2f}, {20,.04f,.05f}, {20,1.1f,1},
    {21,0,1}, {21,11,10}, {21,1.5f,3}, {22,41,40}, {22,-1,0}, {23,.0001f,.0005f},
    {23,.3f,.2f}, {24,-.1f,2}, {25,1.1f,1}, {26,5.1f,5},
    {27,-1,0}, {27,61,60}, {28,9,10}, {28,1001,1000}, {29,.2f,.9f}, {29,5.1f,5},
    {30,.05f,.1f}, {30,1.5f,.5f}, {31,39,40}, {31,58,57.5f}, {32,57,57.5f}, {32,61,60},
    {33,.5f,1}, {33,40,39}, {34,2,1}, {34,.5f,0}, {35,2,1}, {36,.04f,.2f}, {36,2.1f,2}, {36,.1f,.2f},
    {37,-.1f,0}, {37,.3f,.2f}, {38,-1,0}, {38,501,500}, {39,-5,-10}, {39,-201,-200},
  };
  for (size_t i=0;i<sizeof(bounds)/sizeof(bounds[0]);++i) {
    const float old = wire[bounds[i].index];
    wire[bounds[i].index] = bounds[i].bad; assert(!Long_DecodeRequest(wire,LONG_REQUEST_COUNT,&r));
    wire[bounds[i].index] = bounds[i].good; assert(Long_DecodeRequest(wire,LONG_REQUEST_COUNT,&r));
    wire[bounds[i].index] = old;
  }
  wire[6]=10;wire[15]=12;assert(!Long_DecodeRequest(wire,LONG_REQUEST_COUNT,&r)); // floor > limit
  wire[6]=480;wire[15]=12;wire[16]=LONG_SQUARE;wire[23]=.064f;
  assert(Long_DecodeRequest(wire,LONG_REQUEST_COUNT,&r));
  near(r.config.min_current,12); assert(r.config.waveform==LONG_SQUARE); near(r.config.ff_gain,.064f);
  wire[27]=20;wire[34]=1;wire[35]=1;wire[38]=40;wire[39]=-30;
  assert(Long_DecodeRequest(wire,LONG_REQUEST_COUNT,&r));
  near(r.config.regen_max_current,20); assert(r.config.regen_rpm_field==1 && r.config.pi_regen);
  near(r.config.regen_kp,40); near(r.config.collapse_phase_low,-30);
  float round[LONG_CONFIG_COUNT];
  Long_EncodeConfig(&r.config,round);
  assert(memcmp(round,wire+3,sizeof(round))==0);
  wire[15]=0;wire[16]=LONG_SINE;wire[23]=0;
  wire[27]=0;wire[34]=0;wire[35]=0;wire[38]=0;wire[39]=-60;
  assert(Long_DecodeRequest(wire,LONG_REQUEST_COUNT,&r));
  LongCalibration cal={.config=Long_DefaultConfig()};
  Long_ApplyConfiguration(&cal,&r,false);assert(cal.result==LONG_UNSAFE && cal.revision==0);
  assert(cal.config.mode==LONG_TORQUE && cal.source==LONG_DEFAULT);
  Long_ApplyConfiguration(&cal,&r,true);assert(cal.result==LONG_APPLIED && cal.revision==1);
  assert(cal.config.mode==LONG_TORQUE && cal.source==LONG_HOST_OVERRIDE);
  Long_ApplyConfiguration(&cal,&r,true);assert(cal.revision==1); // duplicate is idempotent
  r.id++;r.config.kp=250;Long_ApplyConfiguration(&cal,&r,true);
  assert(cal.result==LONG_CONFLICT);near(cal.config.kp,180); // no partial update
  r.expected_revision=1;Long_ApplyConfiguration(&cal,&r,true);
  near(cal.config.kp,250);assert(cal.revision==2);
  assert(Long_CanConfigure(true,true,0,0,0,0));
  assert(!Long_CanConfigure(false,true,0,0,0,0));
  assert(!Long_CanConfigure(true,false,0,0,0,0));
  assert(!Long_CanConfigure(true,true,.1f,0,0,0));
  assert(!Long_CanConfigure(true,true,0,1,0,0));
  assert(!Long_CanConfigure(true,true,0,0,10,0));
  assert(!Long_CanConfigure(true,true,0,0,0,10));
  assert(!Long_CanConfigure(true,true,NAN,0,0,0));
}

static void test_can_encoding(void)
{
  uint8_t frame[8];
  EZkontrol_EncodeCommand(2000,100,1,0,255,frame);
  assert(frame[0]==0xe8 && frame[1]==0x80 && frame[2]==0xd0 && frame[3]==0x84);
  assert(frame[4]==1 && frame[5]==0 && frame[6]==0 && frame[7]==255);
  EZkontrol_EncodeCommand(2000,480,1,1,0,frame);
  assert(frame[4]==3); // native Speed: current limit and signed RPM, bit1=1
  assert(((unsigned)frame[1]*256+frame[0])==36800);
  EZkontrol_EncodeCommand(-1500,-480,1,0,1,frame);
  assert(frame[4]==1 && ((unsigned)frame[1]*256+frame[0])==27200);
  assert(((unsigned)frame[3]*256+frame[2])==30500); // existing MANUAL reverse
  EZkontrol_EncodeCommand(-1500,480,1,1,2,frame);
  assert(frame[4]==3 && ((unsigned)frame[3]*256+frame[2])==30500); // AUTO Speed reverse
  EZkontrol_EncodeCommand(0,0,0,0,3,frame);assert(frame[4]==0);
  assert(((unsigned)frame[1]*256+frame[0])==32000);
  EZkontrol_EncodeCommand(0,0,0,1,4,frame);assert(frame[4]==2);
  // A 12 A floor survives the 0.1 A wire quantisation exactly.
  EZkontrol_EncodeCommand(4500,12,1,0,5,frame);
  assert(((unsigned)frame[1]*256+frame[0])==32120);
}

int main(void)
{
  test_speed_envelope();
  test_external_inputs();
  test_pi();
  test_seeding();
  test_release();
  test_hold_and_floor();
  test_floor_all_paths();
  test_pedal();
  test_reverse_debounce();
  test_collapse();
  test_regen();
  test_pi_regen();
  test_feedforward();
  test_decode();
  test_can_encoding();
  puts("longitudinal PI, rolling floor/seeding/release, pedal map, reverse debounce, "
       "collapse detector, regen limit/fade/guard/slew, PI regen, feedforward, "
       "calibration atomic commit/rejection: PASS");
}
