#include "longitudinal_controller.h"
#include "../../common/ezkontrol_command.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>

static void near(float a, float b) { assert(fabsf(a-b) < .001f); }

int main(void)
{
  LongConfig c = Long_DefaultConfig();
  LongController s = {0};
  assert(c.mode == LONG_SPEED && c.max_current == 480);
  c.mode = LONG_TORQUE;
  near(Long_Update(&s, &c, 2, 2, .05f), 0);
  near(Long_Update(&s, &c, 6, 0, .05f), 120);
  assert(s.anti_windup && s.saturated); near(s.i, 0);
  for (int n=0;n<200;n++) Long_Update(&s,&c,6,0,.05f);
  near(s.current,480); near(s.i,0);
  Long_Update(&s,&c,6,7,.05f); near(s.current,240);
  Long_Update(&s,&c,6,7,.05f); near(s.current,0);
  for (int n=0;n<200;n++) Long_Update(&s,&c,6,7,.05f);
  near(s.i,0); assert(s.coasting && s.anti_windup);
  Long_Update(&s,&c,6,5.95f,.05f); near(s.current,0); assert(s.coasting);
  Long_Update(&s,&c,6,5.8f,.05f); near(s.current,10); assert(!s.coasting);
  Long_Update(&s,&c,6,5.8f,.05f); near(s.current,20);
  Long_Update(&s,&c,6,5.8f,.05f); assert(s.current>30);
  // Integral builds after output catches up and survives ordinary target steps.
  Long_Reset(&s);
  for (int n=0;n<100;n++) Long_Update(&s,&c,6,5.9f,.05f);
  assert(s.i>0); float before=s.i;
  Long_Update(&s,&c,6.02f,6,.05f); near(s.i,before);
  Long_Update(&s,&c,5,6,.05f); near(s.i,0); // overspeed erases stored drive torque
  Long_Update(&s,&c,0,4,.05f); near(s.current,0); near(s.i,0); assert(!s.coasting);
  // Stop/reset does not slew: all command/controller history clears immediately.
  Long_Update(&s,&c,8,0,.05f); Long_Reset(&s);
  near(s.current,0); near(s.p,0); near(s.i,0);

  LongGate gate = {0}; bool discard = false;
  Long_Update(&s,&c,8,0,.05f);
  assert(Long_CheckInhibit(&s,&gate,true,false,true,false,8,0,&discard)==LONG_FEEDBACK);
  near(s.current,0); near(s.i,0);
  assert(Long_CheckInhibit(&s,&gate,true,true,true,false,8,0,&discard)==LONG_WAIT_COMMAND);
  assert(discard); // even a recent pre-recovery command must be discarded
  assert(Long_CheckInhibit(&s,&gate,true,true,false,false,8,0,&discard)==LONG_WAIT_COMMAND);
  assert(Long_CheckInhibit(&s,&gate,true,true,true,false,8,0,&discard)==LONG_OK);
  assert(Long_CheckInhibit(&s,&gate,true,true,true,true,8,0,&discard)==LONG_STOP);
  assert(Long_CheckInhibit(&s,&gate,false,true,true,false,8,0,&discard)==LONG_DISABLED);
  assert(Long_CheckInhibit(&s,&gate,true,true,false,false,8,0,&discard)==LONG_WAIT_COMMAND);
  assert(Long_CheckInhibit(&s,&gate,true,true,true,false,-1,0,&discard)==LONG_REVERSE);
  assert(Long_CheckInhibit(&s,&gate,true,true,true,false,1,-1,&discard)==LONG_REVERSE);

  float wire[LONG_REQUEST_COUNT]={1,123,0};
  Long_EncodeConfig(&c,wire+3);
  LongRequest r;
  assert(Long_DecodeRequest(wire,LONG_REQUEST_COUNT,&r));
  for (size_t n=0;n<LONG_REQUEST_COUNT;n++) {
    float old=wire[n];wire[n]=NAN;assert(!Long_DecodeRequest(wire,LONG_REQUEST_COUNT,&r));wire[n]=old;
  }
  wire[6]=481;assert(!Long_DecodeRequest(wire,LONG_REQUEST_COUNT,&r));wire[6]=480;
  wire[3]=2;assert(!Long_DecodeRequest(wire,LONG_REQUEST_COUNT,&r));wire[3]=1;
  assert(!Long_DecodeRequest(wire,LONG_REQUEST_COUNT-1,&r));
  assert(Long_DecodeRequest(wire,LONG_REQUEST_COUNT,&r));
  LongCalibration cal={.config=Long_DefaultConfig()};
  Long_ApplyConfiguration(&cal,&r,false);assert(cal.result==LONG_UNSAFE && cal.revision==0);
  assert(cal.config.mode==LONG_SPEED && cal.source==LONG_DEFAULT);
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
  puts("longitudinal PI, slew, coast, reset, calibration atomic commit/rejection: PASS");
}
