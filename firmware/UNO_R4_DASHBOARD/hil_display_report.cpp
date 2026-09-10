#include "hil_display_report.h"
#include "dashboard_ui.h"
#include "ft6336_touch.h"
#include "swipe_test.h"
#include <Arduino.h>
#include <stdio.h>
#include <string.h>

namespace {
bool active=false;
bool touch_active=false;
char input[16]{};
uint8_t input_size=0;
bool input_overflow=false;
char report[512]{};
size_t report_size=0, report_sent=0;
uint32_t last_report_ms=0;
}
namespace HilDisplayReport {
bool enabled() {return active || touch_active;}
void poll(uint32_t now_ms,uint32_t snapshots,int32_t can_age_ms) {
  if(!Serial) {
    active=touch_active=false;input_size=0;input_overflow=false;report_size=report_sent=0;
    return;
  }
  // Bounded input processing; USB activity must not starve CAN reception.
  for(uint8_t count=0;count<32 && Serial.available();++count) {
    const char ch=char(Serial.read());
    if(ch=='\r')continue;
    if(ch=='\n') {
      input[input_size]='\0';
      if(!input_overflow) {
        if(strcmp(input,"HIL_LCD ON")==0) {active=true;touch_active=false;last_report_ms=now_ms-200U;}
        else if(strcmp(input,"HIL_LCD OFF")==0) active=false;
        else if(strcmp(input,"HIL_TOUCH ON")==0) {touch_active=true;active=false;last_report_ms=now_ms-200U;}
        else if(strcmp(input,"HIL_TOUCH OFF")==0) touch_active=false;
        else if(strcmp(input,"SWIPE_TEST ON")==0) SwipeTest::setEnabled(true);
        else if(strcmp(input,"SWIPE_TEST OFF")==0) SwipeTest::setEnabled(false);
      }
      input_size=0;input_overflow=false;
    } else if(input_size<sizeof(input)-1) input[input_size++]=ch;
    else input_overflow=true;
  }
  if(!enabled()) {report_size=report_sent=0;return;}
  if(report_sent==report_size && uint32_t(now_ms-last_report_ms)>=(touch_active?100U:200U)) {
    int written;
    if(touch_active) {
      const auto &t=Ft6336Touch::state();
      const auto &p=t.points;
      written=snprintf(report,sizeof(report),
        "HIL_TOUCH {\"addr\":56,\"identified\":%s,\"valid\":%s,"
        "\"vendor\":%u,\"chip\":[%u,%u,%u],\"fw\":%u,"
        "\"irq\":%u,\"irq_edges\":%lu,\"samples\":%lu,\"errors\":%lu,"
        "\"sample_ms\":%lu,\"count\":%u,\"points\":[[%u,%u,%u,%u],[%u,%u,%u,%u]],"
        "\"can_seq\":%lu,\"can_age_ms\":%ld}\n",
        t.identified?"true":"false",t.valid?"true":"false",
        t.vendor,t.chip_high,t.chip_mid,t.chip_low,t.firmware,
        t.irq_level,(unsigned long)t.irq_edges,(unsigned long)t.samples,(unsigned long)t.errors,
        (unsigned long)t.sample_ms,t.count,
        p[0].id,p[0].event,p[0].x,p[0].y,p[1].id,p[1].event,p[1].x,p[1].y,
        (unsigned long)snapshots,(long)can_age_ms);
    } else {
    const DashboardValues &v=DashboardUi::displayedValues();
    written=snprintf(report,sizeof(report),
      "HIL_LCD {\"v\":1,\"uptime_ms\":%lu,\"seq\":%lu,\"can_lost\":%s,\"can_age_ms\":%ld,"
      "\"page\":\"%s\",\"values\":[%u,%u,%u,%u,%d,%u,%d,%d,%d,%d,%lu]}\n",
      (unsigned long)now_ms,(unsigned long)snapshots,DashboardUi::canLost()?"true":"false",
      (long)can_age_ms,DashboardUi::pageName(),v.status_flags,v.drive_flags,v.pedal_percent,
      v.gnss_state,v.speed_centi_mps,v.bus_voltage_decivolt,v.bus_current_deciamp,
      v.phase_current_deciamp,v.controller_temperature_decic,v.motor_temperature_decic,
      (unsigned long)v.motor_fault_bits);
    }
    report_size=(written>0 && size_t(written)<sizeof(report))?size_t(written):0;
    report_sent=0;last_report_ms=now_ms;
  }
  // CDC TX space can be smaller than one report. Queue chunks without waiting.
  const int space=Serial.availableForWrite();
  if(space>0 && report_sent<report_size) {
    const size_t remaining=report_size-report_sent;
    const size_t chunk=remaining<size_t(space)?remaining:size_t(space);
    report_sent+=Serial.write((const uint8_t*)report+report_sent,chunk);
  }
}
}
