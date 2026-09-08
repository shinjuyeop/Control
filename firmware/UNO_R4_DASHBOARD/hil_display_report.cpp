#include "hil_display_report.h"
#include "dashboard_ui.h"
#include <Arduino.h>
#include <stdio.h>
#include <string.h>

namespace {
bool active=false;
char input[16]{};
uint8_t input_size=0;
bool input_overflow=false;
char report[320]{};
size_t report_size=0, report_sent=0;
uint32_t last_report_ms=0;
}
namespace HilDisplayReport {
bool enabled() {return active;}
void poll(uint32_t now_ms,uint32_t snapshots,int32_t can_age_ms) {
  if(!Serial) {
    active=false;input_size=0;input_overflow=false;report_size=report_sent=0;
    return;
  }
  // Bounded input processing; USB activity must not starve CAN reception.
  for(uint8_t count=0;count<32 && Serial.available();++count) {
    const char ch=char(Serial.read());
    if(ch=='\r')continue;
    if(ch=='\n') {
      input[input_size]='\0';
      if(!input_overflow) {
        if(strcmp(input,"HIL_LCD ON")==0) {active=true;last_report_ms=now_ms-200U;}
        else if(strcmp(input,"HIL_LCD OFF")==0) active=false;
      }
      input_size=0;input_overflow=false;
    } else if(input_size<sizeof(input)-1) input[input_size++]=ch;
    else input_overflow=true;
  }
  if(!active) {report_size=report_sent=0;return;}
  if(report_sent==report_size && uint32_t(now_ms-last_report_ms)>=200U) {
    const DashboardValues &v=DashboardUi::displayedValues();
    const int written=snprintf(report,sizeof(report),
      "HIL_LCD {\"v\":1,\"uptime_ms\":%lu,\"seq\":%lu,\"can_lost\":%s,\"can_age_ms\":%ld,"
      "\"page\":\"%s\",\"values\":[%u,%u,%u,%u,%d,%u,%d,%d,%d,%d,%lu]}\n",
      (unsigned long)now_ms,(unsigned long)snapshots,DashboardUi::canLost()?"true":"false",
      (long)can_age_ms,DashboardUi::pageName(),v.status_flags,v.drive_flags,v.pedal_percent,
      v.gnss_state,v.speed_centi_mps,v.bus_voltage_decivolt,v.bus_current_deciamp,
      v.phase_current_deciamp,v.controller_temperature_decic,v.motor_temperature_decic,
      (unsigned long)v.motor_fault_bits);
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
