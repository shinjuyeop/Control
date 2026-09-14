#include "include/dashboard_timing.h"

#if defined(DASHBOARD_HOST)

#include "host_runtime.h"

namespace DashboardTiming
{
uint32_t beginMeasure() { return 0U; }
void statePrepared(uint32_t, Page page)
{
  DashboardHostRuntime::chargeState((uint8_t)page);
}
void rowComposed(uint32_t, Page page, uint16_t y)
{
  DashboardHostRuntime::chargeRow((uint8_t)page, y);
}
void transportTransactionBegin(uint32_t)
{
  DashboardHostRuntime::chargeTransportTransactionBegin();
}
void transportTransactionEnd(uint32_t)
{
  DashboardHostRuntime::chargeTransportTransactionEnd();
}
void transportRgb565Pack(uint32_t, uint16_t pixel_count)
{
  DashboardHostRuntime::chargeTransportRgb565Pack(pixel_count);
}
void transportSolidPack(uint32_t, uint16_t pixel_count)
{
  DashboardHostRuntime::chargeTransportSolidPack(pixel_count);
}
void transportSpiByte(uint32_t)
{
  DashboardHostRuntime::chargeTransportSpiByte();
}
void transportSpiBuffer(uint32_t, uint16_t bytes)
{
  DashboardHostRuntime::chargeTransportSpiBuffer(bytes);
}
void startupEvent(bool done, uint32_t time_us)
{
  DashboardHostRuntime::recordStartup(done, time_us);
}
void startupFrameEvent(bool done, uint16_t elapsed_ms, uint32_t time_us)
{
  DashboardHostRuntime::recordStartupFrame(done, elapsed_ms, time_us);
}
void regionCompleted(Page page, uint16_t region, uint32_t completed_ms)
{
  DashboardHostRuntime::recordRegion((uint8_t)page, region, completed_ms);
}
void modeEvent(bool done, bool direction_auto, uint16_t boundary,
               uint8_t alive, uint8_t flags, uint32_t time_ms)
{
  DashboardHostRuntime::recordMode(
      done, direction_auto, boundary, alive, flags, time_ms);
}
void serviceCompleted(uint32_t) {}
void beginSegment(uint32_t, Page) {}
void endSegment() {}
void serviceReport(uint32_t) {}
void beginBench(Page, uint32_t, uint16_t, uint32_t) {}
bool reportDone() { return true; }
}  // namespace DashboardTiming

#elif defined(DASHBOARD_TIMING_PROFILE)

#include <stdio.h>
#include <string.h>

#include "include/dashboard_serial.h"

#ifndef DASHBOARD_TIMING_PAGE
#define DASHBOARD_TIMING_PAGE 1
#endif

namespace
{
struct Metric
{
  uint32_t count;
  uint32_t sum_us;
  uint32_t maximum_us;
};

struct SpiMetric
{
  Metric elapsed;
  uint32_t total_bytes;
  uint32_t overhead_us;
};

struct RegionEvent
{
  uint32_t completed_ms;
  uint16_t region;
};

DashboardTiming::Page selected_page =
    (DashboardTiming::Page)DASHBOARD_TIMING_PAGE;
static constexpr uint32_t SPI_HZ = 12000000U;

Metric state_metric = {};
Metric row_metrics[320] = {};
SpiMetric spi_metrics[4] = {};
Metric service_metric = {};
RegionEvent region_events[128] = {};
uint16_t region_event_count = 0U;
bool region_event_overflow = false;
uint32_t segment_started_ms = 0U;
uint32_t startup_started_us = 0U;
uint32_t startup_completed_us = 0U;
uint16_t report_line = 0U;
bool collecting = false;
bool reporting = false;
bool finished = false;
uint32_t segment_duration_ms = 10000U;
uint16_t selected_region_mask = 0xFFFFU;

void add(Metric *metric, uint32_t elapsed_us)
{
  metric->count++;
  metric->sum_us += elapsed_us;
  if (elapsed_us > metric->maximum_us) metric->maximum_us = elapsed_us;
}

const char *pageName()
{
  if (selected_page == DashboardTiming::Page::Startup) return "startup";
  if (selected_page == DashboardTiming::Page::Auto) return "auto";
  return "manual";
}

bool enqueueMetric(const char *name, const Metric &metric)
{
  char line[DashboardSerial::MESSAGE_SIZE];
  (void)snprintf(line, sizeof(line),
                 "TIMING %s count=%lu sum_us=%lu avg_us=%lu max_us=%lu",
                 name, (unsigned long)metric.count,
                 (unsigned long)metric.sum_us,
                 (unsigned long)(metric.count == 0U ? 0U :
                                 metric.sum_us / metric.count),
                 (unsigned long)metric.maximum_us);
  return DashboardSerial::enqueue(line);
}
}  // namespace

namespace DashboardTiming
{
uint32_t beginMeasure() { return micros(); }

void statePrepared(uint32_t started_us, Page page)
{
  if (collecting && page == selected_page)
    add(&state_metric, (uint32_t)(micros() - started_us));
}

void rowComposed(uint32_t started_us, Page page, uint16_t y)
{
  if (collecting && page == selected_page && y < 320U)
    add(&row_metrics[y], (uint32_t)(micros() - started_us));
}

void recordLegacyTransport(uint32_t started_us, size_t bytes)
{
  if (!collecting) return;
  const uint32_t elapsed_us = (uint32_t)(micros() - started_us);
  const uint8_t bin = bytes == 0U ? 0U : bytes <= 8U ? 1U :
                      bytes <= 256U ? 2U : 3U;
  SpiMetric &metric = spi_metrics[bin];
  add(&metric.elapsed, elapsed_us);
  metric.total_bytes += bytes;
  const uint32_t nominal_us = (uint32_t)(
      ((uint64_t)bytes * 8000000ULL + SPI_HZ - 1U) / SPI_HZ);
  if (elapsed_us > nominal_us) metric.overhead_us += elapsed_us - nominal_us;
}

void transportTransactionBegin(uint32_t started_us)
{
  recordLegacyTransport(started_us, 0U);
}

void transportTransactionEnd(uint32_t started_us)
{
  recordLegacyTransport(started_us, 0U);
}

void transportRgb565Pack(uint32_t started_us, uint16_t)
{
  recordLegacyTransport(started_us, 0U);
}

void transportSolidPack(uint32_t started_us, uint16_t)
{
  recordLegacyTransport(started_us, 0U);
}

void transportSpiByte(uint32_t started_us)
{
  recordLegacyTransport(started_us, 1U);
}

void transportSpiBuffer(uint32_t started_us, uint16_t bytes)
{
  recordLegacyTransport(started_us, bytes);
}

void startupEvent(bool done, uint32_t time_us)
{
  if (!collecting || selected_page != Page::Startup) return;
  if (done) startup_completed_us = time_us;
  else startup_started_us = time_us;
}

void startupFrameEvent(bool, uint16_t, uint32_t) {}

void regionCompleted(Page page, uint16_t region, uint32_t completed_ms)
{
  if (collecting && page == selected_page &&
      (region & selected_region_mask) != 0U)
  {
    if (region_event_count < sizeof(region_events) / sizeof(region_events[0]))
      region_events[region_event_count++] = {completed_ms, region};
    else
      region_event_overflow = true;
  }
}

void modeEvent(bool done,
               bool direction_auto,
               uint16_t boundary,
               uint8_t alive,
               uint8_t flags,
               uint32_t time_ms)
{
  char line[DashboardSerial::MESSAGE_SIZE];
  (void)snprintf(line, sizeof(line),
                 "MODE%s time=%lu alive=%u flags=0x%02X direction=%s boundary=%u",
                 done ? " DONE" : "", (unsigned long)time_ms,
                 alive, flags, direction_auto ? "UP_AUTO" : "DOWN_MANUAL",
                 boundary);
  (void)DashboardSerial::enqueue(line);
}

void serviceCompleted(uint32_t started_us)
{
  if (collecting)
    add(&service_metric, (uint32_t)(micros() - started_us));
}

void beginSegment(uint32_t now_ms, Page page)
{
  if (page != selected_page || collecting || reporting || finished) return;
  memset(&state_metric, 0, sizeof(state_metric));
  memset(row_metrics, 0, sizeof(row_metrics));
  memset(spi_metrics, 0, sizeof(spi_metrics));
  memset(&service_metric, 0, sizeof(service_metric));
  region_event_count = 0U;
  region_event_overflow = false;
  startup_started_us = 0U;
  startup_completed_us = 0U;
  segment_started_ms = now_ms;
  report_line = 0U;
  collecting = true;
}

void endSegment()
{
  if (!collecting) return;
  collecting = false;
  reporting = true;
  finished = true;
}

void serviceReport(uint32_t now_ms)
{
  if (collecting &&
      (uint32_t)(now_ms - segment_started_ms) >= segment_duration_ms)
    endSegment();
  if (!reporting || !DashboardSerial::hasCapacity()) return;

  char line[DashboardSerial::MESSAGE_SIZE];
  if (report_line == 0U)
  {
    (void)snprintf(line, sizeof(line),
                   "TIMING page=%s region_overflow=%u",
                   pageName(), region_event_overflow ? 1U : 0U);
    if (!DashboardSerial::enqueue(line)) return;
  }
  else if (report_line == 1U)
  {
    if (!enqueueMetric("state", state_metric)) return;
  }
  else if (report_line == 2U)
  {
    (void)snprintf(line, sizeof(line),
                   "TIMING startup start_us=%lu completed_us=%lu",
                   (unsigned long)startup_started_us,
                   (unsigned long)startup_completed_us);
    if (!DashboardSerial::enqueue(line)) return;
  }
  else if (report_line >= 3U && report_line < 323U)
  {
    const uint16_t y = (uint16_t)(report_line - 3U);
    (void)snprintf(line, sizeof(line),
                   "TIMING row y=%u count=%lu sum_us=%lu max_us=%lu",
                   y, (unsigned long)row_metrics[y].count,
                   (unsigned long)row_metrics[y].sum_us,
                   (unsigned long)row_metrics[y].maximum_us);
    if (!DashboardSerial::enqueue(line)) return;
  }
  else if (report_line >= 323U && report_line < 323U + region_event_count)
  {
    const RegionEvent &event = region_events[report_line - 323U];
    (void)snprintf(line, sizeof(line),
                   "TIMING region bit=0x%04X completed_ms=%lu",
                   event.region, (unsigned long)event.completed_ms);
    if (!DashboardSerial::enqueue(line)) return;
  }
  else if (report_line >= 323U + region_event_count &&
           report_line < 327U + region_event_count)
  {
    const uint8_t bin =
        (uint8_t)(report_line - 323U - region_event_count);
    const SpiMetric &metric = spi_metrics[bin];
    (void)snprintf(line, sizeof(line),
                   "TIMING spi bin=%u count=%lu bytes=%lu sum_us=%lu overhead_us=%lu max_us=%lu",
                   bin, (unsigned long)metric.elapsed.count,
                   (unsigned long)metric.total_bytes,
                   (unsigned long)metric.elapsed.sum_us,
                   (unsigned long)metric.overhead_us,
                   (unsigned long)metric.elapsed.maximum_us);
    if (!DashboardSerial::enqueue(line)) return;
  }
  else if (report_line == 327U + region_event_count)
  {
    if (!enqueueMetric("service", service_metric)) return;
  }
  else
  {
    reporting = false;
    return;
  }
  report_line++;
}

void beginBench(Page page,
                uint32_t duration_ms,
                uint16_t region_mask,
                uint32_t now_ms)
{
  collecting = false;
  reporting = false;
  finished = false;
  selected_page = page;
  segment_duration_ms = duration_ms;
  selected_region_mask = region_mask;
  beginSegment(now_ms, page);
}

bool reportDone()
{
  return !collecting && !reporting;
}
}  // namespace DashboardTiming

#elif defined(DASHBOARD_TIMING_EVENTS_ONLY)

#include <stdio.h>
#include <string.h>

#include "include/dashboard_serial.h"

namespace
{
struct RegionEvent
{
  uint32_t completed_ms;
  uint16_t region;
};

struct ModeEvent
{
  uint32_t time_ms;
  uint16_t boundary;
  uint8_t alive;
  uint8_t flags;
  bool done;
  bool direction_auto;
};

struct StartupFrameEvent
{
  uint32_t time_us;
  uint16_t elapsed_ms;
  bool done;
};

DashboardTiming::Page selected_page = DashboardTiming::Page::Manual;
RegionEvent region_events[128] = {};
ModeEvent mode_events[16] = {};
StartupFrameEvent startup_frame_events[32] = {};
uint16_t region_event_count = 0U;
uint8_t mode_event_count = 0U;
uint8_t startup_frame_event_count = 0U;
bool region_event_overflow = false;
bool mode_event_overflow = false;
bool startup_frame_event_overflow = false;
uint32_t startup_started_us = 0U;
uint32_t startup_completed_us = 0U;
uint32_t segment_started_ms = 0U;
uint32_t segment_duration_ms = 10000U;
uint16_t selected_region_mask = 0xFFFFU;
uint16_t report_line = 0U;
bool collecting = false;
bool reporting = false;

const char *pageName()
{
  if (selected_page == DashboardTiming::Page::Startup) return "startup";
  if (selected_page == DashboardTiming::Page::Auto) return "auto";
  if (selected_page == DashboardTiming::Page::Transport) return "transport";
  return "manual";
}
}  // namespace

namespace DashboardTiming
{
void startupEvent(bool done, uint32_t time_us)
{
  if (!collecting || selected_page != Page::Startup) return;
  if (done) startup_completed_us = time_us;
  else startup_started_us = time_us;
}

void startupFrameEvent(bool done, uint16_t elapsed_ms, uint32_t time_us)
{
  if (!collecting || selected_page != Page::Startup) return;
  if (startup_frame_event_count <
      sizeof(startup_frame_events) / sizeof(startup_frame_events[0]))
  {
    startup_frame_events[startup_frame_event_count++] = {
        time_us, elapsed_ms, done};
  }
  else
  {
    startup_frame_event_overflow = true;
  }
}

void regionCompleted(Page page, uint16_t region, uint32_t completed_ms)
{
  if (!collecting || page != selected_page ||
      (region & selected_region_mask) == 0U)
    return;
  if (region_event_count < sizeof(region_events) / sizeof(region_events[0]))
    region_events[region_event_count++] = {completed_ms, region};
  else
    region_event_overflow = true;
}

void modeEvent(bool done,
               bool direction_auto,
               uint16_t boundary,
               uint8_t alive,
               uint8_t flags,
               uint32_t time_ms)
{
  if (!collecting) return;
  if (mode_event_count < sizeof(mode_events) / sizeof(mode_events[0]))
  {
    mode_events[mode_event_count++] = {
        time_ms, boundary, alive, flags, done, direction_auto};
  }
  else
  {
    mode_event_overflow = true;
  }
}

void endSegment()
{
  if (!collecting) return;
  collecting = false;
  reporting = true;
}

void beginSegment(uint32_t, Page) {}

void serviceReport(uint32_t now_ms)
{
  if (collecting &&
      (uint32_t)(now_ms - segment_started_ms) >= segment_duration_ms)
    endSegment();
  if (!reporting || !DashboardSerial::hasCapacity()) return;

  char line[DashboardSerial::MESSAGE_SIZE];
  if (report_line == 0U)
  {
    (void)snprintf(line, sizeof(line),
                   "TIMING page=%s region_overflow=%u mode_overflow=%u startup_frame_overflow=%u events_only=1",
                   pageName(), region_event_overflow ? 1U : 0U,
                   mode_event_overflow ? 1U : 0U,
                   startup_frame_event_overflow ? 1U : 0U);
  }
  else if (report_line == 1U)
  {
    (void)snprintf(line, sizeof(line),
                   "TIMING startup start_us=%lu completed_us=%lu",
                   (unsigned long)startup_started_us,
                   (unsigned long)startup_completed_us);
  }
  else if (report_line < 2U + startup_frame_event_count)
  {
    const StartupFrameEvent &event = startup_frame_events[report_line - 2U];
    (void)snprintf(line, sizeof(line),
                   "TIMING startup_frame phase=%s elapsed_ms=%u time_us=%lu",
                   event.done ? "done" : "start", event.elapsed_ms,
                   (unsigned long)event.time_us);
  }
  else if (report_line <
           2U + startup_frame_event_count + region_event_count)
  {
    const RegionEvent &event = region_events[
        report_line - 2U - startup_frame_event_count];
    (void)snprintf(line, sizeof(line),
                   "TIMING region bit=0x%04X completed_ms=%lu",
                   event.region, (unsigned long)event.completed_ms);
  }
  else if (report_line <
           2U + startup_frame_event_count + region_event_count +
               mode_event_count)
  {
    const ModeEvent &event = mode_events[
        report_line - 2U - startup_frame_event_count - region_event_count];
    (void)snprintf(line, sizeof(line),
                   "MODE%s time=%lu alive=%u flags=0x%02X direction=%s boundary=%u",
                   event.done ? " DONE" : "", (unsigned long)event.time_ms,
                   event.alive, event.flags,
                   event.direction_auto ? "UP_AUTO" : "DOWN_MANUAL",
                   event.boundary);
  }
  else
  {
    reporting = false;
    return;
  }
  if (DashboardSerial::enqueue(line)) report_line++;
}

void beginBench(Page page,
                uint32_t duration_ms,
                uint16_t region_mask,
                uint32_t now_ms)
{
  selected_page = page;
  segment_duration_ms = duration_ms;
  selected_region_mask = region_mask;
  region_event_count = 0U;
  mode_event_count = 0U;
  startup_frame_event_count = 0U;
  region_event_overflow = false;
  mode_event_overflow = false;
  startup_frame_event_overflow = false;
  startup_started_us = 0U;
  startup_completed_us = 0U;
  segment_started_ms = now_ms;
  report_line = 0U;
  reporting = false;
  collecting = true;
}

bool reportDone()
{
  return !collecting && !reporting;
}
}  // namespace DashboardTiming

#elif defined(DASHBOARD_TIMING_TRANSPORT_PROFILE)

#include <stdio.h>
#include <string.h>

#include "include/dashboard_serial.h"

namespace
{
struct Metric
{
  uint32_t sum_us;
  uint16_t count;
  uint16_t maximum_us;
};

Metric transaction_begin_metric = {};
Metric transaction_end_metric = {};
Metric spi_byte_metric = {};
Metric spi_buffer_metrics[257] = {};
Metric rgb565_pack_metrics[129] = {};
Metric solid_pack_metrics[129] = {};
uint32_t segment_started_ms = 0U;
uint32_t segment_duration_ms = 10000U;
uint16_t report_line = 0U;
bool collecting = false;
bool reporting = false;

void add(Metric *metric, uint32_t elapsed_us)
{
  metric->count++;
  metric->sum_us += elapsed_us;
  if (elapsed_us > metric->maximum_us)
    metric->maximum_us = (uint16_t)elapsed_us;
}

bool enqueueMetric(const char *kind, uint16_t length, const Metric &metric)
{
  char line[DashboardSerial::MESSAGE_SIZE];
  if (length == 0U)
  {
    (void)snprintf(line, sizeof(line),
                   "TIMING transport kind=%s count=%lu sum_us=%lu max_us=%lu",
                   kind, (unsigned long)metric.count,
                   (unsigned long)metric.sum_us,
                   (unsigned long)metric.maximum_us);
  }
  else
  {
    (void)snprintf(line, sizeof(line),
                   "TIMING transport kind=%s length=%u count=%lu sum_us=%lu max_us=%lu",
                   kind, length, (unsigned long)metric.count,
                   (unsigned long)metric.sum_us,
                   (unsigned long)metric.maximum_us);
  }
  return DashboardSerial::enqueue(line);
}
}  // namespace

namespace DashboardTiming
{
uint32_t beginMeasure() { return micros(); }

void transportTransactionBegin(uint32_t started_us)
{
  if (collecting)
    add(&transaction_begin_metric, (uint32_t)(micros() - started_us));
}

void transportTransactionEnd(uint32_t started_us)
{
  if (collecting)
    add(&transaction_end_metric, (uint32_t)(micros() - started_us));
}

void transportRgb565Pack(uint32_t started_us, uint16_t pixel_count)
{
  if (collecting && pixel_count <= 128U)
    add(&rgb565_pack_metrics[pixel_count],
        (uint32_t)(micros() - started_us));
}

void transportSolidPack(uint32_t started_us, uint16_t pixel_count)
{
  if (collecting && pixel_count <= 128U)
    add(&solid_pack_metrics[pixel_count],
        (uint32_t)(micros() - started_us));
}

void transportSpiByte(uint32_t started_us)
{
  if (collecting)
    add(&spi_byte_metric, (uint32_t)(micros() - started_us));
}

void transportSpiBuffer(uint32_t started_us, uint16_t bytes)
{
  if (collecting && bytes <= 256U)
    add(&spi_buffer_metrics[bytes], (uint32_t)(micros() - started_us));
}

void startupEvent(bool, uint32_t) {}
void startupFrameEvent(bool, uint16_t, uint32_t) {}
void regionCompleted(Page, uint16_t, uint32_t) {}
void modeEvent(bool, bool, uint16_t, uint8_t, uint8_t, uint32_t) {}
void beginSegment(uint32_t, Page) {}

void endSegment()
{
  if (!collecting) return;
  collecting = false;
  reporting = true;
}

void serviceReport(uint32_t now_ms)
{
  if (collecting &&
      (uint32_t)(now_ms - segment_started_ms) >= segment_duration_ms)
    endSegment();
  if (!reporting || !DashboardSerial::hasCapacity()) return;

  if (report_line == 0U)
  {
    if (!DashboardSerial::enqueue(
            "TIMING page=transport transport_profile=1")) return;
  }
  else if (report_line == 1U)
  {
    if (!enqueueMetric("transaction_begin", 0U,
                       transaction_begin_metric)) return;
  }
  else if (report_line == 2U)
  {
    if (!enqueueMetric("transaction_end", 0U,
                       transaction_end_metric)) return;
  }
  else if (report_line == 3U)
  {
    if (!enqueueMetric("spi_byte", 0U, spi_byte_metric)) return;
  }
  else if (report_line < 259U)
  {
    const uint16_t bytes = (uint16_t)(report_line - 2U);
    if (!enqueueMetric("spi_buffer", bytes,
                       spi_buffer_metrics[bytes])) return;
  }
  else if (report_line < 387U)
  {
    const uint16_t pixels = (uint16_t)(report_line - 258U);
    if (!enqueueMetric("rgb565_pack", pixels,
                       rgb565_pack_metrics[pixels])) return;
  }
  else if (report_line < 515U)
  {
    const uint16_t pixels = (uint16_t)(report_line - 386U);
    if (!enqueueMetric("solid_pack", pixels,
                       solid_pack_metrics[pixels])) return;
  }
  else
  {
    reporting = false;
    return;
  }
  report_line++;
}

void beginBench(Page page,
                uint32_t duration_ms,
                uint16_t,
                uint32_t now_ms)
{
  if (page != Page::Transport) return;
  memset(&transaction_begin_metric, 0, sizeof(transaction_begin_metric));
  memset(&transaction_end_metric, 0, sizeof(transaction_end_metric));
  memset(&spi_byte_metric, 0, sizeof(spi_byte_metric));
  memset(spi_buffer_metrics, 0, sizeof(spi_buffer_metrics));
  memset(rgb565_pack_metrics, 0, sizeof(rgb565_pack_metrics));
  memset(solid_pack_metrics, 0, sizeof(solid_pack_metrics));
  segment_started_ms = now_ms;
  segment_duration_ms = duration_ms;
  report_line = 0U;
  reporting = false;
  collecting = true;
}

bool reportDone()
{
  return !collecting && !reporting;
}
}  // namespace DashboardTiming

#elif defined(DASHBOARD_TIMING_STARTUP_PROFILE)

#include <stdio.h>
#include <string.h>

#include "include/dashboard_serial.h"

namespace
{
struct Metric
{
  uint32_t count;
  uint32_t sum_us;
  uint32_t maximum_us;
};

Metric state_metric = {};
Metric row_metrics[320] = {};
uint16_t selected_elapsed_ms = 0U;
uint16_t report_y = 0U;
uint8_t report_stage = 0U;
bool collecting = false;
bool reporting = false;

void add(Metric *metric, uint32_t elapsed_us)
{
  metric->count++;
  metric->sum_us += elapsed_us;
  if (elapsed_us > metric->maximum_us) metric->maximum_us = elapsed_us;
}
}  // namespace

namespace DashboardTiming
{
uint32_t beginMeasure() { return micros(); }

void statePrepared(uint32_t started_us, Page page)
{
  if (collecting && page == Page::Startup)
    add(&state_metric, (uint32_t)(micros() - started_us));
}

void rowComposed(uint32_t started_us, Page page, uint16_t y)
{
  if (collecting && page == Page::Startup && y < 320U)
    add(&row_metrics[y], (uint32_t)(micros() - started_us));
}

void startupEvent(bool, uint32_t) {}
void startupFrameEvent(bool, uint16_t, uint32_t) {}
void regionCompleted(Page, uint16_t, uint32_t) {}
void modeEvent(bool, bool, uint16_t, uint8_t, uint8_t, uint32_t) {}
void beginSegment(uint32_t, Page) {}

void endSegment()
{
  if (!collecting) return;
  collecting = false;
  reporting = true;
}

void serviceReport(uint32_t)
{
  if (!reporting || !DashboardSerial::hasCapacity()) return;
  char line[DashboardSerial::MESSAGE_SIZE];
  if (report_stage == 0U)
  {
    (void)snprintf(line, sizeof(line),
                   "TIMING page=startup_fixed elapsed_ms=%u startup_profile=1",
                   selected_elapsed_ms);
    if (!DashboardSerial::enqueue(line)) return;
    report_stage = 1U;
    return;
  }
  if (report_stage == 1U)
  {
    (void)snprintf(line, sizeof(line),
                   "TIMING state count=%lu sum_us=%lu avg_us=%lu max_us=%lu",
                   (unsigned long)state_metric.count,
                   (unsigned long)state_metric.sum_us,
                   (unsigned long)(state_metric.count == 0U ? 0U :
                                   state_metric.sum_us / state_metric.count),
                   (unsigned long)state_metric.maximum_us);
    if (!DashboardSerial::enqueue(line)) return;
    report_stage = 2U;
    return;
  }
  while (report_y < 320U && row_metrics[report_y].count == 0U) report_y++;
  if (report_y < 320U)
  {
    const Metric &metric = row_metrics[report_y];
    (void)snprintf(line, sizeof(line),
                   "TIMING row y=%u count=%lu sum_us=%lu max_us=%lu",
                   report_y, (unsigned long)metric.count,
                   (unsigned long)metric.sum_us,
                   (unsigned long)metric.maximum_us);
    if (DashboardSerial::enqueue(line)) report_y++;
    return;
  }
  reporting = false;
}

void beginBench(Page page,
                uint32_t,
                uint16_t selector,
                uint32_t)
{
  if (page != Page::StartupFixed) return;
  memset(&state_metric, 0, sizeof(state_metric));
  memset(row_metrics, 0, sizeof(row_metrics));
  selected_elapsed_ms = selector;
  report_y = 0U;
  report_stage = 0U;
  reporting = false;
  collecting = true;
}

bool reportDone()
{
  return !collecting && !reporting;
}
}  // namespace DashboardTiming

#endif
