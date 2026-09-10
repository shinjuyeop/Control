#ifndef DASHBOARD_TIMING_H
#define DASHBOARD_TIMING_H

#include <Arduino.h>

namespace DashboardTiming
{

enum class Page : uint8_t
{
  Startup,
  Manual,
  Auto,
  Transport,
  StartupFixed
};

#if defined(DASHBOARD_HOST) || defined(DASHBOARD_TIMING_PROFILE) || \
    defined(DASHBOARD_TIMING_TRANSPORT_PROFILE) || \
    defined(DASHBOARD_TIMING_STARTUP_PROFILE)
uint32_t beginMeasure();
#else
inline uint32_t beginMeasure() { return 0U; }
#endif

#if defined(DASHBOARD_HOST) || defined(DASHBOARD_TIMING_PROFILE) || \
    defined(DASHBOARD_TIMING_STARTUP_PROFILE)
void statePrepared(uint32_t started_us, Page page);
void rowComposed(uint32_t started_us, Page page, uint16_t y);
#else
inline void statePrepared(uint32_t, Page) {}
inline void rowComposed(uint32_t, Page, uint16_t) {}
#endif

#if defined(DASHBOARD_HOST) || defined(DASHBOARD_TIMING_PROFILE) || \
    defined(DASHBOARD_TIMING_TRANSPORT_PROFILE)
void transportTransactionBegin(uint32_t started_us);
void transportTransactionEnd(uint32_t started_us);
void transportRgb565Pack(uint32_t started_us, uint16_t pixel_count);
void transportSolidPack(uint32_t started_us, uint16_t pixel_count);
void transportSpiByte(uint32_t started_us);
void transportSpiBuffer(uint32_t started_us, uint16_t bytes);
#else
inline void transportTransactionBegin(uint32_t) {}
inline void transportTransactionEnd(uint32_t) {}
inline void transportRgb565Pack(uint32_t, uint16_t) {}
inline void transportSolidPack(uint32_t, uint16_t) {}
inline void transportSpiByte(uint32_t) {}
inline void transportSpiBuffer(uint32_t, uint16_t) {}
#endif

#if defined(DASHBOARD_HOST) || defined(DASHBOARD_TIMING_PROFILE) || \
    defined(DASHBOARD_TIMING_EVENTS_ONLY) || \
    defined(DASHBOARD_TIMING_TRANSPORT_PROFILE) || \
    defined(DASHBOARD_TIMING_STARTUP_PROFILE)
void startupEvent(bool done, uint32_t time_us);
void startupFrameEvent(bool done, uint16_t elapsed_ms, uint32_t time_us);
void regionCompleted(Page page, uint16_t region, uint32_t completed_ms);
void modeEvent(bool done,
               bool direction_auto,
               uint16_t boundary,
               uint8_t alive,
               uint8_t flags,
               uint32_t time_ms);
void beginSegment(uint32_t now_ms, Page page);
void endSegment();
void serviceReport(uint32_t now_ms);
void beginBench(Page page,
                uint32_t duration_ms,
                uint16_t region_mask,
                uint32_t now_ms);
bool reportDone();
#else
inline void startupEvent(bool, uint32_t) {}
inline void startupFrameEvent(bool, uint16_t, uint32_t) {}
inline void regionCompleted(Page, uint16_t, uint32_t) {}
inline void modeEvent(bool, bool, uint16_t, uint8_t, uint8_t, uint32_t) {}
inline void beginSegment(uint32_t, Page) {}
inline void endSegment() {}
inline void serviceReport(uint32_t) {}
inline void beginBench(Page, uint32_t, uint16_t, uint32_t) {}
inline bool reportDone() { return true; }
#endif

#if defined(DASHBOARD_HOST) || defined(DASHBOARD_TIMING_PROFILE)
void serviceCompleted(uint32_t started_us);
class ServiceScope
{
 public:
  ServiceScope() : started_us_(beginMeasure()) {}
  ~ServiceScope() { serviceCompleted(started_us_); }
 private:
  uint32_t started_us_;
};
#else
inline void serviceCompleted(uint32_t) {}
class ServiceScope {};
#endif

}  // namespace DashboardTiming

#endif  // DASHBOARD_TIMING_H
