#include <atomic>
#include <stdint.h>
#include <exception>
#include <thread>

#include "host_runtime.h"
#include "auto_tft_ui.h"
#include "dashboard_ui.h"
#include "dashboard_timing.h"
#include "manual_tft_ui.h"

void setup();
void loop();

namespace
{

std::thread firmware_thread;
std::atomic<bool> thread_running(false);
std::atomic<bool> replay_startup(false);
std::atomic<bool> replay_active(false);
std::atomic<bool> published_transition_active(false);
std::atomic<bool> published_auto_displayed(false);
std::atomic<uint16_t> published_transition_boundary(320U);

void publishUiState()
{
  published_transition_active.store(DashboardUi::isTransitionActive());
  published_auto_displayed.store(DashboardUi::isAutoDisplayed());
  published_transition_boundary.store(DashboardUi::transitionBoundary());
}

void runFirmware()
{
  thread_running.store(true);
  try
  {
    setup();
    publishUiState();
    DashboardHostRuntime::setReady();
    while (!DashboardHostRuntime::stopRequested())
    {
      if (replay_startup.exchange(false))
      {
        replay_active.store(true);
        ManualTftUi::playStartup();
        DashboardTiming::regionCompleted(
            DashboardTiming::Page::Startup, 1U, millis());
        DashboardUi::redraw();
        publishUiState();
        replay_active.store(false);
      }
      loop();
      publishUiState();
      DashboardHostRuntime::idleLoop();
    }
  }
  catch (const DashboardHostRuntime::StopException &)
  {
  }
  catch (const std::exception &error)
  {
    DashboardHostRuntime::setError(error.what());
  }
  catch (...)
  {
    DashboardHostRuntime::setError("unknown native firmware exception");
  }
  thread_running.store(false);
}

}  // namespace

extern "C"
{

void dashboard_host_publish_ui_state()
{
  publishUiState();
}

void dashboard_start()
{
  if (firmware_thread.joinable()) return;
  DashboardHostRuntime::reset();
  firmware_thread = std::thread(runFirmware);
}

void dashboard_shutdown()
{
  DashboardHostRuntime::requestStop();
  if (firmware_thread.joinable()) firmware_thread.join();
}

int dashboard_run_until(uint64_t time_us)
{
  DashboardHostRuntime::setAllowedTimeUs(time_us);
  return DashboardHostRuntime::waitUntilUs(time_us) ? 0 : -1;
}

const char *dashboard_last_error()
{
  return DashboardHostRuntime::lastError();
}

uint64_t dashboard_now_us()
{
  return DashboardHostRuntime::nowUs();
}

int dashboard_ready()
{
  return DashboardHostRuntime::ready() ? 1 : 0;
}

void dashboard_copy_framebuffer(uint16_t *destination)
{
  DashboardHostRuntime::copyFramebuffer(destination);
}

void dashboard_schedule_can(uint64_t arrival_us,
                            uint32_t identifier,
                            const uint8_t *payload)
{
  DashboardHostRuntime::scheduleCan(arrival_us, identifier, payload);
}

void dashboard_set_timing_profile(const uint32_t *state_us,
                                  const uint32_t *row_us,
                                  uint32_t spi_call_us,
                                  uint32_t loop_us)
{
  DashboardHostRuntime::setTimingProfile(
      state_us, reinterpret_cast<const uint32_t (*)[320]>(row_us),
      spi_call_us, loop_us);
}

void dashboard_set_transport_profile(
    uint32_t transaction_begin_ns,
    uint32_t transaction_end_ns,
    uint32_t spi_byte_extra_ns,
    const uint32_t *spi_buffer_extra_ns,
    const uint32_t *rgb565_pack_ns,
    const uint32_t *solid_pack_ns)
{
  DashboardHostRuntime::setTransportProfile(
      transaction_begin_ns, transaction_end_ns, spi_byte_extra_ns,
      spi_buffer_extra_ns, rgb565_pack_ns, solid_pack_ns);
}

void dashboard_set_startup_profile(const uint16_t *elapsed_ms,
                                   const uint32_t *state_us,
                                   const uint32_t *row_us,
                                   uint16_t count)
{
  DashboardHostRuntime::setStartupProfile(
      elapsed_ms, state_us, row_us, count);
}

void dashboard_replay_startup()
{
  replay_startup.store(true);
}

int dashboard_replay_active()
{
  return replay_startup.load() || replay_active.load() ? 1 : 0;
}

int dashboard_transition_active()
{
  return published_transition_active.load() ? 1 : 0;
}

int dashboard_auto_displayed()
{
  return published_auto_displayed.load() ? 1 : 0;
}

uint16_t dashboard_transition_boundary()
{
  return published_transition_boundary.load();
}

uint32_t dashboard_auto_history_count()
{
  return (uint32_t)AutoTftUi::historyCount();
}

uint32_t dashboard_startup_start_us()
{
  return DashboardHostRuntime::startupStartUs();
}

uint32_t dashboard_startup_completed_us()
{
  return DashboardHostRuntime::startupCompletedUs();
}

void dashboard_clear_events(uint16_t region_mask)
{
  DashboardHostRuntime::clearEvents(region_mask);
}

uint32_t dashboard_copy_events(DashboardHostRuntime::Event *destination,
                               uint32_t capacity)
{
  return (uint32_t)DashboardHostRuntime::copyEvents(destination, capacity);
}

uint32_t dashboard_copy_startup_frames(
    DashboardHostRuntime::StartupFrameEvent *destination,
    uint32_t capacity)
{
  return (uint32_t)DashboardHostRuntime::copyStartupFrames(
      destination, capacity);
}

}  // extern "C"
