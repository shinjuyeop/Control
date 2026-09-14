#ifndef MANUAL_TFT_UI_H
#define MANUAL_TFT_UI_H

#include "dashboard_values.h"

namespace ManualTftUi
{

void playStartup();
#if defined(DASHBOARD_BENCH_INPUT) && \
    defined(DASHBOARD_TIMING_STARTUP_PROFILE)
void renderStartupFrameForTiming(uint16_t elapsed_ms);
#endif
void observe(const DashboardValues &values,
             bool dashboard_can_lost,
             uint32_t now_ms);
void begin(const DashboardValues &values, bool dashboard_can_lost);
void update(const DashboardValues &values,
            bool dashboard_can_lost,
            bool urgent_change = false);
void service(uint32_t now_ms);
void renderPreparedRows(uint16_t y0, uint16_t y1);
void synchronize();
void resetState();

}  // namespace ManualTftUi

#endif  // MANUAL_TFT_UI_H
