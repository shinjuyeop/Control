#ifndef AUTO_TFT_UI_H
#define AUTO_TFT_UI_H

#include <stddef.h>

#include "dashboard_values.h"

namespace AutoTftUi
{

void begin(const DashboardValues &values,
           const AutoDashboardValues &auto_values,
           bool dashboard_can_lost,
           uint32_t now_ms);
void queueUpdate(const DashboardValues &values,
                 const AutoDashboardValues &auto_values,
                 bool dashboard_can_lost,
                 uint32_t now_ms,
                 bool urgent_change = false);
void service(uint32_t now_ms);
void renderPreparedRows(uint16_t y0, uint16_t y1, uint32_t now_ms);
void synchronize(uint32_t now_ms);
void observe(const AutoDashboardValues &auto_values, uint32_t now_ms);
void resetState();
size_t historyCount();

}  // namespace AutoTftUi

#endif  // AUTO_TFT_UI_H
