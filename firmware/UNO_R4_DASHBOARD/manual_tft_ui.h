#ifndef MANUAL_TFT_UI_H
#define MANUAL_TFT_UI_H

#include "dashboard_values.h"

namespace ManualTftUi
{

void playStartup();
void begin(const DashboardValues &values, bool dashboard_can_lost);
void update(const DashboardValues &values, bool dashboard_can_lost);

}  // namespace ManualTftUi

#endif  // MANUAL_TFT_UI_H
