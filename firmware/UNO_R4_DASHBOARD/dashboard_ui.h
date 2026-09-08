#ifndef DASHBOARD_UI_H
#define DASHBOARD_UI_H

#include "dashboard_values.h"

namespace DashboardUi
{

void begin(const DashboardValues &values, bool dashboard_can_lost);
void update(const DashboardValues &values, bool dashboard_can_lost);
// State whose drawing calls have completed; used by the read-only HIL report.
const DashboardValues &displayedValues();
bool canLost();
const char *pageName();

}  // namespace DashboardUi

#endif  // DASHBOARD_UI_H
