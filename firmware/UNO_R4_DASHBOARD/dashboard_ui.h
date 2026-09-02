#ifndef DASHBOARD_UI_H
#define DASHBOARD_UI_H

#include "dashboard_values.h"

namespace DashboardUi
{

void begin(const DashboardValues &values, bool dashboard_can_lost);
void update(const DashboardValues &values, bool dashboard_can_lost);

}  // namespace DashboardUi

#endif  // DASHBOARD_UI_H
