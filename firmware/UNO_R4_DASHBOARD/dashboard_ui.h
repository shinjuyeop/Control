#ifndef DASHBOARD_UI_H
#define DASHBOARD_UI_H

#include "dashboard_values.h"

namespace DashboardUi
{

void begin(const DashboardValues &values, bool dashboard_can_lost);
void update(const DashboardValues &values, bool dashboard_can_lost);
// Applied dashboard state, or latest received values while the swipe test is shown.
// pageName() identifies the actual screen for the read-only HIL report.
const DashboardValues &displayedValues();
bool canLost();
const char *pageName();
// '\0' restores the dashboard; '-' waits for a swipe; 'L'/'R' show its result.
void showSwipeTest(char result);

}  // namespace DashboardUi

#endif  // DASHBOARD_UI_H
