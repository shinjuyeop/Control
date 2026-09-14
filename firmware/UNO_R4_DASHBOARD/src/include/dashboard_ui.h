#ifndef DASHBOARD_UI_H
#define DASHBOARD_UI_H

#include "dashboard_values.h"

namespace DashboardUi
{

void begin(const DashboardValues &values,
           const AutoDashboardValues &auto_values,
           bool dashboard_can_lost);
void update(const DashboardValues &values,
            const AutoDashboardValues &auto_values,
            bool dashboard_can_lost,
            uint8_t input_alive);
void service(uint32_t now_ms);
void redraw();
void observeAuto(const AutoDashboardValues &auto_values, uint32_t now_ms);
void resetState();
bool isTransitionActive();
bool isAutoDisplayed();
uint16_t transitionBoundary();
const DashboardValues &displayedValues();
const AutoDashboardValues &displayedAutoValues();
bool canLost();
const char *pageName();
void showSwipeTest(char result);

}  // namespace DashboardUi

#endif  // DASHBOARD_UI_H
