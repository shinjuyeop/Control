#ifndef DASHBOARD_SERIAL_H
#define DASHBOARD_SERIAL_H

#include <Arduino.h>

namespace DashboardSerial
{

static constexpr size_t MESSAGE_SIZE = 112U;

bool enqueue(const char *message);
bool hasCapacity();
bool idle();
void service();

}  // namespace DashboardSerial

#endif  // DASHBOARD_SERIAL_H
