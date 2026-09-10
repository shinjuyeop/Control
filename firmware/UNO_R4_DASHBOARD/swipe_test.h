#ifndef SWIPE_TEST_H
#define SWIPE_TEST_H
#include "touch_navigation.h"
namespace SwipeTest
{
void setEnabled(bool enabled);
void poll(uint32_t now_ms);
void virtualTouch(const TouchNavigation::TouchSample &sample, uint32_t now_ms);
void cancelVirtual();
}
#endif
