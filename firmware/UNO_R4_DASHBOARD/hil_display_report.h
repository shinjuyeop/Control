#ifndef HIL_DISPLAY_REPORT_H
#define HIL_DISPLAY_REPORT_H
#include <stdint.h>
namespace HilDisplayReport {
bool enabled();
void poll(uint32_t now_ms, uint32_t snapshots, int32_t can_age_ms);
}
#endif
