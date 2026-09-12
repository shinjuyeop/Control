#ifndef RUNTIME_STATS_ROS_H
#define RUNTIME_STATS_ROS_H

#include "runtime_metrics.h"
#include <rcl/rcl.h>
#include <kaiev26_msgs/msg/runtime_stats.h>

/* Static, owned exclusively by each board's existing uROS task. */
typedef struct {
  rcl_publisher_t publisher;
  bool initialized;
  uint8_t board_id;
  uint32_t last_attempt_ms;
  RuntimeMetricsSnapshot snapshot;
  kaiev26_msgs__msg__RuntimeStats message;
} RuntimeStatsRos;

void RuntimeStatsRos_Init(RuntimeStatsRos *state, rcl_node_t *node, uint8_t board_id);
void RuntimeStatsRos_Fini(RuntimeStatsRos *state, rcl_node_t *node);
void RuntimeStatsRos_PublishIfDue(RuntimeStatsRos *state);

#endif
