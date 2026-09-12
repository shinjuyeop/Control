"""PC-side metadata matching STM_A/B Core/Src/app_freertos.c task attributes.

Heap reservation matches Core/Inc/FreeRTOSConfig.h, not linker SRAM usage.
Task IDs are kaiev26_msgs/TaskRuntimeStats IDs, not array display positions.
Update this metadata when a future firmware allocation changes.
"""

RUNTIME_STALE_SECONDS = 3.0  # ~1 Hz best-effort diagnostics; no safety gate.
HEAP_RESERVED_BYTES = 65536
# Capacity colors are UI advice only, never firmware safety thresholds.
CAPACITY_WARNING_PERCENT = 80
CAPACITY_DANGER_PERCENT = 95

BOARDS = {
    "/stm_a/runtime_stats": {
        "board_id": 1, "name": "STM_A", "role": "DRIVE / INPUT MCU",
        "tasks": {0: ("Control", 2048), 1: ("Inputs", 1024),
                  2: ("uROS", 6144), 3: ("BRx", 1024),
                  4: ("BTx", 1024), 8: ("PerfMon", 1024)},
    },
    "/stm_b/runtime_stats": {
        "board_id": 2, "name": "STM_B", "role": "SAFETY / BRAKE MCU",
        "tasks": {5: ("Brake", 2048), 6: ("EStop", 1024),
                  2: ("uROS", 5120), 7: ("Display", 2048),
                  3: ("ARx", 1024), 4: ("ATx", 1024), 8: ("PerfMon", 1024)},
    },
}
