#ifndef MAIN_H
#define MAIN_H

#include <stdint.h>
#include <stdbool.h>

typedef enum {
    SCHED_PRIORITY_NONE   = 0,
    SCHED_PRIORITY_BLE    = 1,
    SCHED_PRIORITY_THREAD = 2,
} sched_priority_t;

typedef enum {
    SCHED_MODE_NORMAL      = 0,
    SCHED_MODE_BLE_ONLY    = 1,
    SCHED_MODE_THREAD_ONLY = 2,
} sched_mode_t;

void mesh_scheduler_request_priority(sched_priority_t transport, uint32_t duration_ms);
void mesh_scheduler_set_mode(sched_mode_t mode);
void mesh_scheduler_pause(void);
void mesh_scheduler_start(void);

#endif /* MAIN_H */