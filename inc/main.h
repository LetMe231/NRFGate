#ifndef MAIN_H
#define MAIN_H

#include <zephyr/kernel.h>

/**
 * @brief Radio scheduler priority transport selection.
 *
 * When a priority window is active the scheduler gives the radio
 * exclusively to one transport for the requested duration, then
 * returns to normal alternating operation automatically.
 */
typedef enum {
    SCHED_PRIORITY_NONE,
    SCHED_PRIORITY_BLE,     /**< BLE Mesh gets radio exclusively */
    SCHED_PRIORITY_THREAD,  /**< Thread gets radio exclusively   */
} sched_priority_t;

typedef enum {
    SCHED_MODE_NORMAL,     /**< Normal alternating operation */
    SCHED_MODE_BLE_ONLY,  /**< BLE Mesh only (Thread suspended) */
    SCHED_MODE_THREAD_ONLY /**< Thread only (BLE Mesh suspended) */
} sched_mode_t;

void mesh_scheduler_pause(void);
void mesh_scheduler_start(void);
void mesh_scheduler_set_timing(uint32_t thread_ms, uint32_t ble_ms);
void mesh_scheduler_set_mode(sched_mode_t mode);

// Testing
void mesh_scheduler_suppress_reports(bool suppress);

extern uint32_t sched_thread_ms;
extern uint32_t sched_ble_ms;
extern sched_mode_t sched_mode;
extern int64_t g_last_mesh_rx_ms;
/**
 * @brief Request priority radio access for one transport.
 *
 * Safe to call from any thread or work handler — uses atomics only.
 * If a priority window is already active this call extends or
 * replaces it.
 *
 * @param transport   Which transport should get the radio
 * @param duration_ms How long to hold priority (ms)
 */
void mesh_scheduler_request_priority(sched_priority_t transport,
                                     uint32_t duration_ms);

int mesh_scheduler_wait_thread_window(uint32_t timeout_ms);
#endif /* MAIN_H */