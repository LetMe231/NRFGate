#pragma once

/**
 * @file scheduler.h
 * @brief Radio scheduler — time-division multiplexing of BLE Mesh and Thread.
 *
 * The nRF54L15 has a single 2.4 GHz radio shared between BLE Mesh and
 * OpenThread. This module alternates between the two protocols using
 * bt_mesh_suspend() / bt_mesh_resume() on a configurable schedule.
 *
 * Modes:
 *   NORMAL      — alternates BLE/Thread at configured intervals, with
 *                 predictive BLE wake for nodes with long publish periods
 *   BLE_ONLY    — stays in BLE, grants Thread a brief scan every 30 s
 *   THREAD_ONLY — stays in Thread, grants BLE a brief scan every 30 s
 *
 * Priority requests temporarily override the schedule for a given transport.
 * Used by command_router and rule_engine to ensure the radio is in the
 * correct mode before sending a time-sensitive command.
 *
 * Predictive wake:
 *   Call scheduler_on_ble_rx() on every received BLE Mesh packet. The
 *   scheduler tracks each node's send interval and pre-emptively opens the
 *   BLE window before the next expected packet, reducing packet loss for
 *   nodes with long publish periods without sacrificing Thread throughput.
 */

#include <stdint.h>

/* ── Transport priority ────────────────────────────────────── */

typedef enum {
    SCHED_PRIORITY_NONE   = 0, /**< No priority — normal scheduling */
    SCHED_PRIORITY_BLE    = 1, /**< Force BLE Mesh active           */
    SCHED_PRIORITY_THREAD = 2, /**< Force Thread active             */
} sched_priority_t;

/* ── Scheduler mode ────────────────────────────────────────── */

typedef enum {
    SCHED_MODE_NORMAL      = 0, /**< Alternating BLE/Thread with prediction */
    SCHED_MODE_BLE_ONLY    = 1, /**< Mostly BLE, brief Thread scans         */
    SCHED_MODE_THREAD_ONLY = 2, /**< Mostly Thread, brief BLE scans         */
} sched_mode_t;

/* ── Statistics (read-only, updated by scheduler) ──────────── */

extern int64_t  g_ble_active_ms;    /**< Cumulative BLE active time (ms)    */
extern int64_t  g_thread_active_ms; /**< Cumulative Thread active time (ms) */
extern uint32_t g_ble_switches;     /**< Total BLE radio acquisitions        */
extern uint32_t g_thread_switches;  /**< Total Thread radio acquisitions     */

/* ── Public API ────────────────────────────────────────────── */

/**
 * @brief Initialize scheduler state. Call once before scheduler_start().
 */
void scheduler_init(void);

/**
 * @brief Start the alternating radio schedule.
 *        Must be called after BLE Mesh and Thread are both initialized.
 */
void scheduler_start(void);

/**
 * @brief Pause the scheduler and restore BLE Mesh (resume bt_mesh).
 *        Used during provisioning and node configuration.
 */
void scheduler_pause(void);

/**
 * @brief Set the scheduler operating mode.
 * @param mode NORMAL, BLE_ONLY, or THREAD_ONLY.
 */
void scheduler_set_mode(sched_mode_t mode);

/**
 * @brief Adjust the time slots for each transport.
 * @param thread_ms Duration of Thread window in milliseconds (50–2000).
 * @param ble_ms    Duration of BLE Mesh window in milliseconds (50–2000).
 */
void scheduler_set_timing(uint32_t thread_ms, uint32_t ble_ms);

/**
 * @brief Temporarily prioritize one transport for a given duration.
 *        The scheduler switches to the requested transport immediately
 *        and holds it for duration_ms before resuming normal scheduling.
 *
 * @param transport SCHED_PRIORITY_BLE or SCHED_PRIORITY_THREAD.
 * @param duration_ms How long to hold priority in milliseconds.
 */
void scheduler_request_priority(sched_priority_t transport,
                                 uint32_t duration_ms);

/**
 * @brief Block until a Thread radio window becomes available.
 *        Used by the CoAP TX thread before sending a command.
 *
 * @param timeout_ms Maximum time to wait in milliseconds.
 * @return 0 if a Thread window was acquired, -EAGAIN on timeout.
 */
int scheduler_wait_thread_window(uint32_t timeout_ms);

/**
 * @brief Notify the scheduler that a BLE Mesh packet was received.
 *        Updates the per-node send-period estimate used for predictive wake.
 *        Must be called from ble_mesh_adapter on every Sensor Status RX.
 *
 * @param node_idx Store index of the node (from gw_store_find_node()).
 */
void scheduler_on_ble_rx(uint16_t mesh_addr);


bool    scheduler_is_ble_active(void);
int64_t scheduler_ms_since_switch(void);

int scheduler_wait_ble_window(uint32_t timeout_ms);

/* ── Status accessors ──────────────────────────────────────── */

/**
 * @brief Get the current scheduler mode.
 * @return Current sched_mode_t value.
 */
sched_mode_t scheduler_get_mode(void);

/**
 * @brief Get the current scheduler timing parameters.
 * @param out_ble_ms      Output for current BLE slot duration (may be NULL).
 * @param out_thread_ms   Output for current Thread slot duration (may be NULL).
 */
void scheduler_get_timing(uint32_t *out_ble_ms, uint32_t *out_thread_ms);
bool scheduler_is_paused(void);