#ifndef SEMANTIC_HANDLER_H
#define SEMANTIC_HANDLER_H

struct node_sensor_data;  // forward declaration to avoid circular dependency
#include "data_handler.h"

/**
 * @file semantic_handler.h
 * @brief Per-node state machine and alert logic.
 *
 * Responsibilities:
 *   - Track the semantic state of each node (IDLE → ACTIVE → ALERT → CRITICAL)
 *   - Apply hysteresis so alerts don't flicker at threshold boundaries
 *   - Detect sustained conditions (time-windowed alerts)
 *   - Detect node loss (no packet for > NODE_LOST_TIMEOUT_MS)
 *   - Detect rapid trends (rising CO2, sudden motion)
 *   - Request radio scheduler priority when needed
 *
 * NOT responsible for:
 *   - Receiving or parsing packets  → data_handler.c
 *   - Forwarding via BLE/LoRa       → data_handler.c
 *   - Mesh configuration            → ble_mesh_handler.c
 */

/* ── Node semantic states ─────────────────────────────────────── */

typedef enum {
    NODE_STATE_UNKNOWN  = 0,   /* No data received yet              */
    NODE_STATE_IDLE,           /* Values within normal range        */
    NODE_STATE_ACTIVE,         /* Values elevated but not critical  */
    NODE_STATE_ALERT,          /* Threshold crossed, action needed  */
    NODE_STATE_CRITICAL,       /* Multiple thresholds or rapid rise */
    NODE_STATE_LOST,           /* No packet for NODE_LOST_TIMEOUT_MS */
} node_state_t;

/* ── Threshold configuration ─────────────────────────────────── */

/* Environmental — can be overridden at runtime via semantic_handler_set_thresholds() */
#define THRESH_CO2_ACTIVE    1000   /* ppm  — elevated  */
extern int32_t thresh_co2_alert;   /* ppm  — alert     */
#define THRESH_CO2_CRITICAL  5000   /* ppm  — critical  */
#define THRESH_CO2_HYST       200   /* ppm  — hysteresis band */

#define THRESH_TVOC_ACTIVE    150   /* ppb */
extern int32_t thresh_tvoc_alert;   /* ppb */
#define THRESH_TVOC_HYST       50   /* ppb */

extern int32_t thresh_temp_alert;   /* m°C = 38.0 °C */
#define THRESH_TEMP_HYST     1000   /* m°C = 1.0 °C  */

/* IMU — magnitude² in (mg/100)² to avoid sqrt */
extern int32_t thresh_motion_active;   /* ~2g  */
extern int32_t thresh_motion_alert;   /* ~10g */
#define THRESH_MOTION_HYST     100

/* Trend: alert if metric rises by this much within the window */
#define TREND_CO2_RISE_PPM    300   /* ppm per window */
#define TREND_WINDOW_MS      5000   /* ms              */

/* Node loss timeout */
extern int32_t node_lost_timeout_ms;   /* 30 s */

/* Minimum time a condition must persist before triggering alert */
#define SUSTAINED_ALERT_MS    5000   /* 5 s */


/* ── Public API ───────────────────────────────────────────────── */

int  semantic_handler_init(void);

/**
 * @brief Process one received sensor packet.
 * Called by data_handler_receive() after logging/forwarding.
 */
void semantic_handler_process(const struct node_sensor_data *d);

/**
 * @brief Periodic tick — call from a timer or work queue.
 * Checks for node loss and clears expired transient states.
 * Recommended interval: 1–5 seconds.
 */
void semantic_handler_tick(void);

/**
 * @brief Get the current semantic state of a node by index.
 * Returns NODE_STATE_UNKNOWN if the index is out of range.
 */
node_state_t semantic_handler_get_state(uint8_t node_idx);

/**
 * @brief Return a human-readable string for a state value.
 */
const char *semantic_handler_state_str(node_state_t state);

void semantic_handler_set_policy(int32_t motion_active, int32_t motion_alert, int32_t co2_alert, int32_t temp_alert, int32_t tvoc_alert);

#endif /* SEMANTIC_HANDLER_H */