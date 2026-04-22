#ifndef SEMANTIC_HANDLER_H
#define SEMANTIC_HANDLER_H

#include "gw_model.h"

/**
 * @brief Initialize the semantic handler.
 *        Registers itself as a listener on event_ingest.
 *        Must be called after event_ingest_init().
 */
void semantic_handler_init(void);

/**
 * @brief Thresholds for state transitions.
 *        Can be adjusted per deployment without recompiling
 *        by calling semantic_handler_set_thresholds().
 */
typedef struct {
    /* Temperature thresholds in milli-°C */
    int32_t temp_alert_mc;      /* default: 35000  = 35.0°C  */
    int32_t temp_critical_mc;   /* default: 40000  = 40.0°C  */

    /* Humidity thresholds in milli-permille */
    int32_t hum_alert_mpermille;    /* default: 800000 = 80.0% */
    int32_t hum_critical_mpermille; /* default: 900000 = 90.0% */

    /* TVOC thresholds in ppb */
    int32_t tvoc_alert_ppb;     /* default: 500  */
    int32_t tvoc_critical_ppb;  /* default: 1000 */

    /* eCO2 thresholds in ppm */
    int32_t eco2_alert_ppm;     /* default: 1000 */
    int32_t eco2_critical_ppm;  /* default: 2000 */

    /* Timeout before node is marked LOST in ms */
    int64_t lost_timeout_ms;    /* default: 90000 = 90s */

    /* Timeout before node is marked IDLE in ms */
    int64_t idle_timeout_ms;    /* default: 10000 = 10s */
} semantic_thresholds_t;

/**
 * @brief Update thresholds at runtime.
 *        Thread-safe.
 *
 * @param t New thresholds — must not be NULL
 */
void semantic_handler_set_thresholds(const semantic_thresholds_t *t);

/**
 * @brief Get current thresholds.
 *
 * @param out Output buffer — must not be NULL
 */
void semantic_handler_get_thresholds(semantic_thresholds_t *out);

/**
 * @brief Periodic tick — checks for LOST/IDLE nodes.
 *        Must be called regularly, e.g. every second from main.
 */
void semantic_handler_tick(void);

#endif /* SEMANTIC_HANDLER_H */