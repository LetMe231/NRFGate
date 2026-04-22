#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

#include "semantic_handler.h"
#include "event_ingest.h"
#include "rule_engine.h"
#include "gw_store.h"

LOG_MODULE_REGISTER(semantic_handler, LOG_LEVEL_INF);

/* ── Default Thresholds ────────────────────────────────────── */

static semantic_thresholds_t s_thresholds = {
    .temp_alert_mc          = 35000,
    .temp_critical_mc       = 40000,
    .hum_alert_mpermille    = 800000,
    .hum_critical_mpermille = 900000,
    .tvoc_alert_ppb         = 500,
    .tvoc_critical_ppb      = 1000,
    .eco2_alert_ppm         = 1000,
    .eco2_critical_ppm      = 2000,
    .lost_timeout_ms        = 90000,
    .idle_timeout_ms        = 30000,
};

static K_MUTEX_DEFINE(s_threshold_lock);

/* ── Threshold API ─────────────────────────────────────────── */

void semantic_handler_set_thresholds(const semantic_thresholds_t *t)
{
    if (!t) {
        return;
    }
    k_mutex_lock(&s_threshold_lock, K_FOREVER);
    s_thresholds = *t;
    k_mutex_unlock(&s_threshold_lock);
    LOG_INF("Thresholds updated");
}

void semantic_handler_get_thresholds(semantic_thresholds_t *out)
{
    if (!out) {
        return;
    }
    k_mutex_lock(&s_threshold_lock, K_FOREVER);
    *out = s_thresholds;
    k_mutex_unlock(&s_threshold_lock);
}

/* ── State Calculation ─────────────────────────────────────── */

static gw_state_t calc_state(const gw_sensor_payload_t *s, const semantic_thresholds_t *t)
{
    gw_state_t state = GW_STATE_ACTIVE;

    // Temperature
    if ((s->present & GW_HAS_TEMP) && s->temp_mc >= t->temp_critical_mc) {
        state = GW_STATE_CRITICAL;
    } else if ((s->present & GW_HAS_TEMP) && s->temp_mc >= t->temp_alert_mc) {
        state = GW_STATE_ALERT;
    }

    // Humidity
    if ((s->present & GW_HAS_HUM) && s->hum_mpermille >= t->hum_critical_mpermille) {
        state = GW_STATE_CRITICAL;
    } else if ((s->present & GW_HAS_HUM) && s->hum_mpermille >= t->hum_alert_mpermille) {
        state = GW_STATE_ALERT;
    }

    // TVOC
    if ((s->present & GW_HAS_TVOC) && s->tvoc_ppb >= t->tvoc_critical_ppb) {
        state = GW_STATE_CRITICAL;
    } else if ((s->present & GW_HAS_TVOC) && s->tvoc_ppb >= t->tvoc_alert_ppb) {
        state = GW_STATE_ALERT;
    }

    // eCO2
    if ((s->present & GW_HAS_ECO2) && s->eco2_ppm >= t->eco2_critical_ppm) {
        state = GW_STATE_CRITICAL;
    } else if ((s->present & GW_HAS_ECO2) && s->eco2_ppm >= t->eco2_alert_ppm) {
        state = GW_STATE_ALERT;
    }

    return state;
}

/* ── Event Listener ─────────────────────────────────────────── */

static void on_event(const gw_event_t *evt, void *ctx)
{
    ARG_UNUSED(ctx);
    LOG_INF("Received event: type=%d from node=%d", evt->type, gw_store_find_node(&evt->src));
    // Relay button events to rule engine
    if (evt->type == GW_EVT_BUTTON) {
        LOG_INF("Rule engine switch: pressed=%d node=%d",
            evt->data.button.pressed,
            gw_store_find_node(&evt->src));
        rule_engine_on_switch(&evt->src, evt->data.button.pressed);
        return;
    }
    
    // Only process sensor events
    if (evt->type != GW_EVT_SENSOR) {
        return;
    }

    // Thresholds snapshot under lock
    semantic_thresholds_t t;
    k_mutex_lock(&s_threshold_lock, K_FOREVER);
    t = s_thresholds;
    k_mutex_unlock(&s_threshold_lock);

    // Calculate new state
    gw_state_t new_state = calc_state(&evt->data.sensor, &t);

    // read current state from store
    int idx = gw_store_find_node(&evt->src);
    if (idx < 0) {
        LOG_WRN("Received event from unknown node");
        return;
    }

    gw_node_record_t rec;
    if (!gw_store_get_node_copy((uint8_t)idx, &rec)) {
        LOG_WRN("Failed to get node record");
        return;
    }

    gw_state_t old_state = rec.state;

    // only update if state changed
    if (new_state == old_state) {
        return;
    }

    // write new state to store
    gw_store_set_state(&evt->src, new_state, evt->rx_ms);
    rule_engine_on_state(&evt->src, new_state);

    LOG_INF("Node[%d] state: %d -> %d", idx, old_state, new_state);
}

/* ── Tick - LOST/IDLE Detection ─────────────────────────────────────────── */

void semantic_handler_tick(void)
{
    semantic_thresholds_t t;
    k_mutex_lock(&s_threshold_lock, K_FOREVER);
    t = s_thresholds; // copy under lock
    k_mutex_unlock(&s_threshold_lock);

    int64_t now_ms = k_uptime_get();
    uint8_t count = gw_store_count();

    for (uint8_t i = 0; i < count; i++) {
        gw_node_record_t rec;
        if (!gw_store_get_node_copy(i, &rec)) {
            continue;
        }
        if (!rec.known) {
            continue;
        }
        // Check if already LOST
        if (rec.state == GW_STATE_LOST) {
            continue; // already lost
        }

        int64_t silent_ms = now_ms - rec.last_seen_ms;

        // Check for LOST
        if (silent_ms >= t.lost_timeout_ms) {
            gw_store_set_state(&rec.addr, GW_STATE_LOST, now_ms);
            rule_engine_on_state(&rec.addr, GW_STATE_LOST);
            LOG_INF("Node[%d] marked LOST due to inactivity", i);
            continue;
        }

        // Check for IDLE
        if (rec.state == GW_STATE_ACTIVE && silent_ms >= t.idle_timeout_ms) {
            gw_store_set_state(&rec.addr, GW_STATE_IDLE, now_ms);
            rule_engine_on_state(&rec.addr, GW_STATE_IDLE);
            LOG_INF("Node[%d] marked IDLE due to inactivity", i);
            continue;
        }

        // Recover from IDLE to active
        if (rec.state == GW_STATE_IDLE &&
            silent_ms < t.idle_timeout_ms) {
            LOG_INF("Node[%d] recovered IDLE → ACTIVE", i);
            gw_store_set_state(&rec.addr, GW_STATE_ACTIVE, now_ms);
            rule_engine_on_state(&rec.addr, GW_STATE_ACTIVE);
        }
    }

}

/* ── Init ──────────────────────────────────────────────────── */

void semantic_handler_init(void)
{
    int err = event_ingest_register_listener(on_event, NULL);
    if (err) {
        LOG_ERR("Failed to register event listener: %d", err);
        return;
    }

    LOG_INF("Semantic handler initialized");
    LOG_INF("  Thresholds: temp=%d/%d°C tvoc=%d/%d ppb eco2=%d/%d ppm",
            s_thresholds.temp_alert_mc    / 1000,
            s_thresholds.temp_critical_mc / 1000,
            s_thresholds.tvoc_alert_ppb,
            s_thresholds.tvoc_critical_ppb,
            s_thresholds.eco2_alert_ppm,
            s_thresholds.eco2_critical_ppm);
}