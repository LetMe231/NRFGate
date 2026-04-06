/**
 * @file semantic_handler.c
 * @brief Per-node semantic state machine.
 */

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <inttypes.h>
#include <stdlib.h>
#include <string.h>

#include "semantic_handler.h"
#include "main.h"
#include "ble_nus.h"
#include "rule_engine.h"

LOG_MODULE_REGISTER(semantic_handler, LOG_LEVEL_INF);


// Runtime mutable thresholds
int32_t thresh_co2_active    = 1000;
int32_t thresh_co2_alert     = 2000;
int32_t thresh_co2_critical  = 5000;
int32_t thresh_tvoc_alert    = 500;
int32_t thresh_motion_active = 400;
int32_t thresh_motion_alert  = 10000;
int32_t thresh_temp_alert    = 38000;  /* m°C = 38.0 °C */
int32_t node_lost_timeout_ms = 30000;
 
// per-node state machine

typedef struct{
    node_state_t state;
    int64_t last_rx_ms;
    int64_t alert_since_ms;
    bool lost_reported;

    struct {
        int64_t ts_ms;
        int32_t value;
    } co2_hist[4]; // for trend detection, simple circular buffer of recent CO2 readings
    uint8_t co2_hist_idx;

    // last known values for hyst
    int32_t last_co2;
    int32_t last_tvoc;
    int32_t last_temp;
    int32_t last_hum;
    int32_t last_mag2;
} semantic_node_t;
static semantic_node_t node_states[MAX_NODES];

// Helpers

static int32_t mag2(int32_t ax, int32_t ay, int32_t az){
    int32_t x = ax/100, y = ay/100, z = az/100;
    return x * x + y * y + z * z;
}

static const char *state_str(node_state_t s){
    switch(s){
        case NODE_STATE_UNKNOWN: return "UNKNOWN";
        case NODE_STATE_IDLE:    return "IDLE";
        case NODE_STATE_ACTIVE:  return "ACTIVE";
        case NODE_STATE_ALERT:   return "ALERT";
        case NODE_STATE_CRITICAL:return "CRITICAL";
        case NODE_STATE_LOST:    return "LOST";
        default: return "?";
    }
}

static void transition(semantic_node_t *n, uint8_t idx,
                       node_state_t new_state, const struct node_sensor_data *d)
{
    if (n->state == new_state) return;

    LOG_INF("[Node %d] %s -> %s", idx, state_str(n->state), state_str(new_state));
    n->state = new_state;

    if (new_state == NODE_STATE_ALERT || new_state == NODE_STATE_CRITICAL) {
        n->alert_since_ms = k_uptime_get();
        if (d) mesh_scheduler_request_priority(d->identity.transport, 10000);
    }

    node_transport_t transport = (d != NULL) ? d->identity.transport : NODE_TRANSPORT_THREAD; // default to Thread if no data
    rule_engine_on_state(idx, new_state, transport);
}

static bool co2_rising_fast(semantic_node_t *n, int32_t current, int64_t now_ms){
    uint8_t idx = n->co2_hist_idx;

    // Store current reading in history
    n->co2_hist[idx].ts_ms = now_ms;
    n->co2_hist[idx].value = current;
    n->co2_hist_idx = (idx + 1) % ARRAY_SIZE(n->co2_hist);

    // find oldest reading within trend window
    for(uint8_t i=0; i<ARRAY_SIZE(n->co2_hist); i++){
        if(n->co2_hist[i].ts_ms == 0) continue;
        if(now_ms - n->co2_hist[i].ts_ms > TREND_WINDOW_MS) continue;
        if(current - n->co2_hist[i].value >= TREND_CO2_RISE_PPM){
            LOG_INF("  CO2 rising fast: %d ppm in %lld ms",
                    current - n->co2_hist[i].value, now_ms - n->co2_hist[i].ts_ms);
            return true;
        }
    }
    return false;
}

// pre-metric state evaluation

static bool above_with_hyst(int32_t current, int32_t last, int32_t thresh, int32_t hyst){
    if(current >= thresh) return true;
    if(last > thresh && current > thresh - hyst) return true;
    return false;
}

// node state machine evaluation

static node_state_t evaluate_env(semantic_node_t *n, const struct sensor_payload *p, int64_t now_ms){
    node_state_t worst = NODE_STATE_IDLE;

    // CO2
    if(p->present & SENSOR_HAS_ECO2) {
        bool fast_rise = co2_rising_fast(n, p->eco2, now_ms);

        if (fast_rise || above_with_hyst(p->eco2, n->last_co2,
                                 thresh_co2_critical, THRESH_CO2_HYST)) {
            worst = MAX(worst, NODE_STATE_CRITICAL);
        } else if (above_with_hyst(p->eco2, n->last_co2,
                                thresh_co2_alert, THRESH_CO2_HYST)) {
            worst = MAX(worst, NODE_STATE_ALERT);
        } else if (above_with_hyst(p->eco2, n->last_co2,
                                thresh_co2_active, THRESH_CO2_HYST)) {
            worst = MAX(worst, NODE_STATE_ACTIVE);
        }
    }

    // TVOC
    if(p->present & SENSOR_HAS_TVOC) {
        if(above_with_hyst(p->tvoc, n->last_tvoc, thresh_tvoc_alert, THRESH_TVOC_HYST)){
            worst = MAX(worst, NODE_STATE_ALERT);
        }
        else if(above_with_hyst(p->tvoc, n->last_tvoc, THRESH_TVOC_ACTIVE, THRESH_TVOC_HYST)){
            worst = MAX(worst, NODE_STATE_ACTIVE);
        }
        n->last_tvoc = p->tvoc;
    }

    // Temperature
    if(p->present & SENSOR_HAS_TEMP) {
        if(above_with_hyst(p->temp, n->last_temp, thresh_temp_alert, THRESH_TEMP_HYST)){
            worst = MAX(worst, NODE_STATE_ALERT);
        }
        n->last_temp = p->temp;
    }


    // PM2.5: >25 µg/m³ = ACTIVE, >75 = ALERT
    // in evaluate_env():
    if (p->present & SENSOR_HAS_PM25) {
        if (p->pm25 >= 75)      worst = MAX(worst, NODE_STATE_ALERT);
        else if (p->pm25 >= 25) worst = MAX(worst, NODE_STATE_ACTIVE);
    }
    if (p->present & SENSOR_HAS_PM10) {
        if (p->pm10 >= 150)     worst = MAX(worst, NODE_STATE_ALERT);
        else if (p->pm10 >= 50) worst = MAX(worst, NODE_STATE_ACTIVE);
    }
    return worst;


}

static node_state_t evaluate_imu(semantic_node_t *n,
                                  const struct sensor_payload *p)
{
    if ((p->present & SENSOR_HAS_ACCEL) != SENSOR_HAS_ACCEL) {
        return NODE_STATE_IDLE;
    }
 
    int32_t m2 = mag2(p->ax, p->ay, p->az);
 
    node_state_t s = NODE_STATE_IDLE;
    if (above_with_hyst(m2, n->last_mag2,
                        thresh_motion_alert, THRESH_MOTION_HYST)) {
        s = NODE_STATE_ALERT;
        /* For motion alerts give Thread priority so the
         * fast-moving IMU node can push its data through */
        mesh_scheduler_request_priority(SCHED_PRIORITY_THREAD, 5000);
    } else if (above_with_hyst(m2, n->last_mag2,
                               thresh_motion_active, THRESH_MOTION_HYST)) {
        s = NODE_STATE_ACTIVE;
    }
 
    n->last_mag2 = m2;
    return s;
}

static node_state_t apply_sustain(semantic_node_t *n, node_state_t candidate, int64_t now_ms){
    if(candidate == NODE_STATE_ACTIVE){
        // Only transition to ACTIVE if condition sustained for SUSTAINED_ALERT_MS
        if(n->alert_since_ms == 0){
            n->alert_since_ms = now_ms;
            return n->state; // stay in current state until sustained
        }
        if(now_ms - n->alert_since_ms < SUSTAINED_ALERT_MS){
            return n->state; // still waiting for sustained condition
        }
    } 
    else if(candidate == NODE_STATE_IDLE){
            n->alert_since_ms = 0; // reset timer if condition cleared
    }
    return candidate;
}

// public API

int semantic_handler_init(void){
    memset(node_states, 0, sizeof(node_states));
    LOG_INF("Semantic handler initialized");
    return 0;
}

void semantic_handler_process(const struct node_sensor_data *d){
    if(d->node_idx >= MAX_NODES) {
        LOG_WRN("Invalid node index %d", d->node_idx);
        return;
    }

    semantic_node_t *n = &node_states[d->node_idx];
    const struct sensor_payload *p = &d->payload;
    int64_t now_ms = d->rx_uptime_ms;

    n->last_rx_ms = now_ms;
    n->lost_reported = false;

    // first packet -> enter idle
    if(n->state == NODE_STATE_UNKNOWN || n->state == NODE_STATE_LOST){
        transition(n, d->node_idx, NODE_STATE_IDLE, d);
    }

    // evaluate metrics and determine worst state
    node_state_t candidate = NODE_STATE_IDLE;
    candidate = MAX(candidate, evaluate_env(n, p, now_ms));
    candidate = MAX(candidate, evaluate_imu(n, p));

    // apply sustained alert logic
    candidate = apply_sustain(n, candidate, now_ms);

    // transition if needed
    transition(n, d->node_idx, candidate, d);
}

void semantic_handler_tick(void){
    int64_t now_ms = k_uptime_get();

    for(uint8_t i=0; i<MAX_NODES; i++){
        semantic_node_t *n = &node_states[i];
        if(n->state == NODE_STATE_UNKNOWN) continue;
        if(n->last_rx_ms == 0) continue; // should not happen, but guard against uninitialized

        int64_t silence = now_ms - n->last_rx_ms;
        
        if(silence >= node_lost_timeout_ms && !n->lost_reported){
            transition(n, i, NODE_STATE_LOST, NULL);
            n->lost_reported = true;
            n->alert_since_ms = 0; // reset sustained alert timer
            
            if(ble_nus_is_ready()){
                char alert_msg[48];
                snprintf(alert_msg, sizeof(alert_msg), "{\"node\":%d,\"state\":\"LOST\"}\n", i);
                ble_nus_send(alert_msg);
            }
        }
    }
}

node_state_t semantic_handler_get_state(uint8_t node_idx){
    if(node_idx >= MAX_NODES) return NODE_STATE_UNKNOWN;
    return node_states[node_idx].state;
}

const char *semantic_handler_state_str(node_state_t s){
    return state_str(s);
}

void semantic_handler_set_policy(int32_t motion_active, int32_t motion_alert, int32_t co2_alert, int32_t temp_alert, int32_t tvoc_alert){
    if(motion_active >= 0){thresh_motion_active = motion_active;}
    if(motion_alert >= 0){thresh_motion_alert = motion_alert;}
    if(co2_alert >= 0){thresh_co2_alert = co2_alert;}
    if(temp_alert >= 0){thresh_temp_alert = temp_alert;}
    if(tvoc_alert >= 0){thresh_tvoc_alert = tvoc_alert;}
}