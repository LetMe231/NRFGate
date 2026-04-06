/**
 * @file rule_engine.c
 * @brief Cross-protocol rule engine.
 *
 * HOW IT WORKS:
 *   1. semantic_handler calls rule_engine_on_state() on every transition
 *   2. data_handler calls rule_engine_on_switch() when sw field is present
 *   3. We iterate the rule table and fire matching rules
 *   4. Actions are dispatched to mesh_ctrl (BLE) or thread_ctrl (Thread)
 *
 * THREAD SAFETY:
 *   Rules are modified from the BLE NUS work queue (data_handler_cmd).
 *   Rules are evaluated from the semantic handler work queue.
 *   We use a simple mutex to protect the table.
 */

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <string.h>
#include <stdio.h>

#include "rule_engine.h"
#include "mesh_ctrl.h"
#include "thread_handler.h"
#include "ble_nus.h"
#include "main.h"
#include "data_handler.h"

LOG_MODULE_REGISTER(rule_engine, LOG_LEVEL_INF);

/* ── Rule table ──────────────────────────────────────────────── */
static gateway_rule_t s_rules[RULE_MAX];
static K_MUTEX_DEFINE(s_mutex);

/* ── Thread CoAP actuator ────────────────────────────────────── */
/* Sends a CoAP PUT to coap://[ipv6]:5683/light with {"on":true/false} */
extern int thread_handler_coap_put_light(const char *ipv6, bool on);


/* Source-transport for changeback after Rule-Fire */
static node_transport_t s_last_src_transport = NODE_TRANSPORT_THREAD;

static void restore_src_priority_fn(struct k_work *w)
{
    sched_priority_t p = (s_last_src_transport == NODE_TRANSPORT_THREAD)
                       ? SCHED_PRIORITY_THREAD : SCHED_PRIORITY_BLE;
    mesh_scheduler_request_priority(p, 5000);
}
static K_WORK_DELAYABLE_DEFINE(restore_src_work, restore_src_priority_fn);

static void fire(const gateway_rule_t *r, node_transport_t src_transport)
{
    s_last_src_transport = src_transport;

    bool light_on = false;
    bool update_cache = true;

    if (r->target_is_thread) {
        mesh_scheduler_request_priority(SCHED_PRIORITY_THREAD, 1000);
        if (r->action == RULE_ACT_THREAD_TOGGLE) {
            light_on = !(r->src_node_idx < MAX_NODES &&
                        node_actuator_state[r->src_node_idx].light_on);
        } else {
            light_on = (r->action == RULE_ACT_THREAD_ON);
        }
        LOG_INF("Rule -> Thread %s [%s]", light_on ? "ON" : "OFF", r->target.ipv6);
        thread_handler_coap_put_light(r->target.ipv6, light_on);
    } else {
        mesh_scheduler_request_priority(SCHED_PRIORITY_BLE, 1000);
        switch (r->action) {
        case RULE_ACT_MESH_ON:     
            light_on = true;
            mesh_ctrl_set_onoff(r->target.mesh_addr, true);  
            break;
        case RULE_ACT_MESH_OFF:    
            light_on = false;
            mesh_ctrl_set_onoff(r->target.mesh_addr, false); 
            break;
        case RULE_ACT_MESH_TOGGLE: 
            light_on = !(r->src_node_idx < MAX_NODES && node_actuator_state[r->src_node_idx].light_on);
            mesh_ctrl_toggle(r->target.mesh_addr);           
            break;
        default: 
            update_cache = false;
            break;
        }
        LOG_INF("Rule -> Mesh %s [0x%04X]", r->action == RULE_ACT_MESH_TOGGLE ? "TOGGLE" : light_on ? "ON" : "OFF", r->target.mesh_addr);
    }

    // Update actuator cache
    if (update_cache && r->src_node_idx < MAX_NODES) {
       node_actuator_state[r->src_node_idx].known = true;
       node_actuator_state[r->src_node_idx].light_on = light_on;
    }
    /* Nach 1000ms zurück auf Source-Transport für Status-Updates */
    k_work_reschedule(&restore_src_work, K_MSEC(1000));
}

/* ── Internal: check trigger against state ───────────────────── */
static bool trigger_matches_state(rule_trigger_t trig, node_state_t state)
{
    switch (trig) {
    case RULE_TRIG_STATE_ACTIVE:   return state == NODE_STATE_ACTIVE;
    case RULE_TRIG_STATE_ALERT:    return state == NODE_STATE_ALERT
                                       || state == NODE_STATE_CRITICAL;  // ← escalation
    case RULE_TRIG_STATE_CRITICAL: return state == NODE_STATE_CRITICAL;
    case RULE_TRIG_STATE_IDLE:     return state == NODE_STATE_IDLE;
    case RULE_TRIG_STATE_LOST:     return state == NODE_STATE_LOST;
    default: return false;
    }
}

/* ── Public API ──────────────────────────────────────────────── */

void rule_engine_init(void)
{
    memset(s_rules, 0, sizeof(s_rules));
    LOG_INF("Rule engine init (max %d rules)", RULE_MAX);
}

void rule_engine_on_state(uint8_t node_idx, node_state_t new_state, node_transport_t src_transport)
{
    k_mutex_lock(&s_mutex, K_FOREVER);
    for (int i = 0; i < RULE_MAX; i++) {
        if (!s_rules[i].active) continue;
        if (s_rules[i].src_node_idx != node_idx) continue;
        if (!trigger_matches_state(s_rules[i].trigger, new_state)) continue;
        LOG_INF("Rule[%d] fired: node=%d state=%d", i, node_idx, new_state);
        fire(&s_rules[i], src_transport);
    }
    k_mutex_unlock(&s_mutex);
}

void rule_engine_on_switch(uint8_t node_idx, bool sw_on, node_transport_t src_transport)
{
    rule_trigger_t trig = sw_on ? RULE_TRIG_SWITCH_ON : RULE_TRIG_SWITCH_OFF;
    k_mutex_lock(&s_mutex, K_FOREVER);
    for (int i = 0; i < RULE_MAX; i++) {
        if (!s_rules[i].active) continue;
        if (s_rules[i].src_node_idx != node_idx) continue;
        if (s_rules[i].trigger != trig) continue;
        LOG_INF("Rule[%d] fired: node=%d switch=%s", i, node_idx,
                sw_on ? "ON" : "OFF");
        fire(&s_rules[i], src_transport);
    }
    k_mutex_unlock(&s_mutex);
}

int rule_engine_add(const gateway_rule_t *rule)
{
    k_mutex_lock(&s_mutex, K_FOREVER);
    for (int i = 0; i < RULE_MAX; i++) {
        if (!s_rules[i].active) {
            s_rules[i] = *rule;
            s_rules[i].active = true;
            k_mutex_unlock(&s_mutex);
            LOG_INF("Rule[%d] added: node=%d trig=%d → %s %s",
                    i, rule->src_node_idx, rule->trigger,
                    rule->target_is_thread ? "Thread" : "Mesh",
                    rule->target_is_thread ? rule->target.ipv6
                    : "addr");
            return i;
        }
    }
    k_mutex_unlock(&s_mutex);
    LOG_WRN("Rule table full");
    return -1;
}

void rule_engine_remove(uint8_t idx)
{
    if (idx >= RULE_MAX) return;
    k_mutex_lock(&s_mutex, K_FOREVER);
    memset(&s_rules[idx], 0, sizeof(s_rules[idx]));
    k_mutex_unlock(&s_mutex);
    LOG_INF("Rule[%d] removed", idx);
}

const gateway_rule_t *rule_engine_get(uint8_t idx)
{
    if (idx >= RULE_MAX) return NULL;
    return s_rules[idx].active ? &s_rules[idx] : NULL;
}

int rule_engine_to_json(char *buf, size_t size)
{
    int off = 0;
    off += snprintf(buf + off, size - off, "{\"rules\":[");

    bool first = true;
    for (int i = 0; i < RULE_MAX; i++) {
        if (!s_rules[i].active) continue;
        const gateway_rule_t *r = &s_rules[i];
        if (!first) off += snprintf(buf + off, size - off, ",");
        first = false;

        if (r->target_is_thread) {
            off += snprintf(buf + off, size - off,
                "{\"idx\":%d,\"src\":%d,\"trig\":%d,"
                "\"act\":%d,\"type\":\"thread\",\"target\":\"%s\"}",
                i, r->src_node_idx, r->trigger,
                r->action, r->target.ipv6);
        } else {
            off += snprintf(buf + off, size - off,
                "{\"idx\":%d,\"src\":%d,\"trig\":%d,"
                "\"act\":%d,\"type\":\"mesh\",\"target\":%d}",
                i, r->src_node_idx, r->trigger,
                r->action, r->target.mesh_addr);
        }
    }
    off += snprintf(buf + off, size - off, "]}\n");
    return off;
}