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

/* ── Cooldown: prevent double-fire when node retransmits same press ── */
#define RULE_COOLDOWN_MS  1200  /* covers full retransmit window (4×100ms) + margin */
static int64_t s_last_fire_ms[RULE_MAX];

/* ── Thread CoAP actuator ────────────────────────────────────── */
/* Sends a CoAP PUT to coap://[ipv6]:5683/light with {"on":true/false} */
extern int thread_handler_coap_put_light(const char *ipv6, bool on);


/* Source-transport for changeback after Rule-Fire */
static node_transport_t s_last_src_transport = NODE_TRANSPORT_THREAD;

static void restore_src_priority_fn(struct k_work *w)
{
    /* Normales alternierend-Scheduling wiederherstellen — kein Priority-Lock mehr.
     * Kein extra BLE-Prio-Request: der Scheduler übernimmt direkt nach Thread-Fenster. */
    ARG_UNUSED(w);
}
static K_WORK_DELAYABLE_DEFINE(restore_src_work, restore_src_priority_fn);

/* ── Mesh action work item ───────────────────────────────────── *
 * mesh_ctrl_set_onoff/toggle call bt_mesh_model_send() which has a
 * deep call stack. Running it directly in the BT RX callback context
 * causes stack overflow. Offload to a dedicated work queue.
 */
#define MESH_ACT_STACK_SIZE 2048
#define MESH_ACT_PRIORITY   K_PRIO_PREEMPT(4)

K_THREAD_STACK_DEFINE(s_mesh_act_stack, MESH_ACT_STACK_SIZE);
static struct k_work_q s_mesh_act_wq;
static bool s_mesh_act_wq_started = false;

struct mesh_act_item {
    struct k_work work;
    uint16_t      addr;
    int           action;   /* 0=off, 1=on, 2=toggle */
};

#define MESH_ACT_POOL_SIZE 4
static struct mesh_act_item s_mesh_act_pool[MESH_ACT_POOL_SIZE];
static atomic_t s_mesh_act_idx = ATOMIC_INIT(0);

static void mesh_act_work_fn(struct k_work *w)
{
    struct mesh_act_item *item = CONTAINER_OF(w, struct mesh_act_item, work);
    if (item->action == 0)      mesh_ctrl_set_onoff(item->addr, false);
    else if (item->action == 1) mesh_ctrl_set_onoff(item->addr, true);
    else                        mesh_ctrl_toggle(item->addr);
}

static void schedule_mesh_action(uint16_t addr, int action)
{
    if (!s_mesh_act_wq_started) {
        k_work_queue_init(&s_mesh_act_wq);
        k_work_queue_start(&s_mesh_act_wq, s_mesh_act_stack,
                           MESH_ACT_STACK_SIZE, MESH_ACT_PRIORITY, NULL);
        s_mesh_act_wq_started = true;
    }
    int idx = (int)(atomic_inc(&s_mesh_act_idx) % MESH_ACT_POOL_SIZE);
    struct mesh_act_item *item = &s_mesh_act_pool[idx];
    k_work_cancel(&item->work);
    item->addr   = addr;
    item->action = action;
    k_work_init(&item->work, mesh_act_work_fn);
    k_work_submit_to_queue(&s_mesh_act_wq, &item->work);
}

/* thread_handler_coap_put_light() ist non-blocking (K_MSGQ). */
static void schedule_coap_put(const char *ipv6, bool on)
{
    thread_handler_coap_put_light(ipv6, on);
}


static void fire(const gateway_rule_t *r, node_transport_t src_transport)
{
    s_last_src_transport = src_transport;

    bool light_on = false;
    bool update_cache = true;

    /* Resolve target node index first — needed for TOGGLE state lookup */
    uint8_t target_idx = r->target_is_thread
        ? data_handler_get_node_idx_by_ipv6(r->target.ipv6)
        : data_handler_get_node_idx_by_mesh_addr(r->target.mesh_addr);

    LOG_INF("fire: target_idx=%d known=%d cached_light=%d action=%d",
            target_idx,
            (target_idx < MAX_NODES) ? node_actuator_state[target_idx].known : -1,
            (target_idx < MAX_NODES) ? node_actuator_state[target_idx].light_on : -1,
            r->action);

    if (r->target_is_thread) {
        if (r->action == RULE_ACT_THREAD_TOGGLE) {
            bool current = (target_idx < MAX_NODES) &&
                        node_actuator_state[target_idx].known &&
                        node_actuator_state[target_idx].light_on;
            light_on = !current;
        } else {
            light_on = (r->action == RULE_ACT_THREAD_ON);
        }

        /* Optimistisches Cache-Update sofort beim Queueing */
        if (target_idx < MAX_NODES) {
            node_actuator_state[target_idx].known = true;
            node_actuator_state[target_idx].light_on = light_on;
        }

        LOG_INF("Rule -> Thread %s [%s]", light_on ? "ON" : "OFF", r->target.ipv6);
        schedule_coap_put(r->target.ipv6, light_on);

    } else {
        mesh_scheduler_request_priority(SCHED_PRIORITY_BLE, 1000);

        switch (r->action) {
        case RULE_ACT_MESH_ON:
            light_on = true;
            schedule_mesh_action(r->target.mesh_addr, 1);
            break;

        case RULE_ACT_MESH_OFF:
            light_on = false;
            schedule_mesh_action(r->target.mesh_addr, 0);
            break;

        case RULE_ACT_MESH_TOGGLE:
            light_on = !(target_idx < MAX_NODES &&
                        node_actuator_state[target_idx].light_on);
            schedule_mesh_action(r->target.mesh_addr, 2);
            break;

        default:
            update_cache = false;
            break;
        }

        LOG_INF("Rule -> Mesh %s [0x%04X]",
                r->action == RULE_ACT_MESH_TOGGLE ? "TOGGLE" :
                light_on ? "ON" : "OFF",
                r->target.mesh_addr);
    }
    if (update_cache && !r->target_is_thread) {
        /* Cache-Update für Thread-Targets macht der coap_tx_thread nach ACK */
        if (target_idx < MAX_NODES) {
            node_actuator_state[target_idx].known = true;
            node_actuator_state[target_idx].light_on = light_on;
        }
    }
    /* Nach 1000ms zurück auf Source-Transport für Status-Updates */
    /* restore_src_work: TX-Thread regelt Timing selbst */
    k_work_reschedule(&restore_src_work, K_MSEC(500));
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
    gateway_rule_t to_fire[RULE_MAX];
    int fire_count = 0;

    // only match under mutex, but fire outside to avoid blocking semantic handler
    k_mutex_lock(&s_mutex, K_FOREVER);
    for (int i = 0; i < RULE_MAX; i++) {
        if (!s_rules[i].active) continue;
        if (s_rules[i].src_node_idx != node_idx) continue;
        if (!trigger_matches_state(s_rules[i].trigger, new_state)) continue;
        to_fire[fire_count++] = s_rules[i];
    }
    k_mutex_unlock(&s_mutex);
    for (int i = 0; i < fire_count; i++) {
        LOG_INF("Rule[%d] fired: node=%d state=%d", i, node_idx, new_state);
        fire(&to_fire[i], src_transport);
    }
}

void rule_engine_on_switch(uint8_t node_idx, bool sw_on, node_transport_t src_transport)
{
    gateway_rule_t to_fire[RULE_MAX];
    int fire_count = 0;
    
    rule_trigger_t trig = sw_on ? RULE_TRIG_SWITCH_ON : RULE_TRIG_SWITCH_OFF;
    int to_fire_slot[RULE_MAX];
    k_mutex_lock(&s_mutex, K_FOREVER);
    for (int i = 0; i < RULE_MAX; i++) {
        if (!s_rules[i].active) continue;
        if (s_rules[i].src_node_idx != node_idx) continue;
        if (s_rules[i].trigger != trig) continue;
        to_fire[fire_count] = s_rules[i];
        to_fire_slot[fire_count] = i;
        fire_count++;
    }
    k_mutex_unlock(&s_mutex);

    int64_t now_ms = k_uptime_get();
    for (int i = 0; i < fire_count; i++) {
        int slot = to_fire_slot[i];
        if (now_ms - s_last_fire_ms[slot] < RULE_COOLDOWN_MS) {
            LOG_DBG("Rule[%d] cooldown skip (%lld ms since last fire)",
                    slot, now_ms - s_last_fire_ms[slot]);
            continue;
        }
        s_last_fire_ms[slot] = now_ms;
        LOG_INF("Rule[%d] fired: node=%d switch=%s", slot, node_idx,
                sw_on ? "ON" : "OFF");
        fire(&to_fire[i], src_transport);
    }
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
    if(!buf || size <= 16) return 0;
    int off = 0;
    off += snprintf(buf + off, size - off, "{\"rules\":[");

    bool first = true;
    for (int i = 0; i < RULE_MAX; i++) {
        if (!s_rules[i].active) continue;
        const gateway_rule_t *r = &s_rules[i];

        if ((size_t)off >= size - 1) break; // check buffer overflow before writing

        if (!first) off += snprintf(buf + off, size - off, ",");
        first = false;
        
        if ((size_t)off >= size - 1) break; // check again before writing rule details

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
    // make sure we don't write beyond buffer size when adding closing brackets
    if ((size_t)off < size - 3) {
        off += snprintf(buf + off, size - off, "]}\n");
    }
    return off;
}