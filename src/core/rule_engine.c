/**
 * @file rule_engine.c
 * @brief Cross-protocol rule engine.
 *
 * HOW IT WORKS:
 *   1. semantic_handler calls rule_engine_on_state() on every state transition.
 *   2. semantic_handler calls rule_engine_on_switch() when a switch payload arrives.
 *   3. Matching rule INDICES are collected under mutex, then posted to the
 *      fire-queue. The fire-thread re-reads the rule under the mutex before
 *      dispatching, so rule mutations (add/remove) are always safe.
 *   4. Each rule may target multiple nodes across BLE Mesh and Thread.
 *      BLE targets are sent first (after a single priority request for all of
 *      them), then Thread targets — this ensures the radio is in BLE mode
 *      before any bt_mesh_model_send() call.
 *
 * THREAD SAFETY:
 *   Rules are modified from the NUS work queue (nus_handler_cmd).
 *   Rules are evaluated from the event_ingest listener context.
 *   A mutex protects the rule table for both reads and writes.
 */

#include <zephyr/kernel.h>
#include <zephyr/net/net_ip.h>
#include <zephyr/logging/log.h>
#include <string.h>
#include <stdio.h>

#include "gw_model.h"
#include "gw_store.h"
#include "gw_addr.h"
#include "command_router.h"
#include "rule_engine.h"
#include "scheduler.h"

LOG_MODULE_REGISTER(rule_engine, LOG_LEVEL_INF);

/* ── Rule table ────────────────────────────────────────────── */

static gateway_rule_t s_rules[RULE_MAX];
static K_MUTEX_DEFINE(s_mutex);

/* ── Per-rule cooldown ─────────────────────────────────────── */

/**
 * Minimum time between consecutive firings of the same rule.
 * Prevents a single button press from triggering multiple fires
 * when the node sends several retransmissions of the same event.
 */
#define RULE_COOLDOWN_MS 100
static int64_t s_last_fire_ms[RULE_MAX];

/* Fire thread stack — calls into mesh transport / CoAP TX which eat
 * 1.5–2 KB of stack by themselves. With a single target 2048 works,
 * but a rule with multiple BLE targets iterates fire() and the peak
 * usage creeps upward. Measured crash at ~14 fires of a 4-target rule
 * → 3072 is the safe bound for RULE_MAX_TARGETS=8. */
#define RULE_FIRE_STACK_SIZE 3072
#define RULE_FIRE_PRIO       5

K_THREAD_STACK_DEFINE(s_rule_fire_stack, RULE_FIRE_STACK_SIZE);
static struct k_thread s_rule_fire_thread;

/* Queue stores only the rule slot index (1 B per slot, 8 slots deep). */
K_MSGQ_DEFINE(s_rule_fire_q, sizeof(uint8_t), 8, 1);

/* ── Fire helper ─────────────────────────────────────── */

static void fire_target(const gw_node_addr_t *t, int slot,
                        gw_cmd_type_t cmd_type, const char *transport_label)
{
    gw_node_addr_t dst = *t;
    char addr_str[GW_IPV6_STR_LEN];
    gw_addr_to_str(t, addr_str, sizeof(addr_str));

    LOG_INF("Rule[%d] fire → %s %s %s",
            slot, transport_label, addr_str,
            cmd_type == GW_CMD_LIGHT_ON  ? "ON"  :
            cmd_type == GW_CMD_LIGHT_OFF ? "OFF" : "TOGGLE");

    command_router_send_to(&dst, cmd_type);
}

/* ── Fire ──────────────────────────────────────────────────── */

/**
 * @brief Dispatch all targets of a rule.
 *
 * BLE Mesh targets are sent first. A single priority request covers all
 * of them so the radio is held in BLE mode for the duration. Thread
 * targets follow — the CoAP TX thread blocks on the Thread window semaphore
 * inside thread_adapter_send_cmd(), so no explicit priority request is
 * needed for Thread.
 *
 * @param r     Rule to fire (local copy, caller already released the lock).
 * @param slot  Rule table index (for logging).
 */
static void fire(const gateway_rule_t *r, int slot)
{
    gw_cmd_type_t cmd_type;
    switch (r->action) {
    case RULE_ACT_ON:     cmd_type = GW_CMD_LIGHT_ON;     break;
    case RULE_ACT_OFF:    cmd_type = GW_CMD_LIGHT_OFF;    break;
    case RULE_ACT_TOGGLE: cmd_type = GW_CMD_LIGHT_TOGGLE; break;
    default:
        LOG_WRN("Rule[%d]: unknown action %d", slot, r->action);
        return;
    }

    /* Count BLE targets so we can request enough priority time. */
    int ble_count = 0;
    for (int i = 0; i < r->target_count; i++) {
        if (r->targets[i].transport == GW_TR_BLE_MESH) ble_count++;
    }

    /*
     * Request BLE priority for all BLE targets at once.
     * Allow 400 ms per target plus a 300 ms guard for scheduler latency.
     * Then wait 80 ms for the scheduler to actually switch to BLE before
     * the first bt_mesh_model_send() call.
     */
    if (ble_count > 0) {
        scheduler_request_priority(SCHED_PRIORITY_BLE,
                                   (uint32_t)ble_count * 400 + 300);
        k_msleep(80);
    }

    /* Pass 1: BLE Mesh targets. */
    for (int i = 0; i < r->target_count; i++) {
        const gw_node_addr_t *t = &r->targets[i];
        if (t->transport == GW_TR_THREAD) continue;

        fire_target(t, slot, cmd_type, "BLE Mesh");

        k_msleep(50); 
    }

    if (ble_count > 0 && r->target_count > ble_count) {
        k_msleep(300);
    }

    
    /* Pass 2: Thread targets. */
    for (int i = 0; i < r->target_count; i++) {
        const gw_node_addr_t *t = &r->targets[i];
        if (t->transport != GW_TR_THREAD) continue;

        fire_target(t, slot, cmd_type, "Thread");
    }
}

/* ── Trigger match ─────────────────────────────────────────── */

static bool trigger_matches_state(rule_trigger_t trig, gw_state_t state)
{
    switch (trig) {
    case RULE_TRIG_STATE_ACTIVE:   return state == GW_STATE_ACTIVE;
    case RULE_TRIG_STATE_ALERT:    return state == GW_STATE_ALERT ||
                                          state == GW_STATE_CRITICAL;
    case RULE_TRIG_STATE_CRITICAL: return state == GW_STATE_CRITICAL;
    case RULE_TRIG_STATE_IDLE:     return state == GW_STATE_IDLE;
    case RULE_TRIG_STATE_LOST:     return state == GW_STATE_LOST;
    default:                       return false;
    }
}

/**
 * @brief Fire-thread entry point.
 *
 * Pulls a rule slot index off the queue and re-reads the rule under
 * mutex into a local copy. This way the queue payload stays at 1 byte
 * and the table can be mutated safely between enqueue and dispatch
 * (e.g. the rule is deleted before it fires — we just skip it).
 */
static void rule_fire_thread_fn(void *a, void *b, void *c)
{
    ARG_UNUSED(a); ARG_UNUSED(b); ARG_UNUSED(c);

    uint8_t        slot;
    gateway_rule_t rule;

    while (1) {
        k_msgq_get(&s_rule_fire_q, &slot, K_FOREVER);

        if (slot >= RULE_MAX) {
            continue;
        }

        /* Snapshot the rule under lock, then fire outside the lock. */
        k_mutex_lock(&s_mutex, K_FOREVER);
        bool ok = s_rules[slot].active;
        if (ok) {
            rule = s_rules[slot];
        }
        k_mutex_unlock(&s_mutex);

        if (ok) {
            fire(&rule, slot);
        } else {
            LOG_DBG("Rule[%d] vanished before fire", slot);
        }
    }
}

/* ── Public API ────────────────────────────────────────────── */

void rule_engine_init(void)
{
    memset(s_rules,         0, sizeof(s_rules));
    memset(s_last_fire_ms,  0, sizeof(s_last_fire_ms));
    k_thread_create(&s_rule_fire_thread, s_rule_fire_stack, RULE_FIRE_STACK_SIZE,
                    rule_fire_thread_fn, NULL, NULL, NULL,
                    RULE_FIRE_PRIO, 0, K_NO_WAIT);
    k_thread_name_set(&s_rule_fire_thread, "rule_fire");
    LOG_INF("rule_fire thread=%p", &s_rule_fire_thread);
    LOG_INF("Rule engine initialized (max %d rules, %d targets/rule)",
            RULE_MAX, RULE_MAX_TARGETS);
}

static bool rule_try_acquire_cooldown(uint8_t slot, int64_t now_ms)
{
    bool allowed = false;

    if (slot >= RULE_MAX) {
        return false;
    }

    k_mutex_lock(&s_mutex, K_FOREVER);

    if ((now_ms - s_last_fire_ms[slot]) >= RULE_COOLDOWN_MS) {
        s_last_fire_ms[slot] = now_ms;
        allowed = true;
    }

    k_mutex_unlock(&s_mutex);
    return allowed;
}

void rule_engine_on_state(const gw_node_addr_t *src, gw_state_t new_state)
{
    if (!src) return;

    int idx = gw_store_find_node(src);
    if (idx < 0) return;

    /* Collect matching rule INDICES under lock (1 B per entry instead of
     * ~360 B when copying the full struct). */
    uint8_t to_fire_slot[RULE_MAX];
    int     fire_count = 0;

    k_mutex_lock(&s_mutex, K_FOREVER);
    for (int i = 0; i < RULE_MAX; i++) {
        if (!s_rules[i].active) continue;
        if (s_rules[i].src_node_idx != (uint8_t)idx) continue;
        if (!trigger_matches_state(s_rules[i].trigger, new_state)) continue;
        to_fire_slot[fire_count++] = (uint8_t)i;
    }
    k_mutex_unlock(&s_mutex);

    /* Enqueue outside the lock; fire thread re-reads under lock. */
    int64_t now_ms = k_uptime_get();
    for (int i = 0; i < fire_count; i++) {
        uint8_t slot = to_fire_slot[i];

        if (!rule_try_acquire_cooldown(slot, now_ms)) {
            LOG_DBG("Rule[%d] cooldown skip", slot);
            continue;
        }

        if (k_msgq_put(&s_rule_fire_q, &slot, K_NO_WAIT) == 0) {
            LOG_INF("Rule[%d] queued: src_node=%d state=%d",
                slot, idx, new_state);
        } else {
            if (slot >= RULE_MAX) {
                return;
            }

            k_mutex_lock(&s_mutex, K_FOREVER);
            if (s_last_fire_ms[slot] == now_ms) {
                s_last_fire_ms[slot] = 0;
            }
            k_mutex_unlock(&s_mutex);
            LOG_WRN("Rule[%d] queue full, failed to enqueue switch event",
                    slot);
        }
    }
}

void rule_engine_on_switch(const gw_node_addr_t *src, bool sw_on)
{
    if (!src) return;

    int idx = gw_store_find_node(src);
    if (idx < 0) return;

    rule_trigger_t trig = sw_on ? RULE_TRIG_SWITCH_ON : RULE_TRIG_SWITCH_OFF;

    uint8_t to_fire_slot[RULE_MAX];
    int     fire_count = 0;

    k_mutex_lock(&s_mutex, K_FOREVER);
    for (int i = 0; i < RULE_MAX; i++) {
        if (!s_rules[i].active) continue;
        if (s_rules[i].src_node_idx != (uint8_t)idx) continue;
        if (s_rules[i].trigger != trig) continue;
        to_fire_slot[fire_count++] = (uint8_t)i;
    }
    k_mutex_unlock(&s_mutex);

    int64_t now_ms = k_uptime_get();
    for (int i = 0; i < fire_count; i++) {
        uint8_t slot = to_fire_slot[i];

        if (!rule_try_acquire_cooldown(slot, now_ms)) {
            LOG_DBG("Rule[%d] cooldown skip", slot);
            continue;
        }

        if (k_msgq_put(&s_rule_fire_q, &slot, K_NO_WAIT) == 0) {
            LOG_INF("Rule[%d] queued: src_node=%d switch=%s",
                slot, idx, sw_on ? "ON" : "OFF");
        } else {
            if (slot >= RULE_MAX) {
                return;
            }

            k_mutex_lock(&s_mutex, K_FOREVER);
            if (s_last_fire_ms[slot] == now_ms) {
                s_last_fire_ms[slot] = 0;
            }
            k_mutex_unlock(&s_mutex);
            LOG_WRN("Rule[%d] queue full, failed to enqueue switch event",
                    slot);
        }
    }
}

int rule_engine_add(const gateway_rule_t *rule)
{
    if (!rule || rule->target_count == 0 ||
        rule->target_count > RULE_MAX_TARGETS) {
        return -EINVAL;
    }

    k_mutex_lock(&s_mutex, K_FOREVER);
    for (int i = 0; i < RULE_MAX; i++) {
        if (!s_rules[i].active) {
            s_rules[i]        = *rule;
            s_rules[i].active = true;
            k_mutex_unlock(&s_mutex);
            LOG_INF("Rule[%d] added: src_node=%d trig=%d act=%d targets=%d",
                    i, rule->src_node_idx, rule->trigger,
                    rule->action, rule->target_count);
            return i;
        }
    }
    k_mutex_unlock(&s_mutex);
    LOG_WRN("Rule table full");
    return -ENOMEM;
}

void rule_engine_remove(uint8_t idx)
{
    if (idx >= RULE_MAX) return;
    k_mutex_lock(&s_mutex, K_FOREVER);
    memset(&s_rules[idx], 0, sizeof(s_rules[idx]));
    s_last_fire_ms[idx] = 0;
    k_mutex_unlock(&s_mutex);
    LOG_INF("Rule[%d] removed", idx);
}

bool rule_engine_get(uint8_t idx, gateway_rule_t *out)
{
    if (idx >= RULE_MAX) return false;

    k_mutex_lock(&s_mutex, K_FOREVER);

    bool ok = s_rules[idx].active;
    if (ok) {
        *out = s_rules[idx];
    }
    k_mutex_unlock(&s_mutex);
    return ok;
}

/**
 * @brief Serialize all active rules to compact JSON.
 *
 * Format per rule:
 *   [idx, src_node_idx, trigger, action, [[type,addr], ...]]
 *
 * Target format:
 *   [transport, mesh_addr]       — transport: 1=BLE Mesh, 2=Thread, 3=LoRaWAN
 *
 * Example:
 *   {"rules":[[0,1,5,2,[[2,fdb1:e25f:ebc5:1000::3],[2,"fdde::1"]]]]}
 */
int rule_engine_to_json(char *buf, size_t size)
{
    if (!buf || size <= 8) {
        return 0;
    }

    /* Snapshot the full rule table under lock. */
    gateway_rule_t rules_snapshot[RULE_MAX];

    k_mutex_lock(&s_mutex, K_FOREVER);
    memcpy(rules_snapshot, s_rules, sizeof(rules_snapshot));
    k_mutex_unlock(&s_mutex);

    int off = 0;
    int n = snprintf(buf + off, size - off, "{\"rules\":[");
    if (n < 0) {
        return 0;
    }
    if ((size_t)n >= size - off) {
        buf[size - 1] = '\0';
        return (int)(size - 1);
    }
    off += n;

    bool first_rule = true;

    for (int i = 0; i < RULE_MAX; i++) {
        const gateway_rule_t *r = &rules_snapshot[i];

        if (!r->active) {
            continue;
        }

        if ((size_t)off >= size - 4) {
            break;
        }

        if (!first_rule) {
            buf[off++] = ',';
        }
        first_rule = false;

        n = snprintf(buf + off, size - off,
                     "[%d,%d,%d,%d,[",
                     i, r->src_node_idx, r->trigger, r->action);
        if (n < 0) {
            break;
        }
        if ((size_t)n >= size - off) {
            off = (int)(size - 1);
            break;
        }
        off += n;

        bool first_tgt = true;
        for (int j = 0; j < r->target_count; j++) {
            const gw_node_addr_t *t = &r->targets[j];

            if ((size_t)off >= size - 8) {
                break;
            }

            if (!first_tgt) {
                buf[off++] = ',';
            }
            first_tgt = false;

            char ipv6_str[GW_IPV6_STR_LEN];
            gw_addr_to_str(t, ipv6_str, sizeof(ipv6_str));
            n = snprintf(buf + off, size - off, "[%d,\"%s\"]",
                         (int)t->transport, ipv6_str);

            if (n < 0) {
                break;
            }
            if ((size_t)n >= size - off) {
                off = (int)(size - 1);
                break;
            }
            off += n;
        }

        if ((size_t)off >= size - 3) {
            break;
        }

        n = snprintf(buf + off, size - off, "]]");
        if (n < 0) {
            break;
        }
        if ((size_t)n >= size - off) {
            off = (int)(size - 1);
            break;
        }
        off += n;
    }

    if ((size_t)off < size - 1) {
        n = snprintf(buf + off, size - off, "]}\n");
        if (n < 0) {
            buf[off] = '\0';
            return off;
        }
        if ((size_t)n >= size - off) {
            buf[size - 1] = '\0';
            return (int)(size - 1);
        }
        off += n;
    } else {
        buf[size - 1] = '\0';
        return (int)(size - 1);
    }

    return off;
}

uint8_t rule_engine_active_count(void)
{
    uint8_t count = 0;
    k_mutex_lock(&s_mutex, K_FOREVER);
    for (int i = 0; i < RULE_MAX; i++) {
        if (s_rules[i].active) count++;
    }
    k_mutex_unlock(&s_mutex);
    return count;
}
