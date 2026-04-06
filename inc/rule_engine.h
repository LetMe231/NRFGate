#ifndef RULE_ENGINE_H
#define RULE_ENGINE_H

#include <zephyr/kernel.h>
#include "semantic_handler.h"
#include "data_handler.h"

/**
 * @file rule_engine.h
 * @brief If-Then rule engine for cross-protocol actuator control.
 *
 * Rules are evaluated on every semantic state transition.
 * A rule fires when:
 *   source node index matches AND new state matches trigger
 *
 * Actions can target:
 *   - BLE Mesh address (Generic OnOff Set via mesh_ctrl)
 *   - Thread node IPv6 (CoAP PUT to /light)
 *
 * Up to RULE_MAX rules are stored in RAM.
 * Rules survive scheduler restarts but NOT reboots (no persistence yet).
 */

#define RULE_MAX  16

/* ── Trigger types ───────────────────────────────────────────── */
typedef enum {
    RULE_TRIG_STATE_ACTIVE   = 0,  /* node enters ACTIVE              */
    RULE_TRIG_STATE_ALERT    = 1,  /* node enters ALERT               */
    RULE_TRIG_STATE_CRITICAL = 2,  /* node enters CRITICAL            */
    RULE_TRIG_STATE_IDLE     = 3,  /* node returns to IDLE            */
    RULE_TRIG_STATE_LOST     = 4,  /* node goes LOST                  */
    RULE_TRIG_SWITCH_ON      = 5,  /* node sends switch=1 payload     */
    RULE_TRIG_SWITCH_OFF     = 6,  /* node sends switch=0 payload     */
} rule_trigger_t;

/* ── Action types ────────────────────────────────────────────── */
typedef enum {
    RULE_ACT_MESH_ON     = 0,  /* Generic OnOff Set ON  → BLE Mesh addr */
    RULE_ACT_MESH_OFF    = 1,  /* Generic OnOff Set OFF → BLE Mesh addr */
    RULE_ACT_MESH_TOGGLE = 2,  /* Toggle last known state               */
    RULE_ACT_THREAD_ON   = 3,  /* CoAP PUT {"on":true}  → Thread node   */
    RULE_ACT_THREAD_OFF  = 4,  /* CoAP PUT {"on":false} → Thread node   */
    RULE_ACT_THREAD_TOGGLE = 5, /* CoAP PUT toggle last known state       */
} rule_action_t;

/* ── Rule descriptor ─────────────────────────────────────────── */
typedef struct {
    bool           active;       /* false = slot unused               */

    /* Source */
    uint8_t        src_node_idx; /* which node triggers this rule     */
    rule_trigger_t trigger;

    /* Target */
    rule_action_t  action;
    bool           target_is_thread; /* false = BLE Mesh, true = Thread */
    union {
        uint16_t mesh_addr;          /* BLE Mesh unicast or group addr  */
        char     ipv6[40];           /* Thread node IPv6 address        */
    } target;
} gateway_rule_t;

/* ── Public API ──────────────────────────────────────────────── */

void rule_engine_init(void);

/**
 * @brief Evaluate rules on a semantic state transition.
 * Call from semantic_handler after transition() completes.
 */
void rule_engine_on_state(uint8_t node_idx, node_state_t new_state, node_transport_t src_transport);

/**
 * @brief Evaluate rules on a switch event.
 * Call from data_handler_receive when sw=1 or sw=0 is in payload.
 */
void rule_engine_on_switch(uint8_t node_idx, bool sw_on, node_transport_t src_transport);

/**
 * @brief Add a rule. Returns rule index (0–15) or -1 if full.
 */
int rule_engine_add(const gateway_rule_t *rule);

/**
 * @brief Remove a rule by index.
 */
void rule_engine_remove(uint8_t idx);

/**
 * @brief Get rule by index. Returns NULL if slot unused.
 */
const gateway_rule_t *rule_engine_get(uint8_t idx);

/**
 * @brief Serialize all active rules to JSON for dashboard.
 * Writes into buf, returns bytes written.
 */
int rule_engine_to_json(char *buf, size_t size);

#endif /* RULE_ENGINE_H */