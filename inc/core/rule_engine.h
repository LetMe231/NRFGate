#ifndef RULE_ENGINE_H
#define RULE_ENGINE_H

#include <zephyr/kernel.h>
#include "semantic_handler.h"
#include "gw_model.h"
#include "gw_store.h"

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
 *
 * RAM footprint (this refactor):
 *   - rule_target_t uses a union {mesh_addr | ipv6[16]} so each target
 *     costs 20 B instead of 44 B (was char ipv6[40] + uint16_t parallel).
 *   - IPv6 is stored in binary form (16 B) and converted to string only
 *     on demand (JSON export, logging). The previous char[40] never
 *     actually held a string — it was always used as a 16 B buffer.
 */

#define RULE_MAX  16
#define RULE_MAX_TARGETS 8

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
    RULE_ACT_ON     = 0,  /* Generic OnOff Set ON  → BLE Mesh addr */
    RULE_ACT_OFF    = 1,  /* Generic OnOff Set OFF → BLE Mesh addr */
    RULE_ACT_TOGGLE = 2,  /* Toggle last known state               */
} rule_action_t;

/* ── Rule descriptor ─────────────────────────────────────────── */
typedef struct {
    bool           active;        /* false = slot unused               */

    /* Source */
    uint8_t        src_node_idx;  /* which node triggers this rule     */
    rule_trigger_t trigger;

    /* Target */
    rule_action_t  action;
    uint8_t        target_count;  /* number of targets in this rule    */
    gw_node_addr_t targets[RULE_MAX_TARGETS];
} gateway_rule_t;

/* ── Public API ──────────────────────────────────────────────── */

void rule_engine_init(void);

/**
 * @brief Evaluate rules on a semantic state transition.
 * Call from semantic_handler after transition() completes.
 */
void rule_engine_on_state(const gw_node_addr_t *src, gw_state_t new_state);

/**
 * @brief Evaluate rules on a switch event.
 * Call from data_handler_receive when sw=1 or sw=0 is in payload.
 */
void rule_engine_on_switch(const gw_node_addr_t *src, bool sw_on);

/**
 * @brief Add a rule. Returns rule index (0–15) or -1 if full.
 */
int rule_engine_add(const gateway_rule_t *rule);

/**
 * @brief Remove a rule by index.
 */
void rule_engine_remove(uint8_t idx);

/**
 * @brief Get rule by index. Returns true if slot is used, false otherwise.
 */
bool rule_engine_get(uint8_t idx, gateway_rule_t *out);

/**
 * @brief Serialize all active rules to JSON for dashboard.
 * Writes into buf, returns bytes written.
 */
int rule_engine_to_json(char *buf, size_t size);

/**
 * @brief Count the number of currently active rules.
 * @return Active rule count, range [0, RULE_MAX].
 */
uint8_t rule_engine_active_count(void);

#endif /* RULE_ENGINE_H */