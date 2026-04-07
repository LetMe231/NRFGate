#ifndef RULE_ENGINE_H
#define RULE_ENGINE_H

#include <stdint.h>
#include <stdbool.h>
#include "data_handler.h"
#include "semantic_handler.h"

#define RULE_MAX 16

typedef enum {
    RULE_TRIG_STATE_ACTIVE   = 0,
    RULE_TRIG_STATE_ALERT    = 1,
    RULE_TRIG_STATE_CRITICAL = 2,
    RULE_TRIG_STATE_IDLE     = 3,
    RULE_TRIG_STATE_LOST     = 4,
    RULE_TRIG_SWITCH_ON      = 5,
    RULE_TRIG_SWITCH_OFF     = 6,
} rule_trigger_t;

typedef enum {
    RULE_ACT_MESH_ON       = 0,
    RULE_ACT_MESH_OFF      = 1,
    RULE_ACT_MESH_TOGGLE   = 2,
    RULE_ACT_THREAD_ON     = 3,
    RULE_ACT_THREAD_OFF    = 4,
    RULE_ACT_THREAD_TOGGLE = 5,
} rule_action_t;

typedef struct {
    bool           active;
    uint8_t        src_node_idx;
    rule_trigger_t trigger;
    rule_action_t  action;
    bool           target_is_thread;
    union {
        uint16_t mesh_addr;
        char     ipv6[40];
    } target;
} gateway_rule_t;

void rule_engine_init(void);
void rule_engine_on_state(uint8_t node_idx, node_state_t new_state, node_transport_t src_transport);
void rule_engine_on_switch(uint8_t node_idx, bool sw_on, node_transport_t src_transport);
int  rule_engine_add(const gateway_rule_t *rule);
void rule_engine_remove(uint8_t idx);
const gateway_rule_t *rule_engine_get(uint8_t idx);
int  rule_engine_to_json(char *buf, size_t size);

#endif /* RULE_ENGINE_H */
