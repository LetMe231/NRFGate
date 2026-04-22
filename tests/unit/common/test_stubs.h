/**
 * @file tests/unit/common/test_stubs.h
 *
 * Compiler-Stubs für Unit-Tests der neuen Gateway-Architektur.
 *
 * Verhindert Include-Fehler für Module die im Test-Build nicht
 * vorhanden sind (kein BT-Stack, kein Zephyr-Netzwerk etc.).
 *
 * Jeder Test kann zusätzlich eigene Mocks in seiner main.c definieren.
 */

#ifndef TEST_STUBS_H
#define TEST_STUBS_H

/* ── Verhindert doppelten Include der echten Headers ─────────── */
#define BLE_NUS_H
#define BLE_MESH_ADAPTER_H
#define BLE_MESH_PROV_H
#define THREAD_ADAPTER_H
#define LORAWAN_ADAPTER_H
#define COMMAND_ROUTER_H
#define RELIABILITY_MANAGER_H
#define RULE_ENGINE_H
#define SEMANTIC_HANDLER_H

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

/* ── Scheduler (aus main.h) ──────────────────────────────────── */

typedef enum {
    SCHED_PRIORITY_NONE   = 0,
    SCHED_PRIORITY_BLE    = 1,
    SCHED_PRIORITY_THREAD = 2,
} sched_priority_t;

typedef enum {
    SCHED_MODE_NORMAL      = 0,
    SCHED_MODE_BLE_ONLY    = 1,
    SCHED_MODE_THREAD_ONLY = 2,
} sched_mode_t;

void mesh_scheduler_request_priority(sched_priority_t transport,
                                     uint32_t duration_ms);
void mesh_scheduler_set_mode(sched_mode_t mode);
void mesh_scheduler_pause(void);
void mesh_scheduler_start(void);
void mesh_scheduler_set_timing(uint32_t thread_ms, uint32_t ble_ms);

/* ── BLE NUS ─────────────────────────────────────────────────── */

bool ble_nus_is_ready(void);
void ble_nus_send(const char *json);

/* ── BLE Mesh Adapter ────────────────────────────────────────── */

#include "gw_model.h"

int  ble_mesh_adapter_send_cmd(const gw_command_t *cmd);
void ble_mesh_adapter_register_prov_cb(void *prov_cb, void *node_cb,
                                       void *link_cb, void *beacon_cb);
const void *ble_mesh_adapter_comp_init(void);
const void *ble_mesh_adapter_prov_get(void);

extern int64_t g_ble_mesh_last_rx_ms;

/* ── BLE Mesh Prov ───────────────────────────────────────────── */

typedef bool (*ble_mesh_prov_node_lost_fn)(uint16_t mesh_addr);

void ble_mesh_prov_init(void);
void ble_mesh_prov_set_lost_cb(ble_mesh_prov_node_lost_fn fn);
void ble_mesh_self_provision(void);
void ble_mesh_prov_start_window(void);
void ble_mesh_prov_unprovision_node(uint16_t mesh_addr);
void ble_mesh_prov_reconfigure_node(uint16_t mesh_addr);
void ble_mesh_prov_purge_lost_nodes(void);
void ble_mesh_prov_full_reset(void);

/* ── Thread Adapter ──────────────────────────────────────────── */

int thread_adapter_init(void);
int thread_adapter_send_cmd(const gw_command_t *cmd);

/* ── LoRaWAN Adapter ─────────────────────────────────────────── */

int  lorawan_adapter_init(void);
int  lorawan_adapter_send_cmd(const gw_command_t *cmd);
void lorawan_adapter_set_enabled(bool enabled);

/* ── Command Router ──────────────────────────────────────────── */

void command_router_init(void);
int  command_router_send(const gw_command_t *cmd);
int  command_router_send_to(const gw_node_addr_t *dst, gw_cmd_type_t type);

/* ── Reliability Manager ─────────────────────────────────────── */

void reliability_manager_init(void);

/* ── Rule Engine ─────────────────────────────────────────────── */

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
    RULE_ACT_ON     = 0,
    RULE_ACT_OFF    = 1,
    RULE_ACT_TOGGLE = 2,
} rule_action_t;

typedef struct {
    bool           active;
    uint8_t        src_node_idx;
    rule_trigger_t trigger;
    rule_action_t  action;
    bool           target_is_thread;
    union {
        uint16_t mesh_addr;
        char     ipv6[GW_IPV6_STR_LEN];
    } target;
} gateway_rule_t;

void                  rule_engine_init(void);
void                  rule_engine_on_state(const gw_node_addr_t *src,
                                           gw_state_t new_state);
void                  rule_engine_on_switch(const gw_node_addr_t *src,
                                            bool sw_on);
int                   rule_engine_add(const gateway_rule_t *rule);
void                  rule_engine_remove(uint8_t idx);
bool                  rule_engine_get(uint8_t idx, gateway_rule_t *out);
int                   rule_engine_to_json(char *buf, size_t size);

/* ── Semantic Handler ────────────────────────────────────────── */

void semantic_handler_init(void);
void semantic_handler_tick(void);

#endif /* TEST_STUBS_H */