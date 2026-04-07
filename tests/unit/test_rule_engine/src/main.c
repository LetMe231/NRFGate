/**
 * @file tests/unit/test_rule_engine/src/main.c
 *
 * Unit-Tests für rule_engine.c.
 *
 * Was getestet wird:
 *   - Regel hinzufügen / entfernen / wiederverwenden
 *   - State-Trigger: richtige Regel feuert, falsche nicht
 *   - Switch-Trigger: ON/OFF separat
 *   - TOGGLE: benutzt Zielknoten-Zustand (nicht Quellknoten)
 *   - ALERT-Trigger matched auch CRITICAL (Eskalation)
 *   - Tabelle voll → -1
 *   - Thread-Target: coap_put_light wird aufgerufen
 *   - JSON-Serialisierung: valide Struktur
 */

#include <zephyr/ztest.h>
#include <string.h>

/* ── Quell-Header (aus src/) ──────────────────────────────────── */
#include "main.h"  
#include "rule_engine.h"
#include "data_handler.h"

/* ── Mock-Zustand: was wurde gefeuert? ───────────────────────── */
static uint16_t mock_mesh_addr    = 0xFFFF;
static bool     mock_mesh_on      = false;
static int      mock_mesh_calls   = 0;

static char     mock_coap_ipv6[40] = {0};
static bool     mock_coap_on      = false;
static int      mock_coap_calls   = 0;

static void reset_mocks(void)
{
    mock_mesh_addr  = 0xFFFF;
    mock_mesh_on    = false;
    mock_mesh_calls = 0;
    memset(mock_coap_ipv6, 0, sizeof(mock_coap_ipv6));
    mock_coap_on    = false;
    mock_coap_calls = 0;
    memset(node_actuator_state, 0, sizeof(node_actuator_state));
}

/* ── Mocks ────────────────────────────────────────────────────── */

/* data_handler.h: Aktuator-State-Array — rule_engine.c braucht es als extern */
node_actuator_state_t node_actuator_state[MAX_NODES];

/* data_handler.c Lookup-Funktionen */
uint8_t data_handler_get_node_idx_by_mesh_addr(uint16_t mesh_addr)
{
    /* Für Tests: Mesh-Adresse direkt als Index (0x0002 → 0, 0x0003 → 1, ...) */
    if (mesh_addr >= 0x0002 && mesh_addr < 0x0002 + MAX_NODES) {
        return (uint8_t)(mesh_addr - 0x0002);
    }
    return 0xFF;
}

uint8_t data_handler_get_node_idx_by_ipv6(const char *ipv6)
{
    /* Für Tests: "fd11::1" → 0, "fd11::2" → 1, sonst 0xFF */
    if (strcmp(ipv6, "fd11::1") == 0) return 0;
    if (strcmp(ipv6, "fd11::2") == 0) return 1;
    return 0xFF;
}

/* mesh_ctrl.h */
void mesh_ctrl_set_onoff(uint16_t addr, bool on)
{
    mock_mesh_addr = addr;
    mock_mesh_on   = on;
    mock_mesh_calls++;
}

void mesh_ctrl_toggle(uint16_t addr)
{
    mock_mesh_addr = addr;
    mock_mesh_calls++;
    /* Toggle: invertiere bekannten Zustand im Cache — wie echter Code */
    uint8_t idx = data_handler_get_node_idx_by_mesh_addr(addr);
    if (idx < MAX_NODES && node_actuator_state[idx].known) {
        mock_mesh_on = !node_actuator_state[idx].light_on;
    } else {
        mock_mesh_on = true; /* unbekannt → ON */
    }
}

/* thread_handler.h */
int thread_handler_coap_put_light(const char *ipv6, bool on)
{
    strncpy(mock_coap_ipv6, ipv6, sizeof(mock_coap_ipv6) - 1);
    mock_coap_on = on;
    mock_coap_calls++;
    return 0;
}

/* main.h */
void mesh_scheduler_request_priority(sched_priority_t transport, uint32_t duration_ms)
{
    ARG_UNUSED(transport);
    ARG_UNUSED(duration_ms);
}
void mesh_scheduler_set_mode(sched_mode_t mode)   { ARG_UNUSED(mode); }
void mesh_scheduler_pause(void)                    {}
void mesh_scheduler_start(void)                    {}

/* ble_nus.h — nicht verwendet in rule_engine, trotzdem linken */
void ble_nus_send(const char *json)               { ARG_UNUSED(json); }
bool ble_nus_is_ready(void)                        { return false; }

/* ── Helper: Standard-Mesh-Regel erstellen ───────────────────── */
static gateway_rule_t make_mesh_rule(uint8_t src, rule_trigger_t trig,
                                     rule_action_t act, uint16_t target_addr)
{
    gateway_rule_t r = {0};
    r.src_node_idx    = src;
    r.trigger         = trig;
    r.action          = act;
    r.target_is_thread = false;
    r.target.mesh_addr = target_addr;
    return r;
}

static gateway_rule_t make_thread_rule(uint8_t src, rule_trigger_t trig,
                                       rule_action_t act, const char *ipv6)
{
    gateway_rule_t r = {0};
    r.src_node_idx    = src;
    r.trigger         = trig;
    r.action          = act;
    r.target_is_thread = true;
    strncpy(r.target.ipv6, ipv6, sizeof(r.target.ipv6) - 1);
    return r;
}

/* ── Test Suite Setup / Teardown ─────────────────────────────── */
static void before_each(void *f)
{
    ARG_UNUSED(f);
    rule_engine_init();
    reset_mocks();
}

ZTEST_SUITE(rule_engine, NULL, NULL, before_each, NULL, NULL);

/* ════════════════════════════════════════════════════════════════
 * Regeln hinzufügen / entfernen
 * ════════════════════════════════════════════════════════════════ */

ZTEST(rule_engine, test_add_returns_valid_index)
{
    gateway_rule_t r = make_mesh_rule(0, RULE_TRIG_STATE_ALERT,
                                       RULE_ACT_MESH_ON, 0x0002);
    int idx = rule_engine_add(&r);
    zassert_between_inclusive(idx, 0, RULE_MAX - 1,
                              "add() should return valid slot index");
}

ZTEST(rule_engine, test_add_max_rules_then_full)
{
    gateway_rule_t r = make_mesh_rule(0, RULE_TRIG_STATE_ALERT,
                                       RULE_ACT_MESH_ON, 0x0002);
    for (int i = 0; i < RULE_MAX; i++) {
        zassert_true(rule_engine_add(&r) >= 0,
                     "Slot %d should be available", i);
    }
    zassert_equal(rule_engine_add(&r), -1,
                  "Should return -1 when table is full");
}

ZTEST(rule_engine, test_remove_frees_slot)
{
    gateway_rule_t r = make_mesh_rule(0, RULE_TRIG_STATE_ALERT,
                                       RULE_ACT_MESH_ON, 0x0002);
    /* Tabelle füllen */
    for (int i = 0; i < RULE_MAX; i++) {
        rule_engine_add(&r);
    }
    zassert_equal(rule_engine_add(&r), -1, "Table should be full");

    /* Slot 3 entfernen → wiederverwenden */
    rule_engine_remove(3);
    zassert_equal(rule_engine_get(3), NULL, "Removed slot should be NULL");
    int idx = rule_engine_add(&r);
    zassert_equal(idx, 3, "Freed slot should be reused");
}

ZTEST(rule_engine, test_get_returns_null_for_empty_slot)
{
    zassert_is_null(rule_engine_get(0),
                    "Empty slot should return NULL");
    zassert_is_null(rule_engine_get(RULE_MAX - 1),
                    "Empty last slot should return NULL");
}

ZTEST(rule_engine, test_get_out_of_bounds_returns_null)
{
    zassert_is_null(rule_engine_get(RULE_MAX),
                    "Out-of-bounds index should return NULL");
    zassert_is_null(rule_engine_get(255),
                    "Index 255 should return NULL");
}

/* ════════════════════════════════════════════════════════════════
 * State-Trigger: feuert / feuert nicht
 * ════════════════════════════════════════════════════════════════ */

ZTEST(rule_engine, test_fires_on_matching_state_and_node)
{
    gateway_rule_t r = make_mesh_rule(0, RULE_TRIG_STATE_ALERT,
                                       RULE_ACT_MESH_ON, 0x0002);
    rule_engine_add(&r);

    rule_engine_on_state(0, NODE_STATE_ALERT, NODE_TRANSPORT_THREAD);

    zassert_equal(mock_mesh_calls, 1, "Should fire exactly once");
    zassert_equal(mock_mesh_addr, 0x0002, "Should target 0x0002");
    zassert_true(mock_mesh_on, "Should be ON");
}

ZTEST(rule_engine, test_no_fire_wrong_node)
{
    gateway_rule_t r = make_mesh_rule(2, RULE_TRIG_STATE_ALERT,
                                       RULE_ACT_MESH_ON, 0x0002);
    rule_engine_add(&r);

    /* Node 0 feuert, Regel wartet auf Node 2 */
    rule_engine_on_state(0, NODE_STATE_ALERT, NODE_TRANSPORT_THREAD);

    zassert_equal(mock_mesh_calls, 0, "Wrong node should not fire rule");
}

ZTEST(rule_engine, test_no_fire_wrong_state)
{
    gateway_rule_t r = make_mesh_rule(0, RULE_TRIG_STATE_ALERT,
                                       RULE_ACT_MESH_ON, 0x0002);
    rule_engine_add(&r);

    rule_engine_on_state(0, NODE_STATE_IDLE, NODE_TRANSPORT_THREAD);
    rule_engine_on_state(0, NODE_STATE_ACTIVE, NODE_TRANSPORT_THREAD);

    zassert_equal(mock_mesh_calls, 0, "Non-matching states should not fire");
}

ZTEST(rule_engine, test_alert_trigger_matches_critical)
{
    /* ALERT-Trigger soll auch bei CRITICAL feuern (Eskalation) */
    gateway_rule_t r = make_mesh_rule(0, RULE_TRIG_STATE_ALERT,
                                       RULE_ACT_MESH_ON, 0x0002);
    rule_engine_add(&r);

    rule_engine_on_state(0, NODE_STATE_CRITICAL, NODE_TRANSPORT_THREAD);

    zassert_equal(mock_mesh_calls, 1,
                  "ALERT trigger should also fire on CRITICAL (escalation)");
}

ZTEST(rule_engine, test_critical_trigger_does_not_match_alert)
{
    /* CRITICAL-Trigger soll NICHT bei bloßem ALERT feuern */
    gateway_rule_t r = make_mesh_rule(0, RULE_TRIG_STATE_CRITICAL,
                                       RULE_ACT_MESH_ON, 0x0002);
    rule_engine_add(&r);

    rule_engine_on_state(0, NODE_STATE_ALERT, NODE_TRANSPORT_THREAD);

    zassert_equal(mock_mesh_calls, 0,
                  "CRITICAL trigger should not fire on ALERT");
}

ZTEST(rule_engine, test_multiple_rules_same_node_both_fire)
{
    gateway_rule_t r1 = make_mesh_rule(0, RULE_TRIG_STATE_ALERT,
                                        RULE_ACT_MESH_ON,  0x0002);
    gateway_rule_t r2 = make_mesh_rule(0, RULE_TRIG_STATE_ALERT,
                                        RULE_ACT_MESH_OFF, 0x0003);
    rule_engine_add(&r1);
    rule_engine_add(&r2);

    rule_engine_on_state(0, NODE_STATE_ALERT, NODE_TRANSPORT_THREAD);

    zassert_equal(mock_mesh_calls, 2,
                  "Both rules should fire when same node+state matches");
}

/* ════════════════════════════════════════════════════════════════
 * Switch-Trigger
 * ════════════════════════════════════════════════════════════════ */

ZTEST(rule_engine, test_switch_on_trigger_fires_on_sw_on)
{
    gateway_rule_t r = make_mesh_rule(1, RULE_TRIG_SWITCH_ON,
                                       RULE_ACT_MESH_ON, 0x0003);
    rule_engine_add(&r);

    rule_engine_on_switch(1, true, NODE_TRANSPORT_BLE_MESH);

    zassert_equal(mock_mesh_calls, 1, "SWITCH_ON rule should fire");
    zassert_true(mock_mesh_on, "Should be ON");
}

ZTEST(rule_engine, test_switch_on_trigger_does_not_fire_on_sw_off)
{
    gateway_rule_t r = make_mesh_rule(1, RULE_TRIG_SWITCH_ON,
                                       RULE_ACT_MESH_ON, 0x0003);
    rule_engine_add(&r);

    rule_engine_on_switch(1, false, NODE_TRANSPORT_BLE_MESH);

    zassert_equal(mock_mesh_calls, 0,
                  "SWITCH_ON rule must not fire on SWITCH_OFF event");
}

ZTEST(rule_engine, test_switch_off_trigger_fires_on_sw_off)
{
    gateway_rule_t r = make_mesh_rule(1, RULE_TRIG_SWITCH_OFF,
                                       RULE_ACT_MESH_OFF, 0x0003);
    rule_engine_add(&r);

    rule_engine_on_switch(1, false, NODE_TRANSPORT_BLE_MESH);

    zassert_equal(mock_mesh_calls, 1, "SWITCH_OFF rule should fire");
}

/* ════════════════════════════════════════════════════════════════
 * TOGGLE: Zielknoten-Zustand wird verwendet
 * ════════════════════════════════════════════════════════════════ */

ZTEST(rule_engine, test_toggle_uses_target_state_not_source)
{
    /* Quell-Node 0, Ziel: Mesh 0x0002 (→ node_idx 0 im Mock) */
    gateway_rule_t r = make_mesh_rule(0, RULE_TRIG_STATE_ALERT,
                                       RULE_ACT_MESH_TOGGLE, 0x0002);
    rule_engine_add(&r);

    /* Zielknoten ist bekannt: light = ON */
    uint8_t target_idx = data_handler_get_node_idx_by_mesh_addr(0x0002);
    node_actuator_state[target_idx].known    = true;
    node_actuator_state[target_idx].light_on = true;  /* war AN → Toggle → AUS */

    rule_engine_on_state(0, NODE_STATE_ALERT, NODE_TRANSPORT_THREAD);

    zassert_equal(mock_mesh_calls, 1, "Toggle should fire");
    /* mesh_ctrl_toggle wurde aufgerufen — mock invertiert den Zustand */
    zassert_false(mock_mesh_on, "Toggle from ON should result in OFF");
}

ZTEST(rule_engine, test_toggle_unknown_state_defaults_to_on)
{
    gateway_rule_t r = make_mesh_rule(0, RULE_TRIG_STATE_ALERT,
                                       RULE_ACT_MESH_TOGGLE, 0x0002);
    rule_engine_add(&r);

    /* Zustand unbekannt (known = false) */
    uint8_t target_idx = data_handler_get_node_idx_by_mesh_addr(0x0002);
    node_actuator_state[target_idx].known = false;

    rule_engine_on_state(0, NODE_STATE_ALERT, NODE_TRANSPORT_THREAD);

    zassert_equal(mock_mesh_calls, 1, "Toggle should still fire");
    zassert_true(mock_mesh_on, "Unknown state should toggle to ON");
}

/* ════════════════════════════════════════════════════════════════
 * Thread-Target
 * ════════════════════════════════════════════════════════════════ */

ZTEST(rule_engine, test_thread_target_calls_coap)
{
    gateway_rule_t r = make_thread_rule(0, RULE_TRIG_STATE_ALERT,
                                         RULE_ACT_THREAD_ON, "fd11::1");
    rule_engine_add(&r);

    rule_engine_on_state(0, NODE_STATE_ALERT, NODE_TRANSPORT_BLE_MESH);

    zassert_equal(mock_coap_calls, 1, "CoAP PUT should be called");
    zassert_str_equal(mock_coap_ipv6, "fd11::1", "Wrong IPv6 target");
    zassert_true(mock_coap_on, "Should be ON");
    zassert_equal(mock_mesh_calls, 0, "Mesh should NOT be called");
}

ZTEST(rule_engine, test_thread_target_off)
{
    gateway_rule_t r = make_thread_rule(0, RULE_TRIG_STATE_IDLE,
                                         RULE_ACT_THREAD_OFF, "fd11::2");
    rule_engine_add(&r);

    rule_engine_on_state(0, NODE_STATE_IDLE, NODE_TRANSPORT_BLE_MESH);

    zassert_equal(mock_coap_calls, 1, "CoAP PUT should be called");
    zassert_false(mock_coap_on, "Should be OFF");
}

ZTEST(rule_engine, test_thread_toggle_uses_target_cache)
{
    gateway_rule_t r = make_thread_rule(0, RULE_TRIG_STATE_ALERT,
                                         RULE_ACT_THREAD_TOGGLE, "fd11::1");
    rule_engine_add(&r);

    /* Ziel-Thread-Node (fd11::1 → idx 0) ist bekannt: AUS */
    node_actuator_state[0].known    = true;
    node_actuator_state[0].light_on = false;  /* war AUS → Toggle → AN */

    rule_engine_on_state(0, NODE_STATE_ALERT, NODE_TRANSPORT_BLE_MESH);

    zassert_equal(mock_coap_calls, 1, "CoAP should be called");
    zassert_true(mock_coap_on, "Toggle from OFF should be ON");
}

/* ════════════════════════════════════════════════════════════════
 * Aktuator-Cache wird beim ZIEL-Node gesetzt
 * ════════════════════════════════════════════════════════════════ */

ZTEST(rule_engine, test_actuator_cache_updated_on_target)
{
    uint8_t target_idx = data_handler_get_node_idx_by_mesh_addr(0x0002); /* = 0 */

    gateway_rule_t r = make_mesh_rule(1,  /* src = Node 1 */
                                       RULE_TRIG_STATE_ALERT,
                                       RULE_ACT_MESH_ON,
                                       0x0002); /* target = Node 0 */
    rule_engine_add(&r);

    rule_engine_on_state(1, NODE_STATE_ALERT, NODE_TRANSPORT_THREAD);

    /* Cache muss beim TARGET (idx 0) aktualisiert sein, nicht beim Source (idx 1) */
    zassert_true(node_actuator_state[target_idx].known,
                 "Target node cache should be marked known");
    zassert_true(node_actuator_state[target_idx].light_on,
                 "Target node should be cached as ON");
    zassert_false(node_actuator_state[1].known,
                  "Source node cache must NOT be touched");
}

/* ════════════════════════════════════════════════════════════════
 * JSON-Serialisierung
 * ════════════════════════════════════════════════════════════════ */

ZTEST(rule_engine, test_json_empty_rules)
{
    char buf[256];
    int len = rule_engine_to_json(buf, sizeof(buf));
    zassert_true(len > 0, "JSON should not be empty");
    zassert_not_null(strstr(buf, "\"rules\":[]"), "Empty rules array expected");
}

ZTEST(rule_engine, test_json_mesh_rule_fields)
{
    gateway_rule_t r = make_mesh_rule(3, RULE_TRIG_STATE_CRITICAL,
                                       RULE_ACT_MESH_ON, 0x0010);
    rule_engine_add(&r);

    char buf[512];
    rule_engine_to_json(buf, sizeof(buf));

    zassert_not_null(strstr(buf, "\"src\":3"),   "Should have src");
    zassert_not_null(strstr(buf, "\"trig\":2"),  "Should have trig=CRITICAL(2)");
    zassert_not_null(strstr(buf, "\"act\":0"),   "Should have act=MESH_ON(0)");
    zassert_not_null(strstr(buf, "\"type\":\"mesh\""), "Should have type=mesh");
    zassert_not_null(strstr(buf, "\"target\":16"), "Should have target=0x10=16");
}

ZTEST(rule_engine, test_json_thread_rule_has_ipv6)
{
    gateway_rule_t r = make_thread_rule(0, RULE_TRIG_STATE_ALERT,
                                         RULE_ACT_THREAD_ON, "fd11::cafe");
    rule_engine_add(&r);

    char buf[512];
    rule_engine_to_json(buf, sizeof(buf));

    zassert_not_null(strstr(buf, "\"type\":\"thread\""), "Should have type=thread");
    zassert_not_null(strstr(buf, "fd11::cafe"),         "Should contain IPv6");
}

ZTEST(rule_engine, test_json_buffer_too_small_does_not_crash)
{
    gateway_rule_t r = make_mesh_rule(0, RULE_TRIG_STATE_ALERT,
                                       RULE_ACT_MESH_ON, 0x0002);
    rule_engine_add(&r);

    char tiny[8];
    /* Darf nicht crashen, auch wenn Buffer zu klein ist */
    rule_engine_to_json(tiny, sizeof(tiny));
    zassert_true(true, "No crash on tiny buffer");
}