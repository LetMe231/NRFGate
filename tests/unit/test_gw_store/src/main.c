/**
 * @file tests/unit/test_gw_store/src/main.c
 * Unit-Tests für gw_store.c — vollständig inkl. Edge Cases
 */

#include <zephyr/ztest.h>
#include <string.h>
#include <stdint.h>

#include "gw_model.h"
#include "gw_store.h"

static void before_each(void *f) { ARG_UNUSED(f); gw_store_init(); }
ZTEST_SUITE(gw_store, NULL, NULL, before_each, NULL, NULL);

/* ── Helpers ──────────────────────────────────────────────────── */

static gw_event_t make_ble_event(uint16_t mesh_addr)
{
    gw_event_t evt = {0};
    evt.type          = GW_EVT_SENSOR;
    evt.src.transport = GW_TR_BLE_MESH;
    evt.src.mesh_addr = mesh_addr;
    evt.rx_ms         = 1000;
    return evt;
}

static gw_event_t make_thread_event(const char *ipv6)
{
    gw_event_t evt = {0};
    evt.type          = GW_EVT_SENSOR;
    evt.src.transport = GW_TR_THREAD;
    evt.rx_ms         = 1000;
    strncpy(evt.src.ipv6, ipv6, GW_IPV6_STR_LEN - 1);
    return evt;
}

static gw_event_t make_lora_event(uint32_t eui_hi, uint32_t eui_lo)
{
    gw_event_t evt = {0};
    evt.type              = GW_EVT_SENSOR;
    evt.src.transport     = GW_TR_LORAWAN;
    evt.src.dev_eui_hi    = eui_hi;
    evt.src.dev_eui_lo    = eui_lo;
    evt.rx_ms             = 1000;
    return evt;
}

/* ════════════════════════════════════════════════════════════════
 * find_node — unbekannte Nodes
 * ════════════════════════════════════════════════════════════════ */

ZTEST(gw_store, test_find_unknown_ble_returns_negative)
{
    gw_node_addr_t addr = { .transport = GW_TR_BLE_MESH, .mesh_addr = 0x0002 };
    zassert_true(gw_store_find_node(&addr) < 0,
                 "Unknown BLE node should return negative");
}

ZTEST(gw_store, test_find_unknown_thread_returns_negative)
{
    gw_node_addr_t addr = { .transport = GW_TR_THREAD };
    strncpy(addr.ipv6, "fd11::9999", GW_IPV6_STR_LEN - 1);
    zassert_true(gw_store_find_node(&addr) < 0,
                 "Unknown Thread node should return negative");
}

ZTEST(gw_store, test_find_null_addr_returns_negative)
{
    int idx = gw_store_find_node(NULL);
    zassert_true(idx < 0, "NULL addr should return negative");
}

ZTEST(gw_store, test_find_after_apply_event)
{
    gw_event_t evt = make_ble_event(0x0002);
    gw_store_apply_event(&evt);
    gw_node_addr_t addr = { .transport = GW_TR_BLE_MESH, .mesh_addr = 0x0002 };
    zassert_true(gw_store_find_node(&addr) >= 0,
                 "Node should be findable after apply_event");
}

ZTEST(gw_store, test_find_thread_node_by_ipv6)
{
    gw_event_t evt = make_thread_event("fd11::cafe:1");
    gw_store_apply_event(&evt);
    gw_node_addr_t addr = { .transport = GW_TR_THREAD };
    strncpy(addr.ipv6, "fd11::cafe:1", GW_IPV6_STR_LEN - 1);
    zassert_true(gw_store_find_node(&addr) >= 0,
                 "Thread node should be findable by IPv6");
}

ZTEST(gw_store, test_find_lora_node_by_eui)
{
    gw_event_t evt = make_lora_event(0xDEADBEEF, 0xCAFEBABE);
    gw_store_apply_event(&evt);
    gw_node_addr_t addr = {
        .transport  = GW_TR_LORAWAN,
        .dev_eui_hi = 0xDEADBEEF,
        .dev_eui_lo = 0xCAFEBABE,
    };
    zassert_true(gw_store_find_node(&addr) >= 0,
                 "LoRa node should be findable by EUI");
}

/* ════════════════════════════════════════════════════════════════
 * Node-Registrierung
 * ════════════════════════════════════════════════════════════════ */

ZTEST(gw_store, test_first_node_gets_index_0)
{
    gw_event_t evt = make_ble_event(0x0002);
    gw_store_apply_event(&evt);
    gw_node_addr_t addr = { .transport = GW_TR_BLE_MESH, .mesh_addr = 0x0002 };
    zassert_equal(gw_store_find_node(&addr), 0,
                  "First node should get index 0");
}

ZTEST(gw_store, test_same_node_same_index)
{
    gw_event_t evt = make_ble_event(0x0004);
    gw_store_apply_event(&evt);
    gw_node_addr_t addr = { .transport = GW_TR_BLE_MESH, .mesh_addr = 0x0004 };
    int idx1 = gw_store_find_node(&addr);
    evt.rx_ms += 1000;
    gw_store_apply_event(&evt);
    int idx2 = gw_store_find_node(&addr);
    zassert_equal(idx1, idx2, "Same node must always get same index");
    zassert_true(idx1 >= 0,   "Index must be valid");
}

ZTEST(gw_store, test_different_nodes_different_indices)
{
    gw_event_t _ea = { .type = GW_EVT_SENSOR,
        .src = { .transport = GW_TR_BLE_MESH, .mesh_addr = 0x0002 }, .rx_ms = 1 };
    gw_store_apply_event(&_ea);
    gw_event_t _eb = { .type = GW_EVT_SENSOR,
        .src = { .transport = GW_TR_BLE_MESH, .mesh_addr = 0x0004 }, .rx_ms = 1 };
    gw_store_apply_event(&_eb);

    gw_node_addr_t a1 = { .transport = GW_TR_BLE_MESH, .mesh_addr = 0x0002 };
    gw_node_addr_t a2 = { .transport = GW_TR_BLE_MESH, .mesh_addr = 0x0004 };
    zassert_not_equal(gw_store_find_node(&a1), gw_store_find_node(&a2),
                      "Different nodes must have different indices");
}

ZTEST(gw_store, test_ble_and_thread_same_addr_different_slots)
{
    /* BLE mesh_addr=2 und Thread mit ipv6 — völlig unterschiedlich */
    gw_event_t ble = make_ble_event(0x0002);
    gw_event_t thr = make_thread_event("fd11::1");
    gw_store_apply_event(&ble);
    gw_store_apply_event(&thr);

    gw_node_addr_t ba = { .transport = GW_TR_BLE_MESH, .mesh_addr = 0x0002 };
    gw_node_addr_t ta = { .transport = GW_TR_THREAD };
    strncpy(ta.ipv6, "fd11::1", GW_IPV6_STR_LEN - 1);

    zassert_not_equal(gw_store_find_node(&ba), gw_store_find_node(&ta),
                      "BLE and Thread nodes must have different slots");
}

ZTEST(gw_store, test_all_three_transports_coexist)
{
    gw_event_t _ec = { .type = GW_EVT_SENSOR,
        .src = { .transport = GW_TR_BLE_MESH, .mesh_addr = 0x0002 }, .rx_ms = 1 };
    gw_store_apply_event(&_ec);
    { gw_event_t _tmp1 = make_thread_event("fd11::1"); gw_store_apply_event(&_tmp1); }
    { gw_event_t _tmp2 = make_lora_event(0x11111111, 0x22222222); gw_store_apply_event(&_tmp2); }

    zassert_equal(gw_store_count(), 3,
                  "All three transports should coexist");
}

/* ════════════════════════════════════════════════════════════════
 * gw_store_count
 * ════════════════════════════════════════════════════════════════ */

ZTEST(gw_store, test_count_after_init)
{
    zassert_equal(gw_store_count(), 0, "Store should be empty after init");
}

ZTEST(gw_store, test_count_increases_with_new_nodes)
{
    gw_event_t _ec = { .type = GW_EVT_SENSOR,
        .src = { .transport = GW_TR_BLE_MESH, .mesh_addr = 0x0002 }, .rx_ms = 1 };
    gw_store_apply_event(&_ec);
    zassert_equal(gw_store_count(), 1, "Count should be 1");

    gw_event_t _ed = { .type = GW_EVT_SENSOR,
        .src = { .transport = GW_TR_BLE_MESH, .mesh_addr = 0x0004 }, .rx_ms = 1 };
    gw_store_apply_event(&_ed);
    zassert_equal(gw_store_count(), 2, "Count should be 2");
}

ZTEST(gw_store, test_count_same_node_twice_no_increment)
{
    gw_event_t evt = make_ble_event(0x0002);
    gw_store_apply_event(&evt);
    gw_store_apply_event(&evt);
    zassert_equal(gw_store_count(), 1,
                  "Same node twice should not increment count");
}

/* ════════════════════════════════════════════════════════════════
 * gw_store_get_node
 * ════════════════════════════════════════════════════════════════ */

ZTEST(gw_store, test_get_node_valid_index)
{
    gw_event_t evt = make_ble_event(0x0006);
    gw_store_apply_event(&evt);
    gw_node_addr_t addr = { .transport = GW_TR_BLE_MESH, .mesh_addr = 0x0006 };
    int idx = gw_store_find_node(&addr);
    const gw_node_record_t *rec = gw_store_get_node((uint8_t)idx);
    zassert_not_null(rec, "get_node should return non-null");
    zassert_equal(rec->addr.mesh_addr, 0x0006, "mesh_addr wrong");
}

ZTEST(gw_store, test_get_node_out_of_range_returns_null)
{
    zassert_is_null(gw_store_get_node(255),
                    "Out-of-range index should return NULL");
}

ZTEST(gw_store, test_get_node_transport_correct)
{
    gw_event_t evt = make_thread_event("fd11::abc");
    gw_store_apply_event(&evt);
    gw_node_addr_t addr = { .transport = GW_TR_THREAD };
    strncpy(addr.ipv6, "fd11::abc", GW_IPV6_STR_LEN - 1);
    int idx = gw_store_find_node(&addr);
    const gw_node_record_t *rec = gw_store_get_node((uint8_t)idx);
    zassert_not_null(rec, "Record should exist");
    zassert_equal(rec->addr.transport, GW_TR_THREAD,
                  "Transport should be GW_TR_THREAD");
}

/* ════════════════════════════════════════════════════════════════
 * last_seen_ms aktualisierung
 * ════════════════════════════════════════════════════════════════ */

ZTEST(gw_store, test_last_seen_ms_updated)
{
    gw_event_t evt = make_ble_event(0x0002);
    evt.rx_ms = 1000;
    gw_store_apply_event(&evt);

    evt.rx_ms = 5000;
    gw_store_apply_event(&evt);

    gw_node_addr_t addr = { .transport = GW_TR_BLE_MESH, .mesh_addr = 0x0002 };
    int idx = gw_store_find_node(&addr);
    const gw_node_record_t *rec = gw_store_get_node((uint8_t)idx);

    zassert_not_null(rec, "Record should exist");
    zassert_equal(rec->last_seen_ms, 5000,
                  "last_seen_ms should be updated to latest rx_ms");
}

/* ════════════════════════════════════════════════════════════════
 * State Updates
 * ════════════════════════════════════════════════════════════════ */

ZTEST(gw_store, test_new_node_state_is_not_lost)
{
    gw_event_t evt = make_ble_event(0x0002);
    gw_store_apply_event(&evt);
    gw_node_addr_t addr = { .transport = GW_TR_BLE_MESH, .mesh_addr = 0x0002 };
    int idx = gw_store_find_node(&addr);
    const gw_node_record_t *rec = gw_store_get_node((uint8_t)idx);
    zassert_not_null(rec, "Record should exist");
    zassert_not_equal(rec->state, GW_STATE_LOST,
                      "New node should not start as LOST");
}

ZTEST(gw_store, test_state_transition_applied)
{
    gw_event_t evt = make_ble_event(0x0002);
    gw_store_apply_event(&evt);
    gw_node_addr_t addr = { .transport = GW_TR_BLE_MESH, .mesh_addr = 0x0002 };
    int idx = gw_store_find_node(&addr);

    gw_event_t st = {0};
    st.type = GW_EVT_STATE_TRANSITION;
    st.src  = addr;
    st.data.state_transition.from = GW_STATE_UNKNOWN;
    st.data.state_transition.to   = GW_STATE_ACTIVE;
    gw_store_apply_event(&st);

    const gw_node_record_t *rec = gw_store_get_node((uint8_t)idx);
    zassert_equal(rec->state, GW_STATE_ACTIVE,
                  "State should be ACTIVE after transition");
}

ZTEST(gw_store, test_all_states_can_be_set)
{
    gw_state_t states[] = {
        GW_STATE_IDLE, GW_STATE_ACTIVE,
        GW_STATE_ALERT, GW_STATE_CRITICAL, GW_STATE_LOST
    };
    gw_event_t evt = make_ble_event(0x0002);
    gw_store_apply_event(&evt);
    gw_node_addr_t addr = { .transport = GW_TR_BLE_MESH, .mesh_addr = 0x0002 };
    int idx = gw_store_find_node(&addr);

    for (int i = 0; i < ARRAY_SIZE(states); i++) {
        gw_event_t st = {0};
        st.type = GW_EVT_STATE_TRANSITION;
        st.src  = addr;
        st.data.state_transition.to = states[i];
        gw_store_apply_event(&st);

        const gw_node_record_t *rec = gw_store_get_node((uint8_t)idx);
        zassert_equal(rec->state, states[i],
                      "State %d not applied", states[i]);
    }
}

/* ════════════════════════════════════════════════════════════════
 * Sensor-Felder
 * ════════════════════════════════════════════════════════════════ */

ZTEST(gw_store, test_sensor_fields_stored)
{
    gw_event_t evt = make_ble_event(0x0002);
    evt.data.sensor.present       = GW_HAS_TEMP | GW_HAS_HUM;
    evt.data.sensor.temp_mc       = 22500;
    evt.data.sensor.hum_mpermille = 650000;
    gw_store_apply_event(&evt);

    gw_node_addr_t addr = { .transport = GW_TR_BLE_MESH, .mesh_addr = 0x0002 };
    int idx = gw_store_find_node(&addr);
    const gw_node_record_t *rec = gw_store_get_node((uint8_t)idx);

    zassert_true(rec->last_sensor.present & GW_HAS_TEMP, "GW_HAS_TEMP missing");
    zassert_true(rec->last_sensor.present & GW_HAS_HUM,  "GW_HAS_HUM missing");
    zassert_equal(rec->last_sensor.temp_mc,       22500,  "temp_mc wrong");
    zassert_equal(rec->last_sensor.hum_mpermille, 650000, "hum_mpermille wrong");
}

ZTEST(gw_store, test_sensor_fields_overwritten_on_update)
{
    gw_event_t evt = make_ble_event(0x0002);
    evt.data.sensor.present = GW_HAS_TEMP;
    evt.data.sensor.temp_mc = 22500;
    gw_store_apply_event(&evt);

    evt.data.sensor.temp_mc = 30000;
    gw_store_apply_event(&evt);

    gw_node_addr_t addr = { .transport = GW_TR_BLE_MESH, .mesh_addr = 0x0002 };
    int idx = gw_store_find_node(&addr);
    const gw_node_record_t *rec = gw_store_get_node((uint8_t)idx);
    zassert_equal(rec->last_sensor.temp_mc, 30000,
                  "Sensor update should overwrite previous value");
}

ZTEST(gw_store, test_air_quality_fields_stored)
{
    gw_event_t evt = make_thread_event("fd11::2");
    evt.data.sensor.present  = GW_HAS_TVOC | GW_HAS_ECO2;
    evt.data.sensor.tvoc_ppb = 350;
    evt.data.sensor.eco2_ppm = 800;
    gw_store_apply_event(&evt);

    gw_node_addr_t addr = { .transport = GW_TR_THREAD };
    strncpy(addr.ipv6, "fd11::2", GW_IPV6_STR_LEN - 1);
    int idx = gw_store_find_node(&addr);
    const gw_node_record_t *rec = gw_store_get_node((uint8_t)idx);

    zassert_equal(rec->last_sensor.tvoc_ppb, 350, "tvoc_ppb wrong");
    zassert_equal(rec->last_sensor.eco2_ppm, 800, "eco2_ppm wrong");
}

/* ════════════════════════════════════════════════════════════════
 * Button / Actuator Events
 * ════════════════════════════════════════════════════════════════ */

ZTEST(gw_store, test_button_event_stored)
{
    gw_event_t evt = {0};
    evt.type               = GW_EVT_BUTTON;
    evt.src.transport      = GW_TR_BLE_MESH;
    evt.src.mesh_addr      = 0x0004;
    evt.rx_ms              = 1000;
    evt.data.button.pressed = true;
    evt.data.button.seq     = 42;
    gw_store_apply_event(&evt);

    gw_node_addr_t addr = { .transport = GW_TR_BLE_MESH, .mesh_addr = 0x0004 };
    int idx = gw_store_find_node(&addr);
    const gw_node_record_t *rec = gw_store_get_node((uint8_t)idx);

    zassert_true(rec->has_last_button,     "has_last_button not set");
    zassert_true(rec->last_button_pressed, "last_button_pressed wrong");
}

ZTEST(gw_store, test_actuator_state_stored)
{
    gw_event_t evt = {0};
    evt.type                    = GW_EVT_ACTUATOR_STATE;
    evt.src.transport           = GW_TR_BLE_MESH;
    evt.src.mesh_addr           = 0x0006;
    evt.rx_ms                   = 1000;
    evt.data.actuator_state.light_on = true;
    gw_store_apply_event(&evt);

    gw_node_addr_t addr = { .transport = GW_TR_BLE_MESH, .mesh_addr = 0x0006 };
    int idx = gw_store_find_node(&addr);
    const gw_node_record_t *rec = gw_store_get_node((uint8_t)idx);

    zassert_true(rec->has_last_actuator_state, "has_last_actuator_state not set");
    zassert_true(rec->last_light_on,           "last_light_on wrong");
}

/* ════════════════════════════════════════════════════════════════
 * gw_store_foreach
 * ════════════════════════════════════════════════════════════════ */

static int s_foreach_count = 0;
static void count_cb(const gw_node_record_t *rec, void *ctx)
{
    ARG_UNUSED(rec); ARG_UNUSED(ctx);
    s_foreach_count++;
}

ZTEST(gw_store, test_foreach_called_for_all_nodes)
{
    gw_event_t _ea = { .type = GW_EVT_SENSOR,
        .src = { .transport = GW_TR_BLE_MESH, .mesh_addr = 0x0002 }, .rx_ms = 1 };
    gw_store_apply_event(&_ea);
    gw_event_t _eb = { .type = GW_EVT_SENSOR,
        .src = { .transport = GW_TR_BLE_MESH, .mesh_addr = 0x0004 }, .rx_ms = 1 };
    gw_store_apply_event(&_eb);
    { gw_event_t _tmp3 = make_thread_event("fd11::1"); gw_store_apply_event(&_tmp3); }

    s_foreach_count = 0;
    gw_store_foreach_node(count_cb, NULL);
    zassert_equal(s_foreach_count, 3,
                  "foreach should visit all 3 nodes");
}

ZTEST(gw_store, test_foreach_empty_store_no_calls)
{
    s_foreach_count = 0;
    gw_store_foreach_node(count_cb, NULL);
    zassert_equal(s_foreach_count, 0,
                  "foreach on empty store should make no calls");
}

/* ════════════════════════════════════════════════════════════════
 * Store voll
 * ════════════════════════════════════════════════════════════════ */

ZTEST(gw_store, test_store_full_rejects_new_node)
{
    for (int i = 0; i < GW_STORE_MAX_NODES; i++) {
        gw_event_t evt = make_ble_event((uint16_t)(0x0002 + i));
        gw_store_apply_event(&evt);
    }
    zassert_equal(gw_store_count(), GW_STORE_MAX_NODES, "Store should be full");

    { gw_event_t _tmp4 = make_ble_event(0x0F00); gw_store_apply_event(&_tmp4); }
    gw_node_addr_t addr = { .transport = GW_TR_BLE_MESH, .mesh_addr = 0x0F00 };
    zassert_true(gw_store_find_node(&addr) < 0,
                 "Overflow node must not be registered");
}

ZTEST(gw_store, test_store_full_existing_node_still_updated)
{
    /* Fülle Store */
    for (int i = 0; i < GW_STORE_MAX_NODES; i++) {
        gw_event_t evt = make_ble_event((uint16_t)(0x0002 + i));
        gw_store_apply_event(&evt);
    }

    /* Update auf bereits existierenden Node muss trotzdem klappen */
    gw_event_t upd = make_ble_event(0x0002);
    upd.data.sensor.present = GW_HAS_TEMP;
    upd.data.sensor.temp_mc = 99999;
    gw_store_apply_event(&upd);

    gw_node_addr_t addr = { .transport = GW_TR_BLE_MESH, .mesh_addr = 0x0002 };
    int idx = gw_store_find_node(&addr);
    const gw_node_record_t *rec = gw_store_get_node((uint8_t)idx);
    zassert_equal(rec->last_sensor.temp_mc, 99999,
                  "Existing node should still be updated when store is full");
}

/* ════════════════════════════════════════════════════════════════
 * apply_event mit NULL — kein Crash
 * ════════════════════════════════════════════════════════════════ */

ZTEST(gw_store, test_apply_null_no_crash)
{
    gw_store_apply_event(NULL);
    zassert_true(true, "apply_event(NULL) must not crash");
}
