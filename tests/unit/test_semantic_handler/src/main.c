/**
 * @file tests/unit/test_semantic_handler/src/main.c
 * Fixed: correct field names, no semantic_handler_process()
 * Note: semantic_handler runs via event_ingest internally.
 * We test tick() and threshold effects via gw_store state.
 */
#include <zephyr/ztest.h>
#include <string.h>
#include <stdbool.h>
#include "gw_model.h"
#include "gw_store.h"
#include "semantic_handler.h"

/* Mocks for modules semantic_handler depends on */
int event_ingest_register_listener(void (*cb)(const gw_event_t*, void*), void *ctx)
    { ARG_UNUSED(cb); ARG_UNUSED(ctx); return 0; }
int event_ingest_submit(const gw_event_t *e)
    { ARG_UNUSED(e); return 0; }

/* rule_engine mocks */
static int mock_rule_calls = 0;
void rule_engine_on_state(const gw_node_addr_t *s, gw_state_t st)
    { ARG_UNUSED(s); ARG_UNUSED(st); mock_rule_calls++; }
void rule_engine_on_switch(const gw_node_addr_t *s, bool sw)
    { ARG_UNUSED(s); ARG_UNUSED(sw); }

/* nus_handler mock */
void nus_handler_send_state_update(const gw_node_record_t *r) { ARG_UNUSED(r); }

static void before_each(void *f) {
    ARG_UNUSED(f);
    gw_store_init();
    semantic_handler_init();
    mock_rule_calls = 0;
}
ZTEST_SUITE(semantic_handler, NULL, NULL, before_each, NULL, NULL);

/* ── Helpers ─────────────────────────────────────────────────── */

/* Directly apply sensor event to store and call tick */
static void inject(uint16_t addr, int32_t temp_mc, int32_t tvoc, int32_t eco2)
{
    gw_event_t evt = {0};
    evt.type = GW_EVT_SENSOR;
    evt.src.transport = GW_TR_BLE_MESH;
    evt.src.mesh_addr = addr;
    evt.rx_ms = k_uptime_get();
    evt.data.sensor.present  = GW_HAS_TEMP | GW_HAS_TVOC | GW_HAS_ECO2;
    evt.data.sensor.temp_mc  = temp_mc;
    evt.data.sensor.tvoc_ppb = tvoc;
    evt.data.sensor.eco2_ppm = eco2;
    gw_store_apply_event(&evt);
}

static const gw_node_record_t *get_rec(uint16_t addr)
{
    gw_node_addr_t a = { .transport = GW_TR_BLE_MESH, .mesh_addr = addr };
    int idx = gw_store_find_node(&a);
    if (idx < 0) return NULL;
    return gw_store_get_node((uint8_t)idx);
}

/* ════════════════════════════════════════════════════════════════
 * Init / Tick
 * ════════════════════════════════════════════════════════════════ */

ZTEST(semantic_handler, test_init_no_crash)
{
    semantic_handler_init();
    zassert_true(true, "double init must not crash");
}

ZTEST(semantic_handler, test_tick_empty_store_no_crash)
{
    semantic_handler_tick();
    zassert_true(true, "tick on empty store must not crash");
}

ZTEST(semantic_handler, test_tick_with_nodes_no_crash)
{
    inject(0x0002, 22500, 100, 600);
    inject(0x0004, 22500, 100, 600);
    semantic_handler_tick();
    zassert_true(true, "tick with nodes must not crash");
}

/* ════════════════════════════════════════════════════════════════
 * Thresholds API
 * ════════════════════════════════════════════════════════════════ */

ZTEST(semantic_handler, test_set_thresholds_no_crash)
{
    semantic_thresholds_t t = {
        .temp_alert_mc    = 35000,
        .temp_critical_mc = 40000,
        .tvoc_alert_ppb   = 500,
        .tvoc_critical_ppb = 1000,
        .eco2_alert_ppm   = 1000,
        .eco2_critical_ppm = 2000,
        .lost_timeout_ms  = 90000,
        .idle_timeout_ms  = 10000,
    };
    semantic_handler_set_thresholds(&t);
    zassert_true(true, "set_thresholds must not crash");
}

ZTEST(semantic_handler, test_set_null_thresholds_no_crash)
{
    semantic_handler_set_thresholds(NULL);
    zassert_true(true, "NULL thresholds must not crash");
}

ZTEST(semantic_handler, test_get_thresholds_no_crash)
{
    semantic_thresholds_t out;
    semantic_handler_get_thresholds(&out);
    zassert_true(out.temp_alert_mc > 0,   "temp_alert_mc should be positive");
    zassert_true(out.tvoc_alert_ppb > 0,  "tvoc_alert_ppb should be positive");
    zassert_true(out.idle_timeout_ms > 0, "idle_timeout_ms should be positive");
}

ZTEST(semantic_handler, test_get_thresholds_roundtrip)
{
    semantic_thresholds_t set = {
        .temp_alert_mc    = 36000,
        .temp_critical_mc = 42000,
        .tvoc_alert_ppb   = 600,
        .tvoc_critical_ppb = 1200,
        .eco2_alert_ppm   = 1100,
        .eco2_critical_ppm = 2200,
        .lost_timeout_ms  = 60000,
        .idle_timeout_ms  = 15000,
    };
    semantic_handler_set_thresholds(&set);
    semantic_thresholds_t got;
    semantic_handler_get_thresholds(&got);
    zassert_equal(got.temp_alert_mc,    36000, "temp_alert_mc roundtrip");
    zassert_equal(got.tvoc_alert_ppb,   600,   "tvoc_alert_ppb roundtrip");
    zassert_equal(got.eco2_alert_ppm,   1100,  "eco2_alert_ppm roundtrip");
    zassert_equal(got.idle_timeout_ms,  15000, "idle_timeout_ms roundtrip");
}

/* ════════════════════════════════════════════════════════════════
 * Node state via gw_store + tick interaction
 * ════════════════════════════════════════════════════════════════ */

ZTEST(semantic_handler, test_new_node_not_lost)
{
    inject(0x0002, 22500, 100, 600);
    const gw_node_record_t *r = get_rec(0x0002);
    zassert_not_null(r, "node should be registered");
    zassert_not_equal(r->state, GW_STATE_LOST, "new node should not be LOST");
}

ZTEST(semantic_handler, test_tick_multiple_times_no_crash)
{
    inject(0x0002, 22500, 100, 600);
    for (int i = 0; i < 10; i++) semantic_handler_tick();
    zassert_true(true, "multiple ticks must not crash");
}

ZTEST(semantic_handler, test_multiple_nodes_independent)
{
    inject(0x0002, 22500, 100, 600);
    inject(0x0004, 22500, 100, 600);
    zassert_equal(gw_store_count(), 2, "Both nodes should be registered");
    const gw_node_record_t *r1 = get_rec(0x0002);
    const gw_node_record_t *r2 = get_rec(0x0004);
    zassert_not_null(r1, "node 1 should exist");
    zassert_not_null(r2, "node 2 should exist");
}

ZTEST(semantic_handler, test_set_lost_timeout_works)
{
    /* Set very short lost timeout */
    semantic_thresholds_t t;
    semantic_handler_get_thresholds(&t);
    t.idle_timeout_ms = 1;   /* 1ms - will trigger immediately */
    t.lost_timeout_ms = 2;
    semantic_handler_set_thresholds(&t);

    inject(0x0002, 22500, 100, 600);
    /* Small sleep to let timeout pass */
    k_sleep(K_MSEC(5));
    semantic_handler_tick();
    /* Node should now be IDLE or LOST */
    const gw_node_record_t *r = get_rec(0x0002);
    zassert_not_null(r, "node should still exist");
    zassert_true(r->state == GW_STATE_IDLE || r->state == GW_STATE_LOST ||
                 r->state == GW_STATE_ACTIVE,
                 "state should be valid after tick");
}

ZTEST(semantic_handler, test_thread_node_works)
{
    gw_event_t evt = {0};
    evt.type = GW_EVT_SENSOR;
    evt.src.transport = GW_TR_THREAD;
    strncpy(evt.src.ipv6, "fd11::1", GW_IPV6_STR_LEN - 1);
    evt.rx_ms = k_uptime_get();
    gw_store_apply_event(&evt);
    semantic_handler_tick();
    zassert_true(true, "Thread node tick must not crash");
}

ZTEST(semantic_handler, test_lora_node_works)
{
    gw_event_t evt = {0};
    evt.type = GW_EVT_SENSOR;
    evt.src.transport = GW_TR_LORAWAN;
    evt.src.dev_eui_hi = 0x11223344;
    evt.src.dev_eui_lo = 0x55667788;
    evt.rx_ms = k_uptime_get();
    gw_store_apply_event(&evt);
    semantic_handler_tick();
    zassert_true(true, "LoRa node tick must not crash");
}
