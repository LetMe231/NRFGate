/**
 * @file tests/unit/test_semantic/src/main.c
 *
 * Unit-Tests für semantic_handler.c.
 *
 * Was getestet wird:
 *   - Erstes Paket → IDLE
 *   - CO2-Schwellwerte: ACTIVE / ALERT / CRITICAL
 *   - Hysterese: Zustand bleibt nach kurzem Rückgang
 *   - SUSTAINED: ACTIVE erst nach SUSTAINED_ALERT_MS
 *   - CO2-Trend: schneller Anstieg → CRITICAL
 *   - IMU: Bewegungsschwellwert → ALERT
 *   - Node Lost: kein Paket für node_lost_timeout_ms
 *   - Nach LOST: neues Paket → wieder IDLE
 *   - semantic_handler_tick() meldet LOST via NUS
 *   - rule_engine_on_state() wird bei Transition aufgerufen
 */

#include <zephyr/ztest.h>
#include <zephyr/kernel.h>
#include <string.h>

#include "semantic_handler.h"
#include "data_handler.h"

/* ── Mock-Zustand ────────────────────────────────────────────── */
static uint8_t     mock_rule_node_idx  = 0xFF;
static node_state_t mock_rule_state    = NODE_STATE_UNKNOWN;
static int          mock_rule_calls    = 0;
static bool         mock_nus_ready     = false;
static char         mock_nus_last[64]  = {0};
static int          mock_nus_calls     = 0;

static void reset_mocks(void)
{
    mock_rule_node_idx = 0xFF;
    mock_rule_state    = NODE_STATE_UNKNOWN;
    mock_rule_calls    = 0;
    mock_nus_ready     = false;
    memset(mock_nus_last, 0, sizeof(mock_nus_last));
    mock_nus_calls     = 0;
}

/* ── Mocks ────────────────────────────────────────────────────── */

/* rule_engine.h */
void rule_engine_on_state(uint8_t node_idx, node_state_t new_state,
                           node_transport_t src_transport)
{
    ARG_UNUSED(src_transport);
    mock_rule_node_idx = node_idx;
    mock_rule_state    = new_state;
    mock_rule_calls++;
}

/* ble_nus.h */
bool ble_nus_is_ready(void)                { return mock_nus_ready; }
void ble_nus_send(const char *json)
{
    strncpy(mock_nus_last, json, sizeof(mock_nus_last) - 1);
    mock_nus_calls++;
}

/* main.h */
void mesh_scheduler_request_priority(sched_priority_t transport, uint32_t duration_ms)
{
    ARG_UNUSED(transport);
    ARG_UNUSED(duration_ms);
}
void mesh_scheduler_set_mode(sched_mode_t mode) { ARG_UNUSED(mode); }
void mesh_scheduler_pause(void)                 {}
void mesh_scheduler_start(void)                 {}

/* data_handler.h — wird von semantic_handler nicht direkt benötigt,
 * aber über data_handler.h inkludierte Typen schon */
node_actuator_state_t node_actuator_state[MAX_NODES];

/* ── Helper: Test-Paket erstellen ────────────────────────────── */
static struct node_sensor_data make_packet(uint8_t node_idx, int64_t uptime_ms)
{
    struct node_sensor_data d = {0};
    d.node_idx      = node_idx;
    d.rx_uptime_ms  = uptime_ms;
    d.identity.transport = NODE_TRANSPORT_THREAD;
    return d;
}

static struct node_sensor_data make_co2_packet(uint8_t node_idx,
                                                int64_t uptime_ms,
                                                int32_t co2_ppm)
{
    struct node_sensor_data d = make_packet(node_idx, uptime_ms);
    d.payload.present = SENSOR_HAS_ECO2;
    d.payload.eco2    = co2_ppm;
    return d;
}

static struct node_sensor_data make_imu_packet(uint8_t node_idx,
                                                int64_t uptime_ms,
                                                int32_t ax, int32_t ay, int32_t az)
{
    struct node_sensor_data d = make_packet(node_idx, uptime_ms);
    d.payload.present = SENSOR_HAS_ACCEL;
    d.payload.ax = ax;
    d.payload.ay = ay;
    d.payload.az = az;
    return d;
}

/* ── Suite Setup ─────────────────────────────────────────────── */
static void before_each(void *f)
{
    ARG_UNUSED(f);
    semantic_handler_init();
    reset_mocks();
    /* Schwellwerte auf Defaults zurücksetzen */
    semantic_handler_set_policy(-1, -1, 2000, -1, -1);
}

ZTEST_SUITE(semantic, NULL, NULL, before_each, NULL, NULL);

/* ════════════════════════════════════════════════════════════════
 * Erster Empfang
 * ════════════════════════════════════════════════════════════════ */

ZTEST(semantic, test_first_packet_transitions_to_idle)
{
    struct node_sensor_data d = make_packet(0, 1000);
    semantic_handler_process(&d);

    zassert_equal(semantic_handler_get_state(0), NODE_STATE_IDLE,
                  "First packet should transition to IDLE");
}

ZTEST(semantic, test_first_packet_fires_rule_engine)
{
    struct node_sensor_data d = make_packet(0, 1000);
    semantic_handler_process(&d);

    /* UNKNOWN → IDLE ist eine Transition → rule_engine wird benachrichtigt */
    zassert_true(mock_rule_calls >= 1,
                 "rule_engine_on_state should be called on UNKNOWN->IDLE");
}

ZTEST(semantic, test_invalid_node_index_ignored)
{
    struct node_sensor_data d = make_packet(MAX_NODES, 1000); /* out of range */
    semantic_handler_process(&d);
    /* Kein Crash, keine Zustandsänderung */
    zassert_equal(semantic_handler_get_state(MAX_NODES), NODE_STATE_UNKNOWN,
                  "Out-of-range node should remain UNKNOWN");
}

/* ════════════════════════════════════════════════════════════════
 * CO2 Schwellwerte
 * ════════════════════════════════════════════════════════════════ */

ZTEST(semantic, test_co2_normal_stays_idle)
{
    /* Erstes Paket: IDLE */
    struct node_sensor_data d = make_co2_packet(0, 1000, 400);
    semantic_handler_process(&d);
    /* Warte SUSTAINED_ALERT_MS */
    d = make_co2_packet(0, 1000 + SUSTAINED_ALERT_MS + 100, 400);
    semantic_handler_process(&d);

    zassert_equal(semantic_handler_get_state(0), NODE_STATE_IDLE,
                  "Normal CO2 should stay IDLE");
}

ZTEST(semantic, test_co2_above_active_threshold)
{
    struct node_sensor_data d = make_co2_packet(0, 1, 950);
    semantic_handler_process(&d);
    /* Erstes Paket über Schwelle — setzt Sustain-Timer */
    d = make_co2_packet(0, 1000, 1100);
    semantic_handler_process(&d);
    /* Zweites Paket nach SUSTAINED_ALERT_MS — jetzt ACTIVE */
    d = make_co2_packet(0, 1000 + SUSTAINED_ALERT_MS + 100, 1100);
    semantic_handler_process(&d);
    zassert_equal(semantic_handler_get_state(0), NODE_STATE_ACTIVE, NULL);
}


ZTEST(semantic, test_co2_above_alert_threshold)
{
    struct node_sensor_data d = make_co2_packet(0, 1, 1900); // nahe 2000
    semantic_handler_process(&d);
    d = make_co2_packet(0, 1000, 2100);
    semantic_handler_process(&d);
    zassert_equal(semantic_handler_get_state(0), NODE_STATE_ALERT, NULL);
}


ZTEST(semantic, test_co2_above_critical_threshold)
{
    struct node_sensor_data d = make_co2_packet(0, 0, 400);
    semantic_handler_process(&d);

    /* CO2 = 5100 ppm (> THRESH_CO2_CRITICAL=5000) → CRITICAL */
    d = make_co2_packet(0, 1000, 5100);
    semantic_handler_process(&d);

    zassert_equal(semantic_handler_get_state(0), NODE_STATE_CRITICAL,
                  "CO2=5100ppm should be CRITICAL");
    zassert_equal(mock_rule_state, NODE_STATE_CRITICAL,
                  "rule_engine should be notified with CRITICAL");
}

/* ════════════════════════════════════════════════════════════════
 * Hysterese
 * ════════════════════════════════════════════════════════════════ */

ZTEST(semantic, test_co2_hysteresis_stays_alert)
{
    struct node_sensor_data d = make_co2_packet(0, 1, 1900);
    semantic_handler_process(&d);
    d = make_co2_packet(0, 1000, 2100);
    semantic_handler_process(&d);
    zassert_equal(semantic_handler_get_state(0), NODE_STATE_ALERT, "Should be ALERT");

    d = make_co2_packet(0, 2000, 1850); // unter 2000 aber über 1800
    semantic_handler_process(&d);
    zassert_equal(semantic_handler_get_state(0), NODE_STATE_ALERT,
                  "Should stay ALERT within hysteresis");
}

ZTEST(semantic, test_co2_below_hysteresis_returns_to_lower_state)
{
    struct node_sensor_data d = make_co2_packet(0, 1, 1900);
    semantic_handler_process(&d);
    d = make_co2_packet(0, 1000, 2100);
    semantic_handler_process(&d);
    zassert_equal(semantic_handler_get_state(0), NODE_STATE_ALERT, "Should be ALERT");
    /* Weit unter Schwelle — klar unter thresh-hyst=1800 */
    d = make_co2_packet(0, 2000, 400);
    semantic_handler_process(&d);
    d = make_co2_packet(0, 3000, 400);
    semantic_handler_process(&d);
    node_state_t state = semantic_handler_get_state(0);
    zassert_true(state < NODE_STATE_ALERT,
                 "Should leave ALERT (got %d)", (int)state);
}

/* ════════════════════════════════════════════════════════════════
 * CO2-Trend (schneller Anstieg)
 * ════════════════════════════════════════════════════════════════ */

ZTEST(semantic, test_co2_fast_rise_triggers_critical)
{
    /* Baseline über thresh_co2_active=1000 damit Trend-Check greift */
    struct node_sensor_data d = make_co2_packet(0, 1, 1100);
    semantic_handler_process(&d);
    /* +350ppm innerhalb 2000ms < TREND_WINDOW_MS=5000ms */
    d = make_co2_packet(0, 2000, 1450);
    semantic_handler_process(&d);
    zassert_equal(semantic_handler_get_state(0), NODE_STATE_CRITICAL,
                  "Fast CO2 rise should trigger CRITICAL");
}

/* ════════════════════════════════════════════════════════════════
 * IMU / Bewegung
 * ════════════════════════════════════════════════════════════════ */

ZTEST(semantic, test_imu_normal_stays_idle)
{
    /* Ruhig: ~1g Beschleunigung (Schwerkraft) */
    struct node_sensor_data d = make_imu_packet(0, 0, 0, 0, 9810);
    semantic_handler_process(&d);
    d = make_imu_packet(0, 6000, 0, 0, 9810);
    semantic_handler_process(&d);

    zassert_equal(semantic_handler_get_state(0), NODE_STATE_IDLE,
                  "Static 1g should be IDLE");
}

ZTEST(semantic, test_imu_high_motion_triggers_alert)
{
    /* Erstes Paket (Referenz) */
    struct node_sensor_data d = make_imu_packet(0, 0, 0, 0, 1000);
    semantic_handler_process(&d);

    /* Starke Bewegung: thresh_motion_alert=10000 (mg/100)²
     * ax=10000mg → (10000/100)² = 100² = 10000 → genau am Schwellwert
     * Nehmen 11000mg um sicher darüber zu sein */
    d = make_imu_packet(0, 1000, 11000, 0, 0);
    semantic_handler_process(&d);

    node_state_t state = semantic_handler_get_state(0);
    zassert_true(state >= NODE_STATE_ALERT,
                 "High motion should be ALERT or CRITICAL (got %d)", (int)state);
}

/* ════════════════════════════════════════════════════════════════
 * Node Lost
 * ════════════════════════════════════════════════════════════════ */

ZTEST(semantic, test_node_lost_after_timeout)
{
    node_lost_timeout_ms = 100;
    struct node_sensor_data d = make_packet(0, k_uptime_get());
    semantic_handler_process(&d);
    zassert_equal(semantic_handler_get_state(0), NODE_STATE_IDLE, "Should start IDLE");
    k_sleep(K_MSEC(150));
    semantic_handler_tick();
    zassert_equal(semantic_handler_get_state(0), NODE_STATE_LOST, "Should be LOST");
    node_lost_timeout_ms = 30000;
}
ZTEST(semantic, test_node_lost_notifies_nus)
{
    node_lost_timeout_ms = 100;
    mock_nus_ready = true;

    struct node_sensor_data d = make_packet(0, k_uptime_get());
    semantic_handler_process(&d);

    k_sleep(K_MSEC(150));
    semantic_handler_tick();

    zassert_true(mock_nus_calls > 0, "LOST should send NUS notification");
    zassert_not_null(strstr(mock_nus_last, "\"state\":\"LOST\""), NULL);
    node_lost_timeout_ms = 30000;
}

ZTEST(semantic, test_node_recovers_from_lost)
{
    node_lost_timeout_ms = 100;

    struct node_sensor_data d = make_packet(0, k_uptime_get());
    semantic_handler_process(&d);

    k_sleep(K_MSEC(150));
    semantic_handler_tick();
    zassert_equal(semantic_handler_get_state(0), NODE_STATE_LOST, "Should be LOST");

    d = make_packet(0, k_uptime_get());
    semantic_handler_process(&d);
    zassert_equal(semantic_handler_get_state(0), NODE_STATE_IDLE,
                  "New packet after LOST should return to IDLE");
    node_lost_timeout_ms = 30000;
}

/* ════════════════════════════════════════════════════════════════
 * rule_engine-Integration
 * ════════════════════════════════════════════════════════════════ */

ZTEST(semantic, test_transition_calls_rule_engine)
{
    struct node_sensor_data d = make_co2_packet(0, 1, 1900);
    semantic_handler_process(&d);
    int calls_after_idle = mock_rule_calls;

    // Kleiner Schritt → ALERT ohne Trend
    d = make_co2_packet(0, 1000, 2100);
    semantic_handler_process(&d);

    zassert_true(mock_rule_calls > calls_after_idle,
                 "rule_engine_on_state must be called on transition");
    zassert_equal(mock_rule_node_idx, 0, "Wrong node index");
    zassert_equal(mock_rule_state, NODE_STATE_ALERT,
                  "Wrong state passed to rule engine");
}

ZTEST(semantic, test_no_rule_call_without_transition)
{
    // Basis bei t=1 damit kein Sprung von 0
    struct node_sensor_data d = make_co2_packet(0, 1, 1900);
    semantic_handler_process(&d);

    // ALERT — kleiner Schritt über Schwelle, kein Trend
    d = make_co2_packet(0, 1000, 2100);
    semantic_handler_process(&d);
    zassert_equal(semantic_handler_get_state(0), NODE_STATE_ALERT, "Should be ALERT");

    int calls_before = mock_rule_calls;

    // Nochmals ALERT — gleicher Zustand, kein Übergang
    d = make_co2_packet(0, 2000, 2100);
    semantic_handler_process(&d);

    zassert_equal(mock_rule_calls, calls_before,
                  "No transition = no rule_engine call");
}

/* ════════════════════════════════════════════════════════════════
 * State-String
 * ════════════════════════════════════════════════════════════════ */

ZTEST(semantic, test_state_str_all_values)
{
    zassert_str_equal(semantic_handler_state_str(NODE_STATE_UNKNOWN),  "UNKNOWN",  NULL);
    zassert_str_equal(semantic_handler_state_str(NODE_STATE_IDLE),     "IDLE",     NULL);
    zassert_str_equal(semantic_handler_state_str(NODE_STATE_ACTIVE),   "ACTIVE",   NULL);
    zassert_str_equal(semantic_handler_state_str(NODE_STATE_ALERT),    "ALERT",    NULL);
    zassert_str_equal(semantic_handler_state_str(NODE_STATE_CRITICAL), "CRITICAL", NULL);
    zassert_str_equal(semantic_handler_state_str(NODE_STATE_LOST),     "LOST",     NULL);
}

/* ════════════════════════════════════════════════════════════════
 * Mehrere Nodes unabhängig
 * ════════════════════════════════════════════════════════════════ */

ZTEST(semantic, test_multiple_nodes_independent)
{
    /* Node 0: ALERT */
    struct node_sensor_data d0 = make_co2_packet(0, 1000, 2500);
    semantic_handler_process(&d0);

    /* Node 1: normal */
    struct node_sensor_data d1 = make_co2_packet(1, 1000, 400);
    semantic_handler_process(&d1);

    zassert_equal(semantic_handler_get_state(0), NODE_STATE_ALERT,
                  "Node 0 should be ALERT");
    zassert_equal(semantic_handler_get_state(1), NODE_STATE_IDLE,
                  "Node 1 should be IDLE — independent of Node 0");
}