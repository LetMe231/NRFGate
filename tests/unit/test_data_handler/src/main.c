/**
 * @file tests/unit/test_data_handler/src/main.c
 *
 * Unit-Tests für data_handler.c.
 *
 * Was getestet wird:
 *   - Node-Registrierung: erster Node, bekannte Nodes, Tabelle voll
 *   - identity_str(): Thread + Mesh, keine statischen Buffer-Kollisionen
 *   - JSON-Builder: alle Felder, minimale Felder, Buffer-Overflow-Schutz
 *   - NUS-Parser: gültige Befehle, kaputte Befehle, Garbage → kein Crash
 */

#include <zephyr/ztest.h>
#include <string.h>
#include <stdint.h>

#include "data_handler.h"
#include "semantic_handler.h"
#include "rule_engine.h"

/* ── Mock-Zustand ────────────────────────────────────────────── */
static uint16_t mock_unprov_addr     = 0;
static bool     mock_prov_started    = false;
static int      mock_rule_add_calls  = 0;
static int      mock_rule_rm_idx     = -1;
static bool     mock_nus_ready_val   = false;
static char     mock_nus_sent[320]   = {0};

static void reset_mocks(void)
{
    mock_unprov_addr    = 0;
    mock_prov_started   = false;
    mock_rule_add_calls = 0;
    mock_rule_rm_idx    = -1;
    mock_nus_ready_val  = false;
    memset(mock_nus_sent, 0, sizeof(mock_nus_sent));
}

/* ── Mocks ────────────────────────────────────────────────────── */

/* model_handler.h */
void model_handler_unprovision_node(uint16_t addr) { mock_unprov_addr = addr; }
void model_handler_start_provisioning(void)         { mock_prov_started = true; }
void model_handler_reconfigure_node(uint16_t addr)  { ARG_UNUSED(addr); }

/* rule_engine.h */
int  rule_engine_add(const gateway_rule_t *r) { ARG_UNUSED(r); return mock_rule_add_calls++; }
void rule_engine_remove(uint8_t idx)          { mock_rule_rm_idx = idx; }
int  rule_engine_to_json(char *b, size_t s)   { return snprintf(b, s, "{\"rules\":[]}\n"); }
void rule_engine_on_switch(uint8_t node_idx, bool sw_on, node_transport_t t)
{
    ARG_UNUSED(node_idx); ARG_UNUSED(sw_on); ARG_UNUSED(t);
}

/* semantic_handler.h */
void semantic_handler_process(const struct node_sensor_data *d) { ARG_UNUSED(d); }
node_state_t semantic_handler_get_state(uint8_t idx)
{
    ARG_UNUSED(idx);
    return NODE_STATE_IDLE;
}
const char *semantic_handler_state_str(node_state_t s) { ARG_UNUSED(s); return "IDLE"; }

/* ble_nus.h */
bool ble_nus_is_ready(void)              { return mock_nus_ready_val; }
void ble_nus_send(const char *json)
{
    strncpy(mock_nus_sent, json, sizeof(mock_nus_sent) - 1);
}

/* lora_handler.h */
void lora_handler_send(const uint8_t *data, uint8_t len)
{
    ARG_UNUSED(data); ARG_UNUSED(len);
}

/* main.h */
void mesh_scheduler_request_priority(sched_priority_t t, uint32_t ms)
{
    ARG_UNUSED(t); ARG_UNUSED(ms);
}
void mesh_scheduler_set_mode(sched_mode_t m) { ARG_UNUSED(m); }
void mesh_scheduler_pause(void)              {}
void mesh_scheduler_start(void)              {}

/* ── Suite Setup ─────────────────────────────────────────────── */
static void before_each(void *f)
{
    ARG_UNUSED(f);
    data_handler_init();
    reset_mocks();
}

ZTEST_SUITE(data_handler, NULL, NULL, before_each, NULL, NULL);

/* ════════════════════════════════════════════════════════════════
 * Node-Registrierung
 * (White-Box via data_handler_receive + data_handler_get_node_idx_*)
 * ════════════════════════════════════════════════════════════════ */

ZTEST(data_handler, test_first_thread_node_gets_index_0)
{
    struct node_sensor_data d = {
        .identity = { .transport = NODE_TRANSPORT_THREAD },
        .rx_uptime_ms = 1000,
        .payload = { .present = 0 },
    };
    strncpy(d.identity.ipv6, "fd11::1", sizeof(d.identity.ipv6));

    data_handler_receive(&d);

    uint8_t idx = data_handler_get_node_idx_by_ipv6("fd11::1");
    zassert_equal(idx, 0, "First Thread node should get index 0");
}

ZTEST(data_handler, test_same_node_gets_same_index)
{
    struct node_sensor_data d = {
        .identity = { .transport = NODE_TRANSPORT_THREAD },
        .rx_uptime_ms = 1000,
        .payload = { .present = 0 },
    };
    strncpy(d.identity.ipv6, "fd11::2", sizeof(d.identity.ipv6));

    data_handler_receive(&d);
    uint8_t idx1 = data_handler_get_node_idx_by_ipv6("fd11::2");

    d.rx_uptime_ms = 2000;
    data_handler_receive(&d);
    uint8_t idx2 = data_handler_get_node_idx_by_ipv6("fd11::2");

    zassert_equal(idx1, idx2, "Same node should always get same index");
    zassert_not_equal(idx1, 0xFF, "Index should not be 0xFF (not found)");
}

ZTEST(data_handler, test_different_nodes_different_indices)
{
    struct node_sensor_data d1 = {
        .identity = { .transport = NODE_TRANSPORT_THREAD },
        .rx_uptime_ms = 1000,
        .payload = { .present = 0 },
    };
    strncpy(d1.identity.ipv6, "fd11::1", sizeof(d1.identity.ipv6));

    struct node_sensor_data d2 = {
        .identity = { .transport = NODE_TRANSPORT_THREAD },
        .rx_uptime_ms = 1000,
        .payload = { .present = 0 },
    };
    strncpy(d2.identity.ipv6, "fd11::2", sizeof(d2.identity.ipv6));

    data_handler_receive(&d1);
    data_handler_receive(&d2);

    uint8_t idx1 = data_handler_get_node_idx_by_ipv6("fd11::1");
    uint8_t idx2 = data_handler_get_node_idx_by_ipv6("fd11::2");

    zassert_not_equal(idx1, idx2, "Different nodes should have different indices");
}

ZTEST(data_handler, test_lookup_unknown_node_returns_0xff)
{
    uint8_t idx = data_handler_get_node_idx_by_ipv6("fd11::9999");
    zassert_equal(idx, 0xFF, "Unknown node should return 0xFF");

    idx = data_handler_get_node_idx_by_mesh_addr(0x0099);
    zassert_equal(idx, 0xFF, "Unknown mesh addr should return 0xFF");
}

ZTEST(data_handler, test_mesh_node_lookup_by_addr)
{
    struct node_sensor_data d = {
        .identity = {
            .transport = NODE_TRANSPORT_BLE_MESH,
            .mesh_addr = 0x0002,
        },
        .rx_uptime_ms = 1000,
        .payload = { .present = 0 },
    };
    data_handler_receive(&d);

    uint8_t idx = data_handler_get_node_idx_by_mesh_addr(0x0002);
    zassert_not_equal(idx, 0xFF, "BLE Mesh node should be findable by address");
}

ZTEST(data_handler, test_node_table_full_returns_0xff)
{
    /* Fülle Tabelle mit MAX_NODES verschiedenen Nodes */
    for (int i = 0; i < MAX_NODES; i++) {
        struct node_sensor_data d = {
            .identity = { .transport = NODE_TRANSPORT_THREAD },
            .rx_uptime_ms = 1000 + i,
            .payload = { .present = 0 },
        };
        snprintf(d.identity.ipv6, sizeof(d.identity.ipv6), "fd11::%d", i + 1);
        data_handler_receive(&d);
    }

    /* Weiterer Node soll 0xFF zurückgeben */
    struct node_sensor_data overflow = {
        .identity = { .transport = NODE_TRANSPORT_THREAD },
        .rx_uptime_ms = 9999,
        .payload = { .present = 0 },
    };
    strncpy(overflow.identity.ipv6, "fd11::99", sizeof(overflow.identity.ipv6));
    data_handler_receive(&overflow); /* Wird gedroppt */

    uint8_t idx = data_handler_get_node_idx_by_ipv6("fd11::99");
    zassert_equal(idx, 0xFF, "Overflow node should not be registered");
}

/* ════════════════════════════════════════════════════════════════
 * identity_str() — kein statischer Buffer mehr
 * ════════════════════════════════════════════════════════════════ */

ZTEST(data_handler, test_identity_str_thread)
{
    node_identity_t id = { .transport = NODE_TRANSPORT_THREAD };
    strncpy(id.ipv6, "fd11::cafe:1234", sizeof(id.ipv6));

    char buf[NODE_ADDR_STR_LEN];
    identity_str(&id, buf, sizeof(buf));

    zassert_str_equal(buf, "fd11::cafe:1234",
                      "Thread identity_str should return IPv6");
}

ZTEST(data_handler, test_identity_str_mesh)
{
    node_identity_t id = { .transport = NODE_TRANSPORT_BLE_MESH, .mesh_addr = 0xABCD };

    char buf[NODE_ADDR_STR_LEN];
    identity_str(&id, buf, sizeof(buf));

    zassert_not_null(strstr(buf, "ABCD"),
                     "Mesh identity_str should contain address in hex");
}

ZTEST(data_handler, test_identity_str_no_shared_buffer)
{
    /* Beide Aufrufe dürfen sich nicht gegenseitig überschreiben */
    node_identity_t id_a = { .transport = NODE_TRANSPORT_THREAD };
    node_identity_t id_b = { .transport = NODE_TRANSPORT_BLE_MESH, .mesh_addr = 0x0002 };
    strncpy(id_a.ipv6, "fd11::aaa", sizeof(id_a.ipv6));

    char buf_a[NODE_ADDR_STR_LEN];
    char buf_b[NODE_ADDR_STR_LEN];

    identity_str(&id_a, buf_a, sizeof(buf_a));
    identity_str(&id_b, buf_b, sizeof(buf_b));

    /* buf_a darf nach dem zweiten Aufruf nicht verändert worden sein */
    zassert_str_equal(buf_a, "fd11::aaa",
                      "Second call must not overwrite first caller's buffer");
}

/* ════════════════════════════════════════════════════════════════
 * NUS-Befehlsparser
 * ════════════════════════════════════════════════════════════════ */

ZTEST(data_handler, test_cmd_start_provisioning)
{
    const char *cmd = "{\"cmd\":\"start_provisioning\"}";
    data_handler_cmd(cmd, strlen(cmd));
    zassert_true(mock_prov_started, "start_provisioning should trigger provisioning");
}

ZTEST(data_handler, test_cmd_unprovision_valid)
{
    const char *cmd = "{\"cmd\":\"unprovision\",\"mesh_addr\":42}";
    data_handler_cmd(cmd, strlen(cmd));
    zassert_equal(mock_unprov_addr, 42, "Should unprovision addr 42");
}

ZTEST(data_handler, test_cmd_unprovision_addr_zero_rejected)
{
    const char *cmd = "{\"cmd\":\"unprovision\",\"mesh_addr\":0}";
    data_handler_cmd(cmd, strlen(cmd));
    zassert_equal(mock_unprov_addr, 0,
                  "mesh_addr=0 should be rejected, unprovision should not be called");
}

ZTEST(data_handler, test_cmd_unprovision_addr_too_high_rejected)
{
    /* 0x8000 ist kein gültiger Unicast-Bereich */
    const char *cmd = "{\"cmd\":\"unprovision\",\"mesh_addr\":32768}";
    data_handler_cmd(cmd, strlen(cmd));
    zassert_equal(mock_unprov_addr, 0, "Out-of-range addr should be rejected");
}

ZTEST(data_handler, test_cmd_remove_rule)
{
    const char *cmd = "{\"cmd\":\"remove_rule\",\"idx\":7}";
    data_handler_cmd(cmd, strlen(cmd));
    zassert_equal(mock_rule_rm_idx, 7, "Should remove rule index 7");
}

ZTEST(data_handler, test_cmd_add_rule_mesh)
{
    int calls_before = mock_rule_add_calls;
    const char *cmd = "{\"cmd\":\"add_rule\",\"src\":0,\"trig\":1,"
                      "\"act\":0,\"type\":\"mesh\",\"target\":2}";
    data_handler_cmd(cmd, strlen(cmd));
    zassert_equal(mock_rule_add_calls, calls_before + 1,
                  "add_rule should call rule_engine_add()");
}

ZTEST(data_handler, test_cmd_add_rule_thread)
{
    int calls_before = mock_rule_add_calls;
    const char *cmd = "{\"cmd\":\"add_rule\",\"src\":1,\"trig\":0,"
                      "\"act\":3,\"type\":\"thread\",\"target\":\"fd11::1\"}";
    data_handler_cmd(cmd, strlen(cmd));
    zassert_equal(mock_rule_add_calls, calls_before + 1,
                  "add_rule with Thread target should call rule_engine_add()");
}

ZTEST(data_handler, test_cmd_lora_enable)
{
    const char *cmd = "{\"cmd\":\"lora_enabled\",\"lora_enabled\":true}";
    data_handler_cmd(cmd, strlen(cmd));
    /* Kein Crash = Erfolg. Atomic-Wert direkt prüfen geht nur bei White-Box */
    zassert_true(true, "lora_enabled=true should not crash");
}

/* ── Garbage-Eingaben dürfen nicht crashen ─────────────────── */

ZTEST(data_handler, test_cmd_garbage_no_crash)
{
    static const char *inputs[] = {
        "",
        "{}",
        "{{{{{{{",
        "AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA",
        "{\"cmd\":}",
        "{\"cmd\":\"unprovision\"}",          /* mesh_addr fehlt */
        "{\"cmd\":\"remove_rule\"}",           /* idx fehlt */
        "{\"cmd\":\"add_rule\",\"src\":0}",   /* unvollständig */
        "\x01\x02\x03\x04\xFF\xFE",
    };
    for (int i = 0; i < ARRAY_SIZE(inputs); i++) {
        data_handler_cmd(inputs[i], strlen(inputs[i]));
    }
    zassert_true(true, "Garbage inputs must not crash");
}

ZTEST(data_handler, test_json_contains_node_index)
{
    mock_nus_ready_val = true;

    struct node_sensor_data d = {
        .identity = { .transport = NODE_TRANSPORT_BLE_MESH, .mesh_addr = 0x0002 },
        .rx_uptime_ms = 1000,
        .payload = { .present = SENSOR_HAS_TEMP, .temp = 22500 },
    };
    data_handler_receive(&d);

    zassert_not_null(strstr(mock_nus_sent, "\"node\":"),
                     "JSON must contain node index");
    zassert_not_null(strstr(mock_nus_sent, "\"temp\":"),
                     "JSON must contain temp field");
}

ZTEST(data_handler, test_json_thread_node_has_ipv6)
{
    mock_nus_ready_val = true;

    struct node_sensor_data d = {
        .identity = { .transport = NODE_TRANSPORT_THREAD },
        .rx_uptime_ms = 1000,
        .payload = { .present = SENSOR_HAS_ECO2, .eco2 = 1200 },
    };
    strncpy(d.identity.ipv6, "fd11::1", sizeof(d.identity.ipv6));
    data_handler_receive(&d);

    zassert_not_null(strstr(mock_nus_sent, "fd11::1"),
                     "Thread node JSON must contain IPv6");
    zassert_not_null(strstr(mock_nus_sent, "\"eco2\":"),
                     "JSON must contain eco2");
}

ZTEST(data_handler, test_json_no_overflow_with_all_fields)
{
    /* Dieser Test prüft den JSON_BUF_SIZE=220 Bug.
     * Wenn build_json überläuft → ble_nus_send wird NICHT aufgerufen
     * (build_json gibt 0 zurück → forward_ble_nus bricht ab) */
    mock_nus_ready_val = true;
    memset(mock_nus_sent, 0, sizeof(mock_nus_sent));

    struct node_sensor_data d = {
        .identity = { .transport = NODE_TRANSPORT_THREAD },
        .rx_uptime_ms = 1000,
        .payload = {
            .present = 0x000FFFFF, /* alle Felder */
            .ax = 10123, .ay = -9999, .az = 1,
            .gx = 1234,  .gy = -5678, .gz = 1,
            .temp = 37500, .hum = 85300,
            .tvoc = 12345, .eco2 = 54321,
            .heart_rate = 180, .spo2 = 98700,
            .pm25 = 9999, .pm10 = 9999,
            .switch_state = 1,
        },
    };
    /* Langer IPv6-String = worst case */
    strncpy(d.identity.ipv6,
            "2001:db8:1234:5678:9abc:def0:1234:5678",
            sizeof(d.identity.ipv6));

    data_handler_receive(&d);

    /* Wenn JSON_BUF_SIZE zu klein → mock_nus_sent bleibt leer */
    zassert_true(strlen(mock_nus_sent) > 0,
                 "JSON must not overflow buffer — increase JSON_BUF_SIZE if this fails");
    /* Muss valides JSON-Ende haben */
    size_t len = strlen(mock_nus_sent);
    zassert_equal(mock_nus_sent[len - 1], '\n',
                  "JSON must end with newline");
}

ZTEST(data_handler, test_json_only_present_fields_emitted)
{
    mock_nus_ready_val = true;

    struct node_sensor_data d = {
        .identity = { .transport = NODE_TRANSPORT_BLE_MESH, .mesh_addr = 0x0003 },
        .rx_uptime_ms = 1000,
        .payload = { .present = SENSOR_HAS_TEMP, .temp = 20000 },
    };
    data_handler_receive(&d);

    zassert_not_null(strstr(mock_nus_sent, "\"temp\":"),
                     "temp should be present");
    zassert_is_null(strstr(mock_nus_sent, "\"ax\":"),
                    "ax must NOT appear when not in present bitmask");
    zassert_is_null(strstr(mock_nus_sent, "\"eco2\":"),
                    "eco2 must NOT appear when not in present bitmask");
}