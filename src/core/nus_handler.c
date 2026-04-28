#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/net/net_ip.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#include "nus_handler.h"
#include "ble_nus.h"
#include "gw_store.h"
#include "event_ingest.h"
#include "command_router.h"
#include "ble_mesh_prov.h"
#include "rule_engine.h"
#include "lorawan_adapter.h"
#include "thread_adapter.h"
#include "ble_mesh_adapter.h"
#include "scheduler.h"
#include "gw_addr.h"

LOG_MODULE_REGISTER(nus_handler, LOG_LEVEL_INF);

/* ── Constants ─────────────────────────────────────── */

#define HEARTBEAT_MS 5000

/* ── internal state ──────────────────────────────────────── */

static struct k_work_delayable s_list_rules_work;
static int64_t s_last_sent_ms[GW_STORE_MAX_NODES] = {0};
static bool s_last_sent_sensor_valid[GW_STORE_MAX_NODES];

/* ── JSON Buffer ──────────────────────────────────────── */

#define JSON_BUF_SIZE     384
#define DASH_TX_KEY_MAX   64
#define DASH_TX_JSON_MAX  512

/* ── Transport String ──────────────────────────────────────── */

static const char *transport_str(gw_transport_t tr)
{
    switch (tr) {
        case GW_TR_BLE_MESH: return "B";
        case GW_TR_THREAD:   return "T";
        case GW_TR_LORAWAN:  return "L";
        default:             return "?";
    }
}

/* ── State String ──────────────────────────────────────── */

static const char *state_str(gw_state_t type)
{
    switch (type) {
        case GW_STATE_ACTIVE:   return "ACTIVE";
        case GW_STATE_ALERT:    return "ALERT";
        case GW_STATE_CRITICAL: return "CRITICAL";
        case GW_STATE_LOST:     return "LOST";
        case GW_STATE_IDLE:     return "IDLE";
        default:                return "UNKNOWN";
    }
}

/* ── Serialize Node Record → JSON ──────────────────────────── */

#include <stdarg.h>

static bool append_json(char *buf, size_t buf_size, int *off, const char *fmt, ...)
{
    if (*off < 0 || (size_t)*off >= buf_size) {
        return false;
    }

    va_list ap;
    va_start(ap, fmt);
    int n = vsnprintf(buf + *off, buf_size - (size_t)*off, fmt, ap);
    va_end(ap);

    if (n < 0) {
        return false;
    }

    if ((size_t)n >= buf_size - (size_t)*off) {
        *off = (int)buf_size - 1;
        buf[buf_size - 1] = '\0';
        return false;
    }

    *off += n;
    return true;
}
static bool make_node_key(const gw_node_record_t *rec, char *out, size_t out_size)
{
    if (!rec || !out || out_size == 0) {
        return false;
    }
    return gw_addr_to_str(&rec->addr, out, out_size) == 0;
}
static bool build_node_update_json(const gw_node_record_t *rec,
                                   char *buf, size_t buf_size)
{
    if (!rec || !rec->known || !buf || buf_size == 0) {
        return false;
    }

    int off = 0;

    if (!append_json(buf, buf_size, &off,
                    "{\"type\":\"node_update\","
                    "\"tr\":\"%s\","
                    "\"state\":\"%s\","
                    "\"pkt_count\":%u,"
                    "\"seq_gaps\":%u,",
                    transport_str(rec->addr.transport),
                    state_str(rec->state),
                    rec->stats.packet_count,
                    rec->stats.seq_gaps)) {
        LOG_WRN("node_update JSON truncated (header)");
        return false;
    }

    char ipv6_str[GW_IPV6_STR_LEN];
    if (gw_addr_to_str(&rec->addr, ipv6_str, sizeof(ipv6_str)) != 0) {
        return false;
    }

    if (!append_json(buf, buf_size, &off,
                    "\"ipv6\":\"%s\",",
                    ipv6_str)) {
        LOG_WRN("node_update JSON truncated (addr)");
        return false;
    }

    if (rec->stats.has_last_rssi) {
        if (!append_json(buf, buf_size, &off,
                        "\"rssi\":%d,",
                        rec->stats.last_rssi_dbm)) {
            LOG_WRN("node_update JSON truncated (RX meta)");
            return false;
        }
    }

    if (rec->stats.has_last_hops) {
        if (!append_json(buf, buf_size, &off,
                        "\"hops\":%u,",
                        rec->stats.last_hops)) {
            LOG_WRN("node_update JSON truncated (RX meta)");
            return false;
        }
    }

        if (rec->has_last_sensor) {
        const gw_sensor_payload_t *s = &rec->last_sensor;

        if (s->present & GW_HAS_SEQ) {
            if (!append_json(buf, buf_size, &off,
                            "\"seq\":%d,", s->seq)) {
                LOG_WRN("node_update JSON truncated (seq)");
                return false;
            }
        }

        if (s->present & GW_HAS_TEMP) {
            int32_t t = s->temp_mc;
            int32_t t_abs = t < 0 ? -t : t;
            if (!append_json(buf, buf_size, &off,
                            "\"temp\":%s%d.%02d,",
                            t < 0 ? "-" : "",
                            t_abs / 1000,
                            (t_abs % 1000) / 10)) {
                LOG_WRN("node_update JSON truncated (temp)");
                return false;
            }
        }

        if (s->present & GW_HAS_HUM) {
            if (!append_json(buf, buf_size, &off,
                            "\"hum\":%d.%02d,",
                            s->hum_mpermille / 1000,
                            (s->hum_mpermille % 1000) / 10)) {
                LOG_WRN("node_update JSON truncated (hum)");
                return false;
            }
        }

        if (s->present & GW_HAS_TVOC) {
            if (!append_json(buf, buf_size, &off,
                            "\"tvoc\":%d,", s->tvoc_ppb)) {
                LOG_WRN("node_update JSON truncated (tvoc)");
                return false;
            }
        }

        if (s->present & GW_HAS_ECO2) {
            if (!append_json(buf, buf_size, &off,
                            "\"eco2\":%d,", s->eco2_ppm)) {
                LOG_WRN("node_update JSON truncated (eco2)");
                return false;
            }
        }

        if (s->present & GW_HAS_ACCEL) {
            if (!append_json(buf, buf_size, &off,
                            "\"ax\":%.3f,\"ay\":%.3f,\"az\":%.3f,",
                            s->ax_mg * 9.81 / 1000.0,
                            s->ay_mg * 9.81 / 1000.0,
                            s->az_mg * 9.81 / 1000.0)) {
                LOG_WRN("node_update JSON truncated (accel)");
                return false;
            }
        }

        if (s->present & GW_HAS_GYRO) {
            if (!append_json(buf, buf_size, &off,
                            "\"gx\":%.3f,\"gy\":%.3f,\"gz\":%.3f,",
                            s->gx_mdps / 1000.0,
                            s->gy_mdps / 1000.0,
                            s->gz_mdps / 1000.0)) {
                LOG_WRN("node_update JSON truncated (gyro)");
                return false;
            }
        }

        if (s->present & GW_HAS_HEART_RATE) {
            if (!append_json(buf, buf_size, &off,
                            "\"hr\":%d,", s->heart_rate_bpm)) {
                LOG_WRN("node_update JSON truncated (hr)");
                return false;
            }
        }

        if (s->present & GW_HAS_SPO2) {
            if (!append_json(buf, buf_size, &off,
                            "\"spo2\":%.1f,",
                            s->spo2_mpermille / 1000.0)) {
                LOG_WRN("node_update JSON truncated (spo2)");
                return false;
            }
        }

        if (s->present & GW_HAS_PM25) {
            if (!append_json(buf, buf_size, &off,
                            "\"pm25\":%d,", s->pm25_ugm3)) {
                LOG_WRN("node_update JSON truncated (pm25)");
                return false;
            }
        }

        if (s->present & GW_HAS_PM10) {
            if (!append_json(buf, buf_size, &off,
                            "\"pm10\":%d,", s->pm10_ugm3)) {
                LOG_WRN("node_update JSON truncated (pm10)");
                return false;
            }
        }

        if (s->present & GW_HAS_SWITCH) {
            if (!append_json(buf, buf_size, &off,
                            "\"sw\":%d,",
                            s->switch_state ? 1 : 0)) {
                LOG_WRN("node_update JSON truncated (switch)");
                return false;
            }
        }

        if (rec->has_last_actuator_state) {
            if (!append_json(buf, buf_size, &off,
                            "\"light\":%d,",
                            rec->last_light_on ? 1 : 0)) {
                LOG_WRN("node_update JSON truncated (light from actuator state)");
                return false;
            }
        } else if (s->present & GW_HAS_LIGHT) {
            if (!append_json(buf, buf_size, &off,
                            "\"light\":%d,",
                            s->light_on ? 1 : 0)) {
                LOG_WRN("node_update JSON truncated (light from sensor)");
                return false;
            }
        }
    } else if (rec->has_last_actuator_state) {
        if (!append_json(buf, buf_size, &off,
                        "\"light\":%d,",
                        rec->last_light_on ? 1 : 0)) {
            LOG_WRN("node_update JSON truncated (light only)");
            return false;
        }
    }

    if (rec->has_pending_light_cmd) {
        if (!append_json(buf, buf_size, &off,
                        "\"pending_light\":%d,"
                        "\"pending_cmd_id\":%u,",
                        rec->pending_light_on ? 1 : 0,
                        rec->pending_cmd_id)) {
            LOG_WRN("node_update JSON truncated (pending state)");
            return false;
        }
    }

    if (off > 0 && buf[off - 1] == ',') {
        off--;
    }

    if (!append_json(buf, buf_size, &off, "}\n")) {
        LOG_WRN("node_update JSON truncated (finalize)");
        return false;
    }
    LOG_INF("JSON build: has_last_sensor=%d present=0x%08x has_last_act=%d last_light=%d",
        rec->has_last_sensor,
        rec->has_last_sensor ? rec->last_sensor.present : 0,
        rec->has_last_actuator_state,
        rec->last_light_on);
    return true;
}

static void send_node_update_latest(const gw_node_record_t *rec)
{
    char buf[JSON_BUF_SIZE];
    char node_key[64];

    if (!build_node_update_json(rec, buf, sizeof(buf))) {
        return;
    }

    if (!make_node_key(rec, node_key, sizeof(node_key))) {
        LOG_WRN("Failed to build node key");
        return;
    }

    ble_nus_publish_latest(node_key, buf);
}

/* ── State-Only Update ─────────────────────────────────────── */

static void send_state_update(const gw_node_record_t *rec)
{
    if (!rec || !rec->known) {
        return;
    }

    char ipv6_str[GW_IPV6_STR_LEN];
    if (gw_addr_to_str(&rec->addr, ipv6_str, sizeof(ipv6_str)) != 0) {
        LOG_WRN("Failed to convert node address to string");
        return;
    }

    char buf[160];
    snprintf(buf, sizeof(buf),
            "{\"type\":\"state_update\","
            "\"ipv6\":\"%s\","
             "\"state\":\"%s\"}\n\"",
            ipv6_str,
            state_str(rec->state));

    ble_nus_send(buf);
}

/* ── Event Listener ────────────────────────────────────────── */

static void on_event(const gw_event_t *evt, void *ctx)
{
    ARG_UNUSED(ctx);

    if (!ble_nus_is_ready()) {
        return;
    }

    int idx = gw_store_find_node(&evt->src);
    if (idx < 0 || idx >= GW_STORE_MAX_NODES) {
        return;
    }

    gw_node_record_t rec;
    if (!gw_store_get_node_copy((uint8_t)idx, &rec)) {
        return;
    }

    switch (evt->type) {
    case GW_EVT_STATE_TRANSITION:
        send_state_update(&rec);
        break;

    case GW_EVT_ACTUATOR_STATE:
        send_node_update_latest(&rec);
        break;    
    case GW_EVT_CMD_PENDING:
    case GW_EVT_TIMEOUT:
        send_state_update(&rec);
        break;

    case GW_EVT_SENSOR:
        send_node_update_latest(&rec);
        break;

    default:
        break;
    }
}

/* ── Simple JSON Field Extractor ───────────────────────────── */

static bool json_get_str(const char *json, const char *key,
                          char *out, size_t out_size)
{
    char search[64];
    // constaining whitespace:
    snprintf(search, sizeof(search), "\"%s\": \"", key);
    const char *p = strstr(json, search);
    if (!p) {
        // Fallback: allow no whitespace after colon:
        snprintf(search, sizeof(search), "\"%s\":\"", key);
        p = strstr(json, search);
    }
    if (!p) {
        return false;
    }

    p += strlen(search);
    const char *end = strchr(p, '"');
    if (!end) {
        return false;
    }

    size_t len = MIN((size_t)(end - p), out_size - 1);
    memcpy(out, p, len);
    out[len] = '\0';
    return true;
}

static bool json_get_int(const char *json, const char *key, int32_t *out)
{
    char search[64];
    snprintf(search, sizeof(search), "\"%s\":", key);

    const char *p = strstr(json, search);
    if (!p) {
        return false;
    }

    p += strlen(search);
    *out = (int32_t)strtol(p, NULL, 10);
    return true;
}

static const char *skip_ws(const char *p)
{
    while (p && (*p == ' ' || *p == '\t' || *p == '\r' || *p == '\n')) {
        p++;
    }
    return p;
}

static bool json_get_array_bounds(const char *json, const char *key,
                                  const char **out_start, const char **out_end)
{
    char search[64];
    snprintf(search, sizeof(search), "\"%s\":", key);

    const char *p = strstr(json, search);
    if (!p) {
        return false;
    }

    p += strlen(search);
    p = skip_ws(p);

    if (*p != '[') {
        return false;
    }

    p++; /* nach '[' */
    const char *end = strchr(p, ']');
    if (!end) {
        return false;
    }

    *out_start = p;
    *out_end = end;
    return true;
}

static const char *json_next_array_string(const char *p, const char *end,
                                          char *out, size_t out_size)
{
    if (!p || !end || p >= end || !out || out_size == 0) {
        return NULL;
    }

    while (p < end && *p != '"') {
        p++;
    }

    if (p >= end) {
        return NULL;
    }

    p++; /* nach erstem '"' */

    const char *q = p;
    while (q < end && *q != '"') {
        q++;
    }

    if (q >= end) {
        return NULL;
    }

    size_t len = MIN((size_t)(q - p), out_size - 1);
    memcpy(out, p, len);
    out[len] = '\0';

    return q + 1;
}

static bool parse_node_ref(const char *ref, gw_node_addr_t *out)
{
    if (!ref || !out || strlen(ref) < 3 || ref[1] != ':') {
        return false;
    }

    memset(out, 0, sizeof(*out));

    if (ref[0] == 'B') {
        out->transport = GW_TR_BLE_MESH;
        uint16_t mesh_addr = (uint16_t)strtol(ref + 2, NULL, 10);
        gw_addr_from_mesh(out, mesh_addr);
        return true;
    }

    if (ref[0] == 'T') {
        struct in6_addr addr6;
        if (net_addr_pton(AF_INET6, ref + 2, &addr6) < 0) {
            return false;
        }

        out->transport = GW_TR_THREAD;
        memcpy(out->ipv6, addr6.s6_addr, GW_IPV6_BIN_LEN);
        return true;
    }

    return false;
}

/* ── Gateway status envelope ─────────────────────────────────
 *
 * The Dashboard's heartbeat sends "list_rules" every ~5 s. We respond
 * with a combined gw_status envelope that contains:
 *   - active rules (embedded as compact JSON array)
 *   - uptime
 *   - per-transport statistics (last_rx_ms, node count)
 *   - mesh provisioning state
 *   - scheduler mode + timing
 *
 * Format:
 *   {
 *     "type": "gw_status",
 *     "uptime_ms": 1234567,
 *     "rules": [...],
 *     "mesh":   {"provisioned":3,"known":3,"last_rx_ms":1234500,"prov_busy":false},
 *     "thread": {"known":4,"last_rx_ms":1234555},
 *     "lora":   {"known":1,"last_rx_ms":0},
 *     "scheduler": {"mode":0,"ble_ms":200,"thread_ms":300},
 *     "rule_engine": {"active":2,"max":16}
 *   }
 *
 * The buffer is 1024 B which leaves room for ~16 rules with a few targets each.
 * ──────────────────────────────────────────────────────────── */

static const char *sched_mode_str(sched_mode_t m)
{
    switch (m) {
    case SCHED_MODE_NORMAL:      return "NORMAL";
    case SCHED_MODE_BLE_ONLY:    return "BLE_ONLY";
    case SCHED_MODE_THREAD_ONLY: return "THREAD_ONLY";
    default:                     return "UNKNOWN";
    }
}

static int build_rules_json_into(char *buf, size_t size)
{
    /* rule_engine_to_json produces "{\"rules\":[...]}\n" — we want just
     * the inner array because we embed it in gw_status. Strip the wrapper. */
    char tmp[640];
    rule_engine_to_json(tmp, sizeof(tmp));

    const char *start = strchr(tmp, '[');
    if (!start) {
        return snprintf(buf, size, "[]");
    }
    /* find matching closing bracket — last ']' before the trailing }. */
    const char *end = strrchr(tmp, ']');
    if (!end || end <= start) {
        return snprintf(buf, size, "[]");
    }

    size_t len = (size_t)(end - start) + 1;
    if (len >= size) {
        return snprintf(buf, size, "[]");
    }
    memcpy(buf, start, len);
    buf[len] = '\0';
    return (int)len;
}

static void list_rules_work_fn(struct k_work *w)
{
    ARG_UNUSED(w);
    char buf[512];
    rule_engine_to_json(buf, sizeof(buf));
    ble_nus_send_immediate(buf);
}

/* ── Command Parser ────────────────────────────────────────── */

void nus_handler_cmd(const char *cmd, size_t len)
{
    ARG_UNUSED(len);

    LOG_INF("NUS cmd: %s", cmd);

    char cmd_str[32] = {0};
    if (!json_get_str(cmd, "cmd", cmd_str, sizeof(cmd_str))) {
        LOG_WRN("NUS: no 'cmd' field in: %s", cmd);
        return;
    }

    /* ── Mesh Reset ────────────────────────────────────────── */
    if (strcmp(cmd_str, "reset_mesh") == 0) {
        LOG_WRN("NUS: full mesh reset requested");
        ble_mesh_prov_full_reset();
        return;
    }

    /* ── Start Provisioning ────────────────────────────────── */
    if (strcmp(cmd_str, "start_prov") == 0) {
        LOG_INF("NUS: opening provisioning window");
        ble_mesh_prov_start_window();
        return;
    }

    /* ── Unprovision Node ──────────────────────────────────── */
    if (strcmp(cmd_str, "unprovision") == 0) {
        int32_t mesh_addr = 0;
        if (!json_get_int(cmd, "mesh_addr", &mesh_addr)) {
            LOG_WRN("NUS: unprovision missing mesh_addr");
            return;
        }
        LOG_INF("NUS: unprovision 0x%04X", (uint16_t)mesh_addr);
        ble_mesh_prov_unprovision_node((uint16_t)mesh_addr);
        return;
    }

    /* ── List Rules ────────────────────────────────────────── */
    if (strcmp(cmd_str, "list_rules") == 0) {
        k_work_schedule(&s_list_rules_work, K_MSEC(500));
    return;
}

    if (strcmp(cmd_str, "set_lora") == 0) {
        bool enabled = strstr(cmd, "\"enabled\": true") != NULL;
        lorawan_adapter_set_enabled(enabled);
        LOG_INF("LoRa %s", enabled ? "enabled" : "disabled");
        return;
    }

    /* ── Light Commands ────────────────────────────────────── */
    gw_cmd_type_t cmd_type = GW_CMD_NONE;
    if (strcmp(cmd_str, "light_on") == 0)     cmd_type = GW_CMD_LIGHT_ON;
    if (strcmp(cmd_str, "light_off") == 0)    cmd_type = GW_CMD_LIGHT_OFF;
    if (strcmp(cmd_str, "light_toggle") == 0) cmd_type = GW_CMD_LIGHT_TOGGLE;

    /* ── Purge Lost Nodes ──────────────────────────────────────── */
    if (strcmp(cmd_str, "purge_nodes") == 0) {
        LOG_INF("NUS: purging lost nodes");
        ble_mesh_prov_purge_lost_nodes();
        return;
    }

    /* ── Remove Rule ───────────────────────────────────────────── */
    if (strcmp(cmd_str, "remove_rule") == 0) {
        int32_t idx = 0;
        if (!json_get_int(cmd, "idx", &idx)) {
            LOG_WRN("NUS: remove_rule missing idx");
            return;
        }
        LOG_INF("NUS: remove rule %d", (int)idx);
        rule_engine_remove((uint8_t)idx);
        return;
    }

/* ── Add Rule ──────────────────────────────────────────────── */
if (strcmp(cmd_str, "add_rule") == 0) {
    char src_ref[GW_IPV6_STR_LEN + 8] = {0};
    int32_t trig = 0, act = 0;

    if (!json_get_str(cmd, "src", src_ref, sizeof(src_ref)) &&
        !json_get_str(cmd, "src_node_id", src_ref, sizeof(src_ref))) {
        LOG_WRN("NUS: add_rule missing src");
        return;
    }

    if (!json_get_int(cmd, "trig", &trig)) {
        LOG_WRN("NUS: add_rule missing trig");
        return;
    }

    if (!json_get_int(cmd, "act", &act)) {
        LOG_WRN("NUS: add_rule missing act");
        return;
    }

    gateway_rule_t rule = {0};
    rule.active  = true;
    rule.trigger = (rule_trigger_t)trig;
    rule.action  = (rule_action_t)act;

    gw_node_addr_t src_addr = {0};
    if (!parse_node_ref(src_ref, &src_addr)) {
        LOG_WRN("NUS: add_rule invalid src ref '%s'", src_ref);
        return;
    }

    int src_idx = gw_store_find_node(&src_addr);
    if (src_idx < 0) {
        LOG_WRN("NUS: add_rule src node not found: %s", src_ref);
        return;
    }

    rule.src_node_idx = (uint8_t)src_idx;

    const char *arr_start = NULL;
    const char *arr_end   = NULL;
    if (!json_get_array_bounds(cmd, "targets", &arr_start, &arr_end)) {
        LOG_WRN("NUS: add_rule missing targets array");
        return;
    }

    const char *p = arr_start;
    while (rule.target_count < RULE_MAX_TARGETS) {
        char dst_ref[GW_IPV6_STR_LEN + 8] = {0};
        const char *next = json_next_array_string(p, arr_end,
                                                  dst_ref, sizeof(dst_ref));
        if (!next) {
            break;
        }

        gw_node_addr_t dst_addr = {0};
        if (!parse_node_ref(dst_ref, &dst_addr)) {
            LOG_WRN("NUS: invalid target ref '%s'", dst_ref);
            p = next;
            continue;
        }

        rule.targets[rule.target_count++] = dst_addr;
        p = next;
    }

    if (rule.target_count == 0) {
        LOG_WRN("NUS: add_rule no valid targets");
        return;
    }

    int idx = rule_engine_add(&rule);
    LOG_INF("NUS: rule added at idx=%d targets=%d", idx, rule.target_count);
    return;
}

    if (cmd_type == GW_CMD_NONE) {
        LOG_WRN("NUS: unknown cmd: %s", cmd_str);
        return;
    }

    /* ── Transport bestimmen ───────────────────────────────── */
    char tr_str_val[4] = {0};
    gw_node_addr_t dst = {0};

    if (!json_get_str(cmd, "tr", tr_str_val, sizeof(tr_str_val))) {
        LOG_WRN("NUS: light cmd missing 'tr' field");
        return;
    }

    if (strcmp(tr_str_val, "B") == 0) {
        int32_t mesh_addr = 0;
        if (!json_get_int(cmd, "mesh_addr", &mesh_addr)) {
            LOG_WRN("NUS: BLE cmd missing mesh_addr");
            return;
        }
        gw_addr_from_mesh(&dst, (uint16_t)mesh_addr);

    } else if (strcmp(tr_str_val, "T") == 0) {
        char ipv6_str[GW_IPV6_STR_LEN] = {0};
        if (!json_get_str(cmd, "ipv6", ipv6_str, sizeof(ipv6_str))) {
            LOG_WRN("NUS: Thread cmd missing ipv6");
            return;
        }
        struct in6_addr dst_in6;
        if (net_addr_pton(AF_INET6, ipv6_str, &dst_in6) < 0) {
            LOG_WRN("NUS: invalid IPv6 address: %s", ipv6_str);
            return;
        }
        gw_addr_from_thread(&dst, &dst_in6);

    } else if (strcmp(tr_str_val, "L") == 0) {
        int32_t dev_eui_lo = 0;
        json_get_int(cmd, "dev_eui_lo", &dev_eui_lo);
        gw_addr_from_lorawan(&dst, (uint64_t)(uint32_t)dev_eui_lo);

    } else {
        LOG_WRN("NUS: unknown transport: %s", tr_str_val);
        return;
    }

    LOG_INF("NUS: routing cmd=%s tr=%s", cmd_str, tr_str_val);
    command_router_send_to(&dst, cmd_type);
}

/* ── Init ──────────────────────────────────────────────────── */

void nus_handler_init(void)
{
    int err = event_ingest_register_listener(on_event, NULL);
    if (err) {
        LOG_ERR("Failed to register NUS listener: %d", err);
        return;
    }
    k_work_init_delayable(&s_list_rules_work, list_rules_work_fn);
    LOG_INF("NUS handler initialized");
}

static void send_node_update_cb(const gw_node_record_t *rec, void *ctx)
{
    ARG_UNUSED(ctx);
    send_node_update_latest(rec);
}

void nus_handler_send_snapshot(void)
{
    gw_store_foreach_node(send_node_update_cb, NULL);
}

int nus_handler_send_sched(const char *mode, const char *phase)
{
    char sched_buf[64];
    snprintf(sched_buf, sizeof(sched_buf),
        "{\"sched\":\"%s\",\"phase\":\"%s\"}\n", mode, phase);
    ble_nus_send_immediate(sched_buf);
    return 0;
}

void nus_handler_on_disconnect(void)
{
    memset(s_last_sent_ms, 0, sizeof(s_last_sent_ms));
    memset(s_last_sent_sensor_valid, 0, sizeof(s_last_sent_sensor_valid));
}