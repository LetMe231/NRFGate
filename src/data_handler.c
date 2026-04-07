/**
 * @file data_handler.c
 * @brief Centralized sensor data processing
 *
 * This is the only file that "knows" what to do with sensor values.
 * thread_handler.c just calls data_handler_receive() and forgets.
 *
 * Current behaviour: log everything.
 * Future: add BLE notification, LoRa forwarding, MQTT publish, etc.
 * — all changes stay in this file.
 */
 
#include <zephyr/kernel.h>
#include <zephyr/sys/atomic.h>
#include <zephyr/logging/log.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>

#include "data_handler.h"
#include "ble_mesh_handler.h"
#include "ble_nus.h"
#include "lora_handler.h"
#include "main.h"
#include "semantic_handler.h"
#include "rule_engine.h"

LOG_MODULE_REGISTER(data_handler, LOG_LEVEL_INF);

typedef struct{
    node_identity_t identity;
    int64_t first_seen_ms;
    int64_t last_seen_ms;
    uint32_t packet_count;
} node_entry_t;

static node_entry_t node_table[MAX_NODES];
static uint8_t      node_count = 0;

static K_MUTEX_DEFINE(s_node_mutex);

node_actuator_state_t node_actuator_state[MAX_NODES] = {0};

uint8_t data_handler_get_node_idx_by_mesh_addr(uint16_t mesh_addr)
{
    for (uint8_t i = 0; i < node_count; i++) {
        if (node_table[i].identity.transport == NODE_TRANSPORT_BLE_MESH &&
            node_table[i].identity.mesh_addr == mesh_addr) {
            return i;
        }
    }
    return 0xFF;
}

uint8_t data_handler_get_node_idx_by_ipv6(const char *ipv6)
{
    k_mutex_lock(&s_node_mutex, K_FOREVER);
    for (uint8_t i = 0; i < node_count; i++) {
        if (node_table[i].identity.transport == NODE_TRANSPORT_THREAD &&
            strncmp(node_table[i].identity.ipv6, ipv6, NODE_ADDR_STR_LEN) == 0) {
            k_mutex_unlock(&s_node_mutex);
            return i;
        }
    }
    k_mutex_unlock(&s_node_mutex);
    return 0xFF;
}

atomic_t lora_enabled = ATOMIC_INIT(0); // LoRa forwarding is disabled by default, enabled via BLE cmd
K_SEM_DEFINE(lora_wake_sem, 0, 1);

//===============================================================
// Identity helper
//===============================================================

static bool identity_match(const node_identity_t *a, const node_identity_t *b){
    if(a->transport != b->transport) return false;
    switch(a->transport){
        case NODE_TRANSPORT_THREAD:
            return strncmp(a->ipv6, b->ipv6, NODE_ADDR_STR_LEN) == 0;
        case NODE_TRANSPORT_BLE_MESH:
            return a->mesh_addr == b->mesh_addr;
        default:
            return false;
    }
}

void identity_str(const node_identity_t *id, char *buf, size_t len){
    switch(id->transport){
        case NODE_TRANSPORT_THREAD:
            strncpy(buf, id->ipv6, len);
            break;
        case NODE_TRANSPORT_BLE_MESH:
            snprintf(buf, len, "ble:0x%04X", id->mesh_addr);
            break;
        default:
            snprintf(buf, len, "Unknown");
            break;
    }
}

/* Returns a short transport label for log prefixes. */
static const char *transport_str(node_transport_t t)
{
    switch (t) {
    case NODE_TRANSPORT_THREAD:   return "Thread";
    case NODE_TRANSPORT_BLE_MESH: return "BLE";
    default:                      return "?";
    }
}

 static uint8_t register_or_find_node(const node_identity_t *id, int64_t now_ms){
    // node already exists?
    k_mutex_lock(&s_node_mutex, K_FOREVER);
    for(uint8_t i=0; i<node_count; i++){
        if(identity_match(&node_table[i].identity, id)){
            node_table[i].last_seen_ms = now_ms;
            node_table[i].packet_count++;
            return i;
        }
    }

    // new node
    if(node_count >= MAX_NODES){
        char id_str[NODE_ADDR_STR_LEN];
        identity_str(id, id_str, sizeof(id_str));
        LOG_ERR("Node table full (%d/%d), dropping %s",
                node_count, MAX_NODES, id_str);
        return 0xFF;
    }
    
    uint8_t idx = node_count++;
    node_table[idx].identity       = *id;
    node_table[idx].first_seen_ms  = now_ms;
    node_table[idx].last_seen_ms   = now_ms;
    node_table[idx].packet_count   = 1;
    
    char id_str[NODE_ADDR_STR_LEN];
    identity_str(id, id_str, sizeof(id_str));
    LOG_INF("New node [%d] %s addr=%s",
            idx, transport_str(id->transport), id_str);
    k_mutex_unlock(&s_node_mutex);
    return idx;
}

//===============================================================
// Sensor Logging
//===============================================================

static void log_sensor_data(const struct node_sensor_data *d){
    const struct sensor_payload *p = &d->payload;
    char src[NODE_ADDR_STR_LEN];
    identity_str(&d->identity, src, sizeof(src));
    const char *tr = transport_str(d->identity.transport);
    char hdr[80];
    snprintf(hdr, sizeof(hdr), "[Node %d | %s | %s] rx=%lld ms seq=%d", d->node_idx, tr, src, (long long )d->rx_uptime_ms, d->payload.seq);

    LOG_INF("%s", hdr);

    // IMU
    if((p->present & SENSOR_HAS_ACCEL) == SENSOR_HAS_ACCEL){
        LOG_INF("  Accel: ax=%d.%03d ay=%d.%03d az=%d.%03d [mg]",
                p->ax / 1000, abs(p->ax % 1000),
                p->ay / 1000, abs(p->ay % 1000),
                p->az / 1000, abs(p->az % 1000));
    }

    if((p->present & SENSOR_HAS_GYRO) == SENSOR_HAS_GYRO){
        LOG_INF("  Gyro: gx=%d.%03d gy=%d.%03d gz=%d.%03d [mdeg/s]",
                p->gx / 1000, abs(p->gx % 1000),
                p->gy / 1000, abs(p->gy % 1000),
                p->gz / 1000, abs(p->gz % 1000));
    }

    // Environmental
    if(p->present & SENSOR_HAS_ENV){
        LOG_INF("  Env: temp=%d.%02d C hum=%d.%02d %%", 
                p->temp / 1000, abs(p->temp % 1000),
                p->hum / 1000, abs(p->hum % 1000));
    }
    // Air Quality
    if(p->present & SENSOR_HAS_AIR){
        LOG_INF("  Air: TVOC=%d ppb eCO2=%d ppm", p->tvoc, p->eco2);
    }

    /* ── Biometric (BLE Mesh only for now) ───────────────── */
    if (p->present & SENSOR_HAS_HEART_RATE) {
        LOG_INF("  HR     %d bpm", p->heart_rate);
    }
    if (p->present & SENSOR_HAS_SPO2) {
        LOG_INF("  SpO2   %d.%01d %%",
                p->spo2 / 1000, abs((p->spo2 % 1000) / 100));
    }
    if (p->present & SENSOR_HAS_RAW_RED) {
        LOG_INF("  Red    %d", p->raw_red);
    }
    if (p->present & SENSOR_HAS_RAW_IR) {
        LOG_INF("  IR     %d", p->raw_ir);
    }
    if (p->present & SENSOR_HAS_PM25) {
    LOG_INF("  PM2.5: %d µg/m³", p->pm25);
    }
    if (p->present & SENSOR_HAS_PM10) {
        LOG_INF("  PM10:  %d µg/m³", p->pm10);
    }
    if(p->present & SENSOR_HAS_SWITCH){
        LOG_INF("  Switch: %s", p->switch_state ? "PRESSED" : "IDLE");
    }
}


/* ── BLE NUS Serializer: JSON ─────────────────────────────────────
 *
 * WHY JSON for BLE NUS:
 *   The downstream consumer (phone app, PC tool) can parse JSON
 *   directly without a custom decoder. BLE NUS has no hard payload
 *   size constraint beyond MTU — and with MTU negotiation a modern
 *   phone easily gets 100+ bytes per packet. Human-readability
 *   during development is a bonus.
 *
 * WHY integer fixed-point instead of %f:
 *   Zephyr's minimal libc does not guarantee %f support and pulling
 *   in float formatting code costs ~2 kB flash. We use the same
 *   value / 1000, abs(value % 1000) pattern used in logging.
 *
 * Only fields present in the bitmask are emitted, so a BLE Mesh
 * temp/hum node doesn't produce `"ax":0.000` etc.
 *
 * Returns bytes written (excl. null terminator), or 0 on overflow.
 */
#define JSON_BUF_SIZE 320

static int build_json(const struct node_sensor_data *d,
                      char *buf, size_t size)
{
    const struct sensor_payload *p = &d->payload;
    int off = 0;

        int rem;
 
    #define JSON_APPEND(...) do {                                    \
        rem = (int)size - off;                                       \
        if (rem <= 0) { goto overflow; }                             \
        off += snprintf(buf + off, (size_t)rem, __VA_ARGS__);        \
    } while (0)
    
    // Header: node index + transport tag
    const char *tr_tag = (d->identity.transport == NODE_TRANSPORT_BLE_MESH) ? "B" : "T";
    const char *state = semantic_handler_state_str(semantic_handler_get_state(d->node_idx));
    JSON_APPEND("{\"node\":%d,\"tr\":\"%s\",\"state\":\"%s\"", d->node_idx, tr_tag, state);
    if (p->present & SENSOR_HAS_SEQ) {
        JSON_APPEND(",\"seq\":%d", p->seq);
    }
    if (d->identity.transport == NODE_TRANSPORT_BLE_MESH) {
        JSON_APPEND(",\"mesh_addr\":%d", d->identity.mesh_addr);
    }
 
    // IMU — only for Thread/IMU nodes
    if ((p->present & SENSOR_HAS_ACCEL) == SENSOR_HAS_ACCEL) {
        JSON_APPEND(",\"ax\":%d.%03d,\"ay\":%d.%03d,\"az\":%d.%03d",
                    p->ax / 1000, abs(p->ax % 1000),
                    p->ay / 1000, abs(p->ay % 1000),
                    p->az / 1000, abs(p->az % 1000));
    }
    if ((p->present & SENSOR_HAS_GYRO) == SENSOR_HAS_GYRO) {
        JSON_APPEND(",\"gx\":%d.%03d,\"gy\":%d.%03d,\"gz\":%d.%03d",
                    p->gx / 1000, abs(p->gx % 1000),
                    p->gy / 1000, abs(p->gy % 1000),
                    p->gz / 1000, abs(p->gz % 1000));
    }
 
    // Environmental — shared
    if (p->present & SENSOR_HAS_TEMP) {
        JSON_APPEND(",\"temp\":%d.%01d",
                    p->temp / 1000, abs((p->temp % 1000) / 100));
    }
    if (p->present & SENSOR_HAS_HUM) {
        JSON_APPEND(",\"hum\":%d.%01d",
                    p->hum / 1000, abs((p->hum % 1000) / 100));
    }
 
    // Air quality — shared
    if (p->present & SENSOR_HAS_TVOC) {
        JSON_APPEND(",\"tvoc\":%d", p->tvoc);
    }
    if (p->present & SENSOR_HAS_ECO2) {
        JSON_APPEND(",\"eco2\":%d", p->eco2);
    }
 
    // Biometric — BLE Mesh only
    if (p->present & SENSOR_HAS_HEART_RATE) {
        JSON_APPEND(",\"hr\":%d", p->heart_rate);
    }
    if (p->present & SENSOR_HAS_SPO2) {
        JSON_APPEND(",\"spo2\":%d.%01d",
                    p->spo2 / 1000, abs((p->spo2 % 1000) / 100));
    }
    if (d->identity.transport == NODE_TRANSPORT_THREAD) {
        JSON_APPEND(",\"ipv6\":\"%s\"", d->identity.ipv6);
    }
    if (p->present & SENSOR_HAS_PM25) { JSON_APPEND(",\"pm25\":%d", p->pm25); }
    if (p->present & SENSOR_HAS_PM10) { JSON_APPEND(",\"pm10\":%d", p->pm10); }
    if (p->present & SENSOR_HAS_SWITCH && p->switch_state) {
        JSON_APPEND(",\"sw\":%d", p->switch_state);
    }
    /* Aktuator-Status: vom Gateway gecacht, nicht vom Node gemeldet */
    if (node_actuator_state[d->node_idx].known) {
        JSON_APPEND(",\"light\":%d",
                    node_actuator_state[d->node_idx].light_on ? 1 : 0);
    }
    // Close + newline
    JSON_APPEND("}\n");
 
#undef JSON_APPEND
 
    return off;
 
overflow:
    LOG_ERR("JSON buffer overflow at off=%d size=%d", off, (int)size);
    return 0;
   
};

//===============================================================
// LoRa send thread and queue
//===============================================================
typedef struct{
    uint8_t data[WIRE_MAX_SIZE];
    uint8_t len;
} lora_frame_t;

#define LORA_STACK_SIZE 2048
#define LORA_PRIORITY   7

K_MSGQ_DEFINE(lora_msgq, sizeof(lora_frame_t), 4, 4);
K_THREAD_STACK_DEFINE(lora_stack, LORA_STACK_SIZE);
static struct k_thread lora_tid;

static void lora_send_thread(void *p1, void *p2, void *p3)
{
    lora_frame_t frame;
    ARG_UNUSED(p1), ARG_UNUSED(p2), ARG_UNUSED(p3);

    while (1) {
        if(!atomic_get(&lora_enabled)){
            LOG_DBG("LoRa forwarding disabled, sleeping...");
            k_sem_take(&lora_wake_sem, K_FOREVER);
            LOG_DBG("LoRa forwarding enabled, waking up...");
        }
        if (k_msgq_get(&lora_msgq, &frame, K_FOREVER) == 0) {
            if (atomic_get(&lora_enabled)) {
                LOG_DBG("LoRa TX %d bytes", frame.len);
                lora_handler_send(frame.data, frame.len);
            }
        }
    }
}


/* ── Forwarding ───────────────────────────────────────────────────
 *
 * BLE NUS → JSON  : human-readable, easy for phone/PC to consume,
 *                   MTU is negotiable so size is not critical.
 *
 * LoRa    → binary: compact wire format, fits in worst-case SF12
 *                   payload with room to spare.
 *
 * Both use K_NO_WAIT / non-blocking sends so data_handler_receive()
 * always returns quickly regardless of radio state.
 */
static void forward_ble_nus(const struct node_sensor_data *d)
{
    char buf[JSON_BUF_SIZE];
    if (build_json(d, buf, sizeof(buf)) == 0) return;

    /*
     * ble_nus_send() returns void — it posts to the BLE stack's
     * internal queue and does not report per-call errors.
     * If no client is connected the stack silently drops the data.
     */
    ble_nus_send(buf);
}

static void forward_lora(const struct node_sensor_data *d)
{
    lora_frame_t frame = {0};
    frame.len = (uint8_t)wire_build(d, frame.data, sizeof(frame.data));
    if(frame.len <= 0) return;
    
    LOG_DBG("Forwarding to LoRa (node %d, %d bytes)", d->node_idx, frame.len);

    int err = k_msgq_put(&lora_msgq, &frame, K_NO_WAIT);
    if (err) {
        LOG_WRN("LoRa queue full, packet dropped (node %d)", d->node_idx);
    }
}


/* ── Public API ───────────────────────────────────────────────── */

int data_handler_init(void){
    memset(node_table, 0, sizeof(node_table));
    node_count = 0;

    // start LoRa send thread
    k_thread_create(&lora_tid, lora_stack, LORA_STACK_SIZE,
                    lora_send_thread, NULL, NULL, NULL,
                    LORA_PRIORITY, 0, K_NO_WAIT);
    k_thread_name_set(&lora_tid, "lora_send");
    
    LOG_INF("Data handler initialized (max nodes: %d)", MAX_NODES);
    return 0;
}

void data_handler_receive(const struct node_sensor_data *data){
    struct node_sensor_data d = *data; // make a local copy

    d.node_idx = register_or_find_node(&d.identity, d.rx_uptime_ms);
    if(d.node_idx == 0xFF){
        LOG_WRN("Received data from unknown node, dropping");
        return;
    }

    log_sensor_data(&d);

    semantic_handler_process(&d);

    if (d.payload.present & SENSOR_HAS_SWITCH) {
        rule_engine_on_switch(d.node_idx, d.payload.switch_state != 0, d.identity.transport);
    }
    
    if(ble_nus_is_ready()){
        forward_ble_nus(&d);
    }
    if(atomic_get(&lora_enabled)){
        forward_lora(&d);
    }

    if (node_count > 0) {
        bool has_ble    = false;
        bool has_thread = false;
        for (uint8_t i = 0; i < node_count; i++) {
            node_state_t state = semantic_handler_get_state(i);
            if (state == NODE_STATE_LOST || state == NODE_STATE_UNKNOWN) continue;
            if (node_table[i].identity.transport == NODE_TRANSPORT_BLE_MESH)
                has_ble = true;
            else
                has_thread = true;
        }
        sched_mode_t target = SCHED_MODE_NORMAL;
        if (has_ble && !has_thread)    target = SCHED_MODE_BLE_ONLY;
        if (has_thread && !has_ble)    target = SCHED_MODE_THREAD_ONLY;
        mesh_scheduler_set_mode(target);
    }
}

void data_handler_cmd(const char *cmd, uint16_t len)
{
    LOG_INF("NUS cmd: %.*s", len, cmd);
 
    /* ── start_provisioning ──────────────────────────────────────*/
    if (strstr(cmd, "\"start_provisioning\"")) {
        LOG_INF("Provisioning started via dashboard");
        ble_mesh_handler_start_provisioning();
        return;
    }

    /* ── unprovision ─────────────────────────────────────────────*/
    if (strstr(cmd, "\"unprovision\"")) {
        const char *p = strstr(cmd, "\"mesh_addr\"");
        if (!p) { LOG_WRN("unprovision: missing mesh_addr"); return; }
        p = strchr(p, ':');
        if (!p) { LOG_WRN("unprovision: malformed"); return; }
        uint16_t mesh_addr = (uint16_t)atoi(p + 1);
 
        if (mesh_addr == 0 || mesh_addr > 0x7FFF) {
            LOG_WRN("unprovision: invalid mesh_addr %d", mesh_addr);
            return;
        }
 
        LOG_INF("Unprovisioning mesh_addr=0x%04X", mesh_addr);
 
        /* Remove from local node table if present */
        for (uint8_t i = 0; i < node_count; i++) {
            if (node_table[i].identity.transport == NODE_TRANSPORT_BLE_MESH &&
                node_table[i].identity.mesh_addr == mesh_addr) {
                if (i < node_count - 1) {
                    memmove(&node_table[i], &node_table[i + 1],
                            (node_count - i - 1) * sizeof(node_entry_t));
                }
                node_count--;
                LOG_INF("  Removed from local table");
                break;
            }
        }
 
        ble_mesh_handler_unprovision_node(mesh_addr);
        return;
    }
   /* ── add_rule ────────────────────────────────────────────────*/
    if (strstr(cmd, "\"add_rule\"")) {
        gateway_rule_t rule = {0};

        const char *f;
        f = strstr(cmd, "\"src\"");
        if (!f) { LOG_WRN("add_rule: missing src"); return; }
        f = strchr(f, ':'); if (!f) { LOG_WRN("add_rule: malformed src"); return; }
        rule.src_node_idx = (uint8_t)atoi(f + 1);

        f = strstr(cmd, "\"trig\"");
        if (!f) { LOG_WRN("add_rule: missing trig"); return; }
        f = strchr(f, ':'); if (!f) { LOG_WRN("add_rule: malformed trig"); return; }
        rule.trigger = (rule_trigger_t)atoi(f + 1);

        f = strstr(cmd, "\"act\"");
        if (!f) { LOG_WRN("add_rule: missing act"); return; }
        f = strchr(f, ':'); if (!f) { LOG_WRN("add_rule: malformed act"); return; }
        rule.action = (rule_action_t)atoi(f + 1);

        f = strstr(cmd, "\"type\"");
        if (!f) { LOG_WRN("add_rule: missing type"); return; }
        f = strchr(f, ':'); if (!f) { LOG_WRN("add_rule: malformed type"); return; }
        while (*++f == ' ' || *f == '"') {}
        rule.target_is_thread = (strncmp(f, "thread", 6) == 0);

        f = strstr(cmd, "\"target\"");
        if (!f) { LOG_WRN("add_rule: missing target"); return; }
        f = strchr(f, ':'); if (!f) { LOG_WRN("add_rule: malformed target"); return; }
        while (*++f == ' ') {}
        if (*f == '"') {
            /* Thread: IPv6 string */
            f++;
            size_t n = 0;
            while (f[n] && f[n] != '"' && n < sizeof(rule.target.ipv6) - 1) n++;
            memcpy(rule.target.ipv6, f, n);
            rule.target.ipv6[n] = '\0';
        } else {
            /* BLE Mesh: integer address */
            rule.target.mesh_addr = (uint16_t)atoi(f);
        }

        int idx = rule_engine_add(&rule);
        if (idx >= 0) {
            LOG_INF("add_rule OK → slot %d", idx);
        }
        return;
    }

    /* ── list_rules ──────────────────────────────────────────────*/
    if (strstr(cmd, "\"list_rules\"")) {
        char buf[512];
        rule_engine_to_json(buf, sizeof(buf));
        ble_nus_send(buf);
        return;
    }

    /* ── remove_rule ─────────────────────────────────────────────*/
    if (strstr(cmd, "\"remove_rule\"")) {
        const char *f = strstr(cmd, "\"idx\"");
        if (!f) { LOG_WRN("remove_rule: missing idx"); return; }
        f = strchr(f, ':'); if (!f) { LOG_WRN("remove_rule: malformed"); return; }
        rule_engine_remove((uint8_t)atoi(f + 1));
        /* Sofort aktualisierte Liste zurückschicken */
        char buf[512];
        rule_engine_to_json(buf, sizeof(buf));
        ble_nus_send(buf);
        return;
    }
    /* ── lora_enabled ────────────────────────────────────────────*/
    {
        const char *p = strstr(cmd, "\"lora_enabled\"");
        if (!p) { LOG_WRN("Unknown command: %.*s", len, cmd); return; }  /* ← return war hier weg */

        p = strchr(p, ':');
        if (!p) { LOG_WRN("Malformed lora_enabled command"); return; }
        while (*++p == ' ' || *p == '\t') {}

        if (strncmp(p, "true", 4) == 0) {
            atomic_set(&lora_enabled, 1);
            k_sem_give(&lora_wake_sem);
            LOG_INF("LoRa enabled");
        } else if (strncmp(p, "false", 5) == 0) {
            atomic_set(&lora_enabled, 0);
            LOG_INF("LoRa disabled");
        }
        return;
    }
    /* ── reconfigure ──────────────────────────────────────────────*/
    if (strstr(cmd, "\"reconfigure\"")) {
        const char *p = strstr(cmd, "\"mesh_addr\"");
        if (!p) { LOG_WRN("reconfigure: missing mesh_addr"); return; }
        p = strchr(p, ':');
        if (!p) { LOG_WRN("reconfigure: malformed"); return; }
        uint16_t mesh_addr = (uint16_t)atoi(p + 1);
        ble_mesh_handler_reconfigure_node(mesh_addr);
        return;
    }

     LOG_WRN("Unknown command: %.*s", len, cmd);
}