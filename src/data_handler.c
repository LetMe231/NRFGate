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
#include "model_handler.h"
#include "ble_nus.h"
#include "lora_handler.h"
#include "model_handler.h"
#include "main.h"
#include "semantic_handler.h"

LOG_MODULE_REGISTER(data_handler, LOG_LEVEL_INF);

typedef struct{
    node_identity_t identity;
    int64_t first_seen_ms;
    int64_t last_seen_ms;
    uint32_t packet_count;
} node_entry_t;

static node_entry_t node_table[MAX_NODES];
static uint8_t      node_count = 0;

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

const char *identity_str(const node_identity_t *id){
    static char buf[NODE_ADDR_STR_LEN];
    switch(id->transport){
        case NODE_TRANSPORT_THREAD:
            return id->ipv6;
        case NODE_TRANSPORT_BLE_MESH:
            snprintf(buf, sizeof(buf), "ble:0x%04X", id->mesh_addr);
            return buf;
        default:
            snprintf(buf, sizeof(buf), "Unknown");
            return buf;
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
    for(uint8_t i=0; i<node_count; i++){
        if(identity_match(&node_table[i].identity, id)){
            node_table[i].last_seen_ms = now_ms;
            node_table[i].packet_count++;
            return i;
        }
    }

    // new node
    if(node_count >= MAX_NODES){
        LOG_ERR("Node table full (%d/%d), dropping %s",
                node_count, MAX_NODES, identity_str(id));
        return 0xFF;
    }
    
    uint8_t idx = node_count++;
    node_table[idx].identity       = *id;
    node_table[idx].first_seen_ms  = now_ms;
    node_table[idx].last_seen_ms   = now_ms;
    node_table[idx].packet_count   = 1;
 
    LOG_INF("New node [%d] %s addr=%s",
            idx, transport_str(id->transport), identity_str(id));
    return idx;
}

//===============================================================
// Sensor Logging
//===============================================================

static void log_sensor_data(const struct node_sensor_data *d){
    const struct sensor_payload *p = &d->payload;
    const char *src = identity_str(&d->identity);
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
#define JSON_BUF_SIZE 220

static int build_json(const struct node_sensor_data *d,
                      char *buf, size_t size)
{
    const struct sensor_payload *p = &d->payload;
    int off = 0;

    // Header: node index + transport tag
    const char *tr_tag = (d->identity.transport == NODE_TRANSPORT_BLE_MESH) ? "B" : "T";
    const char *state = semantic_handler_state_str(semantic_handler_get_state(d->node_idx));
    off += snprintf(buf + off, size - off, "{\"node\":%d,\"tr\":\"%s\",\"state\":\"%s\"", d->node_idx, tr_tag, state);
    if(p->present & SENSOR_HAS_SEQ){
        off += snprintf(buf + off, size - off, ",\"seq\":%d", p->seq);
    }

    if (d->identity.transport == NODE_TRANSPORT_BLE_MESH) {
        off += snprintf(buf + off, size - off,
                        ",\"mesh_addr\":%d", d->identity.mesh_addr);
    }

    // IMU — only for Thread/IMU nodes 
    if ((p->present & SENSOR_HAS_ACCEL) == SENSOR_HAS_ACCEL) {
        off += snprintf(buf + off, size - off,
                        ",\"ax\":%d.%03d,\"ay\":%d.%03d,\"az\":%d.%03d",
                        p->ax / 1000, abs(p->ax % 1000),
                        p->ay / 1000, abs(p->ay % 1000),
                        p->az / 1000, abs(p->az % 1000));
    }
    if ((p->present & SENSOR_HAS_GYRO) == SENSOR_HAS_GYRO) {
        off += snprintf(buf + off, size - off,
                        ",\"gx\":%d.%03d,\"gy\":%d.%03d,\"gz\":%d.%03d",
                        p->gx / 1000, abs(p->gx % 1000),
                        p->gy / 1000, abs(p->gy % 1000),
                        p->gz / 1000, abs(p->gz % 1000));
    }
 
    // Environmental — shared
    if (p->present & SENSOR_HAS_TEMP) {
        off += snprintf(buf + off, size - off, ",\"temp\":%d.%01d",
                        p->temp / 1000, abs((p->temp % 1000) / 100));
    }
    if (p->present & SENSOR_HAS_HUM) {
        off += snprintf(buf + off, size - off, ",\"hum\":%d.%01d",
                        p->hum / 1000, abs((p->hum % 1000) / 100));
    }
 
    // Air quality — shared 
    if (p->present & SENSOR_HAS_TVOC) {
        off += snprintf(buf + off, size - off, ",\"tvoc\":%d", p->tvoc);
    }
    if (p->present & SENSOR_HAS_ECO2) {
        off += snprintf(buf + off, size - off, ",\"eco2\":%d", p->eco2);
    }
 
    // Biometric — BLE Mesh only 
    if (p->present & SENSOR_HAS_HEART_RATE) {
        off += snprintf(buf + off, size - off, ",\"hr\":%d", p->heart_rate);
    }
    if (p->present & SENSOR_HAS_SPO2) {
        off += snprintf(buf + off, size - off, ",\"spo2\":%d.%01d",
                        p->spo2 / 1000, abs((p->spo2 % 1000) / 100));
    }
 
    // Close + newline (BLE NUS convention from original code) 
    off += snprintf(buf + off, size - off, "}\n");
 
    if (off >= (int)size) {
        LOG_ERR("JSON buffer overflow (%d >= %d)", off, (int)size);
        return 0;
    }
    return off;
};

// TLV serializer for LoRa binary forwarding
/* Frame layout (all fields present, worst case = 36 bytes):
 *
 *   Offset  Size  Field
 *   ──────  ────  ──────────────────────────────────────────────
 *   0       1     node_idx
 *   1       1     transport  (0 = Thread, 1 = BLE Mesh)
 *   2       2     present bitmask, little-endian
 *   ── then only the fields whose bit is set, in this order: ──
 *   4       2     ax  [int16_t, mg,     little-endian]
 *   6       2     ay
 *   8       2     az
 *   10      2     gx  [int16_t, mdeg/s, little-endian]
 *   12      2     gy
 *   14      2     gz
 *   16      2     temp [int16_t, m°C / 10 → 0.1°C resolution]
 *   18      2     hum  [int16_t, m%RH / 10]
 *   20      2     tvoc [uint16_t, ppb]
 *   22      2     eco2 [uint16_t, ppm]
 *   24      1     heart_rate [uint8_t, bpm]
 *   25      2     spo2 [uint16_t, m% / 10 → 0.1% resolution]
*/
#define WIRE_MAX_SIZE 48   /* comfortably above worst-case 36 bytes */
 
/* Helper macros — write little-endian without casting noise */
#define WRITE_I16(buf, off, val) do { \
    int16_t _v = (int16_t)(val);      \
    (buf)[(off)++] = (uint8_t)(_v);   \
    (buf)[(off)++] = (uint8_t)(_v >> 8); \
} while (0)
 
#define WRITE_U16(buf, off, val) do { \
    uint16_t _v = (uint16_t)(val);    \
    (buf)[(off)++] = (uint8_t)(_v);   \
    (buf)[(off)++] = (uint8_t)(_v >> 8); \
} while (0)
 
static int build_wire(const struct node_sensor_data *d,
                      uint8_t *buf, size_t size)
{
    if (size < WIRE_MAX_SIZE) {
        LOG_ERR("Wire buffer too small (%d < %d)", (int)size, WIRE_MAX_SIZE);
        return 0;
    }
 
    const struct sensor_payload *p = &d->payload;
    int off = 0;
 
    /* 4-byte header */
    buf[off++] = d->node_idx;
    buf[off++] = (uint8_t)d->identity.transport;
    buf[off++] = (uint8_t)(p->present & 0xFF);
    buf[off++] = (uint8_t)((p->present >> 8) & 0xFF);
 
    /* IMU — 2 bytes each, only if all three axes present */
    if ((p->present & SENSOR_HAS_ACCEL) == SENSOR_HAS_ACCEL) {
        WRITE_I16(buf, off, p->ax);
        WRITE_I16(buf, off, p->ay);
        WRITE_I16(buf, off, p->az);
    }
    if ((p->present & SENSOR_HAS_GYRO) == SENSOR_HAS_GYRO) {
        WRITE_I16(buf, off, p->gx);
        WRITE_I16(buf, off, p->gy);
        WRITE_I16(buf, off, p->gz);
    }
 
    /* Environmental — divide by 10 to fit int16_t range */
    if (p->present & SENSOR_HAS_TEMP) { WRITE_I16(buf, off, p->temp / 10); }
    if (p->present & SENSOR_HAS_HUM)  { WRITE_I16(buf, off, p->hum  / 10); }
 
    /* Air quality — plain integers, fit in uint16_t */
    if (p->present & SENSOR_HAS_TVOC) { WRITE_U16(buf, off, p->tvoc); }
    if (p->present & SENSOR_HAS_ECO2) { WRITE_U16(buf, off, p->eco2); }
 
    /* Biometric */
    if (p->present & SENSOR_HAS_HEART_RATE) {
        buf[off++] = (uint8_t)p->heart_rate;   /* 1 byte, 0–255 bpm */
    }
    if (p->present & SENSOR_HAS_SPO2) { WRITE_U16(buf, off, p->spo2 / 10); }
 
    return off;
}
 
#undef WRITE_I16
#undef WRITE_U16
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
            LOG_DBG("LoRa TX %d bytes", frame.len);
            lora_handler_send(frame.data, frame.len);
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
    frame.len = (uint8_t)build_wire(d, frame.data, sizeof(frame.data));
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
        model_handler_start_provisioning();
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
 
        model_handler_unprovision_node(mesh_addr);
        return;
    }
    /* ── lora_enabled ────────────────────────────────────────────*/
    const char *p = strstr(cmd, "\"lora_enabled\"");
    if (!p) { LOG_WRN("Unknown command: %.*s", len, cmd); return; }
 
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

    /* ── set policy ───────────────────────────────────────
         * {"cmd":"set_policy","motion_thr":2.0,"alert_thr":15.0,"co2_thr":1000,...}
     */
    if (strstr(cmd, "\"set_policy\"")) {
        int32_t motion_active = -1, motion_alert = -1, co2_alert = -1;
        int32_t temp_alert = -1, tvoc_alert = -1;
        int32_t thread_ms = -1, ble_ms = -1;
 
        const char *p;
 
        p = strstr(cmd, "\"motion_thr\"");
        if (p) { p = strchr(p, ':'); if (p) motion_active = (int32_t)(atof(p+1) * 100); }
 
        p = strstr(cmd, "\"alert_thr\"");
        if (p) { p = strchr(p, ':'); if (p) motion_alert = (int32_t)(atof(p+1) * 100); }
 
        p = strstr(cmd, "\"co2_thr\"");
        if (p) { p = strchr(p, ':'); if (p) co2_alert = (int32_t)atoi(p+1); }
 
        p = strstr(cmd, "\"temp_alert\"");
        if (p) { p = strchr(p, ':'); if (p) temp_alert = (int32_t)(atof(p+1) * 1000); }
 
        p = strstr(cmd, "\"tvoc_alert\"");
        if (p) { p = strchr(p, ':'); if (p) tvoc_alert = (int32_t)atoi(p+1); }
 
        p = strstr(cmd, "\"thread_ms\"");
        if (p) { p = strchr(p, ':'); if (p) thread_ms = (int32_t)atoi(p+1); }
 
        p = strstr(cmd, "\"ble_ms\"");
        if (p) { p = strchr(p, ':'); if (p) ble_ms = (int32_t)atoi(p+1); }

        p = strstr(cmd, "\"idle_interval\"");
        if (p) { p = strchr(p, ':'); if (p) node_lost_timeout_ms = (int32_t)atoi(p+1)*1000; }
 
        semantic_handler_set_policy(motion_active, motion_alert, co2_alert,
                                    temp_alert, tvoc_alert);
 
        if (thread_ms > 0 || ble_ms > 0) {
            mesh_scheduler_set_timing(
                thread_ms > 0 ? (uint32_t)thread_ms : sched_thread_ms,
                ble_ms    > 0 ? (uint32_t)ble_ms    : sched_ble_ms);
        }
 
        LOG_INF("Policy applied");
        return;
    }
}