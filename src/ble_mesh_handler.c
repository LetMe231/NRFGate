/**
 * @file ble_mesh_handler.c
 * @brief BLE Mesh provisioner and sensor client
 *
 * Handles:
 *   - Gateway self-provisioning with hardcoded network/app keys
 *   - Auto-provisioning of ESP32 BLE Mesh sensor nodes (UUID prefix filter)
 *   - Sensor Status message parsing (temperature, humidity, eCO2, TVOC, etc.)
 *   - Configuration of provisioned nodes (AppKey, bind, publication)
 *
 * Provisioning flow:
 *   1. Button 2 opens a 30 s scanning window
 *   2. Unprovisioned beacons with ESP32 UUID prefix are auto-provisioned
 *   3. Each node is configured with AppKey + Sensor Server bind + publication
 *   4. Radio scheduler resumes after window closes
 */

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/bluetooth/mesh.h>
#include <zephyr/drivers/hwinfo.h>
#include <zephyr/settings/settings.h>
#include <zephyr/bluetooth/bluetooth.h>
#include <hal/nrf_ficr.h>
#include <dk_buttons_and_leds.h>

#include "ble_mesh_handler.h"
#include "data_handler.h"
#include "ble_nus.h"
#include "mesh_ctrl.h"
#include "data_handler.h"
#include "semantic_handler.h"
#include "mesh_parser.h"

LOG_MODULE_REGISTER(ble_mesh_handler, LOG_LEVEL_INF);

static bool s_unprov_pending = false;       // True when device has been unprovisioned -> pending guard
K_MUTEX_DEFINE(s_pending_mutex);

/* Scheduler control (defined in main.c) */
extern void mesh_scheduler_pause(void);
extern void mesh_scheduler_start(void);

/* ── Sensor Property IDs (BLE Mesh Sensor Model) ────────────── */

#define PROP_TEMPERATURE 0x004F
#define PROP_HUMIDITY    0x0076
#define PROP_ECO2        0x0008
#define PROP_HEART_RATE  0x0100
#define PROP_SPO2        0x0101
#define PROP_TVOC        0x0102
#define PROP_RAW_RED     0x0103
#define PROP_RAW_IR      0x0104
#define PROP_SWITCH      0x0105
#define PROP_LIGHT_STATE 0x0106
#define PROP_SENSOR_SEQ  0x07FF

/* ── Network Credentials ────────────────────────────────────── */

static const uint8_t net_key[16] = {
    0xF3, 0x43, 0xBB, 0xCD, 0x11, 0x48, 0x9F, 0x37,
    0x21, 0xF3, 0x23, 0xAC, 0xD0, 0x72, 0x9E, 0xBA,
};

static const uint8_t app_key[16] = {
    0xFC, 0x00, 0x10, 0x2B, 0xC3, 0xBD, 0x0E, 0x62,
    0x19, 0xCC, 0xB1, 0x90, 0xB1, 0x0A, 0x88, 0xA9,
};

#define NET_IDX      0x0000
#define APP_IDX      0x0000
#define GROUP_ADDR   0xC000
#define GATEWAY_ADDR 0x0001

/* ── Auto-Provisioning Configuration ────────────────────────── */

#define ESP32_UUID_PREFIX_0 0x32
#define ESP32_UUID_PREFIX_1 0x10
#define PROV_WINDOW_SECONDS 30
#define FIRST_NODE_ADDR     0x0002
#define SENSOR_CLI_MODEL_ID 0x1102

/* Global timestamp for last received Mesh message */
int64_t g_last_mesh_rx_ms = 0; 

/* ── Sensor Status Opcode ────────────────────────────────────── */

#define BT_MESH_MODEL_OP_SENSOR_STATUS BT_MESH_MODEL_OP_1(0x52)

static uint16_t unprov_target_addr;
static struct k_work_delayable unprovision_work;

/* ── Sensor Status Payload Parser ────────────────────────────── */

static void process_sensor_status(const uint8_t *data, uint16_t len,
                                  uint16_t src_addr, int8_t rssi, uint8_t hops)
{
    struct sensor_payload p = {0};
    bool ok = mesh_parse_sensor_status(data, len, &p);
    if (!ok) {
        LOG_WRN("No recognised properties from 0x%04X", src_addr);
        return;
    }

    struct node_sensor_data nd = {
        .identity = {
            .transport = NODE_TRANSPORT_BLE_MESH,
            .mesh_addr = src_addr,
        },
        .rx_uptime_ms = k_uptime_get(),
        .payload = p,
        .rssi  = rssi,
        .hops  = hops,
    };

    data_handler_receive(&nd);
}

/* ── Sensor Client Opcode Handler ────────────────────────────── */

/* Publication TTL (set in configure_node_handler) */
#define MESH_PUB_TTL 5

static int sensor_status_handler(const struct bt_mesh_model *model,
                                 struct bt_mesh_msg_ctx *ctx,
                                 struct net_buf_simple *buf)
{
    g_last_mesh_rx_ms = k_uptime_get();

    /* Hop count: how many relay hops did this packet take?
     * recv_ttl = TTL remaining when packet arrived at gateway.
     * hops = pub_TTL - recv_ttl  (0 = direct, 1 = one relay, etc.) */
    uint8_t hops = (ctx->recv_ttl <= MESH_PUB_TTL)
                   ? (MESH_PUB_TTL - ctx->recv_ttl)
                   : 0;
    int8_t rssi = ctx->recv_rssi;

    LOG_INF("Sensor Status from 0x%04X (%u bytes) rssi=%d hops=%u",
            ctx->addr, buf->len, rssi, hops);

    process_sensor_status(buf->data, buf->len, ctx->addr, rssi, hops);
    return 0;
}

static const struct bt_mesh_model_op sensor_cli_ops[] = {
    {BT_MESH_MODEL_OP_SENSOR_STATUS, 0, sensor_status_handler},
    BT_MESH_MODEL_OP_END,
};

/* ── Generic OnOff Client ────────────────────────────────────── */
/* Status handler — called when a lamp sends its OnOff state back.
 * We forward to mesh_ctrl so it can update its state cache. */
static int onoff_status_handler(const struct bt_mesh_model *model,
                                 struct bt_mesh_msg_ctx *ctx,
                                 struct net_buf_simple *buf)
{
    uint8_t present = net_buf_simple_pull_u8(buf);
    mesh_ctrl_on_status(ctx->addr, present);

    // actualize local cache of actuator states in data handler (for rules engine)
    uint8_t idx = data_handler_get_node_idx_by_mesh_addr(ctx->addr);
    if (idx != 0xFF) {
        node_actuator_state[idx].known    = true;
        node_actuator_state[idx].light_on = (present != 0);
        LOG_INF("Light state confirmed: node_idx=%d %s",
                idx, present ? "ON" : "OFF");
    }
    return 0;
}

static const struct bt_mesh_model_op onoff_cli_ops[] = {
    { BT_MESH_MODEL_OP_GEN_ONOFF_STATUS, BT_MESH_LEN_MIN(1), onoff_status_handler },
    BT_MESH_MODEL_OP_END,
};

BT_MESH_MODEL_PUB_DEFINE(onoff_cli_pub, NULL, 4);

/* ── Device UUID (from FICR) ─────────────────────────────────── */

static void get_device_uuid(uint8_t dev_uuid[16])
{
    uint32_t id0 = nrf_ficr_deviceid_get(NRF_FICR, 0);
    uint32_t id1 = nrf_ficr_deviceid_get(NRF_FICR, 1);

    memcpy(&dev_uuid[0], &id0, 4);
    memcpy(&dev_uuid[4], &id1, 4);
    memset(&dev_uuid[8], 0x00, 8);
}

/* ── Work Queue for Configuration Tasks ──────────────────────── */

#define CFG_WQ_STACK_SIZE   4096
#define RETRY_BASE_MS       500
#define RETRY_MAX_SHIFT     5

static K_THREAD_STACK_DEFINE(config_wq_stack, CFG_WQ_STACK_SIZE);
static struct k_work_q config_wq;

static struct k_work_delayable configure_work;
static int configure_retry_count;

static struct k_work_delayable configure_node_work;
static int node_retry_count;

static struct k_work_delayable prov_timeout_work;

/* ── Provisioning State ──────────────────────────────────────── */

static bool prov_scanning;
static bool prov_in_progress;
static uint16_t next_node_addr = FIRST_NODE_ADDR;
static uint16_t pending_node_addr;

/* ── Gateway Self-Configuration ──────────────────────────────── */

static void schedule_gateway_retry(void)
{
    int shift = MIN(configure_retry_count, RETRY_MAX_SHIFT);
    uint32_t delay_ms = RETRY_BASE_MS << shift;

    LOG_WRN("Gateway config retry #%d in %u ms",
            configure_retry_count + 1, delay_ms);
    configure_retry_count++;
    k_work_schedule_for_queue(&config_wq, &configure_work, K_MSEC(delay_ms));
}

static void configure_handler(struct k_work *work)
{
    int err;
    uint8_t status;

    LOG_INF("Configuring Gateway (addr 0x%04X)", GATEWAY_ADDR);

    err = bt_mesh_app_key_add(APP_IDX, NET_IDX, app_key);
    if (err && err != -EALREADY) {
        LOG_ERR("AppKey add failed: %d", err);
        schedule_gateway_retry();
        return;
    }

    err = bt_mesh_cfg_cli_mod_app_bind(NET_IDX, GATEWAY_ADDR,
                                       GATEWAY_ADDR, APP_IDX,
                                       SENSOR_CLI_MODEL_ID, &status);
    if (err) {
        LOG_ERR("AppKey bind failed (Sensor CLI): %d", err);
        schedule_gateway_retry();
        return;
    }

    /* Bind Generic OnOff Client — required to send ON/OFF to lamp nodes */
    err = bt_mesh_cfg_cli_mod_app_bind(NET_IDX, GATEWAY_ADDR,
                                       GATEWAY_ADDR, APP_IDX,
                                       MESH_CTRL_ONOFF_CLI_ID, &status);
    if (err && err != -EALREADY) {
        LOG_ERR("AppKey bind failed (OnOff CLI): %d", err);
        schedule_gateway_retry();
        return;
    }
    LOG_INF("Generic OnOff Client bound to AppKey 0x%04X", APP_IDX);

    err = bt_mesh_cfg_cli_mod_sub_add(NET_IDX, GATEWAY_ADDR,
                                      GATEWAY_ADDR, GROUP_ADDR,
                                      SENSOR_CLI_MODEL_ID, &status);
    if (err || status) {
        LOG_ERR("Subscribe failed: err=%d status=0x%02X", err, status);
        schedule_gateway_retry();
        return;
    }

    LOG_INF("Sensor Client bound to AppKey, subscribed to 0x%04X", GROUP_ADDR);

    if (IS_ENABLED(CONFIG_SETTINGS)) {
        settings_save();
    }
}

#define MAX_PENDING_NODES 5
static uint16_t pending_addrs[MAX_PENDING_NODES];
static int pending_count = 0;

/* ── Node Configuration (after provisioning) ─────────────────── */

static void schedule_node_retry(void)
{
    int shift = MIN(node_retry_count, RETRY_MAX_SHIFT);
    uint32_t delay_ms = RETRY_BASE_MS << shift;

    LOG_WRN("Node config retry #%d in %u ms",
            node_retry_count + 1, delay_ms);
    if (ble_nus_is_ready()) {
        char buf[80];
        snprintf(buf, sizeof(buf),
                 "{\"type\":\"prov_event\",\"event\":\"retry\","
                 "\"addr\":%u,\"attempt\":%d}\n",
                 pending_node_addr, node_retry_count + 1);
        ble_nus_send(buf);
    }
    node_retry_count++;
    mesh_scheduler_start();
    k_work_schedule_for_queue(&config_wq, &configure_node_work,
                              K_MSEC(delay_ms));
}

static void configure_node_handler(struct k_work *work)
{
    int err;
    uint8_t status;
    uint16_t addr = pending_node_addr;

    mesh_scheduler_pause();

    LOG_INF("Configuring node 0x%04X...", addr);

    /* Clear RPL entry for this node before configuration.
     * After re-provisioning, the node resets seq=0 but the gateway's
     * RPL still has old (higher) seq numbers → all config messages
     * are blocked as replays → AppKey/bind/pub silently fail.
     * Deleting the RPL entry lets the gateway accept seq=0 again. */
    char rpl_key[24];
    snprintf(rpl_key, sizeof(rpl_key), "bt/mesh/RPL/%x", addr);
    settings_delete(rpl_key);
    LOG_INF("  RPL entry cleared for 0x%04X", addr);

    /* Brief delay so the RPL deletion takes effect before sending */
    k_msleep(100);

    /* 1. Add AppKey */
    err = bt_mesh_cfg_cli_app_key_add(NET_IDX, addr,
                                      NET_IDX, APP_IDX,
                                      app_key, &status);
    if (err == -ETIMEDOUT) {
        LOG_WRN("  AppKey add timed out (RPL block?) — assuming success");
    } else if (err && err != -EALREADY) {
        LOG_ERR("  AppKey add failed: err=%d status=0x%02X", err, status);
        schedule_node_retry();
        return;
    } else {
        LOG_INF("  AppKey added to 0x%04X", addr);
    }

    /* 2. Bind to Sensor Server (0x1100) */
    err = bt_mesh_cfg_cli_mod_app_bind(NET_IDX, addr, addr, APP_IDX,
                                       BT_MESH_MODEL_ID_SENSOR_SRV,
                                       &status);
    if (err == -ETIMEDOUT) {
        LOG_WRN("  Bind timed out (RPL block?) — assuming success");
    } else if (err || status) {
        LOG_ERR("  Bind failed: err=%d status=0x%02X", err, status);
        schedule_node_retry();
        return;
    } else {
        LOG_INF("  Bound to Sensor Server");
    }

    /* Bind Generic OnOff Server (falls vorhanden) */
    err = bt_mesh_cfg_cli_mod_app_bind(NET_IDX, addr, addr,
                                        APP_IDX,
                                        BT_MESH_MODEL_ID_GEN_ONOFF_SRV,
                                        &status);
    if (err && err != -ETIMEDOUT) {
        LOG_WRN("OnOff bind failed: %d", err);
    } else {
        LOG_INF("Generic OnOff Server bound");
    }

    /* 3. Enable Relay — allows multi-hop mesh.
     *
     * retransmit = BT_MESH_TRANSMIT(0, 0) means each relay node
     * forwards a packet exactly ONCE (no retransmit).
     *
     * Compare: original sender uses TRANSMIT(2,20) = 3 total TX.
     * Relay nodes forward only 1×, so total packets per hop = N_relays × 1
     * instead of N_relays × 3. The gateway RPL deduplicates silently.
     *
     * TTL=5 on publication limits flood radius to 5 hops max.
     */
    uint8_t relay_status, transmit_status;
    err = bt_mesh_cfg_cli_relay_set(NET_IDX, addr,
                                    BT_MESH_RELAY_ENABLED,
                                    BT_MESH_TRANSMIT(0, 0),
                                    &relay_status, &transmit_status);
    if (err == -ETIMEDOUT) {
        LOG_WRN("  Relay set timed out — assuming enabled");
    } else if (err) {
        LOG_WRN("  Relay set failed: %d (non-fatal)", err);
    } else {
        LOG_INF("  Relay enabled on 0x%04X (1x forward, no retransmit)", addr);
    }

    /* 4. Set publication to group address */
    struct bt_mesh_cfg_cli_mod_pub pub = {
        .addr      = GROUP_ADDR,
        .app_idx   = APP_IDX,
        .ttl       = 5,  /* 5 hops max — enough for most deployments */
        .period    = 0,
        .transmit  = BT_MESH_TRANSMIT(2, 20),  /* 3 total TX, 20ms interval */
        .cred_flag = false,
    };

    err = bt_mesh_cfg_cli_mod_pub_set(NET_IDX, addr, addr,
                                      BT_MESH_MODEL_ID_SENSOR_SRV,
                                      &pub, &status);
    if (err == -ETIMEDOUT) {
        LOG_WRN("  Pub set timed out (RPL block?) — assuming success");
    } else if (err || status) {
        LOG_ERR("  Pub set failed: err=%d status=0x%02X", err, status);
        schedule_node_retry();
        return;
    } else {
        LOG_INF("  Publication set to 0x%04X", GROUP_ADDR);
    }

   LOG_INF("=== Node 0x%04X fully configured ===", addr);

    if (ble_nus_is_ready()) {
        char buf[64];
        snprintf(buf, sizeof(buf),
                 "{\"type\":\"prov_event\",\"event\":\"configured\",\"addr\":%u}\n",
                 addr);
        ble_nus_send(buf);
    }

#if IS_ENABLED(CONFIG_BT_MESH_DF_CLI)
    /* Configure Directed Forwarding path: node → gateway.
     * This tells the node to discover and cache the optimal path
     * to GATEWAY_ADDR (0x0001) so only path nodes forward messages.
     *
     * Discovery uses RSSI — nodes that can't hear each other won't
     * be selected as intermediate hops. Multi-hop only activates
     * when the direct path is unavailable. */
    struct bt_mesh_df_path_metric pm = {
        .metric    = BT_MESH_DF_METRIC_TYPE_NODE_COUNT,
        .threshold = 5,
    };
    err = bt_mesh_df_cli_directed_forwarding_table_entries_delete(
              NET_IDX, addr, GATEWAY_ADDR);
    (void)err;  /* ignore if no entry exists */

    uint16_t df_status;
    err = bt_mesh_df_cli_path_request(
              NET_IDX, addr,
              GATEWAY_ADDR,   /* destination: gateway */
              0x00FF,         /* dependent originator: unicast only */
              &pm,
              &df_status);
    if (err == -ETIMEDOUT) {
        LOG_WRN("  DF path request timed out — flooding fallback");
    } else if (err) {
        LOG_WRN("  DF path request failed: %d (node may not support DF)", err);
    } else {
        LOG_INF("  DF path established: 0x%04X → 0x%04X (status=%d)",
                addr, GATEWAY_ADDR, df_status);
    }
#endif

    if (IS_ENABLED(CONFIG_SETTINGS)) {
        settings_save();
    }

    /* Process next node in queue */
    pending_count--;
    if (pending_count > 0) {
        memmove(&pending_addrs[0], &pending_addrs[1],
                pending_count * sizeof(uint16_t));
        pending_node_addr = pending_addrs[0];
        node_retry_count = 0;
        k_work_schedule_for_queue(&config_wq, &configure_node_work,
                                  K_SECONDS(2));
    } else if (!prov_scanning) {
        mesh_scheduler_start();
    }
}
/* ── Address Management ──────────────────────────────────────── */

static void init_next_node_addr(void)
{
    uint16_t addr = bt_mesh_cdb_free_addr_get(1);

    if (addr == BT_MESH_ADDR_UNASSIGNED) {
        LOG_ERR("No free node address available");
        next_node_addr = BT_MESH_ADDR_UNASSIGNED;
        return;
    }

    next_node_addr = addr;
    LOG_INF("Next available node address: 0x%04X", next_node_addr);
}

// Debugging
static uint8_t dump_node(struct bt_mesh_cdb_node *node, void *user_data)
{
    LOG_INF("CDB node: addr=0x%04X elem=%u net_idx=0x%04X configured=%d",
            node->addr, node->num_elem, node->net_idx,
            atomic_test_bit(node->flags, BT_MESH_CDB_NODE_CONFIGURED));
    return BT_MESH_CDB_ITER_CONTINUE;
}

static void dump_cdb_nodes(void)
{
    bt_mesh_cdb_node_foreach(dump_node, NULL);
}

static uint8_t evict_lost_node_cb(struct bt_mesh_cdb_node *node, void *user_data)
{
    // Prüfe ob dieser Node im data_handler als LOST gilt
    uint8_t idx = data_handler_get_node_idx_by_mesh_addr(node->addr);
    if (idx != 0xFF && semantic_handler_get_state(idx) == NODE_STATE_LOST) {
        LOG_WRN("Evicting LOST node 0x%04X from CDB", node->addr);
        bt_mesh_cdb_node_del(node, true);
        return BT_MESH_CDB_ITER_STOP;
    }
    return BT_MESH_CDB_ITER_CONTINUE;
}

/* ── Provisioning Callbacks ──────────────────────────────────── */

static void unprovisioned_beacon_cb(uint8_t uuid[16],
                                    bt_mesh_prov_oob_info_t oob_info,
                                    uint32_t *uri_hash)
{
    LOG_DBG("Beacon received: %02X%02X scanning=%d", 
            uuid[0], uuid[1], prov_scanning);
    
    if (!prov_scanning || prov_in_progress) {
        return;
    }

    /* UUID prefix filter */
    if (uuid[0] != ESP32_UUID_PREFIX_0 ||
        uuid[1] != ESP32_UUID_PREFIX_1) {
        LOG_WRN("UUID prefix mismatch: %02X%02X != %02X%02X",
                uuid[0], uuid[1], ESP32_UUID_PREFIX_0, ESP32_UUID_PREFIX_1);
        return;
    }

    /* Skip already-provisioned UUIDs */
    uint16_t a = FIRST_NODE_ADDR;
    struct bt_mesh_cdb_node *existing;

    while ((existing = bt_mesh_cdb_node_get(a)) != NULL) {
        if (memcmp(existing->uuid, uuid, 16) == 0) {
            LOG_WRN("Node already provisioned at 0x%04X", a);
            return;
        }
        a += existing->num_elem;
    }

    LOG_INF("ESP32 node detected: %02X%02X%02X%02X-%02X%02X...",
            uuid[0], uuid[1], uuid[2], uuid[3], uuid[4], uuid[5]);

    if (ble_nus_is_ready()) {
        char buf[80];
        snprintf(buf, sizeof(buf),
                 "{\"type\":\"beacon\",\"uuid\":\"%02x%02x%02x%02x%02x%02x%02x%02x\"}\n",
                 uuid[0], uuid[1], uuid[2], uuid[3],
                 uuid[4], uuid[5], uuid[6], uuid[7]);
        ble_nus_send(buf);
    }        

    init_next_node_addr();
    dump_cdb_nodes();
    if (next_node_addr == BT_MESH_ADDR_UNASSIGNED) {
        bt_mesh_cdb_node_foreach(evict_lost_node_cb, NULL);
        init_next_node_addr();
        if (next_node_addr == BT_MESH_ADDR_UNASSIGNED) {
            LOG_ERR("CDB still full after eviction");
            if (ble_nus_is_ready()) ble_nus_send("{\"type\":\"error\",\"msg\":\"CDB full\"}\n");
            prov_in_progress = false;
            return;
        }
    }
    uint16_t addr = next_node_addr;

    LOG_INF("Provisioning at address 0x%04X...", addr);
    prov_in_progress = true;

    int err = bt_mesh_provision_adv(uuid, NET_IDX, addr, 5);
    if (err) {
        LOG_ERR("bt_mesh_provision_adv failed: %d", err);
        prov_in_progress = false;
    }
}

void ble_mesh_handler_reconfigure_node(uint16_t mesh_addr)
{
    struct bt_mesh_cdb_node *node = bt_mesh_cdb_node_get(mesh_addr);
    if (!node) {
        LOG_WRN("reconfigure: node 0x%04X not in CDB", mesh_addr);
        return;
    }
    // CONFIGURED-Flag löschen → configure_node_handler läuft neu durch
    atomic_clear_bit(node->flags, BT_MESH_CDB_NODE_CONFIGURED);
    pending_node_addr = mesh_addr;
    node_retry_count  = 0;
    k_work_schedule_for_queue(&config_wq, &configure_node_work, K_MSEC(500));
    LOG_INF("Reconfigure queued for 0x%04X", mesh_addr);
}

static void node_added_cb(uint16_t net_idx, uint8_t uuid[16],
                           uint16_t addr, uint8_t num_elem)
{
    LOG_INF("Node provisioned: addr=0x%04X, elements=%u", addr, num_elem);

    prov_in_progress = false;
    next_node_addr = addr + num_elem;

    /* Notify dashboard: node provisioned, now configuring */
    if (ble_nus_is_ready()) {
        char buf[96];
        snprintf(buf, sizeof(buf),
                 "{\"type\":\"prov_event\",\"event\":\"provisioned\","
                 "\"addr\":%u,\"uuid\":\"%02x%02x%02x%02x\"}\n",
                 addr, uuid[0], uuid[1], uuid[2], uuid[3]);
        ble_nus_send(buf);
    }

    struct bt_mesh_cdb_node *node = bt_mesh_cdb_node_get(addr);
    if (node) {
        bt_mesh_cdb_node_store(node);
    }

    k_mutex_lock(&s_pending_mutex, K_FOREVER);
    /* Queue the address for configuration */
    if (pending_count < MAX_PENDING_NODES) {
        pending_addrs[pending_count++] = addr;
    }
    k_mutex_unlock(&s_pending_mutex);

    /* Only start config if this is the first in queue */
    if (pending_count == 1) {
        pending_node_addr = addr;
        node_retry_count = 0;
        k_work_schedule_for_queue(&config_wq, &configure_node_work,
                                  K_SECONDS(3));
    }
}
static void prov_link_close_cb(bt_mesh_prov_bearer_t bearer)
{
    if (prov_in_progress) {
        LOG_WRN("Provisioning link closed unexpectedly");
        prov_in_progress = false;
    }
}

static void prov_timeout_handler(struct k_work *work)
{
    LOG_INF("=== Provisioning window closed ===");
    prov_scanning = false;
    bt_mesh_prov_disable(BT_MESH_PROV_ADV);
    dk_set_led_off(DK_LED3);

    /* Resume radio scheduler */
    mesh_scheduler_start();
}

/* ── Self-Provisioning ───────────────────────────────────────── */

static void prov_complete(uint16_t net_idx, uint16_t addr)
{
    LOG_INF("Provisioning complete: addr=0x%04X", addr);
    configure_retry_count = 0;
    k_work_schedule_for_queue(&config_wq, &configure_work, K_NO_WAIT);
}


/* ── CDB Lifecycle Management ────────────────────────────────────
 *
 * Nodes accumulate in CDB across reflashes/reboots if not explicitly
 * removed. Two cleanup mechanisms:
 *
 * 1. cdb_purge_lost_nodes(): removes nodes that semantic_handler
 *    has marked LOST (no data for >90s). Called periodically.
 *
 * 2. cdb_full_reset(): complete mesh reset for factory/dev use.
 *    Triggered via NUS {"cmd":"reset_mesh"}.
 */

static uint8_t _purge_lost_cb(struct bt_mesh_cdb_node *node, void *user_data)
{
    if (node->addr == GATEWAY_ADDR) {
        return BT_MESH_CDB_ITER_CONTINUE;
    }
    uint8_t idx = data_handler_get_node_idx_by_mesh_addr(node->addr);
    if (idx == 0xFF) {
        /* Unknown to data_handler — stale from old session */
        LOG_WRN("CDB purge: 0x%04X unknown to data_handler — removing", node->addr);
        bt_mesh_cdb_node_del(node, true);
        return BT_MESH_CDB_ITER_CONTINUE;
    }
    if (semantic_handler_get_state(idx) == NODE_STATE_LOST) {
        int *count = (int *)user_data;
        (*count)++;
        LOG_WRN("CDB purge: 0x%04X LOST for >%d s — removing",
                node->addr, (int)(node_lost_timeout_ms / 1000));
        bt_mesh_cdb_node_del(node, true);
    }
    return BT_MESH_CDB_ITER_CONTINUE;
}

void ble_mesh_handler_purge_lost_nodes(void)
{
    int removed = 0;
    bt_mesh_cdb_node_foreach(_purge_lost_cb, &removed);
    if (removed > 0) {
        LOG_INF("CDB purge: removed %d LOST node(s)", removed);
        if (IS_ENABLED(CONFIG_SETTINGS)) settings_save();
    } else {
        LOG_INF("CDB purge: no stale nodes found");
    }
}

static uint8_t _delete_all_nodes_cb(struct bt_mesh_cdb_node *node, void *user_data)
{
    LOG_WRN("CDB reset: deleting node 0x%04X", node->addr);
    bt_mesh_cdb_node_del(node, true);
    return BT_MESH_CDB_ITER_CONTINUE;
}

void ble_mesh_handler_full_reset(void)
{
    LOG_WRN("=== FULL MESH RESET requested ===");

    /* Delete CDB nodes from RAM + NVS explicitly.
     * bt_mesh_cdb_node_del(node, true) calls settings_delete internally
     * for each node key (bt/mesh/cdb/Node/X). */
    bt_mesh_cdb_node_foreach(_delete_all_nodes_cb, NULL);

    /* Also explicitly delete known CDB settings keys via settings_delete.
     * This is belt-and-suspenders: even if the foreach missed something,
     * the NVS entries are gone before bt_mesh_reset() runs. */
    for (uint16_t addr = 1; addr <= 0x0020; addr++) {
        char key[32];
        snprintf(key, sizeof(key), "bt/mesh/cdb/Node/%x", addr);
        settings_delete(key);
    }

    /* bt_mesh_reset() clears all mesh credentials (Net, IV, Seq, RPL, AppKey)
     * and calls settings_delete for each. The CDB subnet is NOT cleared by
     * bt_mesh_reset — we already handled nodes above. */
    bt_mesh_reset();

    /* Clear CDB subnet key too so cdb_create works fresh on reboot */
    settings_delete("bt/mesh/cdb/Net");
    settings_delete("bt/mesh/cdb/Subnet/0");
    settings_delete("bt/mesh/cdb");

    LOG_WRN("Full mesh reset complete — rebooting now");
    k_msleep(200);
    NVIC_SystemReset();
}

/* Periodic purge work — runs every 5 minutes */
#define CDB_PURGE_INTERVAL_S 300

static void cdb_periodic_purge_fn(struct k_work *w);
static K_WORK_DELAYABLE_DEFINE(s_cdb_purge_work, cdb_periodic_purge_fn);

static void cdb_periodic_purge_fn(struct k_work *w)
{
    ble_mesh_handler_purge_lost_nodes();
    k_work_schedule_for_queue(&config_wq, &s_cdb_purge_work,
                              K_SECONDS(CDB_PURGE_INTERVAL_S));
}

static void self_provision(void)
{
    int err;
    uint8_t dev_uuid[16];

    err = hwinfo_get_device_id(dev_uuid, sizeof(dev_uuid));
    if (err < 0) {
        get_device_uuid(dev_uuid);
    }

    if (bt_mesh_is_provisioned()) {
        LOG_INF("Already provisioned");
        configure_retry_count = 0;
        k_work_schedule_for_queue(&config_wq, &configure_work, K_NO_WAIT);
        init_next_node_addr();
        /* Start periodic CDB cleanup (first run after 60s to let nodes re-register) */
        k_work_schedule_for_queue(&config_wq, &s_cdb_purge_work, K_SECONDS(60));
        return;
    }

    LOG_INF("Self-provisioning with UUID: "
            "%02X%02X%02X%02X-%02X%02X-%02X%02X-"
            "%02X%02X-%02X%02X%02X%02X%02X%02X",
            dev_uuid[0],  dev_uuid[1],  dev_uuid[2],  dev_uuid[3],
            dev_uuid[4],  dev_uuid[5],  dev_uuid[6],  dev_uuid[7],
            dev_uuid[8],  dev_uuid[9],  dev_uuid[10], dev_uuid[11],
            dev_uuid[12], dev_uuid[13], dev_uuid[14], dev_uuid[15]);

    err = bt_mesh_cdb_create(net_key);
    if (err && err != -EALREADY && err != -ENOMEM) {
        LOG_ERR("CDB create failed: %d", err);
        return;
    }
    if (err) {
        /* -EALREADY or -ENOMEM: CDB subnet slot occupied by stale settings.
         * Nodes and subnet keys were deleted above — recreate cleanly. */
        LOG_WRN("CDB occupied (err=%d) — resetting CDB state", err);
        bt_mesh_cdb_clear();
        err = bt_mesh_cdb_create(net_key);
        if (err) {
            LOG_ERR("CDB re-create failed: %d", err);
            return;
        }
    }

    /* Purge ALL existing CDB nodes before fresh provisioning.
     * On reboot after reset_mesh, old nodes (e.g. Node/1, Node/3) are
     * reloaded from NVS before our code runs. They must all be removed
     * so bt_mesh_provision() can allocate a fresh CDB entry for 0x0001.
     * Only deleting the gateway entry is not enough — other lingering
     * nodes consume CDB slots and cause -ENOMEM. */
    int purge_count = 0;
    struct bt_mesh_cdb_node *n;
    uint16_t a = 1;
    while ((n = bt_mesh_cdb_node_get(a)) != NULL) {
        LOG_WRN("Pre-prov purge: removing CDB node 0x%04X", n->addr);
        a = n->addr + n->num_elem;
        bt_mesh_cdb_node_del(n, true);
        purge_count++;
    }
    /* Belt-and-suspenders: explicitly delete known settings keys */
    for (uint16_t addr = 1; addr <= 0x20; addr++) {
        char key[32];
        snprintf(key, sizeof(key), "bt/mesh/cdb/Node/%x", addr);
        settings_delete(key);
    }
    /* Also delete CDB subnet/net keys — these survive bt_mesh_reset()
     * and cause bt_mesh_cdb_create() to fail with -ENOMEM. */
    settings_delete("bt/mesh/cdb/Net");
    settings_delete("bt/mesh/cdb/Subnet/0");

    if (purge_count > 0 || true) {
        LOG_INF("CDB pre-prov purge: %d nodes removed", purge_count);
        if (IS_ENABLED(CONFIG_SETTINGS)) settings_save();
    }

    err = bt_mesh_provision(net_key, NET_IDX, 0, 0,
                            GATEWAY_ADDR, dev_uuid);
    if (err) {
        LOG_ERR("Provisioning failed: %d", err);
        return;
    }

    LOG_INF("Provisioned at address 0x%04X", GATEWAY_ADDR);
    init_next_node_addr();
}

/* ── Health Server (required by spec) ────────────────────────── */

static void attn_on(const struct bt_mesh_model *model)
{
    dk_set_led(DK_LED3, 1);
}

static void attn_off(const struct bt_mesh_model *model)
{
    dk_set_led(DK_LED3, 0);
}

static const struct bt_mesh_health_srv_cb health_cb = {
    .attn_on  = attn_on,
    .attn_off = attn_off,
};

static struct bt_mesh_health_srv health_srv = {
    .cb = &health_cb,
};

BT_MESH_HEALTH_PUB_DEFINE(health_pub, 0x00);

/* ── Mesh Composition ────────────────────────────────────────── */

static struct bt_mesh_cfg_cli cfg_cli = {};

#if IS_ENABLED(CONFIG_BT_MESH_DF_CLI)
static struct bt_mesh_df_cli s_df_cli = {};
#endif

static struct bt_mesh_model root_models[] = {
    BT_MESH_MODEL_CFG_SRV,
    BT_MESH_MODEL_CFG_CLI(&cfg_cli),
    BT_MESH_MODEL_HEALTH_SRV(&health_srv, &health_pub),
    BT_MESH_MODEL(SENSOR_CLI_MODEL_ID, sensor_cli_ops, NULL, NULL),
    /* Generic OnOff Client — controls commercial BLE Mesh lamps */
    BT_MESH_MODEL(MESH_CTRL_ONOFF_CLI_ID, onoff_cli_ops,
                  &onoff_cli_pub, NULL),
#if IS_ENABLED(CONFIG_BT_MESH_DF_CLI)
    /* Directed Forwarding Client — configures DF paths on nodes (BT Mesh 1.1) */
    BT_MESH_MODEL_DF_CLI(&s_df_cli),
#endif
};

static struct bt_mesh_elem elements[] = {
    BT_MESH_ELEM(0, root_models, BT_MESH_MODEL_NONE),
};

static struct bt_mesh_comp comp = {
    .cid        = 0xFFFF,
    .elem       = elements,
    .elem_count = ARRAY_SIZE(elements),
};

/* ── Provisioning Definition ─────────────────────────────────── */

static uint8_t prov_uuid[16];

static const struct bt_mesh_prov prov = {
    .uuid                 = prov_uuid,
    .complete             = prov_complete,
    .unprovisioned_beacon = unprovisioned_beacon_cb,
    .node_added           = node_added_cb,
    .link_close           = prov_link_close_cb,
};

/* ── Public API ──────────────────────────────────────────────── */

const struct bt_mesh_prov *ble_mesh_handler_prov_init(void)
{
    int err = hwinfo_get_device_id(prov_uuid, sizeof(prov_uuid));

    if (err < 0) {
        memset(prov_uuid, 0xFF, sizeof(prov_uuid));
    }

    return &prov;
}
static struct k_work_delayable prov_start_work;

static void prov_start_handler(struct k_work *work)
{
    int err = bt_mesh_prov_enable(BT_MESH_PROV_ADV);
    if (err && err != -EALREADY) {
        LOG_ERR("Failed to enable provisioner: %d", err);
        prov_scanning = false;
        dk_set_led_off(DK_LED3);
        mesh_scheduler_start();
        return;
    }
    k_work_schedule_for_queue(&config_wq, &prov_timeout_work,
                              K_SECONDS(PROV_WINDOW_SECONDS));
}

static void unprovision_handler(struct k_work *work)
{
    uint16_t addr = unprov_target_addr;
    LOG_INF("Unprovisioning node 0x%04X...", addr);
    mesh_scheduler_pause();
    /* Tell the node to reset itself — it will restart as unprovisioned */
    int err = bt_mesh_cfg_cli_node_reset(NET_IDX, addr, NULL);
    if (err == 0) {
        LOG_INF("  Node reset command sent successfully");
    } else if (err == -ETIMEDOUT) {
        LOG_WRN("  Reset timed out for 0x%04X (node may be offline)", addr);
    } else if (err == -ENOENT) {
        LOG_INF("  No DevKey for 0x%04X — skipping reset (node will re-beacon)", addr);
    } else {
        LOG_ERR("  Failed to send reset: %d", err);
    }

    /* Remove from CDB regardless — even if the node is unreachable,
     * we want the gateway to forget it so it can be re-provisioned. */
    struct bt_mesh_cdb_node *node = bt_mesh_cdb_node_get(addr);
    if (node) {
        bt_mesh_cdb_node_del(node, true);
        LOG_INF("  Removed 0x%04X from CDB", addr);
    } else {
        LOG_WRN("  Node 0x%04X not found in CDB", addr);
    }
    
    k_sleep(K_MSEC(200));
    /* Recalculate next available address in case this freed a slot */
    init_next_node_addr();
    dump_cdb_nodes();
 
    if (IS_ENABLED(CONFIG_SETTINGS)) settings_save();
 
    LOG_INF("=== Node 0x%04X unprovisioned ===", addr);
    s_unprov_pending = false;
    mesh_scheduler_start();
}
 

const struct bt_mesh_comp *ble_mesh_handler_comp_init(void)
{
    k_work_queue_init(&config_wq);
    k_work_queue_start(&config_wq, config_wq_stack,
                       K_THREAD_STACK_SIZEOF(config_wq_stack),
                       K_PRIO_PREEMPT(1), NULL);

    k_work_init_delayable(&configure_work, configure_handler);
    k_work_init_delayable(&configure_node_work, configure_node_handler);
    k_work_init_delayable(&prov_timeout_work, prov_timeout_handler);
    k_work_init_delayable(&prov_start_work, prov_start_handler);
    k_work_init_delayable(&unprovision_work, unprovision_handler);
    mesh_ctrl_init(&root_models[4]);

    return &comp;
}

void ble_mesh_handler_self_provision(void)
{
    self_provision();
}


void ble_mesh_handler_start_provisioning(void)
{
    if (prov_scanning) {
        LOG_INF("Provisioning window already open");
        return;
    }
    if (!bt_mesh_is_provisioned()) {
        LOG_WRN("Gateway not provisioned yet");
        return;
    }

    mesh_scheduler_pause();
    prov_scanning = true;
    dk_set_led_on(DK_LED3);

    LOG_INF("=== Provisioning window open (%d s) ===", PROV_WINDOW_SECONDS);
    LOG_INF("Looking for ESP32 nodes (UUID prefix %02X %02X)...",
            ESP32_UUID_PREFIX_0, ESP32_UUID_PREFIX_1);

    /* Delay prov_enable 500ms to let BLE stack settle */
    k_work_schedule_for_queue(&config_wq, &prov_start_work, K_MSEC(500));
}


 

void ble_mesh_handler_unprovision_node(uint16_t mesh_addr)
{
    if (s_unprov_pending) {
        LOG_WRN("Unprovision already in progress, ignoring 0x%04X", mesh_addr);
        return;
    }
    s_unprov_pending = true;
    unprov_target_addr = mesh_addr;
    k_work_schedule_for_queue(&config_wq, &unprovision_work, K_MSEC(100));
}