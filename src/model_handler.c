/**
 * @file model_handler.c
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

#include "model_handler.h"

LOG_MODULE_REGISTER(model_handler, LOG_LEVEL_INF);

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

/* ── Sensor Status Opcode ────────────────────────────────────── */

#define BT_MESH_MODEL_OP_SENSOR_STATUS BT_MESH_MODEL_OP_1(0x52)

/* ── MPID Parser (Format A) ─────────────────────────────────── */

/**
 * Parse a Format A Marshalled Property ID (16-bit header).
 *
 * Layout:
 *   Bit 0      = format (0 = Format A)
 *   Bits 1–4   = length (zero-based, 0 = 1 byte)
 *   Bits 5–15  = Property ID (11 bits)
 */
static bool parse_mpid_format_a(uint16_t mpid, uint16_t *prop_id,
                                uint8_t *data_len)
{
    if (mpid & 0x01) {
        return false;
    }
    *data_len = ((mpid >> 1) & 0x0F) + 1;
    *prop_id  = (mpid >> 5) & 0x7FF;
    return true;
}

/* ── Little-Endian Readers ───────────────────────────────────── */

static int32_t read_le_signed(const uint8_t *data, uint8_t len)
{
    switch (len) {
    case 1: return (int8_t)data[0];
    case 2: return (int16_t)(data[0] | (data[1] << 8));
    case 4: return (int32_t)(data[0] | (data[1] << 8) |
                             (data[2] << 16) | (data[3] << 24));
    default: return 0;
    }
}

static uint32_t read_le_unsigned(const uint8_t *data, uint8_t len)
{
    switch (len) {
    case 1: return data[0];
    case 2: return data[0] | (data[1] << 8);
    case 4: return data[0] | (data[1] << 8) |
                   (data[2] << 16) | (data[3] << 24);
    default: return 0;
    }
}

/* ── Sensor Status Payload Parser ────────────────────────────── */

static void process_sensor_status(const uint8_t *data, uint16_t len)
{
    uint16_t offset = 0;

    while (offset + 2 <= len) {
        uint16_t mpid = data[offset] | (data[offset + 1] << 8);
        uint16_t prop_id;
        uint8_t data_len;

        if (!parse_mpid_format_a(mpid, &prop_id, &data_len)) {
            break;
        }

        offset += 2;

        if (offset + data_len > len) {
            LOG_WRN("Property 0x%04X truncated", prop_id);
            break;
        }

        const uint8_t *val = &data[offset];
        offset += data_len;

        switch (prop_id) {
        case PROP_TEMPERATURE: {
            int32_t t = read_le_signed(val, data_len);
            LOG_INF("Temperature: %d.%02d C", t / 100, t % 100);
            break;
        }
        case PROP_HUMIDITY: {
            uint32_t h = read_le_unsigned(val, data_len);
            LOG_INF("Humidity: %u.%02u %%", h / 100, h % 100);
            break;
        }
        case PROP_ECO2:
            LOG_INF("eCO2: %u ppm", read_le_unsigned(val, data_len));
            break;
        case PROP_TVOC:
            LOG_INF("TVOC: %u ppb", read_le_unsigned(val, data_len));
            break;
        case PROP_HEART_RATE:
            LOG_INF("Heart Rate: %u bpm", read_le_unsigned(val, data_len));
            break;
        case PROP_SPO2: {
            uint32_t s = read_le_unsigned(val, data_len);
            LOG_INF("SpO2: %u.%02u %%", s / 100, s % 100);
            break;
        }
        case PROP_RAW_RED:
            LOG_INF("Raw red: %u", read_le_unsigned(val, data_len));
            break;
        case PROP_RAW_IR:
            LOG_INF("Raw IR: %u", read_le_unsigned(val, data_len));
            break;
        default:
            LOG_WRN("Unknown Property 0x%04X (%u bytes)", prop_id, data_len);
        }
    }
}

/* ── Sensor Client Opcode Handler ────────────────────────────── */

static int sensor_status_handler(const struct bt_mesh_model *model,
                                 struct bt_mesh_msg_ctx *ctx,
                                 struct net_buf_simple *buf)
{
    LOG_INF("Sensor Status from 0x%04X (%u bytes)", ctx->addr, buf->len);
    process_sensor_status(buf->data, buf->len);
    return 0;
}

static const struct bt_mesh_model_op sensor_cli_ops[] = {
    {BT_MESH_MODEL_OP_SENSOR_STATUS, 0, sensor_status_handler},
    BT_MESH_MODEL_OP_END,
};

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
        LOG_ERR("AppKey bind failed: %d", err);
        schedule_gateway_retry();
        return;
    }

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
    node_retry_count++;
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

    /* 1. Add AppKey */
    err = bt_mesh_cfg_cli_app_key_add(NET_IDX, addr,
                                      NET_IDX, APP_IDX,
                                      app_key, &status);
    if (err || status) {
        LOG_ERR("  AppKey add failed: err=%d status=0x%02X", err, status);
        schedule_node_retry();
        return;
    }
    LOG_INF("  AppKey added to 0x%04X", addr);

    /* 2. Bind to Sensor Server (0x1100) */
    err = bt_mesh_cfg_cli_mod_app_bind(NET_IDX, addr, addr, APP_IDX,
                                       BT_MESH_MODEL_ID_SENSOR_SRV,
                                       &status);
    if (err || status) {
        LOG_ERR("  Bind failed: err=%d status=0x%02X", err, status);
        schedule_node_retry();
        return;
    }
    LOG_INF("  Bound to Sensor Server");

    /* 3. Set publication to group address */
    struct bt_mesh_cfg_cli_mod_pub pub = {
        .addr      = GROUP_ADDR,
        .app_idx   = APP_IDX,
        .ttl       = 7,
        .period    = 0,
        .transmit  = 0,
        .cred_flag = false,
    };

    err = bt_mesh_cfg_cli_mod_pub_set(NET_IDX, addr, addr,
                                      BT_MESH_MODEL_ID_SENSOR_SRV,
                                      &pub, &status);
    if (err || status) {
        LOG_ERR("  Pub set failed: err=%d status=0x%02X", err, status);
        schedule_node_retry();
        return;
    }
    LOG_INF("  Publication set to 0x%04X", GROUP_ADDR);

   LOG_INF("=== Node 0x%04X fully configured ===", addr);

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
    next_node_addr = FIRST_NODE_ADDR;

    struct bt_mesh_cdb_node *node;

    while ((node = bt_mesh_cdb_node_get(next_node_addr)) != NULL) {
        next_node_addr += node->num_elem;
    }

    LOG_INF("Next available node address: 0x%04X", next_node_addr);
}

/* ── Provisioning Callbacks ──────────────────────────────────── */

static void unprovisioned_beacon_cb(uint8_t uuid[16],
                                    bt_mesh_prov_oob_info_t oob_info,
                                    uint32_t *uri_hash)
{
    if (!prov_scanning || prov_in_progress) {
        return;
    }

    /* UUID prefix filter */
    if (uuid[0] != ESP32_UUID_PREFIX_0 ||
        uuid[1] != ESP32_UUID_PREFIX_1) {
        return;
    }

    /* Skip already-provisioned UUIDs */
    uint16_t a = FIRST_NODE_ADDR;
    struct bt_mesh_cdb_node *existing;

    while ((existing = bt_mesh_cdb_node_get(a)) != NULL) {
        if (memcmp(existing->uuid, uuid, 16) == 0) {
            return;
        }
        a += existing->num_elem;
    }

    LOG_INF("ESP32 node detected: %02X%02X%02X%02X-%02X%02X...",
            uuid[0], uuid[1], uuid[2], uuid[3], uuid[4], uuid[5]);

    uint16_t addr = next_node_addr;

    LOG_INF("Provisioning at address 0x%04X...", addr);
    prov_in_progress = true;

    int err = bt_mesh_provision_adv(uuid, NET_IDX, addr, 5);
    if (err) {
        LOG_ERR("bt_mesh_provision_adv failed: %d", err);
        prov_in_progress = false;
    }
}



static void node_added_cb(uint16_t net_idx, uint8_t uuid[16],
                           uint16_t addr, uint8_t num_elem)
{
    LOG_INF("Node provisioned: addr=0x%04X, elements=%u", addr, num_elem);

    prov_in_progress = false;
    next_node_addr = addr + num_elem;

    struct bt_mesh_cdb_node *node = bt_mesh_cdb_node_get(addr);
    if (node) {
        bt_mesh_cdb_node_store(node);
    }

    /* Queue the address for configuration */
    if (pending_count < MAX_PENDING_NODES) {
        pending_addrs[pending_count++] = addr;
    }

    /* Only start config if this is the first in queue */
    if (pending_count == 1) {
        pending_node_addr = addr;
        node_retry_count = 0;
        k_work_schedule_for_queue(&config_wq, &configure_node_work,
                                  K_SECONDS(2));
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
    dk_set_led_off(DK_LED2);

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
    if (err && err != -EALREADY) {
        LOG_ERR("CDB create failed: %d", err);
        return;
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

static struct bt_mesh_model root_models[] = {
    BT_MESH_MODEL_CFG_SRV,
    BT_MESH_MODEL_CFG_CLI(&cfg_cli),
    BT_MESH_MODEL_HEALTH_SRV(&health_srv, &health_pub),
    BT_MESH_MODEL(SENSOR_CLI_MODEL_ID, sensor_cli_ops, NULL, NULL),
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

const struct bt_mesh_prov *model_handler_prov_init(void)
{
    int err = hwinfo_get_device_id(prov_uuid, sizeof(prov_uuid));

    if (err < 0) {
        memset(prov_uuid, 0xFF, sizeof(prov_uuid));
    }

    return &prov;
}

const struct bt_mesh_comp *model_handler_comp_init(void)
{
    k_work_queue_init(&config_wq);
    k_work_queue_start(&config_wq, config_wq_stack,
                       K_THREAD_STACK_SIZEOF(config_wq_stack),
                       K_PRIO_PREEMPT(1), NULL);

    k_work_init_delayable(&configure_work, configure_handler);
    k_work_init_delayable(&configure_node_work, configure_node_handler);
    k_work_init_delayable(&prov_timeout_work, prov_timeout_handler);

    return &comp;
}

void model_handler_self_provision(void)
{
    self_provision();
}

void model_handler_start_provisioning(void)
{
    if (prov_scanning) {
        LOG_INF("Provisioning window already open");
        return;
    }

    if (!bt_mesh_is_provisioned()) {
        LOG_WRN("Gateway not provisioned yet");
        return;
    }

    /* Pause scheduler — mesh needs uninterrupted radio for provisioning */
    mesh_scheduler_pause();

    LOG_INF("=== Provisioning window open (%d s) ===", PROV_WINDOW_SECONDS);
    LOG_INF("Looking for ESP32 nodes (UUID prefix %02X %02X)...",
            ESP32_UUID_PREFIX_0, ESP32_UUID_PREFIX_1);

    prov_scanning = true;
    dk_set_led_on(DK_LED2);

    int err = bt_mesh_prov_enable(BT_MESH_PROV_ADV);

    if (err && err != -EALREADY) {
        LOG_ERR("Failed to enable provisioner: %d", err);
        prov_scanning = false;
        dk_set_led_off(DK_LED2);
        mesh_scheduler_start();
        return;
    }

    k_work_schedule_for_queue(&config_wq, &prov_timeout_work,
                              K_SECONDS(PROV_WINDOW_SECONDS));
}