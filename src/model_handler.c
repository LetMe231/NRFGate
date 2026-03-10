#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/bluetooth/mesh.h>
#include <zephyr/drivers/hwinfo.h>
#include <zephyr/settings/settings.h>
#include <zephyr/bluetooth/bluetooth.h>
#include <dk_buttons_and_leds.h>
#include "model_handler.h"

LOG_MODULE_REGISTER(model_handler, LOG_LEVEL_DBG);

// ============================================================================
// Sensor Property IDs
// ============================================================================

#define PROP_TEMPERATURE    0x004F
#define PROP_HUMIDITY       0x0076
#define PROP_ECO2           0x0008
#define PROP_HEART_RATE     0x0100
#define PROP_SPO2           0x0101
#define PROP_TVOC           0x0102
#define PROP_RAW_RED        0x0103
#define PROP_RAW_IR         0x0104

// ============================================================================
// Network Credentials
// ============================================================================
static const uint8_t net_key[16] = {
    0xF3, 0x43, 0xBB, 0xCD, 0x11, 0x48, 0x9F, 0x37,
    0x21, 0xF3, 0x23, 0xAC, 0xD0, 0x72, 0x9E, 0xBA,
};

static const uint8_t app_key[16] = {
    0xFC, 0x00, 0x10, 0x2B, 0xC3, 0xBD, 0x0E, 0x62,
    0x19, 0xCC, 0xB1, 0x90, 0xB1, 0x0A, 0x88, 0xA9,
};

#define NET_IDX         0x0000
#define APP_IDX         0x0000
#define GROUP_ADDR      0xC000
#define GATEWAY_ADDR    0x0001

// ============================================================================
// Auto-Provisioning Configuration
// ============================================================================

/** UUID prefix that ESP32 sensor nodes advertise (from dev_uuid[] in ESP32 main.c) */
#define ESP32_UUID_PREFIX_0  0x32
#define ESP32_UUID_PREFIX_1  0x10

/** How long the provisioning window stays open after pressing Button 2 */
#define PROV_WINDOW_SECONDS  30

/** First unicast address to assign to sensor nodes */
#define FIRST_NODE_ADDR      0x0002

// ============================================================================
// Sensor Status Opcode
// ============================================================================

#define BT_MESH_MODEL_OP_SENSOR_STATUS BT_MESH_MODEL_OP_1(0x52)

// ============================================================================
// RAW Sensor Status Parser
// ============================================================================

/**
*   @brief Parse a Format A MPID (16-bit header). 
*
 * Format A layout:
 *  - Bit 0     = format (0 for Format A)
 *  - Bits 1-4  = length, zero based (0 = 1 byte, 15 = 16 bytes)
 *  - Bits 5-15 = Property ID (11 bits, max value 0x7FF)
*/
static bool parse_mpid_format_a(uint16_t mpid, uint16_t *prop_id, uint8_t *data_len)
{
    if (mpid & 0x01) {
        return false; // Not Format A
    }
    *data_len = ((mpid >> 1) & 0x0F) + 1; // Length is zero-based
    *prop_id = (mpid >> 5) & 0x7FF; // Extract Property ID
    return true;
}
/**
 *  @brief read a little-endian signed integer from a byte array.
 * 
 *  Used for lengths of 1, 2, or 4 bytes.
 */
static int32_t read_le_signed(const uint8_t *data, uint8_t len)
{
    switch (len) {
        case 1:
            return (int8_t)data[0];
        case 2:
            return (int16_t)((uint16_t)data[0] 
                           | ((uint16_t)data[1] << 8));
        case 4:
            return (int32_t)((uint32_t)data[0] 
                           | ((uint32_t)data[1] << 8) 
                           | ((uint32_t)data[2] << 16) 
                           | ((uint32_t)data[3] << 24));
        default:
            return 0; // Unsupported length
    }
}

/**
 * @brief Read a little-endian unsigned integer from a byte array.
 */
static uint32_t read_le_unsigned(const uint8_t *data, uint8_t len)
{
    switch (len) {
        case 1:
            return data[0];
        case 2:
            return (uint16_t)data[0] 
                 | ((uint16_t)data[1] << 8);
        case 4:
            return (uint32_t)data[0] 
                 | ((uint32_t)data[1] << 8) 
                 | ((uint32_t)data[2] << 16) 
                 | ((uint32_t)data[3] << 24);
        default:
            return 0; // Unsupported length
    }
}

/**
 * @brief Walk through a Sensor Status payload and log each property.
 */
static void process_sensor_status(struct bt_mesh_msg_ctx *ctx, const uint8_t *data, uint16_t len)
{
    uint16_t offset = 0;
    while (offset + 2 <= len) { // Need at least 2 bytes for MPID
        uint16_t mpid = data[offset] | (data[offset + 1] << 8);
        uint16_t prop_id;
        uint8_t data_len;
        if (!parse_mpid_format_a(mpid, &prop_id, &data_len)) {
            LOG_WRN("Format B MPID at offset %u, skipping", offset);
            break;
        }

        offset += 2; // Move past MPID

        if (offset + data_len > len) {
            LOG_WRN("Property 0x%04X truncated (expected %u bytes, got %u)", prop_id, data_len, len - offset);
            break;
        }

        const uint8_t *val = &data[offset];
        offset += data_len; // Move past value

        // Log the property based on its ID
        switch (prop_id) {
            case PROP_TEMPERATURE: {
                int32_t temp = read_le_signed(val, data_len);
                LOG_INF("Temperature: %d.%02d C", temp / 100, temp % 100);
                break;
            }
            case PROP_HUMIDITY: {
                uint32_t hum = read_le_unsigned(val, data_len);
                LOG_INF("Humidity: %u.%02u %%", hum / 100, hum % 100);
                break;
            }
            case PROP_ECO2: {
                uint32_t eco2 = read_le_unsigned(val, data_len);
                LOG_INF("eCO2: %u ppm", eco2);
                break;
            }
            case PROP_TVOC: {
                uint32_t tvoc = read_le_unsigned(val, data_len);
                LOG_INF("TVOC: %u ppb", tvoc);
                break;
            }
            case PROP_HEART_RATE: {
                uint32_t hr = read_le_unsigned(val, data_len);
                LOG_INF("Heart Rate: %u bpm", hr);
                break;
            }
            case PROP_SPO2: {
                uint32_t spo2 = read_le_unsigned(val, data_len);
                LOG_INF("SpO2: %u.%02u %%", spo2 / 100, spo2 % 100);
                break;
            }
            case PROP_RAW_RED:
            {
                uint32_t raw = read_le_unsigned(val, data_len);
                LOG_INF("Raw red: %u", raw);
                break;
            }
            case PROP_RAW_IR: {
                uint32_t raw = read_le_unsigned(val, data_len);
                LOG_INF("Raw IR: %u", raw);
                break;
            }
            default:
                LOG_WRN("Unknown Property ID 0x%04X with length %u bytes", prop_id, data_len);
        }
    }
}

// ============================================================================
// Custom Sensor Client Model (raw opcode handler)
// ============================================================================
static int sensor_status_handler(const struct bt_mesh_model *model,
                                  struct bt_mesh_msg_ctx *ctx,
                                  struct net_buf_simple *buf)
{
    LOG_DBG("Received Sensor Status message from 0x%04X with %u bytes of data", ctx->addr, buf->len);
    process_sensor_status(ctx, buf->data, buf->len);
    return 0;
}

static const struct bt_mesh_model_op sensor_cli_ops[] = {
    { BT_MESH_MODEL_OP_SENSOR_STATUS, 0, sensor_status_handler },
    BT_MESH_MODEL_OP_END,
};

// ============================================================================
// Workqueue & Deferred Work Items
// ============================================================================

#define CONFIG_WQ_STACK_SIZE 4096
#define CONFIG_RETRY_BASE_MS 500
#define CONFIG_RETRY_MAX_SHIFT 5

static K_THREAD_STACK_DEFINE(config_wq_stack, CONFIG_WQ_STACK_SIZE);
static struct k_work_q config_wq;

/* Gateway self-configuration */
static struct k_work_delayable configure_work;
static int configure_retry_count = 0;

/* New node configuration */
static struct k_work_delayable configure_node_work;
static int node_retry_count = 0;

/* Provisioning window timeout */
static struct k_work_delayable prov_timeout_work;

#define SENSOR_CLI_MODEL_ID 0x1102

// ============================================================================
// Auto-Provisioning State
// ============================================================================

/** true while the provisioning window is open */
static bool prov_scanning;

/** true while actively provisioning a node */
static bool prov_in_progress;

/** Next unicast address to assign */
static uint16_t next_node_addr = FIRST_NODE_ADDR;

/** Address of the most recently provisioned node (for configuration) */
static uint16_t pending_node_addr;

// ============================================================================
// Gateway Self-Configuration (exponential backoff)
// ============================================================================

static void schedule_gateway_retry(void)
{
    int shift = MIN(configure_retry_count, CONFIG_RETRY_MAX_SHIFT);
    uint32_t delay_ms = CONFIG_RETRY_BASE_MS << shift;
    LOG_WRN("Gateway config retry #%d in %u ms",
            configure_retry_count + 1, delay_ms);
    configure_retry_count++;
    k_work_schedule_for_queue(&config_wq, &configure_work, K_MSEC(delay_ms));
}

static void configure_handler(struct k_work *work)
{
    int err;
    uint8_t status;

    LOG_INF("Configuring Gateway (address 0x%04X)", GATEWAY_ADDR);

    // Add AppKey
    err = bt_mesh_app_key_add(APP_IDX, NET_IDX, app_key);
    if (err && err != -EALREADY) {
        LOG_ERR("Failed to add AppKey: %d", err);
        schedule_gateway_retry();
        return;
    }

    // Add AppKey to the network
    err = bt_mesh_cfg_cli_mod_app_bind(NET_IDX, GATEWAY_ADDR, 
                                       GATEWAY_ADDR, APP_IDX, 
                                       SENSOR_CLI_MODEL_ID, &status);
    if (err) {
        LOG_ERR("Failed to bind AppKey: %d", err);
        schedule_gateway_retry();
        return;
    }

    // Subscribe to group address
    err = bt_mesh_cfg_cli_mod_sub_add(NET_IDX, GATEWAY_ADDR, GATEWAY_ADDR, GROUP_ADDR, SENSOR_CLI_MODEL_ID, &status);
    if (err || status) {
        LOG_ERR("Failed to subscribe: err=%d status=0x%02X", err, status);
        schedule_gateway_retry();
        return;
    }

    LOG_INF("Sensor Client bound to AppKey 0x%03X", APP_IDX);
    LOG_INF("Subscribed to group address 0x%04X", GROUP_ADDR);

    if (IS_ENABLED(CONFIG_SETTINGS)) {
        settings_save();
        LOG_INF("Configuration saved to flash");
    }
}

// ============================================================================
// New Node Configuration (AppKey + Bind + Publication)
// ============================================================================

static void schedule_node_retry(void)
{
    int shift = MIN(node_retry_count, CONFIG_RETRY_MAX_SHIFT);
    uint32_t delay_ms = CONFIG_RETRY_BASE_MS << shift;
    LOG_WRN("Node config retry #%d in %u ms", node_retry_count + 1, delay_ms);
    node_retry_count++;
    k_work_schedule_for_queue(&config_wq, &configure_node_work, K_MSEC(delay_ms));
}

/**
 * @brief Configure a freshly provisioned sensor node.
 *
 * Sends Config Client messages to the new node:
 *   1. AppKey Add
 *   2. Bind AppKey to Sensor Server (0x1100)
 *   3. Set publication address to GROUP_ADDR (0xC000)
 */
static void configure_node_handler(struct k_work *work)
{
    int err;
    uint8_t status;
    uint16_t addr = pending_node_addr;

    LOG_INF("Configuring sensor node 0x%04x...", addr);

    /* 1. Add AppKey to the new node */
    err = bt_mesh_cfg_cli_app_key_add(NET_IDX, addr,
                                      NET_IDX, APP_IDX,
                                      app_key, &status);
    if (err || status) {
        LOG_ERR("  AppKey add failed: err=%d status=0x%02x", err, status);
        schedule_node_retry();
        return;
    }
    LOG_INF("  AppKey added to node 0x%04x", addr);

    /* 2. Bind AppKey to Sensor Server model (0x1100) */
    err = bt_mesh_cfg_cli_mod_app_bind(NET_IDX, addr,
                                       addr, APP_IDX,
                                       BT_MESH_MODEL_ID_SENSOR_SRV, &status);
    if (err || status) {
        LOG_ERR("  Bind failed: err=%d status=0x%02x", err, status);
        schedule_node_retry();
        return;
    }
    LOG_INF("  AppKey bound to Sensor Server (0x1100)");

    /* 3. Set publication to GROUP_ADDR */
    struct bt_mesh_cfg_cli_mod_pub pub = {
        .addr      = GROUP_ADDR,
        .app_idx   = APP_IDX,
        .ttl       = 7,
        .period    = 0,       /* ESP32 uses its own 2 s timer */
        .transmit  = 0,
        .cred_flag = false,
    };
    err = bt_mesh_cfg_cli_mod_pub_set(NET_IDX, addr,
                                      addr, BT_MESH_MODEL_ID_SENSOR_SRV,
                                      &pub, &status);
    if (err || status) {
        LOG_ERR("  Pub set failed: err=%d status=0x%02x", err, status);
        schedule_node_retry();
        return;
    }
    LOG_INF("  Publication set to 0x%04x", GROUP_ADDR);

    LOG_INF("=== Node 0x%04x fully configured ===", addr);

    if (IS_ENABLED(CONFIG_SETTINGS)) {
        settings_save();
    }
}

// ============================================================================
// Auto-Provisioning Callbacks
// ============================================================================

/**
 * @brief Called when an unprovisioned device beacon is received.
 *
 * Filters by UUID prefix, allocates a CDB node, and starts provisioning.
 */
static void unprovisioned_beacon_cb(uint8_t uuid[16],
                                     bt_mesh_prov_oob_info_t oob_info,
                                     uint32_t *uri_hash)
{
    if (!prov_scanning || prov_in_progress) {
        return;
    }

    /* UUID prefix filter — only accept ESP32 sensor nodes */
    if (uuid[0] != ESP32_UUID_PREFIX_0 || uuid[1] != ESP32_UUID_PREFIX_1) {
        LOG_DBG("Ignoring non-ESP32 beacon: %02x%02x...", uuid[0], uuid[1]);
        return;
    }

    LOG_INF("ESP32 node detected: %02x%02x%02x%02x-%02x%02x...",
            uuid[0], uuid[1], uuid[2], uuid[3], uuid[4], uuid[5]);

    /* Allocate CDB node entry */
    uint16_t addr = next_node_addr;
    LOG_INF("Provisioning node at address 0x%04x...", addr);
    prov_in_progress = true;

    int err = bt_mesh_provision_adv(uuid, NET_IDX, addr, 5);
    if (err) {
        LOG_ERR("bt_mesh_provision_adv failed: %d", err);
        prov_in_progress = false;
    }
}

/**
 * @brief Called when a node has been successfully added to the network.
 */
static void node_added_cb(uint16_t net_idx, uint8_t uuid[16],
                           uint16_t addr, uint8_t num_elem)
{
    LOG_INF("Node provisioned: addr=0x%04x, elements=%u", addr, num_elem);

    prov_in_progress = false;
    next_node_addr = addr + num_elem;

    /* Persist CDB entry */
    struct bt_mesh_cdb_node *node = bt_mesh_cdb_node_get(addr);
    if (node) {
        bt_mesh_cdb_node_store(node);
    }

    /* Schedule node configuration */
    pending_node_addr = addr;
    node_retry_count = 0;
    k_work_schedule_for_queue(&config_wq, &configure_node_work, K_SECONDS(2));
}

/**
 * @brief Called when a provisioning link closes.
 */
static void prov_link_close_cb(bt_mesh_prov_bearer_t bearer)
{
    if (prov_in_progress) {
        LOG_WRN("Provisioning link closed unexpectedly");
        prov_in_progress = false;
    }
}

/**
 * @brief Provisioning window timeout — close after PROV_WINDOW_SECONDS.
 */
static void prov_timeout_handler(struct k_work *work)
{
    LOG_INF("=== Provisioning window closed ===");
    prov_scanning = false;
    bt_mesh_prov_disable(BT_MESH_PROV_ADV);
    dk_set_led_off(DK_LED2);
}

// ============================================================================
// Self-Provisioning (gateway provisions itself on first boot)
// ============================================================================

static void prov_complete(uint16_t net_idx, uint16_t addr)
{
    LOG_INF("Provisioning complete for node 0x%04X on net_idx 0x%04X", addr, net_idx);
    configure_retry_count = 0;
    k_work_schedule_for_queue(&config_wq, &configure_work, K_NO_WAIT);
}

// ============================================================================
// Self-Provisioning
// ============================================================================

static void self_provision(void)
{
    int err;
    uint8_t dev_uuid[16];

    err = hwinfo_get_device_id(dev_uuid, sizeof(dev_uuid));
    if (err) {
        LOG_ERR("Failed to get device ID: %d, using fallback UUID", err);
        memset(dev_uuid, 0xFF, sizeof(dev_uuid));
    }

    // Already provisioned - re-apply configuration
    if (bt_mesh_is_provisioned()) {
        LOG_INF("Device is already provisioned");
        configure_retry_count = 0;
        k_work_schedule_for_queue(&config_wq, &configure_work, K_NO_WAIT);
        return;
    }

    // Not provisioned - start provisioning with static OOB using device UUID as the static OOB value
    LOG_INF("Provisioning device with UUID: %02X%02X%02X%02X-%02X%02X-%02X%02X-%02X%02X-%02X%02X%02X%02X%02X%02X",
            dev_uuid[0], dev_uuid[1], dev_uuid[2], dev_uuid[3], 
            dev_uuid[4], dev_uuid[5], dev_uuid[6], dev_uuid[7],
            dev_uuid[8], dev_uuid[9], dev_uuid[10], dev_uuid[11], 
            dev_uuid[12], dev_uuid[13], dev_uuid[14], dev_uuid[15]);

    err = bt_mesh_cdb_create(net_key);
    if (err && err != -EALREADY) {
        LOG_ERR("Failed to create CDB: %d", err);
        return;
    }

    err = bt_mesh_provision(net_key, NET_IDX, 
                          0, // flags
                          0, // iv_index
                          GATEWAY_ADDR, 
                          dev_uuid);

    if (err) {
        LOG_ERR("Provisioning failed: %d", err);
        return;
    }
    LOG_INF("Provisioned successfully with address 0x%04X", GATEWAY_ADDR);
}

// ============================================================================
// Health Server Callbacks (Not used at the moment, but defined for completeness)
// ============================================================================

static void attn_on(const struct bt_mesh_model *model)
{
    LOG_INF("Health Server Attention ON");
    dk_set_led(DK_LED3, 1);
}

static void attn_off(const struct bt_mesh_model *model)
{
    LOG_INF("Health Server Attention OFF");
    dk_set_led(DK_LED3, 0);
}

static const struct bt_mesh_health_srv_cb health_cb = {
    .attn_on = attn_on,
    .attn_off = attn_off,
};

static struct bt_mesh_health_srv health_srv = {
    .cb = &health_cb,
};

BT_MESH_HEALTH_PUB_DEFINE(health_pub, 0x00);

// ============================================================================
// Mesh Models & Composition
// ============================================================================

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
    .cid = 0xFFFF, // Provisionally set to TEST CID
    .elem = elements,
    .elem_count = ARRAY_SIZE(elements),
};

// ============================================================================
// Provisioning Definition
// ============================================================================

static uint8_t prov_uuid[16];

static const struct bt_mesh_prov prov = {
    .uuid                 = prov_uuid,
    .complete             = prov_complete,
    .unprovisioned_beacon = unprovisioned_beacon_cb,
    .node_added           = node_added_cb,
    .link_close           = prov_link_close_cb,
};

// ============================================================================
// Public API
// ============================================================================

const struct bt_mesh_prov *model_handler_prov_init(void)
{
    int err = hwinfo_get_device_id(prov_uuid, sizeof(prov_uuid));
    if (err < 0) {
        LOG_ERR("Failed to get device ID for provisioning UUID: %d, using fallback UUID", err);
        memset(prov_uuid, 0xFF, sizeof(prov_uuid));
    }
    return &prov;
}

const struct bt_mesh_comp *model_handler_comp_init(void)
{
    k_work_queue_init(&config_wq);
    k_work_queue_start(&config_wq, config_wq_stack, K_THREAD_STACK_SIZEOF(config_wq_stack), K_PRIO_PREEMPT(1), NULL);
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
        return;
    }

    k_work_schedule_for_queue(&config_wq, &prov_timeout_work,
                              K_SECONDS(PROV_WINDOW_SECONDS));
}