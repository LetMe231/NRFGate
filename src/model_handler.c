#include <zephyr/kernel.h>
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/mesh.h>
#include <bluetooth/mesh/models.h>
#include <zephyr/drivers/hwinfo.h>
#include <zephyr/settings/settings.h>
#include <dk_buttons_and_leds.h>
#include "model_handler.h"

/* Logging */
#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(model_handler, LOG_LEVEL_INF);

// ============================================================================
// Sensor Property IDs
// Must match the IDs used in the ESP32 nodes for the Sensor Server model
// ============================================================================

#define PROP_TEMPERATURE  0x004F
#define PROP_HUMIDITY     0x0076
#define PROP_ECO2         0x0008
#define PROP_HEART_RATE   0x0100
#define PROP_SPO2         0x0101
#define PROP_TVOC         0x0102
#define PROP_RAW_RED      0x0103
#define PROP_RAW_IR       0x0104

// ====================================================================================
// net_key and app_key must match the keys used in the ESP32 nodes for provisioning
// ====================================================================================

/* NetKey */
static const uint8_t net_key[16] = {
    0xF3, 0x43, 0xBB, 0xCD, 0x11, 0x48, 0x9F, 0x37,
    0x21, 0xF3, 0x23, 0xAC, 0xD0, 0x72, 0x9E, 0xBA,
};

/* AppKey */
static const uint8_t app_key[16] = {
    0xFC, 0x00, 0x10, 0x2B, 0xC3, 0xBD, 0x0E, 0x62,
    0x19, 0xCC, 0xB1, 0x90, 0xB1, 0x0A, 0x88, 0xA9,
};

#define NET_IDX    0x000   /* Network Key Index */
#define APP_IDX    0x000   /* Application Key Index */
#define GROUP_ADDR 0xC000  /* Group Address for Sensor Publishing */
#define GATEWAY_ADDR 0x0001 /* Unicast-Adress of Gateway */

/* Dedicated workqueue for mesh configuration (avoids system workqueue deadlock) */
#define CONFIG_WQ_STACK_SIZE 4096
static K_THREAD_STACK_DEFINE(config_wq_stack, CONFIG_WQ_STACK_SIZE);
static struct k_work_q config_wq;

/* Delayed work for post-provisioning configuration */
static struct k_work_delayable configure_work;

static void configure_handler(struct k_work *work)
{
    int err;
    uint8_t status;

    /* Add AppKey */
    err = bt_mesh_app_key_add(APP_IDX, NET_IDX, app_key);
    if (err && err != -EALREADY) {
        LOG_ERR("Failed to add AppKey (err %d), retrying...", err);
        k_work_schedule_for_queue(&config_wq, &configure_work, K_SECONDS(2));
        return;
    }

    /* Bind AppKey to Sensor Client model */
    err = bt_mesh_cfg_cli_mod_app_bind(NET_IDX, GATEWAY_ADDR,
                                       GATEWAY_ADDR, APP_IDX,
                                       BT_MESH_MODEL_ID_SENSOR_CLI, &status);
    if (err) {
        LOG_ERR("Failed to bind AppKey (err %d), retrying...", err);
        k_work_schedule_for_queue(&config_wq, &configure_work, K_SECONDS(2));
        return;
    }

    /* Subscribe to group address for receiving sensor publications */
    err = bt_mesh_cfg_cli_mod_sub_add(NET_IDX, GATEWAY_ADDR,
                                      GATEWAY_ADDR, GROUP_ADDR,
                                      BT_MESH_MODEL_ID_SENSOR_CLI, &status);
    if (err) {
        LOG_ERR("Failed to subscribe to group address (err %d), retrying...", err);
        k_work_schedule_for_queue(&config_wq, &configure_work, K_SECONDS(2));
        return;
    }

    LOG_INF("Sensor Client model bound to AppKey 0x%03X", APP_IDX);
    LOG_INF("Subscribed to Group Address 0x%04X for receiving sensor data", GROUP_ADDR);

    /* Save settings to persistent storage */
    if (IS_ENABLED(CONFIG_SETTINGS)) {
        settings_save();
        LOG_INF("Settings saved to persistent storage");
    }
}

/* Forward declaration of the sensor data callback */
static void sensor_data_cb(struct bt_mesh_sensor_cli *cli,
                           struct bt_mesh_msg_ctx *ctx,
                           const struct bt_mesh_sensor_type *sensor,
                           const struct bt_mesh_sensor_value *value)

{
    if (!sensor) {
        LOG_WRN("sensor_data_cb: sensor type is NULL");
        return;
    }

    uint16_t prop_id = sensor->id;
    int32_t raw = 0;
    int64_t micro_val = 0;

    /* Read out first channel */
    if (sensor->channel_count > 0) {
        const struct bt_mesh_sensor_format *fmt = sensor->channels[0].format;
        if (fmt && fmt->cb && fmt->cb->to_micro) {
            fmt->cb->to_micro(&value[0], &micro_val);
            raw = (int32_t) (micro_val /  1000000LL); /* Convert from micro to integer */
        }
    }
    switch (prop_id)
    {
    case PROP_TEMPERATURE:
        LOG_INF("Temperature: %d °C", raw);
        break;
    case PROP_HUMIDITY:
        LOG_INF("Humidity: %d %%", raw);
        break;
    case PROP_ECO2:
        LOG_INF("eCO2: %d ppm", raw);
        break;
    case PROP_HEART_RATE:
        LOG_INF("Heart Rate: %d bpm", raw);
        break;
    case PROP_SPO2:
        LOG_INF("SpO2: %d %%", raw);
        break;
    case PROP_TVOC:
        LOG_INF("TVOC: %d ppb", raw);
        break;
    case PROP_RAW_IR:
        LOG_INF("Raw IR: %d", raw);
        break;    
    default:
        LOG_WRN("Unknown sensor property ID: 0x%04X", prop_id);
        break;
    }
}

/* Callback for receiving a series of sensor values (if the node supports it) */
static void sensor_series_entry_cb(struct bt_mesh_sensor_cli *cli,
                                   struct bt_mesh_msg_ctx *ctx,
                                   const struct bt_mesh_sensor_type *sensor,
                                   uint8_t index, uint8_t count,
                                   const struct bt_mesh_sensor_series_entry *entry)
{
    LOG_INF("Series entry [%u/%u] von 0x%04X", index, count, ctx->addr);
}

static void sensor_desc_cb(struct bt_mesh_sensor_cli *cli,
                           struct bt_mesh_msg_ctx *ctx,
                           const struct bt_mesh_sensor_info *sensor)
{
    if (sensor) {
        LOG_INF("Sensor gefunden: Property ID 0x%04X", sensor->id);
    }
}

static const struct bt_mesh_sensor_cli_handlers sensor_cli_handlers = {
    .data = sensor_data_cb,
    .sensor = sensor_desc_cb,
    .series_entry = sensor_series_entry_cb,
};

static struct bt_mesh_sensor_cli sensor_cli = BT_MESH_SENSOR_CLI_INIT(&sensor_cli_handlers);

/* Health Server callback (not used in this example, but required for composition) */
static struct bt_mesh_health_srv health_srv = {};

BT_MESH_HEALTH_PUB_DEFINE(health_pub, 0x00); /* No publication */


static struct bt_mesh_cfg_cli cfg_cli = {};
/* Model publication context for the Sensor Client model */
static struct bt_mesh_model root_models[] = {
    BT_MESH_MODEL_CFG_SRV,
    BT_MESH_MODEL_CFG_CLI(&cfg_cli),
    BT_MESH_MODEL_HEALTH_SRV(&health_srv, &health_pub),
    BT_MESH_MODEL_SENSOR_CLI(&sensor_cli),
};

/* Elements */
static struct bt_mesh_elem elements[] = {
    BT_MESH_ELEM(0, root_models, BT_MESH_MODEL_NONE),
};

/* Composition */
static const struct bt_mesh_comp comp = {
    .cid = 0xFFFF, /* Use a test Company ID */
    .elem = elements,
    .elem_count = ARRAY_SIZE(elements),
};

static void self_provision(void)
{
    int err;
    uint8_t dev_uuid[16];

    /* Generate a unique Device UUID based on the hardware ID */
    err = hwinfo_get_device_id(dev_uuid, sizeof(dev_uuid));
    if (err < 0) {
        LOG_WRN("Failed to get device ID, using fallback UUID (err %d)", err);
        memset(dev_uuid, 0xFF, sizeof(dev_uuid)); /* Fallback UUID */
    }

    /* Check if the device is already provisioned */
    if (bt_mesh_is_provisioned()) {
        LOG_INF("Device is already provisioned");
        return;
    }

    /* Provision the device */
    LOG_INF("Provisioning device with UUID: %02X:%02X:%02X:%02X:%02X:%02X:%02X:%02X:%02X:%02X:%02X:%02X:%02X:%02X:%02X:%02X",
            dev_uuid[0], dev_uuid[1], dev_uuid[2], dev_uuid[3], dev_uuid[4], dev_uuid[5], dev_uuid[6], dev_uuid[7],
            dev_uuid[8], dev_uuid[9], dev_uuid[10], dev_uuid[11], dev_uuid[12], dev_uuid[13], dev_uuid[14], dev_uuid[15]);
    
    /* Add NetKey */
    err = bt_mesh_cdb_create(net_key);
    if (err && err != -EALREADY) {
        LOG_ERR("Failed to create NetKey (err %d)", err);
        return;
    }

    /* self-provision the device */
    err = bt_mesh_provision(net_key, NET_IDX,
                            0,              /* Flags */
                            0x000000,       /* IV Index */
                            GATEWAY_ADDR,   /* Unicast = 0x0001 */
                            dev_uuid);
    if (err) {
        LOG_ERR("Failed to self-provision (err %d)", err);
        return;
    }

    LOG_INF("Device provisioned successfully with address 0x%04X", GATEWAY_ADDR);

    /* Schedule configuration with delay to let mesh stack fully initialize */
    /* Initialize dedicated workqueue and schedule configuration */
    k_work_queue_init(&config_wq);
    k_work_queue_start(&config_wq, config_wq_stack,
                       K_THREAD_STACK_SIZEOF(config_wq_stack),
                       K_PRIO_PREEMPT(1), NULL);
    k_work_init_delayable(&configure_work, configure_handler);
    k_work_schedule_for_queue(&config_wq, &configure_work, K_SECONDS(2));
}

// =============================================================================
// Bluetooth and Mesh Initialization
// =============================================================================
static uint8_t prov_uuid[16];

static const struct bt_mesh_prov prov = {
    .uuid = prov_uuid,
};

// =============================================================================
// Open API for main.c
// =============================================================================

const struct bt_mesh_prov *model_handler_prov_init(void)
{
    int err = hwinfo_get_device_id(prov_uuid, sizeof(prov_uuid));
    if (err < 0) {
        LOG_WRN("Failed to get device ID, using fallback UUID (err %d)", err);
        memset(prov_uuid, 0xFF, sizeof(prov_uuid));
    }
    return &prov;
}

const struct bt_mesh_comp *model_handler_comp_init(void)
{
    return &comp;
}

void model_handler_self_provision(void)
{
    self_provision();
}