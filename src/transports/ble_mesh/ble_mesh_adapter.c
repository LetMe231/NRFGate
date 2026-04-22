/**
 * @file ble_mesh_adapter.c
 * @brief BLE Mesh adapter for the generic gateway abstraction.
 *
 * This module integrates the BLE Mesh stack into the generic gateway adapter
 * interface (`gw_adapter_t`). It provides:
 * - reception and decoding of BLE Mesh messages
 * - forwarding parsed events into the ingest pipeline
 * - transmission of Generic OnOff commands
 * - access to provisioning and composition data
 */

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/bluetooth/mesh.h>
#include <zephyr/drivers/hwinfo.h>

#include <hal/nrf_ficr.h>
#include <bluetooth/mesh/gen_onoff_cli.h>
#include <string.h>

#include "scheduler.h"
#include "ble_mesh_adapter.h"
#include "ble_mesh_prov.h"
#include "event_ingest.h"
#include "ble_mesh_codec.h"
#include "scheduler.h"

LOG_MODULE_REGISTER(ble_mesh_adapter, LOG_LEVEL_INF);

/* ─────────────────────────────────────────────────────────────
 * Constants and configuration
 * ───────────────────────────────────────────────────────────── */

/**
 * @brief Model indices inside the root model array.
 *
 * These indices are used to retrieve specific model instances from the
 * composition table.
 */
enum {
    ROOT_MODEL_IDX_CFG_SRV = 0,
    ROOT_MODEL_IDX_CFG_CLI,
    ROOT_MODEL_IDX_HEALTH_SRV,
    ROOT_MODEL_IDX_SENSOR_CLI,
    ROOT_MODEL_IDX_ONOFF_CLI,
};

/** @brief Default publication TTL used for hop estimation. */
#define MESH_PUB_TTL 5

/** @brief Sensor Status opcode. */
#define BT_MESH_MODEL_OP_SENSOR_STATUS BT_MESH_MODEL_OP_1(0x52)

/* ─────────────────────────────────────────────────────────────
 * Internal adapter type
 * ───────────────────────────────────────────────────────────── */

/**
 * @brief Concrete BLE Mesh adapter instance.
 *
 * This structure embeds the generic adapter base and keeps all BLE Mesh
 * specific runtime state in one place.
 */
typedef struct ble_mesh_adapter {
    /** @brief Generic gateway adapter base object. */
    gw_adapter_t base;

    /** @brief Serialize outgoing BLE Mesh client transmissions. */
    struct k_mutex tx_lock;

    /** @brief Timestamp of the last received BLE Mesh message in ms. */
    int64_t last_rx_ms;

    /** @brief Provisioning UUID derived from the hardware device ID. */
    uint8_t prov_uuid[16];

    /** @brief Nordic Generic OnOff Client model instance. */
    struct bt_mesh_onoff_cli onoff_cli;

    /** @brief Health Server instance used during provisioning and health reporting. */
    struct bt_mesh_health_srv health_srv;

    /** @brief Provisioning context with callbacks configured externally. */
    struct bt_mesh_prov prov;

    /** @brief Configuration Client instance for node configuration. */
    struct bt_mesh_cfg_cli cfg_cli;
} ble_mesh_adapter_t;

/* ─────────────────────────────────────────────────────────────
 * Generic OnOff Client handlers (Nordic High-Level API)
 * ───────────────────────────────────────────────────────────── */

static void ble_mesh_onoff_status_cb(struct bt_mesh_onoff_cli *cli,
                                     struct bt_mesh_msg_ctx *ctx,
                                     const struct bt_mesh_onoff_status *status);

/* ─────────────────────────────────────────────────────────────
 * Static module state
 * ───────────────────────────────────────────────────────────── */

/** @brief Single static BLE Mesh adapter instance. */
static ble_mesh_adapter_t s_ble;

static ble_mesh_adapter_t s_ble = {
    .onoff_cli = BT_MESH_ONOFF_CLI_INIT(&ble_mesh_onoff_status_cb),
};

/* ─────────────────────────────────────────────────────────────
 * Forward declarations
 * ───────────────────────────────────────────────────────────── */

static void ble_mesh_adapter_init(ble_mesh_adapter_t *self,
                                  gw_adapter_emit_fn emit,
                                  void *emit_user);
static void ble_mesh_require_setup(void);

static void ble_mesh_emit(ble_mesh_adapter_t *self, const gw_event_t *evt);
static void ble_mesh_emit_to_ingest(const gw_event_t *evt, void *user);
static void ble_mesh_codec_emit_to_adapter(const gw_event_t *evt, void *user);

static int ble_mesh_send_cmd(gw_adapter_t *base, const gw_command_t *cmd);
static int64_t ble_mesh_last_rx_ms(gw_adapter_t *base);

static ble_mesh_adapter_t *ble_mesh_model_adapter(const struct bt_mesh_model *model);
static uint8_t ble_mesh_rx_hops(const struct bt_mesh_msg_ctx *ctx);
static gw_rx_meta_t ble_mesh_rx_meta(const struct bt_mesh_msg_ctx *ctx);

static int ble_mesh_sensor_status_handler(const struct bt_mesh_model *model,
                                          struct bt_mesh_msg_ctx *ctx,
                                          struct net_buf_simple *buf);

static int ble_mesh_onoff_from_cmd(const gw_command_t *cmd, uint8_t *onoff);
static struct bt_mesh_msg_ctx ble_mesh_msg_ctx_for_dst(uint16_t dst);

/* ─────────────────────────────────────────────────────────────
 * Adapter API table
 * ───────────────────────────────────────────────────────────── */

/** @brief Vtable for the generic gateway adapter interface. */
static const gw_adapter_api_t s_ble_mesh_api = {
    .send_cmd   = ble_mesh_send_cmd,
    .last_rx_ms = ble_mesh_last_rx_ms,
};

/* ─────────────────────────────────────────────────────────────
 * Internal initialization helpers
 * ───────────────────────────────────────────────────────────── */

/**
 * @brief Initialize a BLE Mesh adapter object.
 *
 * @param self       Adapter instance to initialize.
 * @param emit       Event callback used by the adapter.
 * @param emit_user  User context passed to @p emit.
 */
static void ble_mesh_adapter_init(ble_mesh_adapter_t *self,
                                  gw_adapter_emit_fn emit,
                                  void *emit_user)
{
    self->last_rx_ms = 0;

    memset(self->prov_uuid, 0, sizeof(self->prov_uuid));
    memset(&self->cfg_cli, 0, sizeof(self->cfg_cli));
    memset(&self->health_srv, 0, sizeof(self->health_srv));

    k_mutex_init(&self->tx_lock);

    self->base.api = &s_ble_mesh_api;
    self->base.emit = emit;
    self->base.emit_user = emit_user;

    self->prov.uuid = self->prov_uuid;
}

/**
 * @brief Ensure the adapter was initialized before being used.
 */
static void ble_mesh_require_setup(void)
{
    __ASSERT(s_ble.base.api != NULL, "BLE mesh adapter used before setup");
}

/* ─────────────────────────────────────────────────────────────
 * Public adapter access
 * ───────────────────────────────────────────────────────────── */

/**
 * @brief Get the generic gateway adapter interface for BLE Mesh.
 *
 * @return Pointer to the embedded gateway adapter base.
 */
gw_adapter_t *ble_mesh_adapter_get(void)
{
    ble_mesh_require_setup();
    return &s_ble.base;
}

/**
 * @brief Return the timestamp of the last received BLE Mesh message.
 *
 * @param base Pointer to the generic gateway adapter base.
 *
 * @return Timestamp in milliseconds since boot.
 */
static int64_t ble_mesh_last_rx_ms(gw_adapter_t *base)
{
    ble_mesh_adapter_t *self = CONTAINER_OF(base, ble_mesh_adapter_t, base);
    return self->last_rx_ms;
}

/* ─────────────────────────────────────────────────────────────
 * Event emission helpers
 * ───────────────────────────────────────────────────────────── */

/**
 * @brief Emit an event through the adapter callback if one is registered.
 *
 * @param self Adapter instance.
 * @param evt  Event to emit.
 */
static void ble_mesh_emit(ble_mesh_adapter_t *self, const gw_event_t *evt)
{
    if (self->base.emit) {
        self->base.emit(evt, self->base.emit_user);
    }
}

/**
 * @brief Forward an event into the ingest pipeline.
 *
 * @param evt   Event to submit.
 * @param user  Unused user pointer.
 */
static void ble_mesh_emit_to_ingest(const gw_event_t *evt, void *user)
{
    ARG_UNUSED(user);
    event_ingest_submit(evt);
}

/**
 * @brief Callback wrapper used by the BLE Mesh codec.
 *
 * The codec emits parsed events through this wrapper so that they can be
 * forwarded through the adapter's configured emit callback.
 *
 * @param evt   Parsed gateway event.
 * @param user  Adapter instance pointer.
 */
static void ble_mesh_codec_emit_to_adapter(const gw_event_t *evt, void *user)
{
    ble_mesh_adapter_t *self = user;
    if (!self) {
        LOG_WRN("ble_mesh_codec_emit_to_adapter: self is NULL");
        return;
    }

    ble_mesh_emit(self, evt);
}

/* ─────────────────────────────────────────────────────────────
 * RX helpers
 * ───────────────────────────────────────────────────────────── */

/**
 * @brief Retrieve the adapter instance stored in a mesh model.
 *
 * @param model BLE Mesh model pointer.
 *
 * @return Adapter pointer on success, NULL otherwise.
 */
static ble_mesh_adapter_t *ble_mesh_model_adapter(const struct bt_mesh_model *model)
{
    if (!model || !model->rt || !model->rt->user_data) {
        return NULL;
    }

    return model->rt->user_data;
}

/**
 * @brief Estimate the number of hops from the receive context.
 *
 * @param ctx BLE Mesh message context.
 *
 * @return Hop count derived from the receive TTL.
 */
static uint8_t ble_mesh_rx_hops(const struct bt_mesh_msg_ctx *ctx)
{
    if (!ctx || ctx->recv_ttl == 0) {
        return 0;
    }

    return (ctx->recv_ttl < MESH_PUB_TTL) ? (MESH_PUB_TTL - ctx->recv_ttl) : 0;
}

/**
 * @brief Build gateway RX metadata from a BLE Mesh message context.
 *
 * @param ctx BLE Mesh message context.
 *
 * @return Filled RX metadata structure.
 */
static gw_rx_meta_t ble_mesh_rx_meta(const struct bt_mesh_msg_ctx *ctx)
{
    gw_rx_meta_t meta = {0};
    uint8_t hops = ble_mesh_rx_hops(ctx);

    meta.has_rssi   = true;
    meta.rssi_dbm   = ctx ? ctx->recv_rssi : 0;
    meta.has_hops   = true;
    meta.hops       = hops;
    meta.has_ttl_rx = true;
    meta.rx_ttl     = ctx ? ctx->recv_ttl : 0;

    return meta;
}

/* ─────────────────────────────────────────────────────────────
 * RX handlers
 * ───────────────────────────────────────────────────────────── */

/**
 * @brief Handle incoming Sensor Status messages.
 *
 * While provisioning is busy, sensor broadcasts are ignored at the application
 * level to reduce queue pressure and avoid interference with provisioning and
 * configuration handshakes.
 *
 * @param model BLE Mesh model receiving the message.
 * @param ctx   BLE Mesh message context.
 * @param buf   Payload buffer.
 *
 * @retval 0 on success or ignored message
 * @retval -EINVAL if the model context is invalid
 */
static int ble_mesh_sensor_status_handler(const struct bt_mesh_model *model,
                                          struct bt_mesh_msg_ctx *ctx,
                                          struct net_buf_simple *buf)
{
    ble_mesh_adapter_t *self = ble_mesh_model_adapter(model);
    if (!self) {
        LOG_WRN("Sensor Status handler called with invalid model/user_data");
        return -EINVAL;
    }

    /*
     * Drop sensor broadcasts while provisioning/config is busy. Processing
     * them steals CPU and congests mesh internals, causing config handshake
     * timeouts for the node being added. Returning early bypasses the
     * parser and event_ingest_submit — the mesh stack still acks/relays,
     * but the gateway's application layer doesn't fan out the event.
     * The physical RF channel remains shared, but the gateway's own queue
     * pressure drops significantly.
     */
    if (ble_mesh_prov_is_busy()) {
        return 0;
    }

    self->last_rx_ms = k_uptime_get();

    uint8_t hops = ble_mesh_rx_hops(ctx);

    LOG_INF("Received Sensor Status from 0x%04X [%u bytes] (RSSI %d dBm, hops %u)",
            ctx->addr, buf->len, ctx->recv_rssi, hops);

    // scheduler_request_priority(SCHED_PRIORITY_BLE, 150);

    int err = ble_mesh_codec_parse_sensor_status(buf->data,
                                                 buf->len,
                                                 ctx->addr,
                                                 ctx->recv_rssi,
                                                 hops,
                                                 ble_mesh_codec_emit_to_adapter,
                                                 self);
    if (err) {
        LOG_WRN("Failed to parse sensor status from 0x%04X: %d", ctx->addr, err);
    }

    return 0;
}

static void ble_mesh_onoff_status_cb(struct bt_mesh_onoff_cli *cli,
                                     struct bt_mesh_msg_ctx *ctx,
                                     const struct bt_mesh_onoff_status *status)
{
    ble_mesh_adapter_t *self = CONTAINER_OF(cli, ble_mesh_adapter_t, onoff_cli);

    if (ble_mesh_prov_is_busy()) {
        return;
    }

    self->last_rx_ms = k_uptime_get();

    gw_event_t evt = {
        .type  = GW_EVT_ACTUATOR_STATE,
        .rx_ms = self->last_rx_ms,
        .src   = {
            .transport = GW_TR_BLE_MESH,
            .mesh_addr = ctx->addr,
        },
        .rx_meta = ble_mesh_rx_meta(ctx),
        .data.actuator_state = {
            .light_on = (status->present_on_off != 0),
        },
    };

    LOG_INF("OnOff Status from 0x%04X: %s rssi=%d hops=%u",
            ctx->addr,
            status->present_on_off ? "ON" : "OFF",
            ctx->recv_rssi,
            evt.rx_meta.hops);

    ble_mesh_emit(self, &evt);
}

/* ─────────────────────────────────────────────────────────────
 * Model operation tables
 * ───────────────────────────────────────────────────────────── */

/** @brief Supported opcodes for the Sensor Client model. */
static const struct bt_mesh_model_op sensor_cli_ops[] = {
    { BT_MESH_MODEL_OP_SENSOR_STATUS, 0, ble_mesh_sensor_status_handler },
    BT_MESH_MODEL_OP_END,
};

/* ─────────────────────────────────────────────────────────────
 * Mesh composition
 * ───────────────────────────────────────────────────────────── */

/** @brief Health publication context. */
BT_MESH_HEALTH_PUB_DEFINE(health_pub, 0);

/** @brief Root element model list. */
static struct bt_mesh_model root_models[] = {
    BT_MESH_MODEL_CFG_SRV,
    BT_MESH_MODEL_CFG_CLI(&s_ble.cfg_cli),
    BT_MESH_MODEL_HEALTH_SRV(&s_ble.health_srv, &health_pub),
    BT_MESH_MODEL(SENSOR_CLI_MODEL_ID, sensor_cli_ops, NULL, &s_ble),
    BT_MESH_MODEL_ONOFF_CLI(&s_ble.onoff_cli),
};

/** @brief Mesh element table containing the root element. */
static struct bt_mesh_elem elements[] = {
    BT_MESH_ELEM(0, root_models, BT_MESH_MODEL_NONE),
};

/**
 * @brief BLE Mesh composition data.
 *
 * A non-commercial company ID is used here because no real vendor model is
 * implemented.
 */
static struct bt_mesh_comp comp = {
    .cid        = 0xFFFF,
    .elem       = elements,
    .elem_count = ARRAY_SIZE(elements),
};

/* ─────────────────────────────────────────────────────────────
 * TX helpers
 * ───────────────────────────────────────────────────────────── */

/**
 * @brief Convert a generic gateway command into an OnOff state.
 *
 * @param cmd    Gateway command.
 * @param onoff  Output OnOff value.
 *
 * @retval 0 on success
 * @retval -EINVAL if the input is invalid or unsupported
 * @retval -ENOTSUP if toggle is requested without a known current state
 */
static int ble_mesh_onoff_from_cmd(const gw_command_t *cmd, uint8_t *onoff)
{
    if (!cmd || !onoff) {
        return -EINVAL;
    }

    switch (cmd->type) {
    case GW_CMD_LIGHT_ON:
        *onoff = 1;
        return 0;

    case GW_CMD_LIGHT_OFF:
        *onoff = 0;
        return 0;

    case GW_CMD_LIGHT_TOGGLE:
        LOG_WRN("Toggle command not supported without current state");
        return -ENOTSUP;

    default:
        LOG_ERR("Unsupported command type: %d", cmd->type);
        return -EINVAL;
    }
}

/**
 * @brief Create a BLE Mesh message context for a destination address.
 *
 * @param dst Destination mesh address.
 *
 * @return Initialized message context.
 */
static struct bt_mesh_msg_ctx ble_mesh_msg_ctx_for_dst(uint16_t dst)
{
    struct bt_mesh_msg_ctx ctx = {
        .net_idx  = 0,
        .app_idx  = 0,
        .addr     = dst,
        .send_ttl = BT_MESH_TTL_DEFAULT,
    };

    return ctx;
}

/* ─────────────────────────────────────────────────────────────
 * TX command API
 * ───────────────────────────────────────────────────────────── */

/**
 * @brief Send a gateway command over BLE Mesh.
 *
 * Currently only Generic OnOff commands are supported.
 *
 * @param base Generic gateway adapter base pointer.
 * @param cmd  Command to send.
 *
 * @retval 0 on success
 * @retval <0 Zephyr or adapter-specific error code on failure
 */
static int ble_mesh_send_cmd(gw_adapter_t *base, const gw_command_t *cmd)
{
    ble_mesh_adapter_t *self = CONTAINER_OF(base, ble_mesh_adapter_t, base);

    if (!cmd) {
        return -EINVAL;
    }

    if (cmd->dst.transport != GW_TR_BLE_MESH) {
        return -EINVAL;
    }

    uint8_t onoff;
    int err = ble_mesh_onoff_from_cmd(cmd, &onoff);
    if (err) {
        return err;
    }

    err = k_mutex_lock(&self->tx_lock, K_MSEC(100));
    if (err) {
        LOG_WRN("Failed to acquire tx_lock: %d", err);
        return err;
    }

    struct bt_mesh_msg_ctx ctx = ble_mesh_msg_ctx_for_dst(cmd->dst.mesh_addr);

    struct bt_mesh_onoff_set set = {
        .on_off      = (onoff != 0),
        .transition  = NULL,      /* instant */
        .reuse_transaction = false,
    };

    err = bt_mesh_onoff_cli_set(&self->onoff_cli, &ctx, &set, NULL);

    k_mutex_unlock(&self->tx_lock);

    if (err) {
        LOG_WRN("OnOff Set to 0x%04X failed: %d", cmd->dst.mesh_addr, err);
    }
    return err;
}

/* ─────────────────────────────────────────────────────────────
 * Public mesh access
 * ───────────────────────────────────────────────────────────── */

/**
 * @brief Return the BLE Mesh composition used by this adapter.
 *
 * @return Pointer to the composition structure.
 */
const struct bt_mesh_comp *ble_mesh_adapter_comp_init(void)
{
    ble_mesh_require_setup();
    return &comp;
}

/**
 * @brief Return the provisioning context for this adapter.
 *
 * The provisioning UUID is derived from the hardware device ID. If this fails,
 * the nRF FICR device ID is used as a fallback.
 *
 * @return Pointer to the provisioning context.
 */
const struct bt_mesh_prov *ble_mesh_adapter_prov_get(void)
{
    ble_mesh_require_setup();

    int err = hwinfo_get_device_id(s_ble.prov_uuid, sizeof(s_ble.prov_uuid));
    if (err < 0) {
        LOG_WRN("Failed to get device ID from hwinfo (err %d), using FICR fallback", err);

        uint32_t id0 = nrf_ficr_deviceid_get(NRF_FICR, 0);
        uint32_t id1 = nrf_ficr_deviceid_get(NRF_FICR, 1);

        memcpy(&s_ble.prov_uuid[0], &id0, 4);
        memcpy(&s_ble.prov_uuid[4], &id1, 4);
        memset(&s_ble.prov_uuid[8], 0x00, 8);
    }

    return &s_ble.prov;
}

/**
 * @brief Register provisioning callbacks in the adapter provisioning context.
 *
 * @param complete   Called when provisioning completes.
 * @param node_added Called when a node is added.
 * @param link_close Called when the provisioning link closes.
 * @param beacon     Called for unprovisioned beacons.
 */
void ble_mesh_adapter_register_prov_cb(
    void (*complete)(uint16_t net_idx, uint16_t addr),
    void (*node_added)(uint16_t net_idx, uint8_t uuid[16],
                       uint16_t addr, uint8_t num_elem),
    void (*link_close)(bt_mesh_prov_bearer_t bearer),
    void (*beacon)(uint8_t uuid[16],
                   bt_mesh_prov_oob_info_t oob_info,
                   uint32_t *uri_hash))
{
    ble_mesh_require_setup();

    s_ble.prov.complete             = complete;
    s_ble.prov.node_added           = node_added;
    s_ble.prov.link_close           = link_close;
    s_ble.prov.unprovisioned_beacon = beacon;
}

/**
 * @brief Initialize the static BLE Mesh adapter instance.
 *
 * This function must be called before accessing the adapter, composition or
 * provisioning data.
 */
void ble_mesh_adapter_setup(void)
{
    ble_mesh_adapter_init(&s_ble, ble_mesh_emit_to_ingest, NULL);
}