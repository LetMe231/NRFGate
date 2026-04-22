/**
 * @file lorawan_adapter.c
 * @brief LoRa adapter for the gateway abstraction layer.
 *
 * This module wraps the Zephyr LoRa driver into the generic gateway adapter
 * interface (`gw_adapter_t`). It supports:
 * - asynchronous LoRa reception
 * - forwarding received events into the ingest pipeline
 * - sending gateway commands over LoRa
 * - relaying non-LoRa events onto the LoRa transport
 */

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/drivers/lora.h>

#include <string.h>

#include "lorawan_adapter.h"
#include "lora_wire_format.h"
#include "event_ingest.h"
#include "gw_adapter.h"

LOG_MODULE_REGISTER(lorawan_adapter, LOG_LEVEL_INF);

/* ─────────────────────────────────────────────────────────────
 * Configuration constants
 * ───────────────────────────────────────────────────────────── */

/** @brief LoRa radio frequency in Hz (EU868). */
#define LORA_FREQ       868000000

/** @brief LoRa bandwidth configuration. */
#define LORA_BW         BW_125_KHZ

/** @brief LoRa spreading factor. */
#define LORA_SF         SF_7

/** @brief LoRa coding rate. */
#define LORA_CR         CR_4_5

/** @brief LoRa preamble length in symbols. */
#define LORA_PREAMBLE   8

/** @brief LoRa transmit power in dBm. */
#define LORA_TX_POWER   4

/** @brief LoRa TX worker stack size in bytes. */
#define LORA_TX_STACK_SIZE 512

/** @brief LoRa Tx worker priority. */
#define LORA_TX_PRIORITY   7

/** @brief Maximum Queued LoRa TX requests. */
#define LORA_TX_QUEUE_LEN  8

/* ─────────────────────────────────────────────────────────────
 * Internal types
 * ───────────────────────────────────────────────────────────── */

/**
 * @brief Queued LoRa TX request.
 */
typedef struct {
    /** @brief Payload buffer. */
    uint8_t buf[WIRE_MAX_SIZE];

    /** @brief Payload length in bytes. */
    uint8_t len;
} lora_tx_req_t;

/**
 * @brief Concrete LoRa adapter instance.
 *
 * This structure embeds the generic gateway adapter base and stores
 * the LoRa-specific runtime state.
 */
typedef struct {
    /** @brief Generic gateway adapter base object. */
    gw_adapter_t base;

    /** @brief Timestamp of the last successfully received packet in ms. */
    int64_t last_rx_ms;

    /** @brief Zephyr LoRa device handle. */
    const struct device *dev;

    /** @brief Whether the relay path is enabled. */
    bool enabled;

    /** @brief Protects TX/RX mode switching and radio access. */
    struct k_mutex radio_mutex;

    /** @brief TX worker thread object. */
    struct k_thread tx_tid;
} lorawan_adapter_t;


/* ─────────────────────────────────────────────────────────────
 * Static module state
 * ───────────────────────────────────────────────────────────── */

/** @brief Single static LoRa adapter instance. */
static lorawan_adapter_t s_lora;

/** @brief LoRa TX request queue. */
K_MSGQ_DEFINE(s_lora_txq, sizeof(lora_tx_req_t), LORA_TX_QUEUE_LEN, 4);

/** @brief LoRa TX worker stack. */
K_THREAD_STACK_DEFINE(s_lora_tx_stack, LORA_TX_STACK_SIZE);

/* ─────────────────────────────────────────────────────────────
 * Forward declarations
 * ───────────────────────────────────────────────────────────── */

static void lora_relay_listener(const gw_event_t *evt, void *ctx);
static void lora_rx_cb(const struct device *dev, uint8_t *data, uint16_t size,
                       int16_t rssi, int8_t snr, void *user_data);

static int lora_set_mode(lorawan_adapter_t *self, bool tx);
static int lora_tx(lorawan_adapter_t *self, const uint8_t *buf, uint8_t len);
static int lora_cmd_bytes(const gw_command_t *cmd, uint8_t buf[2]);

static void lora_emit_to_ingest(const gw_event_t *evt, void *user);
static void lora_emit(lorawan_adapter_t *self, const gw_event_t *evt);
static void lorawan_require_setup(void);
static void lorawan_adapter_obj_init(lorawan_adapter_t *self,
                                     gw_adapter_emit_fn emit,
                                     void *emit_user);

static int lora_send_cmd(gw_adapter_t *base, const gw_command_t *cmd);
static int64_t lora_last_rx_ms(gw_adapter_t *base);

static void lora_tx_thread_fn(void *p1, void *p2, void *p3);
static int lora_enqueue_tx(lorawan_adapter_t *self, const uint8_t *buf, uint8_t len);

/* ─────────────────────────────────────────────────────────────
 * Gateway adapter API table
 * ───────────────────────────────────────────────────────────── */

/** @brief Vtable for the generic gateway adapter interface. */
static const gw_adapter_api_t s_lora_api = {
    .send_cmd   = lora_send_cmd,
    .last_rx_ms = lora_last_rx_ms,
};

/* ─────────────────────────────────────────────────────────────
 * Internal object lifecycle helpers
 * ───────────────────────────────────────────────────────────── */

/**
 * @brief Initialize a LoRa adapter object.
 *
 * @param self       Adapter instance to initialize.
 * @param emit       Event callback used by the adapter.
 * @param emit_user  User context passed to @p emit.
 */
static void lorawan_adapter_obj_init(lorawan_adapter_t *self,
                                     gw_adapter_emit_fn emit,
                                     void *emit_user)
{
    memset(self, 0, sizeof(*self));

    self->base.api       = &s_lora_api;
    self->base.emit      = emit;
    self->base.emit_user = emit_user;

    self->dev = NULL;
    self->enabled = false;

    k_mutex_init(&self->radio_mutex);
}

/**
 * @brief Ensure the module has been initialized via @ref lorawan_adapter_setup.
 */
static void lorawan_require_setup(void)
{
    __ASSERT(s_lora.base.api != NULL, "LoRa adapter used before setup");
}

/* ─────────────────────────────────────────────────────────────
 * Radio configuration helpers
 * ───────────────────────────────────────────────────────────── */

/**
 * @brief Configure the LoRa modem either for TX or RX mode.
 *
 * @param self  Adapter instance.
 * @param tx    true to configure for TX, false for RX.
 *
 * @retval 0 on success
 * @retval <0 Zephyr error code on failure
 */
static int lora_set_mode(lorawan_adapter_t *self, bool tx)
{
    struct lora_modem_config cfg = {
        .frequency    = LORA_FREQ,
        .bandwidth    = LORA_BW,
        .datarate     = LORA_SF,
        .coding_rate  = LORA_CR,
        .preamble_len = LORA_PREAMBLE,
        .tx_power     = LORA_TX_POWER,
        .tx           = tx,
    };

    int err = lora_config(self->dev, &cfg);
    if (err < 0) {
        LOG_ERR("lora_config(%s) failed: %d", tx ? "TX" : "RX", err);
    }

    return err;
}

/* ─────────────────────────────────────────────────────────────
 * Event emission helpers
 * ───────────────────────────────────────────────────────────── */

/**
 * @brief Forward a received event into the ingest pipeline.
 *
 * @param evt   Event to submit.
 * @param user  Unused user pointer.
 */
static void lora_emit_to_ingest(const gw_event_t *evt, void *user)
{
    ARG_UNUSED(user);
    event_ingest_submit(evt);
}

/**
 * @brief Emit an event through the adapter callback if present.
 *
 * @param self  Adapter instance.
 * @param evt   Event to emit.
 */
static void lora_emit(lorawan_adapter_t *self, const gw_event_t *evt)
{
    if (self->base.emit) {
        self->base.emit(evt, self->base.emit_user);
    }
}

/**
 * @brief Queue a LoRa payload for transmission by the TX worker.
 *
 * @param self Adapter instance.
 * @param buf  Payload buffer.
 * @param len  Payload length in bytes.
 *
 * @retval 0 on success
 * @retval -EINVAL if parameters are invalid
 * @retval -ENOSPC if the TX queue is full
 */
static int lora_enqueue_tx(lorawan_adapter_t *self, const uint8_t *buf, uint8_t len)
{
    if (!self || !buf || len == 0 || len > WIRE_MAX_SIZE) {
        return -EINVAL;
    }

    lora_tx_req_t req = {0};
    memcpy(req.buf, buf, len);
    req.len = len;

    int err = k_msgq_put(&s_lora_txq, &req, K_NO_WAIT);
    if (err < 0) {
        LOG_WRN("LoRa TX queue full, dropping packet");
        return -ENOSPC;
    }

    return 0;
}

/**
 * @brief Worker thread that serializes all LoRa transmissions.
 *
 * @param p1 Adapter instance pointer.
 * @param p2 Unused.
 * @param p3 Unused.
 */
static void lora_tx_thread_fn(void *p1, void *p2, void *p3)
{
    lorawan_adapter_t *self = p1;
    ARG_UNUSED(p2);
    ARG_UNUSED(p3);

    if (!self) {
        LOG_ERR("LoRa TX worker started with NULL adapter");
        return;
    }

    while (1) {
        lora_tx_req_t req;
        int err = k_msgq_get(&s_lora_txq, &req, K_FOREVER);
        if (err != 0) {
            LOG_ERR("Failed to get LoRa TX request from queue: %d", err);
            continue;
        }

        err = lora_tx(self, req.buf, req.len);
        if (err < 0) {
            LOG_ERR("Queued LoRa TX failed: %d", err);
        }
    }
}

/* ─────────────────────────────────────────────────────────────
 * Public setup / access functions
 * ───────────────────────────────────────────────────────────── */

/**
 * @brief Initialize the static LoRa adapter instance.
 *
 * Must be called before @ref lorawan_adapter_get or
 * @ref lorawan_adapter_start.
 */
void lorawan_adapter_setup(void)
{
    lorawan_adapter_obj_init(&s_lora, lora_emit_to_ingest, NULL);
}

/**
 * @brief Get the generic gateway adapter interface for the LoRa adapter.
 *
 * @return Pointer to the embedded gateway adapter base object.
 */
gw_adapter_t *lorawan_adapter_get(void)
{
    lorawan_require_setup();
    return &s_lora.base;
}

/**
 * @brief Return the timestamp of the last received LoRa packet.
 *
 * @param base Pointer to generic gateway adapter base.
 *
 * @return Timestamp in milliseconds since boot.
 */
static int64_t lora_last_rx_ms(gw_adapter_t *base)
{
    lorawan_adapter_t *self = CONTAINER_OF(base, lorawan_adapter_t, base);
    return self->last_rx_ms;
}

/* ─────────────────────────────────────────────────────────────
 * RX path
 * ───────────────────────────────────────────────────────────── */

/**
 * @brief Asynchronous LoRa receive callback.
 *
 * Parses incoming wire data into a gateway event, enriches it with RSSI/SNR
 * metadata and forwards it via the adapter emit callback.
 *
 * @param dev        LoRa device.
 * @param data       Received payload buffer.
 * @param size       Payload size in bytes.
 * @param rssi       Received signal strength.
 * @param snr        Signal-to-noise ratio.
 * @param user_data  Adapter instance pointer.
 */
static void lora_rx_cb(const struct device *dev, uint8_t *data, uint16_t size,
                       int16_t rssi, int8_t snr, void *user_data)
{
    ARG_UNUSED(dev);

    lorawan_adapter_t *self = user_data;
    if (!self || !data || size == 0) {
        return;
    }

    self->last_rx_ms = k_uptime_get();

    gw_event_t evt = {0};
    if (!wire_parse(data, size, &evt)) {
        LOG_WRN("Invalid LoRa packet (len %d rssi %d)", size, rssi);
        return;
    }

    evt.rx_ms = self->last_rx_ms;
    evt.rx_meta.has_rssi   = true;
    evt.rx_meta.rssi_dbm   = (int8_t)rssi;
    evt.rx_meta.has_snr    = true;
    evt.rx_meta.snr_db_x10 = (int16_t)(snr * 10);

    lora_emit(self, &evt);
}

/* ─────────────────────────────────────────────────────────────
 * TX path
 * ───────────────────────────────────────────────────────────── */

/**
 * @brief Send a raw LoRa payload.
 *
 * The function temporarily stops asynchronous RX, switches the modem into TX
 * mode, transmits the payload and then restores RX mode again.
 *
 * @param self  Adapter instance.
 * @param buf   Payload buffer.
 * @param len   Payload length in bytes.
 *
 * @retval 0 on success
 * @retval <0 Zephyr error code on failure
 */
static int lora_tx(lorawan_adapter_t *self, const uint8_t *buf, uint8_t len)
{
    int err;

    if (!self->dev) {
        return -ENODEV;
    }
    if (len == 0) {
        return -EINVAL;
    }

    k_mutex_lock(&self->radio_mutex, K_FOREVER);

    LOG_INF("LoRa TX: %d bytes", len);

    /* Stop RX to avoid self-reception and free the radio for TX. */
    err = lora_recv_async(self->dev, NULL, NULL);
    if (err < 0) {
        LOG_ERR("lora_recv_async(NULL) failed: %d", err);
        goto out;
    }

    /* Reconfigure modem for TX. */
    err = lora_set_mode(self, true);
    if (err < 0) {
        goto restart_rx;
    }

    /* Transmit payload. */
    err = lora_send(self->dev, (uint8_t *)buf, len);
    if (err < 0) {
        LOG_ERR("lora_send failed: %d (len=%u)", err, len);
        goto restart_rx;
    }

    LOG_INF("LoRa TX OK: %u bytes", len);

restart_rx:
    /* Restore RX mode and restart asynchronous reception. */
    {
        int err2 = lora_set_mode(self, false);
        if (err2 < 0) {
            LOG_ERR("lora_set_mode(RX) failed: %d", err2);
            if (err == 0) {
                err = err2;
            }
        }

        int err3 = lora_recv_async(self->dev, lora_rx_cb, self);
        if (err3 < 0) {
            LOG_ERR("lora_recv_async(restart) failed: %d", err3);
            if (err == 0) {
                err = err3;
            }
        }
    }

out:
    k_mutex_unlock(&self->radio_mutex);
    return err;
}

/* ─────────────────────────────────────────────────────────────
 * Adapter startup
 * ───────────────────────────────────────────────────────────── */

/**
 * @brief Start the LoRa adapter and enable asynchronous reception.
 *
 * This function:
 * - resolves the LoRa device from devicetree
 * - configures the modem into RX mode
 * - starts asynchronous reception
 * - registers the relay listener with the ingest pipeline
 *
 * @retval 0 on success
 * @retval -EALREADY if the adapter was already started
 * @retval -ENODEV if the LoRa device is not ready
 * @retval <0 other Zephyr error code on failure
 */
int lorawan_adapter_start(void)
{
    int err;

    lorawan_require_setup();

    if (s_lora.dev != NULL) {
        LOG_WRN("LoRa adapter already started");
        return -EALREADY;
    }

    s_lora.dev = DEVICE_DT_GET(DT_NODELABEL(lora0));
    if (!device_is_ready(s_lora.dev)) {
        LOG_ERR("LoRa device not found");
        s_lora.dev = NULL;
        return -ENODEV;
    }

    /* Start in RX mode. */
    err = lora_set_mode(&s_lora, false);
    if (err < 0) {
        s_lora.dev = NULL;
        return err;
    }

    err = lora_recv_async(s_lora.dev, lora_rx_cb, &s_lora);
    if (err < 0) {
        LOG_ERR("lora_recv_async failed: %d", err);
        s_lora.dev = NULL;
        return err;
    }

    event_ingest_register_listener(lora_relay_listener, NULL);

    k_thread_create(&s_lora.tx_tid,
                    s_lora_tx_stack,
                    LORA_TX_STACK_SIZE,
                    lora_tx_thread_fn,
                    &s_lora,
                    NULL,
                    NULL,
                    LORA_TX_PRIORITY,
                    0,
                    K_NO_WAIT);

    k_thread_name_set(&s_lora.tx_tid, "lora_tx");

    LOG_INF("LoRaWAN adapter initialized (%.1f MHz SF %d BW125)",
            LORA_FREQ / 1e6, LORA_SF);

    return 0;
}

/* ─────────────────────────────────────────────────────────────
 * Relay path
 * ───────────────────────────────────────────────────────────── */

/**
 * @brief Relay non-LoRa events over the LoRa transport.
 *
 * Events that already originated from LoRa are ignored to prevent looping.
 *
 * @param evt  Event to relay.
 * @param ctx  Unused listener context.
 */
static void lora_relay_listener(const gw_event_t *evt, void *ctx)
{
    ARG_UNUSED(ctx);

    if (!s_lora.enabled) {
        return;
    }

    if (evt->src.transport == GW_TR_LORAWAN) {
        return;
    }

    uint8_t buf[WIRE_MAX_SIZE];
    int len = wire_build(evt, buf, sizeof(buf));
    if (len <= 0) {
        LOG_WRN("LoRa serialize failed: %d", len);
        return;
    }

    if (len > 255) {
        LOG_WRN("LoRa payload too large: %d", len);
        return;
    }

    int err = lora_enqueue_tx(&s_lora, buf, (uint8_t)len);
    if (err < 0) {
        LOG_ERR("Failed to queue LoRa relay TX: %d", err);
    }
}

/* ─────────────────────────────────────────────────────────────
 * Command encoding / send API
 * ───────────────────────────────────────────────────────────── */

/**
 * @brief Encode a gateway command into a 2-byte LoRa payload.
 *
 * @param cmd  Command to encode.
 * @param buf  Output buffer with size 2.
 *
 * @retval 0 on success
 * @retval -EINVAL if input is invalid or command is unsupported
 * @retval -ENOTSUP if toggle command is not resolved here
 */
static int lora_cmd_bytes(const gw_command_t *cmd, uint8_t buf[2])
{
    if (!cmd || !buf) {
        return -EINVAL;
    }

    switch (cmd->type) {
    case GW_CMD_LIGHT_ON:
        buf[0] = 0x10;
        buf[1] = 0x01;
        return 0;

    case GW_CMD_LIGHT_OFF:
        buf[0] = 0x10;
        buf[1] = 0x00;
        return 0;

    case GW_CMD_LIGHT_TOGGLE:
        LOG_WRN("TOGGLE not resolved — use command_router");
        return -ENOTSUP;

    default:
        return -EINVAL;
    }
}

/**
 * @brief Send a gateway command over the LoRa transport.
 *
 * @param base  Generic gateway adapter base pointer.
 * @param cmd   Command to send.
 *
 * @retval 0 on success
 * @retval <0 Zephyr or adapter-specific error code on failure
 */
static int lora_send_cmd(gw_adapter_t *base, const gw_command_t *cmd)
{
    lorawan_adapter_t *self = CONTAINER_OF(base, lorawan_adapter_t, base);

    if (!cmd || cmd->dst.transport != GW_TR_LORAWAN) {
        return -EINVAL;
    }

    uint8_t buf[2];
    int err = lora_cmd_bytes(cmd, buf);
    if (err < 0) {
        return err;
    }

    return lora_enqueue_tx(self, buf, 2);
}

/* ─────────────────────────────────────────────────────────────
 * Public control API
 * ───────────────────────────────────────────────────────────── */

/**
 * @brief Enable or disable event relaying over LoRa.
 *
 * @param enabled true to enable relaying, false to disable it.
 */
void lorawan_adapter_set_enabled(bool enabled)
{
    s_lora.enabled = enabled;
    LOG_INF("LoRa adapter %s", enabled ? "ENABLED" : "DISABLED");
}