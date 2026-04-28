/**
 * @file thread_adapter.c
 * @brief Thread / CoAP adapter for the generic gateway abstraction.
 *
 * This module integrates a Thread-based CoAP transport into the generic
 * gateway adapter interface (`gw_adapter_t`). It supports:
 * - receiving CoAP sensor updates from Thread nodes
 * - forwarding parsed events into the ingest pipeline
 * - sending light commands as CoAP PUT requests
 * - applying a static OpenThread operational dataset
 */

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/net/socket.h>
#include <zephyr/net/coap.h>
#include <zephyr/net/net_if.h>
#include <zephyr/net/openthread.h>

#include <openthread/dataset.h>
#include <openthread/ip6.h>
#include <openthread/thread.h>

#include <errno.h>
#include <string.h>
#include <stdatomic.h>

#include "thread_adapter.h"
#include "thread_codec.h"
#include "event_ingest.h"
#include "scheduler.h"

LOG_MODULE_REGISTER(thread_adapter, LOG_LEVEL_INF);

/* ─────────────────────────────────────────────────────────────
 * Configuration constants
 * ───────────────────────────────────────────────────────────── */

/** @brief UDP port used for CoAP communication. */
#define COAP_PORT      5683

/** @brief RX thread stack size in bytes. */
#define RX_STACK_SIZE  4096

/** @brief TX thread stack size in bytes. */
#define TX_STACK_SIZE  4096

/** @brief RX thread priority. */
#define RX_PRIORITY    10

/** @brief TX thread priority. */
#define TX_PRIORITY    7

/** @brief General receive / parse buffer size in bytes. */
#define BUF_SIZE       256

/** @brief Static Thread channel. */
#define THREAD_CHANNEL 11

/** @brief Static Thread PAN ID. */
#define THREAD_PANID   0xABCD

/** @brief Static Thread network name. */
#define THREAD_NET_NAME "NRFGate"

/* ─────────────────────────────────────────────────────────────
 * Static Thread dataset
 * ───────────────────────────────────────────────────────────── */

/** @brief Static Thread network key. */
static const uint8_t thread_net_key[16] = {
    0x00, 0x11, 0x22, 0x33, 0x44, 0x55, 0x66, 0x77,
    0x88, 0x99, 0xaa, 0xbb, 0xcc, 0xdd, 0xee, 0xff,
};

/** @brief Static Thread Extended PAN ID. */
static const uint8_t thread_ext_pan_id[8] = {
    0xde, 0xad, 0xbe, 0xef, 0xca, 0xfe, 0xba, 0xbe,
};

/* ─────────────────────────────────────────────────────────────
 * Internal types
 * ───────────────────────────────────────────────────────────── */

/**
 * @brief Concrete Thread adapter instance.
 *
 * This structure embeds the generic gateway adapter base and stores all
 * Thread / CoAP-specific runtime state.
 */
typedef struct {
    /** @brief Generic gateway adapter base object. */
    gw_adapter_t base;

    /** @brief Timestamp of the last received packet in ms. */
    int64_t last_rx_ms;

    /** @brief Bound UDP socket used for CoAP reception. */
    int sock;

    /** @brief Indicates whether RX/TX worker threads should keep running. */
    bool running;

    /** @brief Command sequence counter for queued TX commands. */
    atomic_t cmd_seq;

    /** @brief CoAP message ID counter. */
    atomic_t msg_id;

    /** @brief RX worker thread object. */
    struct k_thread rx_tid;

    /** @brief TX worker thread object. */
    struct k_thread tx_tid;
} thread_adapter_t;

/**
 * @brief Queued CoAP light command.
 */
struct coap_cmd {
    /** @brief Destination IPv6 address in binary form. */
    uint8_t ipv6[GW_IPV6_BIN_LEN];

    /** @brief Desired light state. */
    bool on;

    /** @brief Internal sequence number for logging. */
    uint8_t seq;

    /** @brief CoAP message ID to use for this command. */
    uint32_t cmd_id;
};

/* ─────────────────────────────────────────────────────────────
 * Static module state
 * ───────────────────────────────────────────────────────────── */

/** @brief Single static Thread adapter instance. */
static thread_adapter_t s_thread;

/* ─────────────────────────────────────────────────────────────
 * ACK signaling: RX thread signals TX thread when matching ACK arrives.
 * TX thread never reads from the socket directly — only the RX thread does.
 * ───────────────────────────────────────────────────────────── */

/** @brief Semaphore signaled by RX thread when pending ACK is received. */
static K_SEM_DEFINE(s_ack_sem, 0, 1);

/** @brief Mutex protecting s_pending_mid. */
static K_MUTEX_DEFINE(s_ack_lock);

/** @brief Message ID the TX thread is currently waiting for. 0 = none. */
static uint16_t s_pending_mid = 0;

/* ─────────────────────────────────────────────────────────────
 * Forward declarations
 * ───────────────────────────────────────────────────────────── */

static void thread_adapter_obj_init(thread_adapter_t *self,
                                    gw_adapter_emit_fn emit,
                                    void *emit_user);
static void thread_require_setup(void);

static void thread_emit(thread_adapter_t *self, const gw_event_t *evt);
static void thread_emit_to_ingest(const gw_event_t *evt, void *user);
static void thread_codec_emit_to_adapter(const gw_event_t *evt, void *user);

static int thread_send_cmd(gw_adapter_t *base, const gw_command_t *cmd);
static int64_t thread_last_rx_ms(gw_adapter_t *base);
static gw_send_mode_t thread_send_mode(gw_adapter_t *base);

static int sensors_post(struct coap_resource *resource,
                        struct coap_packet *request,
                        struct sockaddr *addr,
                        socklen_t addr_len);

static void coap_rx_thread(void *p1, void *p2, void *p3);
static void coap_tx_thread_fn(void *p1, void *p2, void *p3);

static bool coap_send_con(thread_adapter_t *self, 
                          const uint8_t *ipv6,
                          bool on,
                          uint16_t msg_id,
                          int ack_timeout_ms);

static int apply_thread_dataset(void);
static int thread_onoff_from_cmd(const gw_command_t *cmd, bool *on);

/* ─────────────────────────────────────────────────────────────
 * Adapter API table
 * ───────────────────────────────────────────────────────────── */

/** @brief Vtable for the generic gateway adapter interface. */
static const gw_adapter_api_t s_thread_api = {
    .send_cmd   = thread_send_cmd,
    .last_rx_ms = thread_last_rx_ms,
    .send_mode  = thread_send_mode,
};

/* ─────────────────────────────────────────────────────────────
 * Queues and stacks
 * ───────────────────────────────────────────────────────────── */

/** @brief TX command queue for outgoing CoAP requests. */
K_MSGQ_DEFINE(s_coap_cmdq, sizeof(struct coap_cmd), 8, 4);

/** @brief RX thread stack. */
K_THREAD_STACK_DEFINE(s_rx_stack, RX_STACK_SIZE);

/** @brief TX thread stack. */
K_THREAD_STACK_DEFINE(s_tx_stack, TX_STACK_SIZE);

/* ─────────────────────────────────────────────────────────────
 * CoAP resources
 * ───────────────────────────────────────────────────────────── */

/** @brief URI path for sensor POST requests. */
static const char *const sensors_path[] = {
    "sensors",
    NULL
};

/** @brief Static CoAP resource table. */
static struct coap_resource resources[] = {
    {
        .get  = NULL,
        .post = sensors_post,
        .put  = NULL,
        .del  = NULL,
        .path = sensors_path,
        .user_data = &s_thread,
    },
    { .path = NULL },
};

/* ─────────────────────────────────────────────────────────────
 * Internal initialization helpers
 * ───────────────────────────────────────────────────────────── */

/**
 * @brief Initialize a Thread adapter object.
 *
 * @param self       Adapter instance to initialize.
 * @param emit       Event callback used by the adapter.
 * @param emit_user  User context passed to @p emit.
 */
static void thread_adapter_obj_init(thread_adapter_t *self,
                                    gw_adapter_emit_fn emit,
                                    void *emit_user)
{
    memset(self, 0, sizeof(*self));

    self->base.api       = &s_thread_api;
    self->base.emit      = emit;
    self->base.emit_user = emit_user;

    self->sock = -1;
    self->running = false;

    atomic_set(&self->cmd_seq, 0);
    atomic_set(&self->msg_id, 1);
}

/**
 * @brief Ensure the Thread adapter was initialized before use.
 */
static void thread_require_setup(void)
{
    __ASSERT(s_thread.base.api != NULL, "Thread adapter used before setup");
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
static void thread_emit(thread_adapter_t *self, const gw_event_t *evt)
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
static void thread_emit_to_ingest(const gw_event_t *evt, void *user)
{
    ARG_UNUSED(user);
    event_ingest_submit(evt);
}

/**
 * @brief Callback wrapper used by the Thread codec.
 *
 * @param evt   Parsed gateway event.
 * @param user  Adapter instance pointer.
 */
static void thread_codec_emit_to_adapter(const gw_event_t *evt, void *user)
{
    thread_adapter_t *self = user;
    if (!self) {
        return;
    }

    thread_emit(self, evt);
}

/**
 * @brief Return the timestamp of the last received Thread packet.
 *
 * @param base Pointer to the generic gateway adapter base.
 *
 * @return Timestamp in milliseconds since boot.
 */
static int64_t thread_last_rx_ms(gw_adapter_t *base)
{
    thread_adapter_t *self = CONTAINER_OF(base, thread_adapter_t, base);
    return self->last_rx_ms;
}

/**
 * @brief Report the synchronicity of the Thread/CoAP send path.
 *
 * @param base Generic gateway adapter base pointer (unused).
 *
 * @return GW_SEND_QUEUED — thread_send_cmd() places the request into the
 *         CoAP TX queue. The actual UDP send and CoAP ACK round-trip happen
 *         later in the dedicated TX worker thread, so a successful send_cmd()
 *         return only means "queued", not "delivered".
 */
static gw_send_mode_t thread_send_mode(gw_adapter_t *base)
{
    ARG_UNUSED(base);
    return GW_SEND_QUEUED;
}

/* ─────────────────────────────────────────────────────────────
 * CoAP RX path
 * ───────────────────────────────────────────────────────────── */

/**
 * @brief Handle incoming CoAP POST requests on `/sensors`.
 *
 * The function acknowledges the request immediately, extracts the sender IPv6
 * address, filters discovery packets and forwards valid sensor JSON payloads
 * to the Thread codec.
 *
 * @param resource  Matched CoAP resource.
 * @param request   Parsed CoAP request.
 * @param addr      Source address.
 * @param addr_len  Source address length.
 *
 * @retval 0 on success
 * @retval -EINVAL if the request payload is missing or invalid
 */
static int sensors_post(struct coap_resource *resource,
                        struct coap_packet *request,
                        struct sockaddr *addr,
                        socklen_t addr_len)
{
    thread_adapter_t *self = resource ? resource->user_data : NULL;
    if (!self) {
        LOG_ERR("sensors_post: adapter instance is NULL");
        return -EINVAL;
    }

    self->last_rx_ms = k_uptime_get();

    /* ACK immediately. */
    uint8_t token[COAP_TOKEN_MAX_LEN];
    uint8_t token_len = coap_header_get_token(request, token);
    uint8_t ack_buf[16];
    struct coap_packet ack;

    coap_packet_init(&ack,
                     ack_buf,
                     sizeof(ack_buf),
                     COAP_VERSION_1,
                     COAP_TYPE_ACK,
                     token_len,
                     token,
                     COAP_RESPONSE_CODE_CHANGED,
                     coap_header_get_id(request));

    zsock_sendto(self->sock, ack_buf, ack.offset, 0, addr, addr_len);

    /* Extract source IPv6 for event metadata and logging. */
    char ipv6[GW_IPV6_STR_LEN];
    struct sockaddr_in6 *src6 = (struct sockaddr_in6 *)addr;
    net_addr_ntop(AF_INET6, &src6->sin6_addr, ipv6, sizeof(ipv6));

    /* Read payload. */
    uint16_t payload_len;
    const uint8_t *payload = coap_packet_get_payload(request, &payload_len);
    if (!payload || payload_len == 0) {
        LOG_WRN("No payload in CoAP request from %s", ipv6);
        return -EINVAL;
    }

    /* Ignore discovery packets. */
    if (payload_len > 11) {
        if (strncmp((const char *)payload, "{\"discover\"", 11) == 0) {
            LOG_DBG("Discovery packet from %s — ignored", ipv6);
            return 0;
        }
    }

    /* Copy payload into a temporary null-terminated JSON buffer. */
    char json_buf[BUF_SIZE];
    size_t copy_len = MIN(payload_len, sizeof(json_buf) - 1);
    memcpy(json_buf, payload, copy_len);
    json_buf[copy_len] = '\0';

    int ret = thread_codec_parse_sensor_json(json_buf,
                                             copy_len,
                                             src6->sin6_addr.s6_addr,
                                             self->last_rx_ms,
                                             thread_codec_emit_to_adapter,
                                             self);
    if (ret < 0) {
        LOG_WRN("Failed to parse JSON from %s: %d", ipv6, ret);
        return -EINVAL;
    }

    LOG_INF("CoAP POST /sensors from %s (%u bytes)", ipv6, payload_len);
    return 0;
}

/**
 * @brief RX worker thread for incoming CoAP packets.
 *
 * This thread waits until the default network interface is up, then listens on
 * the bound socket, parses CoAP packets and dispatches them to the resource
 * table.
 *
 * @param p1 Adapter instance pointer.
 * @param p2 Unused.
 * @param p3 Unused.
 */
static void coap_rx_thread(void *p1, void *p2, void *p3)
{
    thread_adapter_t *self = p1;
    ARG_UNUSED(p2);
    ARG_UNUSED(p3);

    if (!self) {
        LOG_ERR("Thread adapter instance is NULL");
        return;
    }

    uint8_t buf[BUF_SIZE];
    struct sockaddr_in6 src;
    socklen_t src_len = sizeof(src);
    struct coap_packet request;

    /* Wait for the default network interface to come up. */
    struct net_if *iface = net_if_get_default();
    if (!iface) {
        LOG_ERR("No network interface found");
        return;
    }

    while (!net_if_is_up(iface)) {
        LOG_INF("Waiting for Thread interface...");
        k_sleep(K_SECONDS(1));
    }

    LOG_INF("Network up — listening for CoAP on port %d", COAP_PORT);

    while (self->running) {
        src_len = sizeof(src);
        ssize_t received = zsock_recvfrom(self->sock,
                                          buf,
                                          sizeof(buf),
                                          0,
                                          (struct sockaddr *)&src,
                                          &src_len);

        if (received < 0) {
            if (errno == EAGAIN || errno == EWOULDBLOCK) {
                k_msleep(10);
                continue;
            }

            LOG_ERR("recvfrom error: %d", errno);
            continue;
        }

        int ret = coap_packet_parse(&request, buf, received, NULL, 0);
        if (ret < 0) {
            LOG_WRN("Failed to parse CoAP packet: %d", ret);
            continue;
        }

        uint8_t coap_type = coap_header_get_type(&request);
        if (coap_type == COAP_TYPE_ACK || coap_type == COAP_TYPE_RESET) {
            uint16_t coap_id = coap_header_get_id(&request);

            k_mutex_lock(&s_ack_lock, K_FOREVER);
            bool is_our_ack = (s_pending_mid != 0 && coap_id == s_pending_mid);
            k_mutex_unlock(&s_ack_lock);

            if (is_our_ack) {
                LOG_DBG("Received ACK for MID %u", coap_id);
                k_sem_give(&s_ack_sem);
                continue;
            } else {
                LOG_DBG("Stray ACK/RST mid=%u (pending=%u)", coap_id, s_pending_mid);
                continue;
            }
            continue;
        }

        struct coap_option options[4];
        uint8_t opt_num = coap_find_options(&request,
                                            COAP_OPTION_URI_PATH,
                                            options,
                                            ARRAY_SIZE(options));

        ret = coap_handle_request(&request,
                                  resources,
                                  options,
                                  opt_num,
                                  (struct sockaddr *)&src,
                                  src_len);

        /* -ENOENT is normal for MPL duplicates — ignore silently. */
        if (ret < 0 && ret != -ENOENT) {
            LOG_WRN("CoAP dispatch error: %d", ret);
        }
    }
}

/* ─────────────────────────────────────────────────────────────
 * CoAP TX path
 * ───────────────────────────────────────────────────────────── */

/**
 * @brief Send a confirmable CoAP PUT request and wait for its ACK.
 *
 * @param ipv6            Destination IPv6 address in binary form.
 * @param on              Desired light state.
 * @param msg_id          CoAP message ID.
 * @param ack_timeout_ms  Poll timeout for the ACK in milliseconds.
 *
 * @return true if an ACK was received, false otherwise.
 */
static bool coap_send_con(thread_adapter_t *self,
                          const uint8_t *ipv6,
                          bool on,
                          uint16_t msg_id,
                          int ack_timeout_ms)
{
    if (!self || self->sock < 0 || !ipv6) {
        LOG_ERR("Invalid parameters for coap_send_con");
        return false;
    }

    uint8_t pkt_buf[128];
    struct coap_packet pkt;

    int r = coap_packet_init(&pkt,
                             pkt_buf,
                             sizeof(pkt_buf),
                             COAP_VERSION_1,
                             COAP_TYPE_CON,
                             COAP_TOKEN_MAX_LEN,
                             coap_next_token(),
                             COAP_METHOD_PUT,
                             msg_id);
    if (r < 0) {
        LOG_ERR("pkt_init failed: %d", r);
        return false;
    }

    r = coap_packet_append_option(&pkt,
                                  COAP_OPTION_URI_PATH,
                                  "light",
                                  strlen("light"));
    if (r < 0) {
        LOG_ERR("URI option failed: %d", r);
        return false;
    }

    uint8_t fmt = COAP_CONTENT_FORMAT_APP_JSON;
    r = coap_packet_append_option(&pkt,
                                  COAP_OPTION_CONTENT_FORMAT,
                                  &fmt,
                                  sizeof(fmt));
    if (r < 0) {
        LOG_ERR("Content format option failed: %d", r);
        return false;
    }

    r = coap_packet_append_payload_marker(&pkt);
    if (r < 0) {
        LOG_ERR("payload marker failed: %d", r);
        return false;
    }

    const char *body = on ? "{\"on\":true}" : "{\"on\":false}";
    r = coap_packet_append_payload(&pkt,
                                   (const uint8_t *)body,
                                   strlen(body));
    if (r < 0) {
        LOG_ERR("payload append failed: %d", r);
        return false;
    }

    struct sockaddr_in6 dst = {
        .sin6_family = AF_INET6,
        .sin6_port   = htons(COAP_PORT),
    };

    memcpy(&dst.sin6_addr, ipv6, GW_IPV6_BIN_LEN);

    /* Arm ACK waiter BEFORE sending, so we don't miss a fast ACK. */
    k_mutex_lock(&s_ack_lock, K_FOREVER);
    s_pending_mid = msg_id;
    k_mutex_unlock(&s_ack_lock);
    k_sem_reset(&s_ack_sem);

    r = zsock_sendto(self->sock,
                     pkt.data,
                     pkt.offset,
                     0,
                     (struct sockaddr *)&dst,
                     sizeof(dst));
    if (r < 0) {
        LOG_ERR("sendto failed: %d", errno);
            /* Disarm */
        k_mutex_lock(&s_ack_lock, K_FOREVER);
        s_pending_mid = 0;
        k_mutex_unlock(&s_ack_lock);
        return false;
    }

    /* Wait for RX thread to signal matching ACK */
    int sr = k_sem_take(&s_ack_sem, K_FOREVER);

    /* Disarm ACK waiter */
    k_mutex_lock(&s_ack_lock, K_FOREVER);
    s_pending_mid = 0;
    k_mutex_unlock(&s_ack_lock);

    return (sr == 0);
}

/**
 * @brief TX worker thread for queued CoAP commands.
 *
 * This thread dequeues outgoing light commands, requests Thread scheduling
 * priority, performs up to four send attempts and emits an actuator state event
 * locally when the command succeeds.
 *
 * @param p1 Adapter instance pointer.
 * @param p2 Unused.
 * @param p3 Unused.
 */
static void coap_tx_thread_fn(void *p1, void *p2, void *p3)
{
    thread_adapter_t *self = p1;
    ARG_UNUSED(p2);
    ARG_UNUSED(p3);

    if (!self) {
        LOG_ERR("coap_tx_thread_fn: self is NULL");
        return;
    }

    while (self->running) {
        struct coap_cmd cmd;
        int ret = k_msgq_get(&s_coap_cmdq, &cmd, K_FOREVER);
        if (ret != 0) {
            LOG_ERR("Failed to get command from queue: %d", ret);
            continue;
        }

        char ipv6_str[GW_IPV6_STR_LEN];
        struct in6_addr a;
        memcpy(a.s6_addr, cmd.ipv6, GW_IPV6_BIN_LEN);
        net_addr_ntop(AF_INET6, &a, ipv6_str, sizeof(ipv6_str));

        LOG_INF("CoAP TX: [%s] %s (seq=%u)",
                ipv6_str,
                cmd.on ? "ON" : "OFF",
                cmd.seq);

        bool success = false;

        for (int attempt = 0; attempt < 4 && !success; attempt++) {
            if (attempt > 0) {
                LOG_WRN("CoAP retry %d/4 — waiting for Thread window", attempt);
                k_msleep(200);
            }

            scheduler_request_priority(SCHED_PRIORITY_THREAD, 700);

            uint16_t mid = (uint16_t)atomic_inc(&self->msg_id);
            success = coap_send_con(self, cmd.ipv6, cmd.on, mid, 500);

            if (success) {
                LOG_INF("CoAP PUT /light [%s] %s OK",
                        ipv6_str,
                        cmd.on ? "ON" : "OFF");

                gw_event_t evt = {
                    .type  = GW_EVT_ACTUATOR_STATE,
                    .rx_ms = k_uptime_get(),
                    .src   = {
                        .transport = GW_TR_THREAD,
                    },
                    .data.actuator_state = {
                        .light_on = cmd.on,
                    },
                };

                memcpy(evt.src.ipv6, cmd.ipv6, GW_IPV6_BIN_LEN);
                thread_emit(self, &evt);
            } else {
                LOG_WRN("No ACK for msg_id=%u (attempt %d)", mid, attempt + 1);
            }
        }
        if (!success) {
            gw_event_t evt = {
                .type  = GW_EVT_TIMEOUT,
                .rx_ms = k_uptime_get(),
                .src   = {
                    .transport = GW_TR_THREAD,
                },
                .data.timeout = {
                    .cmd_id = cmd.cmd_id,
                },
            };
            memcpy(evt.src.ipv6, cmd.ipv6, GW_IPV6_BIN_LEN);
            thread_emit(self, &evt);
        }

        // /* Brief yield between commands to let OT stack settle, */
        // /* prevents queue-burst causing back-to-back priority requests. */
        // k_msleep(100);
    }
}

/* ─────────────────────────────────────────────────────────────
 * Thread dataset helpers
 * ───────────────────────────────────────────────────────────── */

/**
 * @brief Apply the static OpenThread operational dataset and enable Thread.
 *
 * @retval 0 on success
 * @retval -ENODEV if the OpenThread instance is unavailable
 */
static int apply_thread_dataset(void)
{
    otInstance *ot = openthread_get_default_instance();
    if (!ot) {
        LOG_ERR("OpenThread instance not available");
        return -ENODEV;
    }

    otOperationalDataset dataset = {0};

    dataset.mActiveTimestamp.mSeconds             = 1;
    dataset.mComponents.mIsActiveTimestampPresent = true;

    dataset.mChannel                              = THREAD_CHANNEL;
    dataset.mComponents.mIsChannelPresent         = true;

    dataset.mPanId                                = THREAD_PANID;
    dataset.mComponents.mIsPanIdPresent           = true;

    memcpy(dataset.mNetworkKey.m8, thread_net_key, sizeof(thread_net_key));
    dataset.mComponents.mIsNetworkKeyPresent      = true;

    memcpy(dataset.mExtendedPanId.m8,
           thread_ext_pan_id,
           sizeof(thread_ext_pan_id));
    dataset.mComponents.mIsExtendedPanIdPresent   = true;

    strcpy((char *)dataset.mNetworkName.m8, THREAD_NET_NAME);
    dataset.mComponents.mIsNetworkNamePresent     = true;

    openthread_mutex_lock();
    otDatasetSetActive(ot, &dataset);
    otIp6SetEnabled(ot, true);
    otThreadSetEnabled(ot, true);
    openthread_mutex_unlock();

    LOG_INF("Thread dataset applied: ch=%d, panid=0x%04X, name=%s",
            THREAD_CHANNEL,
            THREAD_PANID,
            THREAD_NET_NAME);

    return 0;
}

/* ─────────────────────────────────────────────────────────────
 * Public setup / startup API
 * ───────────────────────────────────────────────────────────── */

/**
 * @brief Initialize the static Thread adapter instance.
 */
void thread_adapter_setup(void)
{
    thread_adapter_obj_init(&s_thread, thread_emit_to_ingest, NULL);
}

/**
 * @brief Start the Thread adapter.
 *
 * This function applies the static Thread dataset, creates and binds the CoAP
 * socket, then starts the RX and TX worker threads.
 *
 * @retval 0 on success
 * @retval -EALREADY if the adapter is already running
 * @retval <0 Zephyr or POSIX-style error code on failure
 */
int thread_adapter_start(void)
{
    thread_require_setup();

    if (s_thread.running || s_thread.sock >= 0) {
        LOG_WRN("Thread adapter already started");
        return -EALREADY;
    }

    int err = apply_thread_dataset();
    if (err < 0) {
        return err;
    }

    struct sockaddr_in6 addr = {
        .sin6_family = AF_INET6,
        .sin6_port   = htons(COAP_PORT),
        .sin6_addr   = IN6ADDR_ANY_INIT,
    };

    s_thread.sock = zsock_socket(AF_INET6, SOCK_DGRAM, IPPROTO_UDP);
    if (s_thread.sock < 0) {
        LOG_ERR("Failed to open socket: %d", errno);
        return -errno;
    }

    if (zsock_bind(s_thread.sock, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
        LOG_ERR("Failed to bind socket: %d", errno);
        zsock_close(s_thread.sock);
        s_thread.sock = -1;
        return -errno;
    }

    s_thread.running = true;

    k_thread_create(&s_thread.rx_tid,
                    s_rx_stack,
                    RX_STACK_SIZE,
                    coap_rx_thread,
                    &s_thread,
                    NULL,
                    NULL,
                    RX_PRIORITY,
                    0,
                    K_NO_WAIT);
    k_thread_name_set(&s_thread.rx_tid, "coap_rx");

    k_thread_create(&s_thread.tx_tid,
                    s_tx_stack,
                    TX_STACK_SIZE,
                    coap_tx_thread_fn,
                    &s_thread,
                    NULL,
                    NULL,
                    TX_PRIORITY,
                    0,
                    K_NO_WAIT);
    k_thread_name_set(&s_thread.tx_tid, "coap_tx");

    LOG_INF("coap_rx thread=%p", &s_thread.rx_tid);
    LOG_INF("coap_tx thread=%p", &s_thread.tx_tid);
    LOG_INF("Thread adapter initialized (CoAP port %d)", COAP_PORT);
    return 0;
}

/**
 * @brief Get the generic gateway adapter interface for the Thread adapter.
 *
 * @return Pointer to the embedded gateway adapter base object.
 */
gw_adapter_t *thread_adapter_get(void)
{
    thread_require_setup();
    return &s_thread.base;
}

/* ─────────────────────────────────────────────────────────────
 * Command encoding / send API
 * ───────────────────────────────────────────────────────────── */

/**
 * @brief Convert a generic gateway command into a boolean light state.
 *
 * @param cmd  Gateway command.
 * @param on   Output light state.
 *
 * @retval 0 on success
 * @retval -EINVAL if the input is invalid or unsupported
 * @retval -ENOTSUP if toggle is not resolved here
 */
static int thread_onoff_from_cmd(const gw_command_t *cmd, bool *on)
{
    if (!cmd || !on) {
        return -EINVAL;
    }

    switch (cmd->type) {
    case GW_CMD_LIGHT_ON:
        *on = true;
        return 0;

    case GW_CMD_LIGHT_OFF:
        *on = false;
        return 0;

    case GW_CMD_LIGHT_TOGGLE:
        LOG_WRN("TOGGLE not resolved in adapter — use command_router");
        return -ENOTSUP;

    default:
        LOG_ERR("Unsupported command type: %d", cmd->type);
        return -EINVAL;
    }
}

/**
 * @brief Queue a gateway command for transmission over Thread / CoAP.
 *
 * @param base Generic gateway adapter base pointer.
 * @param cmd  Command to send.
 *
 * @retval 0 on success
 * @retval <0 adapter-specific error code on failure
 */
static int thread_send_cmd(gw_adapter_t *base, const gw_command_t *cmd)
{
    thread_adapter_t *self = CONTAINER_OF(base, thread_adapter_t, base);

    if (!cmd) {
        return -EINVAL;
    }

    if (cmd->dst.transport != GW_TR_THREAD) {
        return -EINVAL;
    }

    if (self->sock < 0) {
        LOG_ERR("Thread adapter not initialized");
        return -ENODEV;
    }

    struct coap_cmd c = {0};
    memcpy(c.ipv6, cmd->dst.ipv6, GW_IPV6_BIN_LEN);
    c.seq = (uint8_t)atomic_inc(&self->cmd_seq);
    c.cmd_id = cmd->cmd_id;

    bool on;
    int err = thread_onoff_from_cmd(cmd, &on);
    if (err) {
        return err;
    }

    c.on = on;

    int r = k_msgq_put(&s_coap_cmdq, &c, K_NO_WAIT);
    if (r < 0) {
        LOG_WRN("CoAP command queue full, dropping command");
        return -ENOSPC;
    }

    char ipv6_str[GW_IPV6_STR_LEN];
    struct in6_addr a;
    memcpy(a.s6_addr, c.ipv6, GW_IPV6_BIN_LEN);
    net_addr_ntop(AF_INET6, &a, ipv6_str, sizeof(ipv6_str));

    LOG_INF("Command queued for %s: %s (seq=%u)",
            ipv6_str,
            c.on ? "ON" : "OFF",
            c.seq);

    return 0;
}