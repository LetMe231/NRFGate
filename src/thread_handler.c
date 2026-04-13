/**
 * @file thread_handler.c
 * @brief Thread/CoAP receiver for ESP32-C6 sensor data
 *
 * Listens on UDP port 5683 for CoAP NON POST /sensors messages
 * containing JSON-encoded IMU data from ESP32-C6 nodes.
 *
 * Expected JSON format:
 *   {"ts":12345,"ax":100,"ay":-50,"az":9810,"gx":0,"gy":0,"gz":0}
 *
 * Values are integers (float * 1000).
 */

#include <zephyr/kernel.h>
#include <zephyr/net/socket.h>
#include <zephyr/net/coap.h>
#include <zephyr/net/net_if.h>
#include <zephyr/data/json.h>
#include <zephyr/logging/log.h>
#include <zephyr/net/openthread.h>
#include <openthread/dataset.h>
#include <openthread/ip6.h>
#include <openthread/thread.h>
#include <stdint.h>

#include "thread_handler.h"
#include "main.h"
#include "data_handler.h"

LOG_MODULE_REGISTER(thread_handler, LOG_LEVEL_INF);

#define COAP_PORT  5683
#define STACK_SIZE 4096
#define PRIORITY   10
#define BUF_SIZE   256

#define COAP_PUT_RETRIES 3
#define COAP_PUT_RETRY_MS 400

static int sock;
static bool running;

K_THREAD_STACK_DEFINE(thread_rx_stack, STACK_SIZE);
static struct k_thread thread_rx_tid;

/* ── CoAP TX Thread — forward declarations (before JSON descr) ───── */
struct coap_cmd {
    char    ipv6[40];
    bool    on;
    uint8_t seq;
};

#define COAP_TX_STACK_SIZE 4096
#define COAP_TX_PRIORITY   7

K_MSGQ_DEFINE(s_coap_cmdq, sizeof(struct coap_cmd), 8, 4);
K_THREAD_STACK_DEFINE(s_coap_tx_stack, COAP_TX_STACK_SIZE);
static struct k_thread  s_coap_tx_tid;
static atomic_t s_cmd_seq = ATOMIC_INIT(0);

/* ── JSON Payload Descriptor ─────────────────────────────────── */

struct sensor_json {
    int32_t seq, ts;
    // IMU
    int32_t ax, ay, az;
    int32_t gx, gy, gz;
    // environmental data
    int32_t temp, hum;

    // air quality data
    int32_t tvoc, eco2;

    // Biometric data
    int32_t heart_rate, spo2;
    int32_t raw_red, raw_ir;

    int32_t pm25, pm10;   // µg/m3

    int32_t sw;          // generic switch input state (0=off, 1=on)
    int32_t light;       // generic light state (0=off, 1=on)
    // add more sensor data here
};

static const struct json_obj_descr sensor_descr[] = {
    JSON_OBJ_DESCR_PRIM(struct sensor_json, seq, JSON_TOK_NUMBER),
    JSON_OBJ_DESCR_PRIM(struct sensor_json, ts, JSON_TOK_NUMBER),
    JSON_OBJ_DESCR_PRIM(struct sensor_json, ax, JSON_TOK_NUMBER),
    JSON_OBJ_DESCR_PRIM(struct sensor_json, ay, JSON_TOK_NUMBER),
    JSON_OBJ_DESCR_PRIM(struct sensor_json, az, JSON_TOK_NUMBER),
    JSON_OBJ_DESCR_PRIM(struct sensor_json, gx, JSON_TOK_NUMBER),
    JSON_OBJ_DESCR_PRIM(struct sensor_json, gy, JSON_TOK_NUMBER),
    JSON_OBJ_DESCR_PRIM(struct sensor_json, gz, JSON_TOK_NUMBER),
    JSON_OBJ_DESCR_PRIM(struct sensor_json, temp, JSON_TOK_NUMBER),
    JSON_OBJ_DESCR_PRIM(struct sensor_json, hum, JSON_TOK_NUMBER),
    JSON_OBJ_DESCR_PRIM(struct sensor_json, tvoc, JSON_TOK_NUMBER),
    JSON_OBJ_DESCR_PRIM(struct sensor_json, eco2, JSON_TOK_NUMBER),
    JSON_OBJ_DESCR_PRIM(struct sensor_json, heart_rate, JSON_TOK_NUMBER),
    JSON_OBJ_DESCR_PRIM(struct sensor_json, spo2, JSON_TOK_NUMBER),
    JSON_OBJ_DESCR_PRIM(struct sensor_json, raw_red, JSON_TOK_NUMBER),
    JSON_OBJ_DESCR_PRIM(struct sensor_json, raw_ir, JSON_TOK_NUMBER),
    JSON_OBJ_DESCR_PRIM(struct sensor_json, pm25, JSON_TOK_NUMBER),
    JSON_OBJ_DESCR_PRIM(struct sensor_json, pm10, JSON_TOK_NUMBER),
    JSON_OBJ_DESCR_PRIM(struct sensor_json, sw, JSON_TOK_NUMBER),
    JSON_OBJ_DESCR_PRIM(struct sensor_json, light, JSON_TOK_NUMBER),
};

/* ── POST /sensors Handler ───────────────────────────────────── */

static int sensors_post(struct coap_resource *resource,
                        struct coap_packet *request,
                        struct sockaddr *addr, socklen_t addr_len)
{
    ARG_UNUSED(resource);

    uint8_t token[COAP_TOKEN_MAX_LEN];
    uint8_t token_len = coap_header_get_token(request, token);

    // Ack the request immediately to free up the sender, then do processing and forwarding asynchronously
    uint8_t ack_buf[16];
    struct coap_packet ack;

    coap_packet_init(&ack, ack_buf, sizeof(ack_buf),
                    COAP_VERSION_1,
                    COAP_TYPE_ACK,
                    token_len, token,                // echo the token back to correlate with request
                    COAP_RESPONSE_CODE_CHANGED,      // 2.04
                    coap_header_get_id(request));    // same Message-ID as request

    zsock_sendto(sock, ack_buf, ack.offset, 0, addr, addr_len);

    // Build Thread identity from source address
    node_identity_t id = { .transport = NODE_TRANSPORT_THREAD };
    struct sockaddr_in6 *src6 = (struct sockaddr_in6 *)addr;
    net_addr_ntop(AF_INET6, &src6->sin6_addr, id.ipv6, sizeof(id.ipv6));

    // Get CoAP payload
    uint16_t payload_len;
    const uint8_t *payload = coap_packet_get_payload(request, &payload_len);
    if (!payload || payload_len == 0) {
        LOG_WRN("No payload in CoAP request from %s", id.ipv6);
        return -EINVAL;
    }

    // Ignore discovery packets
    if (payload_len > 11){
        const char *pl = (const char *)payload;
        if (memchr(pl, 'd', payload_len) && strncmp(pl, "{\"discover\"", MIN(payload_len, 11)) == 0){
            LOG_INF("Ignoring discovery packet from %s", id.ipv6);
            return 0;
        }
    }


    // Parse JSON payload into sensor struct
    struct sensor_json s = {
        .seq        = -1,
        .ts         = -1,
        .ax         = INT32_MIN,
        .ay         = INT32_MIN,
        .az         = INT32_MIN,
        .gx         = INT32_MIN,
        .gy         = INT32_MIN,
        .gz         = INT32_MIN,
        .temp       = INT32_MIN,
        .hum        = INT32_MIN,
        .tvoc       = INT32_MIN,
        .eco2       = INT32_MIN,
        .heart_rate = INT32_MIN,
        .spo2       = INT32_MIN,
        .raw_red    = INT32_MIN,
        .raw_ir     = INT32_MIN,
        .pm25       = INT32_MIN,
        .pm10       = INT32_MIN,
        .sw         = -1,
        .light      = -1,
    };

    char json_buf[BUF_SIZE];
    size_t copy_len = MIN(payload_len, sizeof(json_buf) - 1);
    memcpy(json_buf, payload, copy_len);
    json_buf[copy_len] = '\0';

    int ret = json_obj_parse(json_buf, copy_len,
                            sensor_descr, ARRAY_SIZE(sensor_descr), &s);
    if (ret < 0) {
        LOG_WRN("Failed to parse JSON from %s: %d", id.ipv6, ret);
        return ret;
    }

    uint32_t present = 0;
    if (s.seq >= 0)                                              present |= SENSOR_HAS_SEQ;
    if (s.ts  >= 0)                                              present |= SENSOR_HAS_TS;
    if (s.ax != INT32_MIN && s.ay != INT32_MIN && s.az != INT32_MIN) present |= SENSOR_HAS_ACCEL;
    if (s.gx != INT32_MIN && s.gy != INT32_MIN && s.gz != INT32_MIN) present |= SENSOR_HAS_GYRO;
    if (s.temp       != INT32_MIN) present |= SENSOR_HAS_TEMP;
    if (s.hum        != INT32_MIN) present |= SENSOR_HAS_HUM;
    if (s.tvoc       != INT32_MIN) present |= SENSOR_HAS_TVOC;
    if (s.eco2       != INT32_MIN) present |= SENSOR_HAS_ECO2;
    if (s.heart_rate != INT32_MIN) present |= SENSOR_HAS_HEART_RATE;
    if (s.spo2       != INT32_MIN) present |= SENSOR_HAS_SPO2;
    if (s.raw_red    != INT32_MIN) present |= SENSOR_HAS_RAW_RED;
    if (s.raw_ir     != INT32_MIN) present |= SENSOR_HAS_RAW_IR;
    if (s.pm25       != INT32_MIN) present |= SENSOR_HAS_PM25;
    if (s.pm10       != INT32_MIN) present |= SENSOR_HAS_PM10;
    if (s.sw    == 0 || s.sw    == 1) present |= SENSOR_HAS_SWITCH;
    if (s.light == 0 || s.light == 1) present |= SENSOR_HAS_LIGHT;
    /* NOTE: present is uint32_t — no "< 0" check needed */
    
    struct node_sensor_data nd = {
        .identity = id,
        .payload = {
            .present    = present,
            .seq        = s.seq,
            .ts         = s.ts,
            .ax = s.ax, .ay = s.ay, .az = s.az,
            .gx = s.gx, .gy = s.gy, .gz = s.gz,
            .temp = s.temp, .hum = s.hum,
            .tvoc = s.tvoc, .eco2 = s.eco2,
            .heart_rate = s.heart_rate, .spo2 = s.spo2,
            .raw_red = s.raw_red, .raw_ir = s.raw_ir,
            .pm25 = s.pm25, .pm10 = s.pm10,
            .switch_state = s.sw,
            .light_on     = (s.light == 1),
        },
        .rx_uptime_ms = k_uptime_get(),
    };

    data_handler_receive(&nd);

    if (present & SENSOR_HAS_LIGHT) {
        uint8_t idx = data_handler_get_node_idx_by_ipv6(id.ipv6);
        if (idx < MAX_NODES) {
            node_actuator_state[idx].known    = true;
            node_actuator_state[idx].light_on = (s.light == 1);
            LOG_DBG("Light cache: node=%d light=%d", idx, s.light);
        } 
    }
    return 0;
}

/* ── CoAP Resource Table ─────────────────────────────────────── */

static const char *const sensors_path[] = {"sensors", NULL};

static struct coap_resource resources[] = {
    {
        .get  = NULL,
        .post = sensors_post,
        .put  = NULL,
        .del  = NULL,
        .path = sensors_path,
    },
    {.path = NULL},
};

/* ── CoAP RX Thread ──────────────────────────────────────────── */

static void coap_rx_thread(void *p1, void *p2, void *p3)
{
    uint8_t buf[BUF_SIZE];
    struct sockaddr_in6 src;
    socklen_t src_len = sizeof(src);
    struct coap_packet request;
    ssize_t received;
    int ret;

    ARG_UNUSED(p1);
    ARG_UNUSED(p2);
    ARG_UNUSED(p3);

    /* Wait for OpenThread interface to come up */
    struct net_if *iface = net_if_get_default();
    if (!iface) { LOG_ERR("No network interface"); return; }

    while (!net_if_is_up(iface)) {
        LOG_INF("Waiting for network interface...");
        k_sleep(K_SECONDS(1));
    }

    LOG_INF("Network up — listening for CoAP on port %d", COAP_PORT);

    while (running) {
        received = zsock_recvfrom(sock, buf, sizeof(buf), 0,
                                 (struct sockaddr *)&src, &src_len);
        if (received < 0) {
            LOG_ERR("recvfrom error: %d", errno);
            continue;
        }

        ret = coap_packet_parse(&request, buf, received, NULL, 0);
        if (ret < 0) {
            continue;
        }

        struct coap_option options[4];
        uint8_t opt_num = coap_find_options(&request,
                                            COAP_OPTION_URI_PATH,
                                            options, 4);

        ret = coap_handle_request(&request, resources, options, opt_num,
                                  (struct sockaddr *)&src, src_len);
        /* -ENOENT is normal for MPL duplicates — ignore silently */
        if (ret < 0 && ret != -ENOENT) {
            LOG_WRN("CoAP dispatch error: %d", ret);
        }
    }
}

/* ── CoAP TX Thread ──────────────────────────────────────────── */

/* Forward declaration */
static bool coap_send_con(const char *ipv6, bool on,
                          uint16_t msg_id, int ack_timeout_ms);

/* ── TX-Thread ────────────────────────────────────────────────
 * Holt Commands aus der Queue, wartet auf Thread-Fenster,
 * sendet CON + wartet auf ACK. Retry auf nächstes Thread-Fenster.
 */
static void coap_tx_thread_fn(void *p1, void *p2, void *p3)
{
    ARG_UNUSED(p1); ARG_UNUSED(p2); ARG_UNUSED(p3);

    static atomic_t s_msg_id = ATOMIC_INIT(1);

    while (1) {
        struct coap_cmd cmd;
        /* Warte auf nächsten Command */
        k_msgq_get(&s_coap_cmdq, &cmd, K_FOREVER);

        /* Neuere Commands für dasselbe Ziel übernehmen (latest-wins) */
        // struct coap_cmd newer;
        // while (k_msgq_peek(&s_coap_cmdq, &newer) == 0
        //        && strcmp(newer.ipv6, cmd.ipv6) == 0) {
        //     k_msgq_get(&s_coap_cmdq, &cmd, K_NO_WAIT);
        // }

        LOG_INF("CoAP TX: [%s] %s (seq=%u)", cmd.ipv6,
                cmd.on ? "ON" : "OFF", cmd.seq);

        bool success = false;

        for (int attempt = 0; attempt < 2 && !success; attempt++) {
            if (attempt > 0) {
                LOG_WRN("CoAP retry %d/3 — warte auf Thread-Fenster", attempt);
            }

            /* Thread-Priorität anfordern: 1500ms reicht für send+ACK */
            mesh_scheduler_request_priority(SCHED_PRIORITY_THREAD, 250);

            int wr = mesh_scheduler_wait_thread_window(350);
            if (wr != 0) {
                LOG_WRN("Thread-Fenster timeout, retry %d", attempt);
                continue;
            }

            /* Kurze Stabilisierungspause nach BLE→Thread-Wechsel */
            k_msleep(80);

            uint16_t mid = (uint16_t)atomic_inc(&s_msg_id);
            success = coap_send_con(cmd.ipv6, cmd.on, mid,
                                    120 /* ms ACK-Timeout */);

            if (!success) {
                LOG_WRN("Kein ACK für msg_id=%u (attempt %d)", mid, attempt+1);
                /* Kurz warten bevor nächster Versuch */
                k_msleep(200);
            }
        }

        if (success) {
            LOG_INF("CoAP PUT /light [%s] %s OK",
                    cmd.ipv6, cmd.on ? "ON" : "OFF");
            /* Aktuator-Cache aktualisieren */
            uint8_t idx = data_handler_get_node_idx_by_ipv6(cmd.ipv6);
            if (idx < MAX_NODES) {
                node_actuator_state[idx].known    = true;
                node_actuator_state[idx].light_on = cmd.on;
            }
        } else {
            LOG_ERR("CoAP PUT /light [%s] FAILED after 2 attempts", cmd.ipv6);
        }
    }
}
/* ── Public API ──────────────────────────────────────────────── */

int thread_handler_init(void)
{
    struct sockaddr_in6 addr = {
        .sin6_family = AF_INET6,
        .sin6_port   = htons(COAP_PORT),
        .sin6_addr   = IN6ADDR_ANY_INIT,
    };

    /* ── Apply Thread dataset ─────────────────────────────── */
    otInstance *ot = openthread_get_default_instance();
    if (ot) {
        otOperationalDataset dataset = {0};
        dataset.mActiveTimestamp.mSeconds             = 1;
        dataset.mComponents.mIsActiveTimestampPresent = true;
        dataset.mChannel                              = 11;
        dataset.mComponents.mIsChannelPresent         = true;
        dataset.mPanId                                = 0xABCD;
        dataset.mComponents.mIsPanIdPresent           = true;

        memcpy(dataset.mNetworkKey.m8,
               (uint8_t[]){0x00,0x11,0x22,0x33,0x44,0x55,0x66,0x77,
                           0x88,0x99,0xaa,0xbb,0xcc,0xdd,0xee,0xff},
               OT_NETWORK_KEY_SIZE);
        dataset.mComponents.mIsNetworkKeyPresent = true;

        memcpy(dataset.mExtendedPanId.m8,
               (uint8_t[]){0xde,0xad,0xbe,0xef,0xca,0xfe,0xba,0xbe},
               OT_EXT_PAN_ID_SIZE);
        dataset.mComponents.mIsExtendedPanIdPresent = true;

        strcpy(dataset.mNetworkName.m8, "NRFGate");
        dataset.mComponents.mIsNetworkNamePresent = true;

        openthread_mutex_lock();
        otDatasetSetActive(ot, &dataset);
        otIp6SetEnabled(ot, true);
        otThreadSetEnabled(ot, true);
        openthread_mutex_unlock();

        LOG_INF("Thread dataset applied: ch=11, key=00112233...");
    } else {
        LOG_ERR("OpenThread instance not available");
    }

    /* ── CoAP Socket + RX Thread ─────────────────────────── */
    sock = zsock_socket(AF_INET6, SOCK_DGRAM, IPPROTO_UDP);
    if (sock < 0) {
        LOG_ERR("Socket open failed: %d", errno);
        return -errno;
    }

    if (zsock_bind(sock, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
        LOG_ERR("Socket bind failed: %d", errno);
        zsock_close(sock);
        return -errno;
    }

    running = true;
    k_thread_create(&thread_rx_tid, thread_rx_stack, STACK_SIZE,
                    coap_rx_thread, NULL, NULL, NULL,
                    PRIORITY, 0, K_NO_WAIT);

    LOG_INF("CoAP RX thread started on port %d", COAP_PORT);

    /* CoAP TX Thread starten */
    k_thread_create(&s_coap_tx_tid, s_coap_tx_stack, COAP_TX_STACK_SIZE,
                    coap_tx_thread_fn, NULL, NULL, NULL,
                    COAP_TX_PRIORITY, 0, K_NO_WAIT);
    k_thread_name_set(&s_coap_tx_tid, "coap_tx");
    LOG_INF("CoAP TX thread started (prio=%d)", COAP_TX_PRIORITY);

    return 0;
}

/* ── Internes Send+ACK ────────────────────────────────────────
 * CON-Paket senden, Socket offen lassen, auf ACK warten (zsock_poll).
 * Gibt true zurück wenn ACK innerhalb ack_timeout_ms ankam.
 */
static bool coap_send_con(const char *ipv6, bool on,
                           uint16_t msg_id, int ack_timeout_ms)
{
    uint8_t pkt_buf[128];
    struct coap_packet pkt;

    int r = coap_packet_init(&pkt, pkt_buf, sizeof(pkt_buf),
                              COAP_VERSION_1,
                              COAP_TYPE_CON,
                              COAP_TOKEN_MAX_LEN,
                              coap_next_token(),
                              COAP_METHOD_PUT,
                              msg_id);
    if (r < 0) { LOG_ERR("pkt_init: %d", r); return false; }

    r = coap_packet_append_option(&pkt, COAP_OPTION_URI_PATH,
                                  "light", strlen("light"));
    if (r < 0) { LOG_ERR("URI: %d", r); return false; }

    uint8_t fmt = COAP_CONTENT_FORMAT_APP_JSON;
    r = coap_packet_append_option(&pkt, COAP_OPTION_CONTENT_FORMAT,
                                  &fmt, sizeof(fmt));
    if (r < 0) { LOG_ERR("fmt: %d", r); return false; }

    /* Payload */
    r = coap_packet_append_payload_marker(&pkt);
    if (r < 0) { LOG_ERR("marker: %d", r); return false; }

    const char *body = on ? "{\"on\":true}" : "{\"on\":false}";
    r = coap_packet_append_payload(&pkt, (uint8_t *)body, strlen(body));
    if (r < 0) { LOG_ERR("payload: %d", r); return false; }

    /* Open ephemeral socket */
    int tx_sock = zsock_socket(AF_INET6, SOCK_DGRAM, IPPROTO_UDP);
    if (tx_sock < 0) { LOG_ERR("socket: %d", errno); return false; }

    /* Destination address */
    struct sockaddr_in6 dst = {
        .sin6_family = AF_INET6,
        .sin6_port   = htons(COAP_PORT),
    };
    if (net_addr_pton(AF_INET6, ipv6, &dst.sin6_addr) < 0) {
        LOG_ERR("bad IPv6: %s", ipv6);
        zsock_close(tx_sock);
        return false;
    }

    /* Senden */
    r = zsock_sendto(tx_sock, pkt.data, pkt.offset, 0,
                     (struct sockaddr *)&dst, sizeof(dst));
    if (r < 0) {
        LOG_WRN("sendto failed: %d", errno);
        zsock_close(tx_sock);
        return false;
    }

    /* Auf ACK warten */
    struct zsock_pollfd pfd = { .fd = tx_sock, .events = ZSOCK_POLLIN };
    int pr = zsock_poll(&pfd, 1, ack_timeout_ms);

    bool acked = false;
    if (pr > 0 && (pfd.revents & ZSOCK_POLLIN)) {
        uint8_t ack_buf[32];
        ssize_t n = zsock_recv(tx_sock, ack_buf, sizeof(ack_buf), 0);
        if (n > 0) {
            acked = true;
            LOG_INF("CoAP ACK received (%d B) for msg_id=%u", (int)n, msg_id);
        }
    }

    zsock_close(tx_sock);
    return acked;
}


/* ── Öffentliche API ──────────────────────────────────────────
 * Gibt sofort zurück — blockiert den Rule-Engine-Work-Queue nicht.
 */
int thread_handler_coap_put_light(const char *ipv6, bool on)
{
    struct coap_cmd cmd = {0};
    strncpy(cmd.ipv6, ipv6, sizeof(cmd.ipv6) - 1);
    cmd.on  = on;
    cmd.seq = (uint8_t)atomic_inc(&s_cmd_seq);

    int r = k_msgq_put(&s_coap_cmdq, &cmd, K_NO_WAIT);
    if (r < 0) {
        LOG_WRN("CoAP queue full — dropping cmd (queue will drain)");
        return -ENOSPC;
    }

    LOG_INF("CoAP cmd queued: [%s] %s", ipv6, on ? "ON" : "OFF");
    return 0;
}