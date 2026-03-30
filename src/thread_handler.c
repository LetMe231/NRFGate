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

#include "thread_handler.h"
#include "data_handler.h"

LOG_MODULE_REGISTER(thread_handler, LOG_LEVEL_INF);

#define COAP_PORT  5683
#define STACK_SIZE 4096
#define PRIORITY   10
#define BUF_SIZE   256

static int sock;
static bool running;

K_THREAD_STACK_DEFINE(thread_rx_stack, STACK_SIZE);
static struct k_thread thread_rx_tid;

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

};

/* ── POST /sensors Handler ───────────────────────────────────── */

static int sensors_post(struct coap_resource *resource,
                        struct coap_packet *request,
                        struct sockaddr *addr, socklen_t addr_len)
{
    ARG_UNUSED(resource);

    // Ack the request immediately to free up the sender, then do processing and forwarding asynchronously
    uint8_t ack_buf[4];
    struct coap_packet ack;

    coap_packet_init(&ack, ack_buf, sizeof(ack_buf),
                    COAP_VERSION_1,
                    COAP_TYPE_ACK,
                    0, NULL,
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

    // Parse JSON payload into sensor struct
    struct sensor_json s = {0};
    char json_buf[BUF_SIZE];
    size_t copy_len = MIN(payload_len, sizeof(json_buf) - 1);   
    memcpy(json_buf, payload, copy_len);
    json_buf[copy_len] = '\0';
    int32_t present = json_obj_parse(json_buf, copy_len, sensor_descr, ARRAY_SIZE(sensor_descr), &s);
    if (present < 0) {
        LOG_WRN("Failed to parse JSON from %s: %d", id.ipv6, present);
        return -EINVAL;
    }
    // Hand off to data handler
    struct node_sensor_data nd = {
        .identity = id,
        .payload = {
            .present = present,
            .seq = s.seq,
            .ts = s.ts,
            .ax = s.ax, .ay = s.ay, .az = s.az,
            .gx = s.gx, .gy = s.gy, .gz = s.gz,
            .temp = s.temp, .hum = s.hum,
            .tvoc = s.tvoc, .eco2 = s.eco2,
            .heart_rate = s.heart_rate, .spo2 = s.spo2,
            .raw_red = s.raw_red, .raw_ir = s.raw_ir,
        },
        .rx_uptime_ms = k_uptime_get(),
    };

    data_handler_receive(&nd);
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

        dataset.mActiveTimestamp.mSeconds = 1;
        dataset.mComponents.mIsActiveTimestampPresent = true;

        dataset.mChannel = 11;
        dataset.mComponents.mIsChannelPresent = true;

        dataset.mPanId = 0xABCD;
        dataset.mComponents.mIsPanIdPresent = true;

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
    return 0;
}