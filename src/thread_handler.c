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

#include "thread_handler.h"

LOG_MODULE_REGISTER(thread_handler, LOG_LEVEL_INF);

#define COAP_PORT  5683
#define STACK_SIZE 4096
#define PRIORITY   5
#define BUF_SIZE   256

static int sock;
static bool running;

K_THREAD_STACK_DEFINE(thread_rx_stack, STACK_SIZE);
static struct k_thread thread_rx_tid;

/* ── JSON Payload Descriptor ─────────────────────────────────── */

struct sensor_json {
    int32_t ts;
    int32_t ax, ay, az;
    int32_t gx, gy, gz;
};

static const struct json_obj_descr sensor_descr[] = {
    JSON_OBJ_DESCR_PRIM(struct sensor_json, ts, JSON_TOK_NUMBER),
    JSON_OBJ_DESCR_PRIM(struct sensor_json, ax, JSON_TOK_NUMBER),
    JSON_OBJ_DESCR_PRIM(struct sensor_json, ay, JSON_TOK_NUMBER),
    JSON_OBJ_DESCR_PRIM(struct sensor_json, az, JSON_TOK_NUMBER),
    JSON_OBJ_DESCR_PRIM(struct sensor_json, gx, JSON_TOK_NUMBER),
    JSON_OBJ_DESCR_PRIM(struct sensor_json, gy, JSON_TOK_NUMBER),
    JSON_OBJ_DESCR_PRIM(struct sensor_json, gz, JSON_TOK_NUMBER),
};

/* ── POST /sensors Handler ───────────────────────────────────── */

static int sensors_post(struct coap_resource *resource,
                        struct coap_packet *request,
                        struct sockaddr *addr, socklen_t addr_len)
{
    const uint8_t *payload;
    uint16_t payload_len;
    struct sensor_json s = {0};

    ARG_UNUSED(resource);

    payload = coap_packet_get_payload(request, &payload_len);
    if (!payload || payload_len == 0) {
        return -EINVAL;
    }

    if (json_obj_parse((char *)payload, payload_len,
                       sensor_descr, ARRAY_SIZE(sensor_descr), &s) < 0) {
        LOG_WRN("JSON parse failed (len=%d)", payload_len);
        return -EINVAL;
    }

    LOG_INF("Received Sensor Data - Timestamp: %d ms, "
            "Accel: (%d, %d, %d), Gyro: (%d, %d, %d)",
            s.ts, s.ax, s.ay, s.az, s.gx, s.gy, s.gz);

    return 0;
}

/* ── CoAP Resource Table ─────────────────────────────────────── */

static const char *const resource_path[] = {"sensors", NULL};

static struct coap_resource resources[] = {
    {
        .get  = NULL,
        .post = sensors_post,
        .put  = NULL,
        .del  = NULL,
        .path = resource_path,
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