#include <zephyr/kernel.h>
#include <zephyr/net/socket.h>
#include <zephyr/net/coap.h>
#include <zephyr/data/json.h>
#include <zephyr/logging/log.h>
#include "thread_handler.h"

/* Realm-local all-nodes multicast – Thread nodes send sensor data here */
#define MCAST_ADDR  "ff03::1"

LOG_MODULE_REGISTER(thread_handler, LOG_LEVEL_INF);

#define COAP_PORT   5683
#define STACK_SIZE  4096
#define PRIORITY    5
#define BUF_SIZE    256

static int  sock;
static bool running;

K_THREAD_STACK_DEFINE(coap_rx_stack, STACK_SIZE);
static struct k_thread coap_rx_tid;

/* ── JSON payload descriptor ────────────────────────────────────────────── *
 * Floats are sent as integers × 1000 (e.g. 9.81 m/s² → 9810).
 * Avoids floating-point JSON serialisation on both ends.
 * ────────────────────────────────────────────────────────────────────────── */

struct sensor_json {
    int32_t ts;             /* timestamp [ms]         */
    int32_t ax, ay, az;     /* acceleration [m/s²×1000] */
    int32_t gx, gy, gz;     /* gyroscope   [°/s ×1000]  */
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

/* ── CoAP 2.04 Changed ACK ──────────────────────────────────────────────── */

static void send_ack(struct coap_packet *req,
                     struct sockaddr    *addr,
                     socklen_t           addr_len)
{
    uint8_t             buf[64];
    struct coap_packet  rsp;

    if (coap_ack_init(&rsp, req, buf, sizeof(buf),
                      COAP_RESPONSE_CODE_CHANGED) < 0) {
        return;
    }
    sendto(sock, rsp.data, rsp.offset, 0, addr, addr_len);
}

/* ── POST /sensors handler ──────────────────────────────────────────────── */

static int sensors_post(struct coap_resource *resource,
                        struct coap_packet   *request,
                        struct sockaddr      *addr,
                        socklen_t             addr_len)
{
    const uint8_t    *payload;
    uint16_t          payload_len;
    struct sensor_json s = {0};

    ARG_UNUSED(resource);

    payload = coap_packet_get_payload(request, &payload_len);
    if (!payload || payload_len == 0) {
        LOG_WRN("/sensors: empty payload");
        return -EINVAL;
    }

    if (json_obj_parse((char *)payload, payload_len,
                       sensor_descr, ARRAY_SIZE(sensor_descr), &s) < 0) {
        LOG_WRN("/sensors: JSON parse failed (len=%u)", payload_len);
        return -EINVAL;
    }

    LOG_INF("[CoAP/sensors] t=%dms | "
            "Accel: %.3f %.3f %.3f m/s² | "
            "Gyro:  %.3f %.3f %.3f °/s",
            s.ts,
            s.ax / 1000.0, s.ay / 1000.0, s.az / 1000.0,
            s.gx / 1000.0, s.gy / 1000.0, s.gz / 1000.0);

    send_ack(request, addr, addr_len);
    return 0;
}

/* ── CoAP resource table ────────────────────────────────────────────────── */

static const char * const sensors_path[] = { "sensors", NULL };

static struct coap_resource resources[] = {
    {
        .post = sensors_post,
        .path = sensors_path,
    },
    { .path = NULL }   /* sentinel */
};

/* ── RX thread ──────────────────────────────────────────────────────────── */

static void coap_rx_thread(void *p1, void *p2, void *p3)
{
    uint8_t             buf[BUF_SIZE];
    struct sockaddr_in6 src;
    socklen_t           src_len = sizeof(src);
    struct coap_packet  request;
    ssize_t             received;
    int                 ret;

    ARG_UNUSED(p1); ARG_UNUSED(p2); ARG_UNUSED(p3);

    /* Join ff03::1 here – Thread interface may not be ready at init time.
     * Retry every second until the stack accepts the subscription. */
    struct ipv6_mreq mreq = { .ipv6mr_ifindex = 0 };
    inet_pton(AF_INET6, MCAST_ADDR, &mreq.ipv6mr_multiaddr);
    while (setsockopt(sock, IPPROTO_IPV6, IPV6_JOIN_GROUP,
                      &mreq, sizeof(mreq)) < 0) {
        LOG_DBG("Multicast join pending (err %d), retrying in 1 s...", errno);
        k_sleep(K_SECONDS(1));
    }
    LOG_INF("Joined multicast group %s", MCAST_ADDR);

    while (running) {
        received = recvfrom(sock, buf, sizeof(buf), 0,
                            (struct sockaddr *)&src, &src_len);
        if (received < 0) {
            LOG_ERR("recvfrom: %d", errno);
            continue;
        }

        ret = coap_packet_parse(&request, buf, received, NULL, 0);
        if (ret < 0) {
            LOG_WRN("CoAP parse error: %d", ret);
            continue;
        }

        ret = coap_handle_request(&request, resources, NULL, 0,
                                  (struct sockaddr *)&src, src_len);
        if (ret < 0) {
            LOG_WRN("CoAP dispatch error: %d", ret);
        }
    }
}

/* ── Public API ─────────────────────────────────────────────────────────── */

int thread_handler_init(void)
{
    struct sockaddr_in6 addr = {
        .sin6_family = AF_INET6,
        .sin6_port   = htons(COAP_PORT),
        .sin6_addr   = IN6ADDR_ANY_INIT,
    };

    sock = socket(AF_INET6, SOCK_DGRAM, IPPROTO_UDP);
    if (sock < 0) {
        LOG_ERR("Socket open failed: %d", errno);
        return -errno;
    }

    if (bind(sock, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
        LOG_ERR("Socket bind failed: %d", errno);
        close(sock);
        return -errno;
    }

    running = true;
    k_thread_create(&coap_rx_tid, coap_rx_stack,
                    K_THREAD_STACK_SIZEOF(coap_rx_stack),
                    coap_rx_thread, NULL, NULL, NULL,
                    PRIORITY, 0, K_NO_WAIT);
    k_thread_name_set(&coap_rx_tid, "coap_rx");

    LOG_INF("CoAP server listening on port %d, resource: /sensors", COAP_PORT);
    return 0;
}
