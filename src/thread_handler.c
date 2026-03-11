#include <zephyr/kernel.h>
#include <zephyr/net/socket.h>
#include <zephyr/logging/log.h>
#include "thread_handler.h"

LOG_MODULE_REGISTER(thread_handler, LOG_LEVEL_INF);

#define UDP_PORT    5683
#define STACK_SIZE  4096
#define PRIORITY    5

/* Muss byte-identisch mit dem ESP32-Struct sein */
typedef struct __attribute__((packed)) {
    uint32_t timestamp_ms;
    float    accel_x;
    float    accel_y;
    float    accel_z;
    float    gyro_x;
    float    gyro_y;
    float    gyro_z;
} sensor_packet_t;

static int  sock;
static bool running;

K_THREAD_STACK_DEFINE(thread_rx_stack, STACK_SIZE);
static struct k_thread thread_rx_tid;

/* ── Multicast beitreten (ff03::1) ──────────────────────────────────────── */

static void join_multicast(void)
{
    struct ipv6_mreq mreq;

    net_addr_pton(AF_INET6, "ff03::1", &mreq.ipv6mr_multiaddr);
    mreq.ipv6mr_ifindex = 0;

    if (setsockopt(sock, IPPROTO_IPV6, IPV6_ADD_MEMBERSHIP,
                   &mreq, sizeof(mreq)) < 0) {
        LOG_WRN("Multicast join failed: %d", errno);
    } else {
        LOG_INF("Joined ff03::1");
    }
}

/* ── Payload verarbeiten ────────────────────────────────────────────────── */

static void handle_packet(const uint8_t *buf, size_t len)
{
    if (len < sizeof(sensor_packet_t)) {
        LOG_WRN("Packet too short: %zu < %zu", len, sizeof(sensor_packet_t));
        return;
    }

    sensor_packet_t pkt;
    memcpy(&pkt, buf, sizeof(pkt));

    LOG_INF("[Thread] t=%ums | Accel: %.2f %.2f %.2f m/s² | Gyro: %.2f %.2f %.2f °/s",
            pkt.timestamp_ms,
            (double)pkt.accel_x, (double)pkt.accel_y, (double)pkt.accel_z,
            (double)pkt.gyro_x,  (double)pkt.gyro_y,  (double)pkt.gyro_z);
}

/* ── Empfangs-Thread ────────────────────────────────────────────────────── */

static void rx_thread(void *p1, void *p2, void *p3)
{
    uint8_t             buf[sizeof(sensor_packet_t)];
    struct sockaddr_in6 src;
    socklen_t           src_len = sizeof(src);
    ssize_t             received;

    ARG_UNUSED(p1); ARG_UNUSED(p2); ARG_UNUSED(p3);

    while (running) {
        received = recvfrom(sock, buf, sizeof(buf), 0,
                            (struct sockaddr *)&src, &src_len);
        if (received < 0) {
            LOG_ERR("recvfrom: %d", errno);
            continue;
        }
        handle_packet(buf, received);
    }
}

/* ── Public API ─────────────────────────────────────────────────────────── */

int thread_handler_init(void)
{
    struct sockaddr_in6 addr = {
        .sin6_family = AF_INET6,
        .sin6_port   = htons(UDP_PORT),
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

    join_multicast();

    running = true;
    k_thread_create(&thread_rx_tid, thread_rx_stack,
                    K_THREAD_STACK_SIZEOF(thread_rx_stack),
                    rx_thread, NULL, NULL, NULL,
                    PRIORITY, 0, K_NO_WAIT);
    k_thread_name_set(&thread_rx_tid, "thread_rx");

    LOG_INF("Thread UDP receiver on port %d", UDP_PORT);
    return 0;
}
