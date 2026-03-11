/**
 * ESP32-H2 / ESP32-C6  –  CoAP Sensor Node
 *
 * Reads BMI088 (accel + gyro) and sends a CoAP POST to /sensors
 * on the Thread gateway (nRF5340) every 100 ms.
 *
 * Payload format (JSON, integers × 1000):
 *   {"ts":12345,"ax":1200,"ay":-300,"az":9810,"gx":100,"gy":0,"gz":-50}
 *
 * Build system: ESP-IDF >= v5.2 with CONFIG_OPENTHREAD_ENABLED=y
 */

#include <stdio.h>
#include <string.h>
#include <stdint.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_openthread.h"
#include "esp_openthread_defaults.h"

/* libcoap is bundled with ESP-IDF (component: coap) */
#include "coap3/coap.h"

/* ── Configuration ──────────────────────────────────────────────────────── */

/* Thread mesh-local address of the nRF5340 gateway.
 * Read it from the gateway serial log after boot:
 *   "Mesh Local EID: fdXX:XX::XX"
 * Replace the placeholder below with the real address.          */
#define GATEWAY_ADDR   "fd11:22::1"
#define COAP_PORT       5683
#define SEND_INTERVAL_MS 100   /* 10 Hz */

static const char *TAG = "coap_node";

/* ── CoAP POST /sensors ─────────────────────────────────────────────────── */

/**
 * @brief Send one sensor frame to the gateway via CoAP CON POST.
 *        Values are passed as floats and encoded as integers × 1000
 *        so the payload stays plain JSON without float formatting.
 */
static void coap_send_sensors(float ax, float ay, float az,
                               float gx, float gy, float gz,
                               uint32_t ts_ms)
{
    coap_context_t  *ctx      = NULL;
    coap_session_t  *session  = NULL;
    coap_pdu_t      *pdu      = NULL;
    coap_optlist_t  *optlist  = NULL;
    coap_address_t   dst;
    char             payload[128];
    int              payload_len;
    uint8_t          cf_json  = 50;   /* CoAP content-format: application/json */

    /* Build JSON payload – integers scaled × 1000 */
    payload_len = snprintf(payload, sizeof(payload),
        "{\"ts\":%lu,"
        "\"ax\":%d,\"ay\":%d,\"az\":%d,"
        "\"gx\":%d,\"gy\":%d,\"gz\":%d}",
        (unsigned long)ts_ms,
        (int)(ax * 1000), (int)(ay * 1000), (int)(az * 1000),
        (int)(gx * 1000), (int)(gy * 1000), (int)(gz * 1000));

    /* Destination: gateway IPv6 + port */
    coap_address_init(&dst);
    dst.addr.sin6.sin6_family = AF_INET6;
    dst.addr.sin6.sin6_port   = htons(COAP_PORT);
    inet_pton(AF_INET6, GATEWAY_ADDR, &dst.addr.sin6.sin6_addr);

    ctx = coap_new_context(NULL);
    if (!ctx) {
        ESP_LOGE(TAG, "coap_new_context failed");
        goto cleanup;
    }

    /* CON (confirmable) → gateway replies with ACK */
    session = coap_new_client_session(ctx, NULL, &dst, COAP_PROTO_UDP);
    if (!session) {
        ESP_LOGE(TAG, "coap_new_client_session failed");
        goto cleanup;
    }

    pdu = coap_pdu_init(COAP_MESSAGE_CON,
                        COAP_REQUEST_CODE_POST,
                        coap_new_message_id(session),
                        256);
    if (!pdu) {
        ESP_LOGE(TAG, "coap_pdu_init failed");
        goto cleanup;
    }

    /* Options: URI-Path + Content-Format */
    coap_insert_optlist(&optlist,
        coap_new_optlist(COAP_OPTION_URI_PATH,
                         strlen("sensors"),
                         (uint8_t *)"sensors"));

    coap_insert_optlist(&optlist,
        coap_new_optlist(COAP_OPTION_CONTENT_FORMAT, 1, &cf_json));

    coap_add_optlist_pdu(pdu, &optlist);
    coap_add_data(pdu, payload_len, (uint8_t *)payload);

    if (coap_send(session, pdu) == COAP_INVALID_MID) {
        ESP_LOGE(TAG, "coap_send failed");
        goto cleanup;
    }

    /* Process I/O until ACK arrives or 2 s timeout */
    coap_run_once(ctx, 2000);
    ESP_LOGD(TAG, "Sent: %s", payload);

cleanup:
    coap_delete_optlist(optlist);
    if (session) coap_session_release(session);
    if (ctx)     coap_free_context(ctx);
}

/* ── Sensor task ────────────────────────────────────────────────────────── */

static void sensor_task(void *arg)
{
    /* TODO: bmi088_init() over SPI or I2C */

    while (1) {
        float ax, ay, az, gx, gy, gz;
        uint32_t ts = xTaskGetTickCount() * portTICK_PERIOD_MS;

        /* TODO: replace with real BMI088 driver calls
         *   bmi088_accel_get_data(&ax, &ay, &az);
         *   bmi088_gyro_get_data(&gx, &gy, &gz);       */
        ax = 0.10f; ay = 0.20f; az = 9.81f;
        gx = 0.00f; gy = 0.00f; gz = 0.00f;

        coap_send_sensors(ax, ay, az, gx, gy, gz, ts);

        vTaskDelay(pdMS_TO_TICKS(SEND_INTERVAL_MS));
    }
}

/* ── Entry point ────────────────────────────────────────────────────────── */

void app_main(void)
{
    ESP_LOGI(TAG, "ESP32 CoAP sensor node starting");

    /* Initialize OpenThread (uses esp_openthread component).
     * The Thread dataset (channel, PAN-ID, masterkey) must match
     * the values in the nRF5340 prj.conf:
     *   channel  = 11
     *   pan-id   = 0xABCD
     *   masterkey = 00112233445566778899aabbccddeeff              */
    esp_openthread_platform_config_t ot_cfg = {
        .radio_config = ESP_OPENTHREAD_DEFAULT_RADIO_CONFIG(),
        .host_config  = ESP_OPENTHREAD_DEFAULT_HOST_CONFIG(),
        .port_config  = ESP_OPENTHREAD_DEFAULT_PORT_CONFIG(),
    };
    esp_openthread_init(&ot_cfg);

    xTaskCreate(sensor_task, "sensor_task", 8192, NULL, 5, NULL);

    /* Hand control to the OpenThread task scheduler */
    esp_openthread_launch_mainloop();
}
