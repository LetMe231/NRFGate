/**
 * @file thread_codec.c
 * @brief JSON decoder for Thread sensor payloads.
 *
 * This module parses JSON payloads received over Thread / CoAP and converts
 * them into generic gateway events (`gw_event_t`).
 */

#include "thread_codec.h"

#include <zephyr/logging/log.h>
#include <zephyr/data/json.h>
#include <zephyr/net/socket.h>

#include <limits.h>
#include <string.h>

LOG_MODULE_REGISTER(thread_codec, LOG_LEVEL_INF);

/* ─────────────────────────────────────────────────────────────
 * JSON payload structure
 * ───────────────────────────────────────────────────────────── */

/**
 * @brief Internal representation of the incoming JSON sensor payload.
 */
struct sensor_json {
    int32_t seq;
    int32_t ts;

    int32_t ax;
    int32_t ay;
    int32_t az;

    int32_t gx;
    int32_t gy;
    int32_t gz;

    int32_t temp;
    int32_t hum;
    int32_t tvoc;
    int32_t eco2;

    int32_t heart_rate;
    int32_t spo2;
    int32_t raw_red;
    int32_t raw_ir;

    int32_t pm25;
    int32_t pm10;

    int32_t sw;
    int32_t light;

    int32_t tx_ms;
    int32_t boot_id;
};

/** @brief JSON descriptor table for @ref sensor_json. */
static const struct json_obj_descr sensor_descr[] = {
    JSON_OBJ_DESCR_PRIM(struct sensor_json, seq,        JSON_TOK_NUMBER),
    JSON_OBJ_DESCR_PRIM(struct sensor_json, ts,         JSON_TOK_NUMBER),
    JSON_OBJ_DESCR_PRIM(struct sensor_json, ax,         JSON_TOK_NUMBER),
    JSON_OBJ_DESCR_PRIM(struct sensor_json, ay,         JSON_TOK_NUMBER),
    JSON_OBJ_DESCR_PRIM(struct sensor_json, az,         JSON_TOK_NUMBER),
    JSON_OBJ_DESCR_PRIM(struct sensor_json, gx,         JSON_TOK_NUMBER),
    JSON_OBJ_DESCR_PRIM(struct sensor_json, gy,         JSON_TOK_NUMBER),
    JSON_OBJ_DESCR_PRIM(struct sensor_json, gz,         JSON_TOK_NUMBER),
    JSON_OBJ_DESCR_PRIM(struct sensor_json, temp,       JSON_TOK_NUMBER),
    JSON_OBJ_DESCR_PRIM(struct sensor_json, hum,        JSON_TOK_NUMBER),
    JSON_OBJ_DESCR_PRIM(struct sensor_json, tvoc,       JSON_TOK_NUMBER),
    JSON_OBJ_DESCR_PRIM(struct sensor_json, eco2,       JSON_TOK_NUMBER),
    JSON_OBJ_DESCR_PRIM(struct sensor_json, heart_rate, JSON_TOK_NUMBER),
    JSON_OBJ_DESCR_PRIM(struct sensor_json, spo2,       JSON_TOK_NUMBER),
    JSON_OBJ_DESCR_PRIM(struct sensor_json, raw_red,    JSON_TOK_NUMBER),
    JSON_OBJ_DESCR_PRIM(struct sensor_json, raw_ir,     JSON_TOK_NUMBER),
    JSON_OBJ_DESCR_PRIM(struct sensor_json, pm25,       JSON_TOK_NUMBER),
    JSON_OBJ_DESCR_PRIM(struct sensor_json, pm10,       JSON_TOK_NUMBER),
    JSON_OBJ_DESCR_PRIM(struct sensor_json, sw,         JSON_TOK_NUMBER),
    JSON_OBJ_DESCR_PRIM(struct sensor_json, light,      JSON_TOK_NUMBER),
    JSON_OBJ_DESCR_PRIM(struct sensor_json, tx_ms,      JSON_TOK_NUMBER),
    JSON_OBJ_DESCR_PRIM(struct sensor_json, boot_id,    JSON_TOK_NUMBER),
};

/* ─────────────────────────────────────────────────────────────
 * Internal helpers
 * ───────────────────────────────────────────────────────────── */

/**
 * @brief Emit a gateway event through the codec callback if registered.
 *
 * @param emit       Event callback.
 * @param emit_user  User context passed to the callback.
 * @param evt        Event to emit.
 */
static void thread_codec_emit(thread_codec_emit_fn emit,
                              void *emit_user,
                              const gw_event_t *evt)
{
    if (emit) {
        emit(evt, emit_user);
    }
}

/**
 * @brief Fill a generic gateway source address for the Thread transport.
 *
 * @param src   Output source address.
 * @param ipv6  Source IPv6 address in binary form.
 */
static void thread_codec_fill_src(gw_node_addr_t *src,
                                  const uint8_t ipv6[GW_IPV6_BIN_LEN])
{
    memset(src, 0, sizeof(*src));
    src->transport = GW_TR_THREAD;
    memcpy(src->ipv6, ipv6, GW_IPV6_BIN_LEN);
}

/**
 * @brief Convert parsed JSON sensor values into gateway events.
 *
 * @param s          Parsed JSON structure.
 * @param ipv6       Source IPv6 address.
 * @param rx_ms      Receive timestamp.
 * @param emit       Event callback.
 * @param emit_user  User context passed to the callback.
 */
static void thread_codec_json_to_events(const struct sensor_json *s,
                                        const uint8_t ipv6[GW_IPV6_BIN_LEN],
                                        int64_t rx_ms,
                                        thread_codec_emit_fn emit,
                                        void *emit_user)
{
    gw_event_t evt = {
        .type  = GW_EVT_SENSOR,
        .rx_ms = rx_ms,
    };

    thread_codec_fill_src(&evt.src, ipv6);

    gw_sensor_payload_t *p = &evt.data.sensor;

    if (s->seq >= 0) {
        p->seq = s->seq;
        p->present |= GW_HAS_SEQ;
    }
    if (s->ts >= 0) {
        p->ts = s->ts;
        p->present |= GW_HAS_TS;
    }
    if (s->temp != INT32_MIN) {
        p->temp_mc = s->temp;
        p->present |= GW_HAS_TEMP;
    }
    if (s->hum != INT32_MIN) {
        p->hum_mpermille = s->hum;
        p->present |= GW_HAS_HUM;
    }
    if (s->ax != INT32_MIN && s->ay != INT32_MIN && s->az != INT32_MIN) {
        p->ax_mg = s->ax;
        p->ay_mg = s->ay;
        p->az_mg = s->az;
        p->present |= GW_HAS_ACCEL;
    }
    if (s->gx != INT32_MIN && s->gy != INT32_MIN && s->gz != INT32_MIN) {
        p->gx_mdps = s->gx;
        p->gy_mdps = s->gy;
        p->gz_mdps = s->gz;
        p->present |= GW_HAS_GYRO;
    }
    if (s->tvoc != INT32_MIN) {
        p->tvoc_ppb = s->tvoc;
        p->present |= GW_HAS_TVOC;
    }
    if (s->eco2 != INT32_MIN) {
        p->eco2_ppm = s->eco2;
        p->present |= GW_HAS_ECO2;
    }
    if (s->heart_rate != INT32_MIN) {
        p->heart_rate_bpm = s->heart_rate;
        p->present |= GW_HAS_HEART_RATE;
    }
    if (s->spo2 != INT32_MIN) {
        p->spo2_mpermille = s->spo2;
        p->present |= GW_HAS_SPO2;
    }
    if (s->raw_red != INT32_MIN) {
        p->raw_red = s->raw_red;
        p->present |= GW_HAS_RAW_RED;
    }
    if (s->raw_ir != INT32_MIN) {
        p->raw_ir = s->raw_ir;
        p->present |= GW_HAS_RAW_IR;
    }
    if (s->pm25 != INT32_MIN) {
        p->pm25_ugm3 = s->pm25;
        p->present |= GW_HAS_PM25;
    }
    if (s->pm10 != INT32_MIN) {
        p->pm10_ugm3 = s->pm10;
        p->present |= GW_HAS_PM10;
    }
    if (s->sw == 0 || s->sw == 1) {
        p->switch_state = (s->sw == 1);
        p->present |= GW_HAS_SWITCH;
    }
    if (s->light == 0 || s->light == 1) {
        p->light_on = (s->light == 1);
        p->present |= GW_HAS_LIGHT;
    }

    if (p->present == GW_HAS_NONE) {
        LOG_WRN("No known fields in Thread JSON payload");
        return;
    }

    if (s->sw == 0 || s->sw == 1) {
        gw_event_t btn_evt = {
            .type  = GW_EVT_BUTTON,
            .rx_ms = rx_ms,
            .src   = evt.src,
            .data.button = {
                .pressed = (s->sw == 1),
                .seq     = (uint8_t)s->seq,
            },
        };

        thread_codec_emit(emit, emit_user, &btn_evt);
    }

    thread_codec_emit(emit, emit_user, &evt);
}

/* ─────────────────────────────────────────────────────────────
 * Public API
 * ───────────────────────────────────────────────────────────── */

/**
 * @brief Parse a Thread JSON sensor payload into gateway events.
 *
 * @param json       Null-terminated or bounded JSON payload.
 * @param len        JSON length in bytes.
 * @param ipv6       Source IPv6 address in binary form.
 * @param rx_ms      Receive timestamp in milliseconds.
 * @param emit       Event callback used to emit parsed events.
 * @param emit_user  User context passed to @p emit.
 *
 * @retval 0 on success
 * @retval -EINVAL if required input pointers are invalid
 * @retval <0 JSON parsing error code
 */
int thread_codec_parse_sensor_json(char *json,
                                   size_t len,
                                   const uint8_t ipv6[GW_IPV6_BIN_LEN],
                                   int64_t rx_ms,
                                   thread_codec_emit_fn emit,
                                   void *emit_user)
{
    if (!json || !ipv6) {
        return -EINVAL;
    }

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
        .tx_ms      = INT32_MIN,
        .boot_id    = INT32_MIN,
    };

    int ret = json_obj_parse(json,
                             len,
                             sensor_descr,
                             ARRAY_SIZE(sensor_descr),
                             &s);
    if (ret < 0) {
        return ret;
    }

    thread_codec_json_to_events(&s, ipv6, rx_ms, emit, emit_user);
    return 0;
}