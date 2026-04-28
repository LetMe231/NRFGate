/**
 * @file ble_mesh_codec.c
 * @brief Decoder for BLE Mesh Sensor Status messages.
 *
 * This module parses raw BLE Mesh sensor payloads into generic gateway events
 * (`gw_event_t`). Depending on the properties contained in the message, it may
 * emit:
 * - a sensor event
 * - a button event
 */

#include "ble_mesh_codec.h"

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/byteorder.h>

LOG_MODULE_REGISTER(ble_mesh_codec, LOG_LEVEL_INF);

/* ─────────────────────────────────────────────────────────────
 * Constants
 * ───────────────────────────────────────────────────────────── */

/** @brief Assumed publication TTL used to reconstruct receive TTL metadata. */
#define MESH_PUB_TTL     5

/** @brief Property ID for temperature. */
#define PROP_TEMPERATURE 0x004F

/** @brief Property ID for humidity. */
#define PROP_HUMIDITY    0x0076

/** @brief Property ID for eCO2. */
#define PROP_ECO2        0x0008

/** @brief Property ID for heart rate. */
#define PROP_HEART_RATE  0x0100

/** @brief Property ID for SpO2. */
#define PROP_SPO2        0x0101

/** @brief Property ID for TVOC. */
#define PROP_TVOC        0x0102

/** @brief Property ID for raw red channel value. */
#define PROP_RAW_RED     0x0103

/** @brief Property ID for raw IR channel value. */
#define PROP_RAW_IR      0x0104

/** @brief Property ID for switch state. */
#define PROP_SWITCH      0x0105

/** @brief Property ID for light state. */
#define PROP_LIGHT_STATE 0x0106

/** @brief Property ID for sensor sequence counter. */
#define PROP_SENSOR_SEQ  0x07FF

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
static void ble_mesh_codec_emit(ble_mesh_codec_emit_fn emit,
                                void *emit_user,
                                const gw_event_t *evt)
{
    if (emit) {
        emit(evt, emit_user);
    }
}

/* ─────────────────────────────────────────────────────────────
 * Public API
 * ───────────────────────────────────────────────────────────── */

/**
 * @brief Parse a BLE Mesh Sensor Status payload into gateway events.
 *
 * The payload is parsed property-by-property using the Mesh Property ID
 * encoding. Known properties are mapped into the generic gateway sensor
 * payload. If a switch property is present, an additional button event is
 * emitted.
 *
 * @param data       Raw payload bytes.
 * @param len        Payload length in bytes.
 * @param src_addr   Source mesh address.
 * @param rssi       Received RSSI in dBm.
 * @param hops       Estimated hop count.
 * @param emit       Event callback used to emit parsed events.
 * @param emit_user  User context passed to @p emit.
 *
 * @retval 0 Always returns success after parsing.
 */
int ble_mesh_codec_parse_sensor_status(const uint8_t *data,
                                       uint16_t len,
                                       uint16_t src_addr,
                                       int8_t rssi,
                                       uint8_t hops,
                                       ble_mesh_codec_emit_fn emit,
                                       void *emit_user)
{
    gw_event_t evt = {
        .type  = GW_EVT_SENSOR,
        .rx_ms = k_uptime_get(),
        .rx_meta = {
            .has_rssi   = true,
            .rssi_dbm   = rssi,
            .has_hops   = true,
            .hops       = hops,
            .has_ttl_rx = true,
            .rx_ttl = (hops <= MESH_PUB_TTL) ? (MESH_PUB_TTL - hops) : 0,
        },
    };

    gw_addr_from_mesh(&evt.src, src_addr);

    gw_sensor_payload_t *s = &evt.data.sensor;
    const uint8_t *p = data;
    const uint8_t *end = data + len;

    while (p + 3 <= end) {
        uint8_t b0 = p[0];
        uint8_t b1 = p[1];

        uint16_t prop_id;
        uint8_t val_len;

        if ((b0 & 0x01) == 0) {
            uint16_t mpid = b0 | ((uint16_t)b1 << 8);
            prop_id = (mpid >> 5) & 0x7FF;
            val_len = ((mpid >> 1) & 0x0F) + 1;
            p += 2;
        } else {
            prop_id = ((uint16_t)b1 << 3) | (b0 >> 5);
            val_len = p[2];
            p += 3;
        }

        switch (prop_id) {
        case PROP_SENSOR_SEQ:
            if (val_len >= 4) {
                s->seq = (int32_t)sys_get_le32(p);
                s->present |= GW_HAS_SEQ;
            }
            break;

        case PROP_TEMPERATURE:
            if (val_len >= 2) {
                int16_t raw = (int16_t)sys_get_le16(p);
                s->temp_mc = (int32_t)raw * 10;
                s->present |= GW_HAS_TEMP;
            }
            break;

        case PROP_HUMIDITY:
            if (val_len >= 2) {
                uint16_t raw = sys_get_le16(p);
                s->hum_mpermille = (int32_t)raw * 10;
                s->present |= GW_HAS_HUM;
            }
            break;

        case PROP_TVOC:
            if (val_len >= 2) {
                s->tvoc_ppb = (int32_t)sys_get_le16(p);
                s->present |= GW_HAS_TVOC;
            }
            break;

        case PROP_ECO2:
            if (val_len >= 2) {
                s->eco2_ppm = (int32_t)sys_get_le16(p);
                s->present |= GW_HAS_ECO2;
            }
            break;

        case PROP_HEART_RATE:
            if (val_len >= 2) {
                s->heart_rate_bpm = (int32_t)sys_get_le16(p);
                s->present |= GW_HAS_HEART_RATE;
            }
            break;

        case PROP_SPO2:
            if (val_len >= 2) {
                uint16_t raw = sys_get_le16(p);
                s->spo2_mpermille = (int32_t)raw * 10;
                s->present |= GW_HAS_SPO2;
            }
            break;

        case PROP_RAW_RED:
            if (val_len >= 4) {
                s->raw_red = (int32_t)sys_get_le32(p);
                s->present |= GW_HAS_RAW_RED;
            }
            break;

        case PROP_RAW_IR:
            if (val_len >= 4) {
                s->raw_ir = (int32_t)sys_get_le32(p);
                s->present |= GW_HAS_RAW_IR;
            }
            break;

        case PROP_SWITCH:
            if (val_len >= 1) {
                s->switch_state = (p[0] != 0);
                s->present |= GW_HAS_SWITCH;

                gw_event_t btn_evt = {
                    .type  = GW_EVT_BUTTON,
                    .rx_ms = k_uptime_get(),
                    .src   = evt.src,
                    .data.button = {
                        .pressed = (p[0] != 0),
                    },
                };

                ble_mesh_codec_emit(emit, emit_user, &btn_evt);
            }
            break;

        case PROP_LIGHT_STATE:
            if (val_len >= 1) {
                s->light_on = (p[0] != 0);
                s->present |= GW_HAS_LIGHT;
            }
            break;

        default:
            LOG_DBG("Unknown property ID: %u (len %u)", prop_id, val_len);
            break;
        }

        if (p + val_len > end) {
            LOG_WRN("Truncated sensor property (need %u, have %zd)", val_len, end - p);
            break;
        }
        p += val_len;
    }

    if (s->present == GW_HAS_NONE) {
        LOG_WRN("Received sensor status with no known properties from 0x%04X",
                src_addr);
    }

    ble_mesh_codec_emit(emit, emit_user, &evt);
    return 0;
}