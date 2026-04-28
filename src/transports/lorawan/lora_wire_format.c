/**
 * @file lora_wire_format.c
 * @brief LoRa binary wire format serializer and deserializer.
 *
 * This module converts generic gateway events to and from the compact LoRa
 * wire format used on the radio link.
 */

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

#include <string.h>

#include "lora_wire_format.h"
#include "gw_addr.h"

LOG_MODULE_REGISTER(lora_wire_format, LOG_LEVEL_WRN);

/* ─────────────────────────────────────────────────────────────
 * Little-endian write helpers
 * ───────────────────────────────────────────────────────────── */

/** @brief Write a signed 16-bit integer in little-endian format. */
#define WRITE_I16(buf, off, val) do {       \
    int16_t _v = (int16_t)(val);            \
    (buf)[(off)++] = (uint8_t)(_v);         \
    (buf)[(off)++] = (uint8_t)(_v >> 8);    \
} while (0)

/** @brief Write an unsigned 16-bit integer in little-endian format. */
#define WRITE_U16(buf, off, val) do {       \
    uint16_t _v = (uint16_t)(val);          \
    (buf)[(off)++] = (uint8_t)(_v);         \
    (buf)[(off)++] = (uint8_t)(_v >> 8);    \
} while (0)

/** @brief Write an unsigned 32-bit integer in little-endian format. */
#define WRITE_U32(buf, off, val) do {       \
    uint32_t _v = (uint32_t)(val);          \
    (buf)[(off)++] = (uint8_t)(_v);         \
    (buf)[(off)++] = (uint8_t)(_v >> 8);    \
    (buf)[(off)++] = (uint8_t)(_v >> 16);   \
    (buf)[(off)++] = (uint8_t)(_v >> 24);   \
} while (0)

/* ─────────────────────────────────────────────────────────────
 * Little-endian read helpers
 * ───────────────────────────────────────────────────────────── */

/** @brief Read a signed 16-bit integer from little-endian format. */
#define READ_I16(buf, off) \
    ((int16_t)((buf)[(off)] | ((uint16_t)(buf)[(off) + 1] << 8)))

/** @brief Read an unsigned 16-bit integer from little-endian format. */
#define READ_U16(buf, off) \
    ((uint16_t)((buf)[(off)] | ((uint16_t)(buf)[(off) + 1] << 8)))

/** @brief Read an unsigned 32-bit integer from little-endian format. */
#define READ_U32(buf, off) \
    ((uint32_t)((buf)[(off)]                           \
              | ((uint32_t)(buf)[(off) + 1] << 8)    \
              | ((uint32_t)(buf)[(off) + 2] << 16)   \
              | ((uint32_t)(buf)[(off) + 3] << 24)))

/* ─────────────────────────────────────────────────────────────
 * Serialization
 * ───────────────────────────────────────────────────────────── */

/**
 * @brief Serialize a sensor event into the LoRa wire format.
 *
 * @param evt   Event to serialize.
 * @param buf   Output buffer.
 * @param size  Output buffer size in bytes.
 *
 * @return Number of bytes written, or 0 on failure.
 */
int wire_build(const gw_event_t *evt, uint8_t *buf, size_t size)
{
    if (!evt || !buf || size < WIRE_MAX_SIZE) {
        LOG_ERR("wire_build: invalid args");
        return 0;
    }

    if (evt->type != GW_EVT_SENSOR) {
        LOG_ERR("wire_build: not a sensor event");
        return 0;
    }

    const gw_sensor_payload_t *p = &evt->data.sensor;
    int off = 0;

    /* Header. */
    buf[off++] = LORA_PKT_SENSOR;
    buf[off++] = (uint8_t)p->seq;

    uint64_t dev_eui;
    int err = gw_addr_to_lorawan(&evt->src, &dev_eui);
    if (err) {
        LOG_ERR("wire_build: failed to convert node address to dev_eui");
        return err;
    }
    /* Node ID from dev_eui_lo. */
    uint16_t node_id = (uint16_t)(dev_eui & 0xFFFF);
    WRITE_U16(buf, off, node_id);

    /* Presence mask. */
    WRITE_U32(buf, off, p->present);

    /* Fields in presence-mask order. */
    if (p->present & GW_HAS_ACCEL) {
        WRITE_I16(buf, off, p->ax_mg / 10);
        WRITE_I16(buf, off, p->ay_mg / 10);
        WRITE_I16(buf, off, p->az_mg / 10);
    }
    if (p->present & GW_HAS_GYRO) {
        WRITE_I16(buf, off, p->gx_mdps / 10);
        WRITE_I16(buf, off, p->gy_mdps / 10);
        WRITE_I16(buf, off, p->gz_mdps / 10);
    }
    if (p->present & GW_HAS_TEMP) {
        WRITE_I16(buf, off, p->temp_mc / 10);
    }
    if (p->present & GW_HAS_HUM) {
        WRITE_U16(buf, off, p->hum_mpermille / 10);
    }
    if (p->present & GW_HAS_TVOC) {
        WRITE_U16(buf, off, p->tvoc_ppb);
    }
    if (p->present & GW_HAS_ECO2) {
        WRITE_U16(buf, off, p->eco2_ppm);
    }
    if (p->present & GW_HAS_HEART_RATE) {
        buf[off++] = (uint8_t)p->heart_rate_bpm;
    }
    if (p->present & GW_HAS_SPO2) {
        WRITE_U16(buf, off, p->spo2_mpermille / 10);
    }
    if (p->present & GW_HAS_PM25) {
        WRITE_U16(buf, off, p->pm25_ugm3);
    }
    if (p->present & GW_HAS_PM10) {
        WRITE_U16(buf, off, p->pm10_ugm3);
    }
    if (p->present & GW_HAS_SWITCH) {
        buf[off++] = p->switch_state ? 1 : 0;
    }
    if (p->present & GW_HAS_LIGHT) {
        buf[off++] = p->light_on ? 1 : 0;
    }

    LOG_DBG("wire_build: %d bytes, present=0x%08X", off, p->present);
    return off;
}

/**
 * @brief Serialize an alert/state-transition packet into the LoRa wire format.
 *
 * @param node_id Node identifier.
 * @param seq     Sequence number.
 * @param from    Previous node state.
 * @param to      New node state.
 * @param buf     Output buffer.
 * @param size    Output buffer size in bytes.
 *
 * @return Number of bytes written, or 0 on failure.
 */
int wire_build_alert(uint16_t node_id,
                     uint8_t seq,
                     gw_state_t from,
                     gw_state_t to,
                     uint8_t *buf,
                     size_t size)
{
    if (!buf || size < 6) {
        LOG_ERR("wire_build_alert: buffer too small");
        return 0;
    }

    int off = 0;
    buf[off++] = LORA_PKT_ALERT;
    buf[off++] = seq;
    WRITE_U16(buf, off, node_id);
    buf[off++] = (uint8_t)from;
    buf[off++] = (uint8_t)to;

    LOG_DBG("wire_build_alert: node=0x%04X %d->%d", node_id, from, to);
    return off;
}

/* ─────────────────────────────────────────────────────────────
 * Deserialization
 * ───────────────────────────────────────────────────────────── */

/**
 * @brief Parse a LoRa wire-format packet into a gateway event.
 *
 * @param buf  Input buffer.
 * @param len  Input length in bytes.
 * @param out  Output event.
 *
 * @return true on successful parse, false otherwise.
 */
bool wire_parse(const uint8_t *buf, size_t len, gw_event_t *out)
{
    if (!buf || !out || len < 4) {
        return false;
    }

    memset(out, 0, sizeof(*out));
    out->src.transport = GW_TR_LORAWAN;

    uint8_t pkt_type = buf[0];
    uint8_t seq = buf[1];
    uint16_t node_id = READ_U16(buf, 2);

    /* Build full 64-bit device EUI */
    gw_addr_from_lorawan(&out->src, (uint64_t)node_id);
    /* ALERT packet. */
    if (pkt_type == LORA_PKT_ALERT) {
        if (len < 6) {
            LOG_WRN("wire_parse: ALERT too short (%d)", (int)len);
            return false;
        }

        out->type = GW_EVT_STATE_TRANSITION;
        out->data.state_transition.from = (gw_state_t)buf[4];
        out->data.state_transition.to   = (gw_state_t)buf[5];

        LOG_DBG("wire_parse: ALERT node=0x%04X %d->%d",
                node_id,
                out->data.state_transition.from,
                out->data.state_transition.to);
        return true;
    }

    /* SENSOR / SNAPSHOT packet. */
    if (pkt_type != LORA_PKT_SENSOR && pkt_type != LORA_PKT_SNAPSHOT) {
        LOG_WRN("wire_parse: unknown packet type 0x%02X", pkt_type);
        return false;
    }

    if (len < 8) {
        LOG_WRN("wire_parse: SENSOR too short (%d)", (int)len);
        return false;
    }

    out->type = GW_EVT_SENSOR;

    gw_sensor_payload_t *p = &out->data.sensor;
    p->seq = seq;
    p->present = READ_U32(buf, 4);

    int off = 8;

    if (p->present & GW_HAS_ACCEL) {
        if (off + 6 > (int)len) {
            return false;
        }
        p->ax_mg = (int32_t)READ_I16(buf, off) * 10; off += 2;
        p->ay_mg = (int32_t)READ_I16(buf, off) * 10; off += 2;
        p->az_mg = (int32_t)READ_I16(buf, off) * 10; off += 2;
    }
    if (p->present & GW_HAS_GYRO) {
        if (off + 6 > (int)len) {
            return false;
        }
        p->gx_mdps = (int32_t)READ_I16(buf, off) * 10; off += 2;
        p->gy_mdps = (int32_t)READ_I16(buf, off) * 10; off += 2;
        p->gz_mdps = (int32_t)READ_I16(buf, off) * 10; off += 2;
    }
    if (p->present & GW_HAS_TEMP) {
        if (off + 2 > (int)len) {
            return false;
        }
        p->temp_mc = (int32_t)READ_I16(buf, off) * 10; off += 2;
    }
    if (p->present & GW_HAS_HUM) {
        if (off + 2 > (int)len) {
            return false;
        }
        p->hum_mpermille = (int32_t)READ_U16(buf, off) * 10; off += 2;
    }
    if (p->present & GW_HAS_TVOC) {
        if (off + 2 > (int)len) {
            return false;
        }
        p->tvoc_ppb = READ_U16(buf, off); off += 2;
    }
    if (p->present & GW_HAS_ECO2) {
        if (off + 2 > (int)len) {
            return false;
        }
        p->eco2_ppm = READ_U16(buf, off); off += 2;
    }
    if (p->present & GW_HAS_HEART_RATE) {
        if (off + 1 > (int)len) {
            return false;
        }
        p->heart_rate_bpm = buf[off++];
    }
    if (p->present & GW_HAS_SPO2) {
        if (off + 2 > (int)len) {
            return false;
        }
        p->spo2_mpermille = (int32_t)READ_U16(buf, off) * 10; off += 2;
    }
    if (p->present & GW_HAS_PM25) {
        if (off + 2 > (int)len) {
            return false;
        }
        p->pm25_ugm3 = READ_U16(buf, off); off += 2;
    }
    if (p->present & GW_HAS_PM10) {
        if (off + 2 > (int)len) {
            return false;
        }
        p->pm10_ugm3 = READ_U16(buf, off); off += 2;
    }
    if (p->present & GW_HAS_SWITCH) {
        if (off + 1 > (int)len) {
            return false;
        }
        p->switch_state = (buf[off++] != 0);
    }
    if (p->present & GW_HAS_LIGHT) {
        if (off + 1 > (int)len) {
            return false;
        }
        p->light_on = (buf[off++] != 0);
    }

    LOG_DBG("wire_parse: SENSOR node=0x%04X present=0x%08X",
            node_id, p->present);
    return true;
}