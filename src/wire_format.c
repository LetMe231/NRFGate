/**
 * @file wire_format.c
 * @brief LoRa binary wire format serializer + deserializer.
 *
 * Extracted from data_handler.c to allow unit testing.
 * data_handler.c should include wire_format.h and call wire_build().
 */

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <string.h>

#include "wire_format.h"

LOG_MODULE_REGISTER(wire_format, LOG_LEVEL_WRN);

/* ── Little-endian write helpers ─────────────────────────────── */

#define WRITE_I16(buf, off, val) do { \
    int16_t _v = (int16_t)(val);      \
    (buf)[(off)++] = (uint8_t)(_v);   \
    (buf)[(off)++] = (uint8_t)(_v >> 8); \
} while (0)

#define WRITE_U16(buf, off, val) do { \
    uint16_t _v = (uint16_t)(val);    \
    (buf)[(off)++] = (uint8_t)(_v);   \
    (buf)[(off)++] = (uint8_t)(_v >> 8); \
} while (0)

/* ── Little-endian read helpers ──────────────────────────────── */

#define READ_I16(buf, off) \
    ((int16_t)((buf)[(off)] | ((buf)[(off)+1] << 8)))

#define READ_U16(buf, off) \
    ((uint16_t)((buf)[(off)] | ((buf)[(off)+1] << 8)))

int wire_build(const struct node_sensor_data *d, uint8_t *buf, size_t size)
{
    if (size < WIRE_MAX_SIZE) {
        LOG_ERR("Wire buffer too small (%d < %d)", (int)size, WIRE_MAX_SIZE);
        return 0;
    }

    const struct sensor_payload *p = &d->payload;
    int off = 0;

    /* 6-byte header */
    buf[off++] = d->node_idx;
    buf[off++] = (uint8_t)d->identity.transport;
    buf[off++] = (uint8_t)(p->present & 0xFF);
    buf[off++] = (uint8_t)((p->present >> 8) & 0xFF);
    buf[off++] = (uint8_t)((p->present >> 16) & 0xFF);
    buf[off++] = (uint8_t)((p->present >> 24) & 0xFF);

    /* IMU — only if all three axes present */
    if ((p->present & SENSOR_HAS_ACCEL) == SENSOR_HAS_ACCEL) {
        WRITE_I16(buf, off, p->ax);
        WRITE_I16(buf, off, p->ay);
        WRITE_I16(buf, off, p->az);
    }
    if ((p->present & SENSOR_HAS_GYRO) == SENSOR_HAS_GYRO) {
        WRITE_I16(buf, off, p->gx);
        WRITE_I16(buf, off, p->gy);
        WRITE_I16(buf, off, p->gz);
    }

    /* Environmental — divide by 10 to fit int16_t */
    if (p->present & SENSOR_HAS_TEMP) { WRITE_I16(buf, off, p->temp / 10); }
    if (p->present & SENSOR_HAS_HUM)  { WRITE_I16(buf, off, p->hum  / 10); }

    /* Air quality */
    if (p->present & SENSOR_HAS_TVOC) { WRITE_U16(buf, off, p->tvoc); }
    if (p->present & SENSOR_HAS_ECO2) { WRITE_U16(buf, off, p->eco2); }

    /* Biometric */
    if (p->present & SENSOR_HAS_HEART_RATE) {
        buf[off++] = (uint8_t)p->heart_rate;
    }
    if (p->present & SENSOR_HAS_SPO2) { WRITE_U16(buf, off, p->spo2 / 10); }
    if (p->present & SENSOR_HAS_PM25) { WRITE_U16(buf, off, p->pm25); }
    if (p->present & SENSOR_HAS_PM10) { WRITE_U16(buf, off, p->pm10); }
    if (p->present & SENSOR_HAS_SWITCH) {
        buf[off++] = p->switch_state;
    }

    return off;
}

bool wire_parse(const uint8_t *buf, size_t len, struct node_sensor_data *out)
{
    if (len < 6) return false;

    int off = 0;
    out->node_idx               = buf[off++];
    out->identity.transport     = (node_transport_t)buf[off++];
    out->payload.present        = (int32_t)( buf[off]           | 
                                            (buf[off+1] << 8)   | 
                                            (buf[off+2] << 16)  | 
                                            (buf[off+3] << 24));
    off += 4;

    int32_t present = out->payload.present;

    if ((present & SENSOR_HAS_ACCEL) == SENSOR_HAS_ACCEL) {
        if (off + 6 > (int)len) return false;
        out->payload.ax = READ_I16(buf, off); off += 2;
        out->payload.ay = READ_I16(buf, off); off += 2;
        out->payload.az = READ_I16(buf, off); off += 2;
    }
    if ((present & SENSOR_HAS_GYRO) == SENSOR_HAS_GYRO) {
        if (off + 6 > (int)len) return false;
        out->payload.gx = READ_I16(buf, off); off += 2;
        out->payload.gy = READ_I16(buf, off); off += 2;
        out->payload.gz = READ_I16(buf, off); off += 2;
    }
    if (present & SENSOR_HAS_TEMP) {
        if (off + 2 > (int)len) return false;
        out->payload.temp = (int32_t)READ_I16(buf, off) * 10; off += 2;
    }
    if (present & SENSOR_HAS_HUM) {
        if (off + 2 > (int)len) return false;
        out->payload.hum = (int32_t)READ_I16(buf, off) * 10; off += 2;
    }
    if (present & SENSOR_HAS_TVOC) {
        if (off + 2 > (int)len) return false;
        out->payload.tvoc = READ_U16(buf, off); off += 2;
    }
    if (present & SENSOR_HAS_ECO2) {
        if (off + 2 > (int)len) return false;
        out->payload.eco2 = READ_U16(buf, off); off += 2;
    }
    if (present & SENSOR_HAS_HEART_RATE) {
        if (off + 1 > (int)len) return false;
        out->payload.heart_rate = buf[off++];
    }
    if (present & SENSOR_HAS_SPO2) {
        if (off + 2 > (int)len) return false;
        out->payload.spo2 = (int32_t)READ_U16(buf, off) * 10; off += 2;
    }
    if (present & SENSOR_HAS_PM25) {
        if (off + 2 > (int)len) return false;
        out->payload.pm25 = READ_U16(buf, off); off += 2;
    }
    if (present & SENSOR_HAS_PM10) {
        if (off + 2 > (int)len) return false;
        out->payload.pm10 = READ_U16(buf, off); off += 2;
    }
    if (present & SENSOR_HAS_SWITCH) {
        if (off + 1 > (int)len) return false;
        out->payload.switch_state = buf[off++];
    }

    return true;
}