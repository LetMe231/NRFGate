/**
 * @file mesh_parser.c
 * @brief BLE Mesh Sensor Status payload parser.
 *
 * Extracted from ble_mesh_handler.c to allow unit testing without
 * pulling in the full BLE Mesh stack.
 */

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <string.h>

#include "mesh_parser.h"

LOG_MODULE_REGISTER(mesh_parser, LOG_LEVEL_WRN);

bool mesh_parse_mpid_format_a(uint16_t mpid, uint16_t *prop_id, uint8_t *data_len)
{
    if (mpid & 0x01) {
        return false;  /* Format B — not supported */
    }
    *data_len = ((mpid >> 1) & 0x0F) + 1;
    *prop_id  = (mpid >> 5) & 0x7FF;
    return true;
}

int32_t mesh_read_le_signed(const uint8_t *data, uint8_t len)
{
    switch (len) {
    case 1: return (int8_t)data[0];
    case 2: return (int16_t)(data[0] | (data[1] << 8));
    case 4: return (int32_t)(data[0] | (data[1] << 8) |
                             (data[2] << 16) | (data[3] << 24));
    default: return 0;
    }
}

uint32_t mesh_read_le_unsigned(const uint8_t *data, uint8_t len)
{
    switch (len) {
    case 1: return data[0];
    case 2: return (uint32_t)(data[0] | (data[1] << 8));
    case 4: return (uint32_t)(data[0] | (data[1] << 8) |
                              (data[2] << 16) | (data[3] << 24));
    default: return 0;
    }
}

bool mesh_parse_sensor_status(const uint8_t *data, uint16_t len,
                              struct sensor_payload *out)
{
    uint16_t offset = 0;

    while (offset + 2 <= len) {
        uint16_t mpid = data[offset] | (data[offset + 1] << 8);
        uint16_t prop_id;
        uint8_t data_len;

        if (!mesh_parse_mpid_format_a(mpid, &prop_id, &data_len)) {
            break;
        }
        offset += 2;

        if (offset + data_len > len) {
            LOG_WRN("Property 0x%04X truncated at offset %d", prop_id, offset);
            break;
        }

        const uint8_t *val = &data[offset];
        offset += data_len;

        switch (prop_id) {
        case PROP_SENSOR_SEQ:
            out->seq = (int32_t)mesh_read_le_unsigned(val, data_len);
            out->present |= SENSOR_HAS_SEQ;
            break;
        case PROP_TEMPERATURE:
            out->temp = mesh_read_le_signed(val, data_len) * 10;
            out->present |= SENSOR_HAS_TEMP;
            break;
        case PROP_HUMIDITY:
            out->hum = (int32_t)mesh_read_le_unsigned(val, data_len) * 10;
            out->present |= SENSOR_HAS_HUM;
            break;
        case PROP_ECO2:
            out->eco2 = (int32_t)mesh_read_le_unsigned(val, data_len);
            out->present |= SENSOR_HAS_ECO2;
            break;
        case PROP_TVOC:
            out->tvoc = (int32_t)mesh_read_le_unsigned(val, data_len);
            out->present |= SENSOR_HAS_TVOC;
            break;
        case PROP_HEART_RATE:
            out->heart_rate = (int32_t)mesh_read_le_unsigned(val, data_len);
            out->present |= SENSOR_HAS_HEART_RATE;
            break;
        case PROP_SPO2:
            out->spo2 = (int32_t)mesh_read_le_unsigned(val, data_len) * 10;
            out->present |= SENSOR_HAS_SPO2;
            break;
        case PROP_RAW_RED:
            out->raw_red = (int32_t)mesh_read_le_unsigned(val, data_len);
            out->present |= SENSOR_HAS_RAW_RED;
            break;
        case PROP_RAW_IR:
            out->raw_ir = (int32_t)mesh_read_le_unsigned(val, data_len);
            out->present |= SENSOR_HAS_RAW_IR;
            break;
        case PROP_SWITCH:
            out->switch_state = (uint8_t)mesh_read_le_unsigned(val, data_len);
            out->present |= SENSOR_HAS_SWITCH;
            break;
        case PROP_LIGHT_STATE:
            out->light_on = (uint8_t)mesh_read_le_unsigned(val, data_len);
            out->present |= SENSOR_HAS_LIGHT;
            break;
        default:
            LOG_WRN("Unknown Property 0x%04X (%u bytes)", prop_id, data_len);
            break;
        }
    }

    return (out->present != 0);
}

uint16_t mesh_encode_property(uint8_t *buf, uint16_t buf_size,
                              uint16_t prop_id, const uint8_t *value,
                              uint8_t val_len)
{
    if (buf_size < (uint16_t)(2 + val_len)) return 0;
    if (val_len < 1 || val_len > 8) return 0;

    /* Format A MPID: bit0=0, bits1-4=len-1, bits5-15=prop_id */
    uint16_t mpid = (uint16_t)(((prop_id & 0x7FF) << 5) |
                               (((val_len - 1) & 0x0F) << 1));
    buf[0] = (uint8_t)(mpid & 0xFF);
    buf[1] = (uint8_t)(mpid >> 8);
    memcpy(&buf[2], value, val_len);
    return (uint16_t)(2 + val_len);
}