#ifndef MESH_PARSER_H
#define MESH_PARSER_H

/**
 * @file mesh_parser.h
 * @brief BLE Mesh Sensor Status payload parser.
 *
 * Extracted from ble_mesh_handler.c to allow unit testing.
 * ble_mesh_handler.c includes this header and calls these functions.
 */

#include <stdint.h>
#include <stdbool.h>
#include "data_handler.h"

/* ── Property IDs (BLE Mesh Sensor Model) ────────────────────── */
#define PROP_TEMPERATURE  0x004F
#define PROP_HUMIDITY     0x0076
#define PROP_ECO2         0x0008
#define PROP_HEART_RATE   0x0100
#define PROP_SPO2         0x0101
#define PROP_TVOC         0x0102
#define PROP_RAW_RED      0x0103
#define PROP_RAW_IR       0x0104
#define PROP_SWITCH       0x0105
#define PROP_LIGHT_STATE  0x0106
#define PROP_SENSOR_SEQ   0x07FF

/**
 * @brief Parse a Format A Marshalled Property ID.
 *
 * Layout:
 *   Bit 0      = format (0 = Format A, 1 = Format B → returns false)
 *   Bits 1–4   = length (zero-based: 0 = 1 byte, 1 = 2 bytes, ...)
 *   Bits 5–15  = Property ID (11 bits)
 *
 * @param mpid     Raw 16-bit MPID value (little-endian from wire)
 * @param prop_id  Output: decoded Property ID
 * @param data_len Output: decoded data length in bytes
 * @return true if Format A, false if Format B (unsupported)
 */
bool mesh_parse_mpid_format_a(uint16_t mpid, uint16_t *prop_id, uint8_t *data_len);

/**
 * @brief Read a little-endian signed integer (1, 2, or 4 bytes).
 */
int32_t mesh_read_le_signed(const uint8_t *data, uint8_t len);

/**
 * @brief Read a little-endian unsigned integer (1, 2, or 4 bytes).
 */
uint32_t mesh_read_le_unsigned(const uint8_t *data, uint8_t len);

/**
 * @brief Parse a full Sensor Status payload into a sensor_payload struct.
 *
 * Iterates over all MPID+value pairs in the payload and fills in
 * the fields of @p out that are recognized. Sets @p out->present bitmask.
 *
 * @param data   Raw payload bytes
 * @param len    Payload length in bytes
 * @param out    Output sensor_payload (must be zero-initialized by caller)
 * @return true if at least one property was recognized, false otherwise
 */
bool mesh_parse_sensor_status(const uint8_t *data, uint16_t len,
                              struct sensor_payload *out);

/**
 * @brief Build a Sensor Status payload for testing/simulation.
 *
 * Encodes a single property into a Format A MPID + value pair.
 * Useful for constructing test vectors.
 *
 * @param buf      Output buffer
 * @param buf_size Buffer size
 * @param prop_id  Property ID to encode
 * @param value    Value bytes (little-endian)
 * @param val_len  Length of value in bytes (1, 2, or 4)
 * @return bytes written, or 0 on error
 */
uint16_t mesh_encode_property(uint8_t *buf, uint16_t buf_size,
                              uint16_t prop_id, const uint8_t *value,
                              uint8_t val_len);

#endif /* MESH_PARSER_H */