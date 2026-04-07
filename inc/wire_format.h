#ifndef WIRE_FORMAT_H
#define WIRE_FORMAT_H

/**
 * @file wire_format.h
 * @brief LoRa binary wire format serializer/deserializer.
 *
 * Extracted from data_handler.c to allow unit testing.
 *
 * Frame layout (worst case = 36 bytes):
 *
 *   Offset  Size  Field
 *   ──────  ────  ──────────────────────────────────────────────
 *   0       1     node_idx
 *   1       1     transport  (0=Thread, 1=BLE Mesh)
 *   2       4     present bitmask (little-endian, low 32 bits)
 *   ── then only fields whose bit is set, in order: ──────────
 *   +0      2     ax  [int16_t, mg, little-endian]
 *   +2      2     ay
 *   +4      2     az
 *   +6      2     gx  [int16_t, mdeg/s]
 *   +8      2     gy
 *   +10     2     gz
 *   +12     2     temp [int16_t, m°C / 10 → 0.1°C resolution]
 *   +14     2     hum  [int16_t, m%RH / 10]
 *   +16     2     tvoc [uint16_t, ppb]
 *   +18     2     eco2 [uint16_t, ppm]
 *   +20     1     heart_rate [uint8_t, bpm]
 *   +21     2     spo2 [uint16_t, m% / 10]
 *   +23     2     pm25 [uint16_t, µg/m³]
 *   +25     2     pm10 [uint16_t, µg/m³]
 *   +27     1     switch_state [uint8_t]
 */

#include <stdint.h>
#include "data_handler.h"

#define WIRE_MAX_SIZE 50  /* 6-byte header + max payload */

/**
 * @brief Serialize sensor data into LoRa binary wire format.
 *
 * @param d    Source sensor data
 * @param buf  Output buffer (must be >= WIRE_MAX_SIZE bytes)
 * @param size Buffer size
 * @return Number of bytes written, or 0 on error
 */
int wire_build(const struct node_sensor_data *d, uint8_t *buf, size_t size);

/**
 * @brief Deserialize LoRa binary wire format back into sensor data.
 * Used for verification in tests and on the receiver side.
 *
 * @param buf  Input buffer
 * @param len  Buffer length
 * @param out  Output sensor data (zeroed by caller)
 * @return true on success, false on malformed input
 */
bool wire_parse(const uint8_t *buf, size_t len, struct node_sensor_data *out);

#endif /* WIRE_FORMAT_H */