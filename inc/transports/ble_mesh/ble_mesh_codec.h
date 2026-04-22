/**
 * @file ble_mesh_codec.h
 * @brief Decoder for BLE Mesh sensor payloads.
 */

#ifndef BLE_MESH_CODEC_H_
#define BLE_MESH_CODEC_H_

#include <zephyr/kernel.h>
#include <stdint.h>

#include "gw_model.h"

/**
 * @brief Event callback type used by the BLE Mesh codec.
 *
 * @param evt       Parsed gateway event.
 * @param user_data User context pointer.
 */
typedef void (*ble_mesh_codec_emit_fn)(const gw_event_t *evt, void *user_data);

/**
 * @brief Parse a BLE Mesh Sensor Status payload into gateway events.
 *
 * Depending on the contained properties, this may emit:
 * - one sensor event
 * - one additional button event
 *
 * @param data       Raw payload bytes.
 * @param len        Payload length in bytes.
 * @param src_addr   Source mesh address.
 * @param rssi       Receive RSSI in dBm.
 * @param hops       Estimated hop count.
 * @param emit       Event callback.
 * @param emit_user  User context passed to @p emit.
 *
 * @retval 0 Parsing completed
 */
int ble_mesh_codec_parse_sensor_status(const uint8_t *data,
                                       uint16_t len,
                                       uint16_t src_addr,
                                       int8_t rssi,
                                       uint8_t hops,
                                       ble_mesh_codec_emit_fn emit,
                                       void *emit_user);

#endif /* BLE_MESH_CODEC_H_ */