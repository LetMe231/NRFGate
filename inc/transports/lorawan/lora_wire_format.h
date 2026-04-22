#ifndef WIRE_FORMAT_H
#define WIRE_FORMAT_H

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include "gw_model.h"

/* ── Packet Types ──────────────────────────────────────────────
 *
 * Byte 0 of every LoRa uplink identifies the packet type.
 */
#define LORA_PKT_SENSOR   0x01   /* Periodic sensor data          */
#define LORA_PKT_ALERT    0x02   /* Immediate state transition     */
#define LORA_PKT_SNAPSHOT 0x03   /* Compressed 60s snapshot        */

/* ── Wire Format — SENSOR / SNAPSHOT ──────────────────────────
 *
 * Byte  0:     Packet type (LORA_PKT_*)
 * Byte  1:     Sequence number (uint8, wraps at 255)
 * Byte  2-3:   Node ID (uint16 little-endian = dev_eui_lo[0:1])
 * Byte  4-7:   Presence mask (uint32 little-endian, GW_HAS_* bits)
 * Byte  8+:    Fields in presence-mask order, little-endian:
 *
 *   GW_HAS_ACCEL:      ax, ay, az  (3x int16, milli-g / 10)
 *   GW_HAS_GYRO:       gx, gy, gz  (3x int16, milli-dps / 10)
 *   GW_HAS_TEMP:       temp_mc     (int16,  milli-°C / 10)
 *   GW_HAS_HUM:        hum_mpermil (uint16, milli-% / 10)
 *   GW_HAS_TVOC:       tvoc_ppb    (uint16)
 *   GW_HAS_ECO2:       eco2_ppm    (uint16)
 *   GW_HAS_HEART_RATE: heart_rate  (uint8,  bpm)
 *   GW_HAS_SPO2:       spo2        (uint16, milli-% / 10)
 *   GW_HAS_PM25:       pm25_ugm3   (uint16)
 *   GW_HAS_PM10:       pm10_ugm3   (uint16)
 *   GW_HAS_SWITCH:     switch_state (uint8, 0/1)
 *   GW_HAS_LIGHT:      light_on    (uint8, 0/1)
 *
 * ── Wire Format — ALERT ────────────────────────────────────────
 *
 * Byte  0:     0x02
 * Byte  1:     Sequence number
 * Byte  2-3:   Node ID (uint16 little-endian)
 * Byte  4:     from_state (gw_state_t)
 * Byte  5:     to_state   (gw_state_t)
 *
 * Total: 6 bytes
 */

/* Maximum wire packet size in bytes */
#define WIRE_MAX_SIZE  64

/**
 * @brief Serialize a sensor event into a LoRa wire packet.
 *
 * @param evt   Source event — must be GW_EVT_SENSOR
 * @param buf   Output buffer
 * @param size  Buffer size — must be >= WIRE_MAX_SIZE
 * @return Number of bytes written, 0 on error
 */
int wire_build(const gw_event_t *evt, uint8_t *buf, size_t size);

/**
 * @brief Serialize a state transition into a LoRa ALERT packet.
 *
 * @param node_id   16-bit node identifier
 * @param seq       Sequence number
 * @param from      Previous state
 * @param to        New state
 * @param buf       Output buffer (must be >= 6 bytes)
 * @return Number of bytes written (always 6), 0 on error
 */
int wire_build_alert(uint16_t node_id, uint8_t seq,
                     gw_state_t from, gw_state_t to,
                     uint8_t *buf, size_t size);

/**
 * @brief Parse a LoRa wire packet into a gateway event.
 *        Handles SENSOR, ALERT and SNAPSHOT packet types.
 *        Node address fields (dev_eui_hi/lo) are set from node_id.
 *
 * @param buf   Raw received bytes
 * @param len   Number of received bytes
 * @param out   Output event — must not be NULL
 * @return true on success, false if packet is malformed or unknown type
 */
bool wire_parse(const uint8_t *buf, size_t len, gw_event_t *out);

#endif /* WIRE_FORMAT_H */