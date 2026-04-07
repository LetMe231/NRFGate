/**
 * @file tests/unit/test_wire_format/src/main.c
 *
 * Unit-Tests für wire_format.c (LoRa Binary Serializer/Deserializer).
 *
 * Was getestet wird:
 *   - Header korrekt (node_idx, transport, present bitmask)
 *   - Nur vorhandene Felder werden serialisiert
 *   - Korrekte Skalierung (temp /10, spo2 /10)
 *   - Little-Endian Byte-Reihenfolge
 *   - Maximale Größe nie überschritten
 *   - Round-Trip: serialize → deserialize → gleiche Werte
 *   - Leerer Payload: nur Header (4 Bytes)
 *   - Truncated Buffer: gibt 0 zurück
 */

#include <zephyr/ztest.h>
#include <string.h>
#include <stdint.h>

#include "wire_format.h"
#include "data_handler.h"

/* ── Helper: Node-Sensor-Data erstellen ──────────────────────── */
static struct node_sensor_data make_d(uint8_t node_idx,
                                      node_transport_t transport)
{
    struct node_sensor_data d = {0};
    d.node_idx               = node_idx;
    d.identity.transport     = transport;
    return d;
}

/* ── Suite Setup ─────────────────────────────────────────────── */
ZTEST_SUITE(wire_format, NULL, NULL, NULL, NULL, NULL);

/* ════════════════════════════════════════════════════════════════
 * Header
 * ════════════════════════════════════════════════════════════════ */

ZTEST(wire_format, test_header_node_idx)
{
    struct node_sensor_data d = make_d(5, NODE_TRANSPORT_THREAD);
    uint8_t buf[WIRE_MAX_SIZE];
    int len = wire_build(&d, buf, sizeof(buf));

    zassert_true(len >= 4, "Must have at least header");
    zassert_equal(buf[0], 5, "node_idx mismatch");
}

ZTEST(wire_format, test_header_transport_thread)
{
    struct node_sensor_data d = make_d(0, NODE_TRANSPORT_THREAD);
    uint8_t buf[WIRE_MAX_SIZE];
    wire_build(&d, buf, sizeof(buf));
    zassert_equal(buf[1], NODE_TRANSPORT_THREAD, "transport mismatch");
}

ZTEST(wire_format, test_header_transport_ble)
{
    struct node_sensor_data d = make_d(0, NODE_TRANSPORT_BLE_MESH);
    uint8_t buf[WIRE_MAX_SIZE];
    wire_build(&d, buf, sizeof(buf));
    zassert_equal(buf[1], NODE_TRANSPORT_BLE_MESH, "transport mismatch");
}

ZTEST(wire_format, test_header_present_bitmask_little_endian)
{
    struct node_sensor_data d = make_d(0, NODE_TRANSPORT_THREAD);
    d.payload.present = SENSOR_HAS_TEMP | SENSOR_HAS_ECO2;
    uint8_t buf[WIRE_MAX_SIZE];
    wire_build(&d, buf, sizeof(buf));

    uint16_t present_from_wire = (uint16_t)(buf[2] | (buf[3] << 8));
    zassert_equal(present_from_wire,
                  (uint16_t)(SENSOR_HAS_TEMP | SENSOR_HAS_ECO2),
                  "Present bitmask not little-endian");
}

/* ════════════════════════════════════════════════════════════════
 * Leerer Payload
 * ════════════════════════════════════════════════════════════════ */

ZTEST(wire_format, test_empty_payload_is_6_bytes)
{
    struct node_sensor_data d = make_d(0, NODE_TRANSPORT_THREAD);
    d.payload.present = 0;
    uint8_t buf[WIRE_MAX_SIZE];
    int len = wire_build(&d, buf, sizeof(buf));
    zassert_equal(len, 6, "Empty payload should be 6 bytes (header)");
}

/* ════════════════════════════════════════════════════════════════
 * Skalierung
 * ════════════════════════════════════════════════════════════════ */

ZTEST(wire_format, test_temp_scaling_divide_by_10)
{
    /* temp = 22500 m°C (22.5°C) → wire: 2250 (int16_t LE) */
    struct node_sensor_data d = make_d(0, NODE_TRANSPORT_THREAD);
    d.payload.present = SENSOR_HAS_TEMP;
    d.payload.temp    = 22500;
    uint8_t buf[WIRE_MAX_SIZE];
    wire_build(&d, buf, sizeof(buf));

    int16_t wire_temp = (int16_t)(buf[6] | (buf[7] << 8));
    zassert_equal(wire_temp, 2250,
                  "temp should be divided by 10 on wire (got %d)", wire_temp);
}

ZTEST(wire_format, test_temp_negative_scaling)
{
    /* temp = -5000 m°C (-5.0°C) → wire: -500 */
    struct node_sensor_data d = make_d(0, NODE_TRANSPORT_THREAD);
    d.payload.present = SENSOR_HAS_TEMP;
    d.payload.temp    = -5000;
    uint8_t buf[WIRE_MAX_SIZE];
    wire_build(&d, buf, sizeof(buf));

    int16_t wire_temp = (int16_t)(buf[6] | (buf[7] << 8));
    zassert_equal(wire_temp, -500, "Negative temp scaling failed");
}

ZTEST(wire_format, test_spo2_scaling_divide_by_10)
{
    /* spo2 = 98000 m% (98.0%) → wire: 9800 */
    struct node_sensor_data d = make_d(0, NODE_TRANSPORT_THREAD);
    d.payload.present    = SENSOR_HAS_SPO2;
    d.payload.spo2       = 98000;
    uint8_t buf[WIRE_MAX_SIZE];
    wire_build(&d, buf, sizeof(buf));

    /* SpO2 steht nach Header (6) */
    uint16_t wire_spo2 = (uint16_t)(buf[6] | (buf[7] << 8));
    zassert_equal(wire_spo2, 9800, "SpO2 scaling failed (got %d)", wire_spo2);
}

ZTEST(wire_format, test_no_scaling_for_eco2)
{
    /* eco2 = 1234 ppm → wire: 1234 (no scaling) */
    struct node_sensor_data d = make_d(0, NODE_TRANSPORT_THREAD);
    d.payload.present = SENSOR_HAS_ECO2;
    d.payload.eco2    = 1234;
    uint8_t buf[WIRE_MAX_SIZE];
    wire_build(&d, buf, sizeof(buf));

    uint16_t wire_eco2 = (uint16_t)(buf[6] | (buf[7] << 8));
    zassert_equal(wire_eco2, 1234, "eCO2 should not be scaled");
}

/* ════════════════════════════════════════════════════════════════
 * Selektive Felder
 * ════════════════════════════════════════════════════════════════ */

ZTEST(wire_format, test_only_present_fields_written)
{
    /* Nur TEMP → 6 Header + 2 Bytes temp */
    struct node_sensor_data d = make_d(0, NODE_TRANSPORT_THREAD);
    d.payload.present = SENSOR_HAS_TEMP;
    d.payload.temp    = 20000;
    uint8_t buf[WIRE_MAX_SIZE];
    int len = wire_build(&d, buf, sizeof(buf));

    zassert_equal(len, 8, "Only temp: header(6) + temp(2) = 8 bytes");
}

ZTEST(wire_format, test_accel_needs_all_three_axes)
{
    /* Nur SENSOR_HAS_AX gesetzt (nicht ACCEL-Gruppe) → kein Schreiben */
    struct node_sensor_data d = make_d(0, NODE_TRANSPORT_THREAD);
    d.payload.present = SENSOR_HAS_AX;  /* nur ein Bit, nicht alle drei */
    d.payload.ax      = 1000;
    uint8_t buf[WIRE_MAX_SIZE];
    int len = wire_build(&d, buf, sizeof(buf));

    zassert_equal(len, 6, "Partial accel should not be written");
}

ZTEST(wire_format, test_field_order_temp_before_eco2)
{
    /* Temp und ECO2 vorhanden — Temp muss zuerst kommen */
    struct node_sensor_data d = make_d(0, NODE_TRANSPORT_THREAD);
    d.payload.present = SENSOR_HAS_TEMP | SENSOR_HAS_ECO2;
    d.payload.temp    = 22500;  /* wire: 2250 */
    d.payload.eco2    = 800;
    uint8_t buf[WIRE_MAX_SIZE];
    wire_build(&d, buf, sizeof(buf));

    int16_t  wire_temp = (int16_t)(buf[6] | (buf[7] << 8));
    uint16_t wire_eco2 = (uint16_t)(buf[8] | (buf[9] << 8));

    zassert_equal(wire_temp, 2250, "Temp at wrong offset");
    zassert_equal(wire_eco2, 800,  "ECO2 at wrong offset");
}

/* ════════════════════════════════════════════════════════════════
 * Maximale Größe
 * ════════════════════════════════════════════════════════════════ */

ZTEST(wire_format, test_max_payload_fits_in_wire_max_size)
{
    struct node_sensor_data d = make_d(7, NODE_TRANSPORT_BLE_MESH);
    d.payload.present = SENSOR_HAS_ACCEL | SENSOR_HAS_GYRO |
                        SENSOR_HAS_TEMP  | SENSOR_HAS_HUM  |
                        SENSOR_HAS_TVOC  | SENSOR_HAS_ECO2 |
                        SENSOR_HAS_HEART_RATE | SENSOR_HAS_SPO2 |
                        SENSOR_HAS_PM25  | SENSOR_HAS_PM10 |
                        SENSOR_HAS_SWITCH;
    d.payload.ax = 1000; d.payload.ay = -500; d.payload.az = 9810;
    d.payload.gx = 100;  d.payload.gy = -50;  d.payload.gz = 10;
    d.payload.temp = 22500; d.payload.hum = 60000;
    d.payload.tvoc = 300;   d.payload.eco2 = 800;
    d.payload.heart_rate = 72; d.payload.spo2 = 98000;
    d.payload.pm25 = 15;    d.payload.pm10 = 30;
    d.payload.switch_state = 1;

    uint8_t buf[WIRE_MAX_SIZE];
    int len = wire_build(&d, buf, sizeof(buf));

    zassert_true(len > 0,             "Full payload should serialize");
    zassert_true(len <= WIRE_MAX_SIZE, "Must fit in WIRE_MAX_SIZE (%d bytes, got %d)",
                 WIRE_MAX_SIZE, len);
}

ZTEST(wire_format, test_buffer_too_small_returns_zero)
{
    struct node_sensor_data d = make_d(0, NODE_TRANSPORT_THREAD);
    d.payload.present = SENSOR_HAS_TEMP;
    uint8_t tiny[3];  /* kleiner als Header */
    int len = wire_build(&d, tiny, sizeof(tiny));
    zassert_equal(len, 0, "Too-small buffer should return 0");
}

/* ════════════════════════════════════════════════════════════════
 * Round-Trip: serialize → deserialize → gleiche Werte
 * ════════════════════════════════════════════════════════════════ */

ZTEST(wire_format, test_roundtrip_temp_hum)
{
    struct node_sensor_data orig = make_d(3, NODE_TRANSPORT_THREAD);
    orig.payload.present = SENSOR_HAS_TEMP | SENSOR_HAS_HUM;
    orig.payload.temp    = 22500;   /* 22.5°C */
    orig.payload.hum     = 65000;   /* 65.0% */

    uint8_t buf[WIRE_MAX_SIZE];
    int len = wire_build(&orig, buf, sizeof(buf));
    zassert_true(len > 0, "Build failed");

    struct node_sensor_data decoded = {0};
    bool ok = wire_parse(buf, len, &decoded);
    zassert_true(ok, "Parse failed");

    zassert_equal(decoded.node_idx,       orig.node_idx,   "node_idx");
    zassert_equal(decoded.payload.present, orig.payload.present, "present");
    zassert_equal(decoded.payload.temp,   orig.payload.temp,    "temp");
    zassert_equal(decoded.payload.hum,    orig.payload.hum,     "hum");
}

ZTEST(wire_format, test_roundtrip_full_imu)
{
    struct node_sensor_data orig = make_d(1, NODE_TRANSPORT_BLE_MESH);
    orig.payload.present = SENSOR_HAS_ACCEL | SENSOR_HAS_GYRO;
    orig.payload.ax = 1234; orig.payload.ay = -567; orig.payload.az = 9810;
    orig.payload.gx = 100;  orig.payload.gy = -50;  orig.payload.gz = 25;

    uint8_t buf[WIRE_MAX_SIZE];
    int len = wire_build(&orig, buf, sizeof(buf));

    struct node_sensor_data decoded = {0};
    wire_parse(buf, len, &decoded);

    zassert_equal(decoded.payload.ax, orig.payload.ax, "ax");
    zassert_equal(decoded.payload.ay, orig.payload.ay, "ay");
    zassert_equal(decoded.payload.az, orig.payload.az, "az");
    zassert_equal(decoded.payload.gx, orig.payload.gx, "gx");
    zassert_equal(decoded.payload.gy, orig.payload.gy, "gy");
    zassert_equal(decoded.payload.gz, orig.payload.gz, "gz");
}

ZTEST(wire_format, test_roundtrip_biometric)
{
    struct node_sensor_data orig = make_d(2, NODE_TRANSPORT_BLE_MESH);
    orig.payload.present    = SENSOR_HAS_HEART_RATE | SENSOR_HAS_SPO2;
    orig.payload.heart_rate = 72;
    orig.payload.spo2       = 98000;  /* 98.0% */

    uint8_t buf[WIRE_MAX_SIZE];
    int len = wire_build(&orig, buf, sizeof(buf));

    struct node_sensor_data decoded = {0};
    wire_parse(buf, len, &decoded);

    zassert_equal(decoded.payload.heart_rate, 72,    "heart_rate");
    zassert_equal(decoded.payload.spo2,       98000, "spo2");
}

ZTEST(wire_format, test_roundtrip_switch)
{
    struct node_sensor_data orig = make_d(0, NODE_TRANSPORT_THREAD);
    orig.payload.present     = SENSOR_HAS_SWITCH;
    orig.payload.switch_state = 1;

    uint8_t buf[WIRE_MAX_SIZE];
    int len = wire_build(&orig, buf, sizeof(buf));

    struct node_sensor_data decoded = {0};
    wire_parse(buf, len, &decoded);

    zassert_equal(decoded.payload.switch_state, 1, "switch_state");
}

ZTEST(wire_format, test_parse_truncated_returns_false)
{
    struct node_sensor_data orig = make_d(0, NODE_TRANSPORT_THREAD);
    orig.payload.present = SENSOR_HAS_TEMP;
    orig.payload.temp    = 22000;

    uint8_t buf[WIRE_MAX_SIZE];
    int len = wire_build(&orig, buf, sizeof(buf));

    /* Nur halbes Paket übergeben */
    struct node_sensor_data decoded = {0};
    bool ok = wire_parse(buf, len / 2, &decoded);
    zassert_false(ok, "Truncated buffer should return false");
}