/**
 * @file tests/unit/test_mesh_parser/src/main.c
 *
 * Unit-Tests für mesh_parser.c (BLE Mesh Sensor Status Parser).
 *
 * Was getestet wird:
 *   - MPID Format A: korrekte Dekodierung von prop_id und data_len
 *   - MPID Format B: wird korrekt abgelehnt
 *   - Little-Endian Reader: signed/unsigned, 1/2/4 Bytes
 *   - Sensor Status Parser:
 *       - Temperatur mit Skalierung (*10)
 *       - Humidity mit Skalierung (*10)
 *       - eCO2, TVOC ohne Skalierung
 *       - Herz, SpO2
 *       - Switch: nur gesetzt wenn state != 0
 *       - Mehrere Properties in einem Paket
 *       - Leeres Paket → false
 *       - Truncated Payload → graceful stop
 *       - Unbekannte Property → wird übersprungen
 */

#include <zephyr/ztest.h>
#include <string.h>
#include <stdint.h>

#include "mesh_parser.h"
#include "data_handler.h"

/* ── Suite Setup ─────────────────────────────────────────────── */
ZTEST_SUITE(mesh_parser, NULL, NULL, NULL, NULL, NULL);

/* ════════════════════════════════════════════════════════════════
 * MPID Format A Decoder
 * ════════════════════════════════════════════════════════════════ */

ZTEST(mesh_parser, test_mpid_format_a_1byte)
{
    /* prop_id=0x004F (TEMPERATURE), len=2 bytes (len_field=1 → +1=2)
     * MPID = (0x004F << 5) | (1 << 1) | 0 = 0x09E2 */
    uint16_t prop_id; uint8_t data_len;
    uint16_t mpid = (uint16_t)(((PROP_TEMPERATURE & 0x7FF) << 5) | ((2-1) << 1));
    bool ok = mesh_parse_mpid_format_a(mpid, &prop_id, &data_len);

    zassert_true(ok, "Format A should return true");
    zassert_equal(prop_id,  PROP_TEMPERATURE, "Wrong prop_id");
    zassert_equal(data_len, 2,                "Wrong data_len");
}

ZTEST(mesh_parser, test_mpid_format_a_4byte)
{
    /* 4 Bytes: len_field = 3 */
    uint16_t prop_id; uint8_t data_len;
    uint16_t mpid = (uint16_t)(((0x0008 & 0x7FF) << 5) | ((4-1) << 1));
    bool ok = mesh_parse_mpid_format_a(mpid, &prop_id, &data_len);

    zassert_true(ok, "Format A 4-byte should return true");
    zassert_equal(prop_id,  0x0008, "Wrong prop_id");
    zassert_equal(data_len, 4,      "Wrong data_len");
}

ZTEST(mesh_parser, test_mpid_format_b_rejected)
{
    /* Format B: bit 0 gesetzt */
    uint16_t prop_id; uint8_t data_len;
    uint16_t mpid = 0x0001;  /* bit 0 = 1 → Format B */
    bool ok = mesh_parse_mpid_format_a(mpid, &prop_id, &data_len);

    zassert_false(ok, "Format B should return false");
}

ZTEST(mesh_parser, test_mpid_all_known_properties)
{
    /* Alle bekannten Properties können korrekt dekodiert werden */
    static const struct { uint16_t id; uint8_t len; } props[] = {
        { PROP_TEMPERATURE, 2 },
        { PROP_HUMIDITY,    2 },
        { PROP_ECO2,        2 },
        { PROP_TVOC,        2 },
        { PROP_HEART_RATE,  1 },
        { PROP_SPO2,        2 },
        { PROP_RAW_RED,     4 },
        { PROP_RAW_IR,      4 },
        { PROP_SWITCH,      1 },
        { PROP_SENSOR_SEQ,  2 },
    };

    for (int i = 0; i < ARRAY_SIZE(props); i++) {
        uint16_t mpid = (uint16_t)(((props[i].id & 0x7FF) << 5) |
                                   ((props[i].len - 1) << 1));
        uint16_t prop_id; uint8_t data_len;
        bool ok = mesh_parse_mpid_format_a(mpid, &prop_id, &data_len);

        zassert_true(ok,                          "prop 0x%04X: Format A rejected", props[i].id);
        zassert_equal(prop_id,  props[i].id,      "prop 0x%04X: wrong prop_id", props[i].id);
        zassert_equal(data_len, props[i].len,     "prop 0x%04X: wrong len", props[i].id);
    }
}

/* ════════════════════════════════════════════════════════════════
 * Little-Endian Reader
 * ════════════════════════════════════════════════════════════════ */

ZTEST(mesh_parser, test_read_le_signed_1byte_positive)
{
    uint8_t data[] = { 0x7F };
    zassert_equal(mesh_read_le_signed(data, 1), 127, "1-byte positive");
}

ZTEST(mesh_parser, test_read_le_signed_1byte_negative)
{
    uint8_t data[] = { 0xFF };
    zassert_equal(mesh_read_le_signed(data, 1), -1, "1-byte negative (0xFF = -1)");
}

ZTEST(mesh_parser, test_read_le_signed_2byte)
{
    /* -500 in little-endian = 0x0C FF (but signed: 0xFF0C = -244... 
     * Let's use a cleaner example: 2250 = 0x08CA → bytes 0xCA, 0x08 */
    uint8_t data[] = { 0xCA, 0x08 };  /* 0x08CA = 2250 */
    zassert_equal(mesh_read_le_signed(data, 2), 2250, "2-byte LE positive");
}

ZTEST(mesh_parser, test_read_le_signed_2byte_negative)
{
    /* -500 = 0xFE0C → bytes 0x0C, 0xFE */
    uint8_t data[] = { 0x0C, 0xFE };
    zassert_equal(mesh_read_le_signed(data, 2), -500, "2-byte LE negative");
}

ZTEST(mesh_parser, test_read_le_unsigned_2byte)
{
    /* 1234 = 0x04D2 → bytes 0xD2, 0x04 */
    uint8_t data[] = { 0xD2, 0x04 };
    zassert_equal(mesh_read_le_unsigned(data, 2), 1234, "2-byte LE unsigned");
}

ZTEST(mesh_parser, test_read_le_unsigned_1byte)
{
    uint8_t data[] = { 0xAB };
    zassert_equal(mesh_read_le_unsigned(data, 1), 0xAB, "1-byte unsigned");
}

ZTEST(mesh_parser, test_read_le_unsigned_4byte)
{
    /* 0x12345678 → bytes: 0x78, 0x56, 0x34, 0x12 */
    uint8_t data[] = { 0x78, 0x56, 0x34, 0x12 };
    zassert_equal(mesh_read_le_unsigned(data, 4), 0x12345678, "4-byte LE unsigned");
}

ZTEST(mesh_parser, test_read_le_invalid_len_returns_zero)
{
    uint8_t data[] = { 0xFF, 0xFF, 0xFF };
    zassert_equal(mesh_read_le_signed(data, 3),   0, "Invalid len signed should be 0");
    zassert_equal(mesh_read_le_unsigned(data, 3), 0, "Invalid len unsigned should be 0");
}

/* ════════════════════════════════════════════════════════════════
 * Sensor Status Parser — Einzelne Properties
 * ════════════════════════════════════════════════════════════════ */

/* Helper: Baue ein Sensor-Status-Paket mit einer einzelnen Property */
static uint16_t build_single_prop(uint8_t *buf, uint16_t buf_size,
                                  uint16_t prop_id,
                                  const uint8_t *value, uint8_t val_len)
{
    return mesh_encode_property(buf, buf_size, prop_id, value, val_len);
}

ZTEST(mesh_parser, test_parse_temperature)
{
    /* 22.5°C → BLE Mesh: 2250 (0.01°C steps) → wire: 2250 LE
     * Gateway: *10 → 22500 m°C */
    int16_t raw = 2250;
    uint8_t val[2] = { (uint8_t)(raw & 0xFF), (uint8_t)(raw >> 8) };
    uint8_t buf[16];
    uint16_t len = build_single_prop(buf, sizeof(buf), PROP_TEMPERATURE, val, 2);

    struct sensor_payload out = {0};
    bool ok = mesh_parse_sensor_status(buf, len, &out);

    zassert_true(ok, "Should parse successfully");
    zassert_true(out.present & SENSOR_HAS_TEMP, "SENSOR_HAS_TEMP not set");
    zassert_equal(out.temp, 22500, "temp: expected 22500, got %d", out.temp);
}

ZTEST(mesh_parser, test_parse_temperature_negative)
{
    /* -5°C → raw: -500 → gateway: -5000 m°C */
    int16_t raw = -500;
    uint8_t val[2] = { (uint8_t)(raw & 0xFF), (uint8_t)((raw >> 8) & 0xFF) };
    uint8_t buf[16];
    uint16_t len = build_single_prop(buf, sizeof(buf), PROP_TEMPERATURE, val, 2);

    struct sensor_payload out = {0};
    mesh_parse_sensor_status(buf, len, &out);

    zassert_equal(out.temp, -5000, "Negative temp failed (got %d)", out.temp);
}

ZTEST(mesh_parser, test_parse_humidity)
{
    /* 65.0%RH → raw: 6500 (0.01% steps) → gateway: *10 = 65000 */
    uint16_t raw = 6500;
    uint8_t val[2] = { (uint8_t)(raw & 0xFF), (uint8_t)(raw >> 8) };
    uint8_t buf[16];
    uint16_t len = build_single_prop(buf, sizeof(buf), PROP_HUMIDITY, val, 2);

    struct sensor_payload out = {0};
    mesh_parse_sensor_status(buf, len, &out);

    zassert_true(out.present & SENSOR_HAS_HUM, "SENSOR_HAS_HUM not set");
    zassert_equal(out.hum, 65000, "hum: expected 65000, got %d", out.hum);
}

ZTEST(mesh_parser, test_parse_eco2_no_scaling)
{
    /* 800 ppm → keine Skalierung */
    uint16_t raw = 800;
    uint8_t val[2] = { (uint8_t)(raw & 0xFF), (uint8_t)(raw >> 8) };
    uint8_t buf[16];
    uint16_t len = build_single_prop(buf, sizeof(buf), PROP_ECO2, val, 2);

    struct sensor_payload out = {0};
    mesh_parse_sensor_status(buf, len, &out);

    zassert_true(out.present & SENSOR_HAS_ECO2, "SENSOR_HAS_ECO2 not set");
    zassert_equal(out.eco2, 800, "eco2 should not be scaled");
}

ZTEST(mesh_parser, test_parse_tvoc_no_scaling)
{
    uint16_t raw = 350;
    uint8_t val[2] = { (uint8_t)(raw & 0xFF), (uint8_t)(raw >> 8) };
    uint8_t buf[16];
    uint16_t len = build_single_prop(buf, sizeof(buf), PROP_TVOC, val, 2);

    struct sensor_payload out = {0};
    mesh_parse_sensor_status(buf, len, &out);

    zassert_equal(out.tvoc, 350, "tvoc should not be scaled");
}

ZTEST(mesh_parser, test_parse_heart_rate)
{
    uint8_t val[1] = { 72 };
    uint8_t buf[8];
    uint16_t len = build_single_prop(buf, sizeof(buf), PROP_HEART_RATE, val, 1);

    struct sensor_payload out = {0};
    mesh_parse_sensor_status(buf, len, &out);

    zassert_true(out.present & SENSOR_HAS_HEART_RATE, "SENSOR_HAS_HEART_RATE not set");
    zassert_equal(out.heart_rate, 72, "heart_rate wrong");
}

ZTEST(mesh_parser, test_parse_spo2_scaling)
{
    /* 98.0% SpO2 → raw: 9800 (0.01% steps) → gateway: *10 = 98000 */
    uint16_t raw = 9800;
    uint8_t val[2] = { (uint8_t)(raw & 0xFF), (uint8_t)(raw >> 8) };
    uint8_t buf[16];
    uint16_t len = build_single_prop(buf, sizeof(buf), PROP_SPO2, val, 2);

    struct sensor_payload out = {0};
    mesh_parse_sensor_status(buf, len, &out);

    zassert_equal(out.spo2, 98000, "SpO2 scaling wrong (got %d)", out.spo2);
}

ZTEST(mesh_parser, test_parse_switch_on)
{
    uint8_t val[1] = { 1 };
    uint8_t buf[8];
    uint16_t len = build_single_prop(buf, sizeof(buf), PROP_SWITCH, val, 1);

    struct sensor_payload out = {0};
    mesh_parse_sensor_status(buf, len, &out);

    zassert_true(out.present & SENSOR_HAS_SWITCH, "Switch ON should set SENSOR_HAS_SWITCH");
    zassert_equal(out.switch_state, 1, "switch_state should be 1");
}

ZTEST(mesh_parser, test_parse_switch_off_not_in_present)
{
    /* Switch=0 soll SENSOR_HAS_SWITCH NICHT setzen */
    uint8_t val[1] = { 0 };
    uint8_t buf[8];
    uint16_t len = build_single_prop(buf, sizeof(buf), PROP_SWITCH, val, 1);

    struct sensor_payload out = {0};
    mesh_parse_sensor_status(buf, len, &out);

    zassert_false(out.present & SENSOR_HAS_SWITCH,
                  "Switch=0 should NOT set SENSOR_HAS_SWITCH");
}

/* ════════════════════════════════════════════════════════════════
 * Mehrere Properties in einem Paket
 * ════════════════════════════════════════════════════════════════ */

ZTEST(mesh_parser, test_parse_multiple_properties)
{
    /* Temp + Humidity + ECO2 in einem Paket */
    uint8_t buf[64];
    uint16_t off = 0;

    int16_t temp_raw = 2250;
    uint8_t temp_val[2] = { (uint8_t)(temp_raw & 0xFF), (uint8_t)(temp_raw >> 8) };
    off += mesh_encode_property(buf + off, sizeof(buf) - off,
                                PROP_TEMPERATURE, temp_val, 2);

    uint16_t hum_raw = 6000;
    uint8_t hum_val[2] = { (uint8_t)(hum_raw & 0xFF), (uint8_t)(hum_raw >> 8) };
    off += mesh_encode_property(buf + off, sizeof(buf) - off,
                                PROP_HUMIDITY, hum_val, 2);

    uint16_t eco2_raw = 1200;
    uint8_t eco2_val[2] = { (uint8_t)(eco2_raw & 0xFF), (uint8_t)(eco2_raw >> 8) };
    off += mesh_encode_property(buf + off, sizeof(buf) - off,
                                PROP_ECO2, eco2_val, 2);

    struct sensor_payload out = {0};
    bool ok = mesh_parse_sensor_status(buf, off, &out);

    zassert_true(ok, "Multi-property parse should succeed");
    zassert_true(out.present & SENSOR_HAS_TEMP, "TEMP missing");
    zassert_true(out.present & SENSOR_HAS_HUM,  "HUM missing");
    zassert_true(out.present & SENSOR_HAS_ECO2, "ECO2 missing");
    zassert_equal(out.temp, 22500, "temp wrong");
    zassert_equal(out.hum,  60000, "hum wrong");
    zassert_equal(out.eco2, 1200,  "eco2 wrong");
}

/* ════════════════════════════════════════════════════════════════
 * Edge Cases
 * ════════════════════════════════════════════════════════════════ */

ZTEST(mesh_parser, test_parse_empty_payload_returns_false)
{
    uint8_t buf[4] = {0};
    struct sensor_payload out = {0};
    bool ok = mesh_parse_sensor_status(buf, 0, &out);
    zassert_false(ok, "Empty payload should return false");
}

ZTEST(mesh_parser, test_parse_only_1_byte_returns_false)
{
    /* Kein vollständiges MPID möglich */
    uint8_t buf[1] = { 0x42 };
    struct sensor_payload out = {0};
    bool ok = mesh_parse_sensor_status(buf, 1, &out);
    zassert_false(ok, "1-byte payload cannot contain a valid MPID");
}

ZTEST(mesh_parser, test_parse_truncated_value_stops_gracefully)
{
    /* MPID für TEMP vorhanden, aber nur 1 statt 2 Bytes Wert */
    uint8_t buf[3];
    uint16_t mpid = (uint16_t)(((PROP_TEMPERATURE & 0x7FF) << 5) | ((2-1) << 1));
    buf[0] = (uint8_t)(mpid & 0xFF);
    buf[1] = (uint8_t)(mpid >> 8);
    buf[2] = 0xCA;  /* nur 1 Byte, braucht 2 */

    struct sensor_payload out = {0};
    /* Soll nicht crashen, gibt false zurück */
    bool ok = mesh_parse_sensor_status(buf, 3, &out);
    zassert_false(ok, "Truncated value should return false (no complete property)");
    /* Kein Crash = Erfolg */
    zassert_true(true, "No crash on truncated input");
}

ZTEST(mesh_parser, test_parse_unknown_property_skipped)
{
    /* Unbekannte Property-ID, gefolgt von bekannter */
    uint8_t buf[32];
    uint16_t off = 0;

    /* Unbekannte Property 0x0FFF mit 2 Bytes Wert */
    uint16_t unknown_prop = 0x0FFF;
    uint8_t unknown_val[2] = { 0xDE, 0xAD };
    off += mesh_encode_property(buf + off, sizeof(buf) - off,
                                unknown_prop, unknown_val, 2);

    /* Danach bekannte Property (TVOC) */
    uint16_t tvoc_raw = 200;
    uint8_t tvoc_val[2] = { (uint8_t)(tvoc_raw & 0xFF), (uint8_t)(tvoc_raw >> 8) };
    off += mesh_encode_property(buf + off, sizeof(buf) - off,
                                PROP_TVOC, tvoc_val, 2);

    struct sensor_payload out = {0};
    bool ok = mesh_parse_sensor_status(buf, off, &out);

    zassert_true(ok, "Should succeed (TVOC recognized)");
    zassert_true(out.present & SENSOR_HAS_TVOC, "TVOC should be parsed after unknown prop");
    zassert_equal(out.tvoc, 200, "tvoc value wrong");
}

ZTEST(mesh_parser, test_parse_format_b_stops_parsing)
{
    /* Format B MPID am Anfang → sofortiger Stop */
    uint8_t buf[8] = { 0x01, 0x00, 0x00, 0x00 };  /* bit0=1 = Format B */
    struct sensor_payload out = {0};
    bool ok = mesh_parse_sensor_status(buf, 4, &out);
    zassert_false(ok, "Format B should cause stop and return false");
}