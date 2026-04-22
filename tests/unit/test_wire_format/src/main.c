/**
 * @file tests/unit/test_wire_format/src/main.c
 * Unit-Tests für wire_format.c (LoRa Binary TLV Serializer)
 */

#include <zephyr/ztest.h>
#include <string.h>
#include <stdint.h>

#include "gw_model.h"
#include "wire_format.h"

ZTEST_SUITE(wire_format, NULL, NULL, NULL, NULL, NULL);

/* ── Helpers ──────────────────────────────────────────────────── */

static gw_event_t make_sensor_event(uint16_t mesh_addr)
{
    gw_event_t evt = {0};
    evt.type          = GW_EVT_SENSOR;
    evt.src.transport = GW_TR_BLE_MESH;
    evt.src.mesh_addr = mesh_addr;
    evt.rx_ms         = 1000;
    return evt;
}

/* ════════════════════════════════════════════════════════════════
 * Roundtrip: wire_build → wire_parse
 * ════════════════════════════════════════════════════════════════ */

ZTEST(wire_format, test_roundtrip_temp_hum)
{
    gw_event_t src = make_sensor_event(0x0002);
    src.data.sensor.present       = GW_HAS_TEMP | GW_HAS_HUM;
    src.data.sensor.temp_mc       = 22500;
    src.data.sensor.hum_mpermille = 650000;

    uint8_t buf[WIRE_MAX_SIZE];
    int len = wire_build(&src, buf, sizeof(buf));
    zassert_true(len > 0, "wire_build should produce output");

    gw_event_t dst = {0};
    bool ok = wire_parse(buf, (size_t)len, &dst);
    zassert_true(ok, "wire_parse should succeed");
    zassert_true(dst.data.sensor.present & GW_HAS_TEMP, "GW_HAS_TEMP missing");
    zassert_true(dst.data.sensor.present & GW_HAS_HUM,  "GW_HAS_HUM missing");
    zassert_equal(dst.data.sensor.temp_mc,       22500,  "temp_mc wrong");
    zassert_equal(dst.data.sensor.hum_mpermille, 650000, "hum_mpermille wrong");
}

ZTEST(wire_format, test_roundtrip_air_quality)
{
    gw_event_t src = make_sensor_event(0x0002);
    src.data.sensor.present  = GW_HAS_TVOC | GW_HAS_ECO2;
    src.data.sensor.tvoc_ppb = 350;
    src.data.sensor.eco2_ppm = 800;

    uint8_t buf[WIRE_MAX_SIZE];
    int len = wire_build(&src, buf, sizeof(buf));
    zassert_true(len > 0, "wire_build should produce output");

    gw_event_t dst = {0};
    bool ok = wire_parse(buf, (size_t)len, &dst);
    zassert_true(ok, "wire_parse should succeed");
    zassert_equal(dst.data.sensor.tvoc_ppb, 350, "tvoc_ppb wrong");
    zassert_equal(dst.data.sensor.eco2_ppm, 800, "eco2_ppm wrong");
}

ZTEST(wire_format, test_roundtrip_switch_event)
{
    /* wire_build only handles GW_EVT_SENSOR — button events return error.
     * Verify this documented limitation. */
    gw_event_t src = {0};
    src.type               = GW_EVT_BUTTON;
    src.src.transport      = GW_TR_BLE_MESH;
    src.src.mesh_addr      = 0x0004;
    src.rx_ms              = 1000;
    src.data.button.pressed = true;

    uint8_t buf[WIRE_MAX_SIZE];
    int len = wire_build(&src, buf, sizeof(buf));
    zassert_true(len <= 0, "wire_build should reject non-sensor event");
}

ZTEST(wire_format, test_roundtrip_all_sensor_fields)
{
    gw_event_t src = make_sensor_event(0x0002);
    src.data.sensor.present       = GW_HAS_TEMP | GW_HAS_HUM |
                                    GW_HAS_TVOC | GW_HAS_ECO2 |
                                    GW_HAS_ACCEL | GW_HAS_SEQ;
    src.data.sensor.temp_mc       = -5000;
    src.data.sensor.hum_mpermille = 850000;
    src.data.sensor.tvoc_ppb      = 1234;
    src.data.sensor.eco2_ppm      = 5678;
    src.data.sensor.ax_mg         = 9810;
    src.data.sensor.ay_mg         = -500;
    src.data.sensor.az_mg         = 0;
    src.data.sensor.seq           = 99;

    uint8_t buf[WIRE_MAX_SIZE];
    int len = wire_build(&src, buf, sizeof(buf));
    zassert_true(len > 0, "wire_build all fields should succeed");
    zassert_true(len <= WIRE_MAX_SIZE, "wire_build must not exceed WIRE_MAX_SIZE");

    gw_event_t dst = {0};
    bool ok = wire_parse(buf, (size_t)len, &dst);
    zassert_true(ok, "wire_parse all fields should succeed");
    zassert_equal(dst.data.sensor.temp_mc, -5000, "temp_mc wrong");
    zassert_equal(dst.data.sensor.tvoc_ppb, 1234, "tvoc_ppb wrong");
}

ZTEST(wire_format, test_roundtrip_negative_temperature)
{
    gw_event_t src = make_sensor_event(0x0002);
    src.data.sensor.present = GW_HAS_TEMP;
    src.data.sensor.temp_mc = -25000;  /* -25°C */

    uint8_t buf[WIRE_MAX_SIZE];
    int len = wire_build(&src, buf, sizeof(buf));
    gw_event_t dst = {0};
    wire_parse(buf, (size_t)len, &dst);

    zassert_equal(dst.data.sensor.temp_mc, -25000, "Negative temp roundtrip failed");
}

ZTEST(wire_format, test_roundtrip_seq_preserves_value)
{
    /* seq is serialized as uint8_t in wire format, max 255 */
    gw_event_t src = make_sensor_event(0x0002);
    src.data.sensor.present = GW_HAS_TEMP | GW_HAS_SEQ;
    src.data.sensor.temp_mc = 22000;
    src.data.sensor.seq     = 42;

    uint8_t buf[WIRE_MAX_SIZE];
    int len = wire_build(&src, buf, sizeof(buf));
    zassert_true(len > 0, "wire_build should succeed");
    gw_event_t dst = {0};
    wire_parse(buf, (size_t)len, &dst);

    zassert_true(dst.data.sensor.present & GW_HAS_SEQ, "GW_HAS_SEQ missing");
    zassert_equal(dst.data.sensor.seq, 42, "seq roundtrip failed");
}

/* ════════════════════════════════════════════════════════════════
 * wire_build — Fehlerbehandlung
 * ════════════════════════════════════════════════════════════════ */

ZTEST(wire_format, test_build_null_evt_returns_error)
{
    uint8_t buf[WIRE_MAX_SIZE];
    int len = wire_build(NULL, buf, sizeof(buf));
    zassert_true(len <= 0, "NULL evt should return error");
}

ZTEST(wire_format, test_build_null_buf_returns_error)
{
    gw_event_t evt = make_sensor_event(0x0002);
    int len = wire_build(&evt, NULL, 0);
    zassert_true(len <= 0, "NULL buf should return error");
}

ZTEST(wire_format, test_build_tiny_buf_returns_error)
{
    gw_event_t evt = make_sensor_event(0x0002);
    evt.data.sensor.present = GW_HAS_TEMP;
    evt.data.sensor.temp_mc = 22500;
    uint8_t buf[2];  /* zu klein */
    int len = wire_build(&evt, buf, sizeof(buf));
    /* Entweder Fehler oder 0 — Hauptsache kein Overflow */
    zassert_true(len <= (int)sizeof(buf),
                 "Must not write more than buf size");
}

ZTEST(wire_format, test_build_empty_present_still_valid)
{
    gw_event_t evt = make_sensor_event(0x0002);
    evt.data.sensor.present = GW_HAS_NONE;
    uint8_t buf[WIRE_MAX_SIZE];
    int len = wire_build(&evt, buf, sizeof(buf));
    /* Darf 0 oder positiv zurückgeben — kein Crash */
    zassert_true(len >= 0, "Empty present must not crash");
}

/* ════════════════════════════════════════════════════════════════
 * wire_parse — Fehlerbehandlung
 * ════════════════════════════════════════════════════════════════ */

ZTEST(wire_format, test_parse_null_buf_returns_false)
{
    gw_event_t dst = {0};
    bool ok = wire_parse(NULL, 0, &dst);
    zassert_false(ok, "NULL buf should return false");
}

ZTEST(wire_format, test_parse_null_evt_returns_false)
{
    uint8_t buf[4] = { 0xAA, 0xBB, 0xCC, 0xDD };
    bool ok = wire_parse(buf, 4, NULL);
    zassert_false(ok, "NULL evt should return false");
}

ZTEST(wire_format, test_parse_zero_len_returns_false)
{
    uint8_t buf[4] = { 0 };
    gw_event_t dst = {0};
    bool ok = wire_parse(buf, 0, &dst);
    zassert_false(ok, "Zero length should return false");
}

ZTEST(wire_format, test_parse_garbage_no_crash)
{
    static const uint8_t garbage[] = {
        0xFF, 0xFE, 0xFD, 0xAA, 0xBB, 0x00, 0x11, 0x22
    };
    gw_event_t dst = {0};
    wire_parse(garbage, sizeof(garbage), &dst);
    zassert_true(true, "Garbage input must not crash");
}

ZTEST(wire_format, test_parse_truncated_no_crash)
{
    gw_event_t src = make_sensor_event(0x0002);
    src.data.sensor.present = GW_HAS_TEMP | GW_HAS_HUM | GW_HAS_ECO2;
    src.data.sensor.temp_mc = 22500;

    uint8_t buf[WIRE_MAX_SIZE];
    int len = wire_build(&src, buf, sizeof(buf));
    zassert_true(len > 4, "Need some bytes to truncate");

    /* Verschiedene Abschneidepunkte testen */
    gw_event_t dst = {0};
    for (int trunc = 1; trunc < len; trunc++) {
        memset(&dst, 0, sizeof(dst));
        wire_parse(buf, (size_t)trunc, &dst);
    }
    zassert_true(true, "All truncated lengths must not crash");
}

ZTEST(wire_format, test_parse_all_zeros_no_crash)
{
    uint8_t buf[32] = {0};
    gw_event_t dst = {0};
    wire_parse(buf, sizeof(buf), &dst);
    zassert_true(true, "All-zeros buffer must not crash");
}

/* ════════════════════════════════════════════════════════════════
 * Länge / Größe
 * ════════════════════════════════════════════════════════════════ */

ZTEST(wire_format, test_build_size_within_bounds)
{
    /* Worst-case: alle Felder gesetzt */
    gw_event_t src = make_sensor_event(0x0002);
    src.data.sensor.present = 0xFFFFFFFF;  /* alle bits */
    uint8_t buf[WIRE_MAX_SIZE];
    int len = wire_build(&src, buf, sizeof(buf));
    if (len > 0) {
        zassert_true(len <= WIRE_MAX_SIZE,
                     "wire_build must not exceed WIRE_MAX_SIZE");
    }
}

ZTEST(wire_format, test_build_single_field_minimal_size)
{
    gw_event_t src = make_sensor_event(0x0002);
    src.data.sensor.present = GW_HAS_SEQ;
    src.data.sensor.seq     = 1;
    uint8_t buf[WIRE_MAX_SIZE];
    int len = wire_build(&src, buf, sizeof(buf));
    /* SEQ alleine braucht mindestens ein paar Bytes */
    zassert_true(len > 0, "Single field must produce some output");
}
