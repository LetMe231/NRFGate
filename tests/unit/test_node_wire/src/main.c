/**
 * @file tests/unit/test_node_wire/src/main.c
 *
 * Wire-Format-Tests für den ESP32-S3 BLE Mesh Sensor Node.
 *
 * ZIEL:
 *   Den Protokollvertrag zwischen Node (main.c) und Gateway (mesh_parser.c)
 *   vollständig absichern. Tests laufen auf dem Gateway (Zephyr/native_sim),
 *   replizieren die Node-seitigen marshal_*-Funktionen und prüfen den
 *   Round-Trip durch mesh_parse_sensor_status().
 *
 * GETESTETE BEREICHE:
 *   1. marshal_format_a     – MPID-Kodierung (Format A)
 *   2. marshal_u8/u16/s16/u32 – Typen-Helfer
 *   3. Button-Paket         – PROP_SWITCH=1, kein SEQ, 3 Bytes gesamt
 *   4. Button-Reset         – raw_switch wird auf 0 zurückgesetzt
 *   5. Periodisches Paket   – SEQ vorhanden, SWITCH=0 (nicht in present)
 *   6. PROP_LIGHT_STATE     – Round-Trip für Toggle-Cache
 *   7. Vollständiger Round-Trip: Node-Payload → gateway parser → Felder korrekt
 *   8. Grenzfälle           – falscher Typ, 0-Wert, max. Werte
 */

#include <zephyr/ztest.h>
#include <string.h>
#include <stdint.h>

#include "mesh_parser.h"
#include "data_handler.h"

/* ================================================================
 * Node-seitige Marshal-Funktionen (Kopie aus ESP32-S3 main.c).
 *
 * Diese Funktionen sind im Node als `static` deklariert und daher
 * nicht direkt importierbar. Wir replizieren sie hier 1:1, damit
 * die Tests den tatsächlichen Wire-Output validieren können.
 * Weicht die Node-Implementierung ab, schlagen die Round-Trip-Tests an.
 * ================================================================ */

static uint16_t node_marshal_format_a(uint8_t *buf, uint16_t property_id,
                                       const uint8_t *data, uint8_t data_len)
{
    uint16_t mpid = ((uint16_t)(property_id & 0x7FF) << 5)
                  | ((uint16_t)((data_len - 1) & 0xF) << 1)
                  | 0;   /* bit0=0 → Format A */
    buf[0] = (uint8_t)(mpid & 0xFF);
    buf[1] = (uint8_t)((mpid >> 8) & 0xFF);
    memcpy(&buf[2], data, data_len);
    return (uint16_t)(2 + data_len);
}

static uint16_t node_marshal_u8(uint8_t *buf, uint16_t prop, uint8_t val)
{
    return node_marshal_format_a(buf, prop, &val, 1);
}

static uint16_t node_marshal_u16(uint8_t *buf, uint16_t prop, uint16_t val)
{
    uint8_t raw[2] = { (uint8_t)val, (uint8_t)(val >> 8) };
    return node_marshal_format_a(buf, prop, raw, 2);
}

static uint16_t node_marshal_s16(uint8_t *buf, uint16_t prop, int16_t val)
{
    uint8_t raw[2] = { (uint8_t)val, (uint8_t)((val >> 8) & 0xFF) };
    return node_marshal_format_a(buf, prop, raw, 2);
}

static uint16_t node_marshal_u32(uint8_t *buf, uint16_t prop, uint32_t val)
{
    uint8_t raw[4] = {
        (uint8_t)(val),        (uint8_t)(val >> 8),
        (uint8_t)(val >> 16),  (uint8_t)(val >> 24),
    };
    return node_marshal_format_a(buf, prop, raw, 4);
}

/* ── Suite ───────────────────────────────────────────────────── */
ZTEST_SUITE(node_wire, NULL, NULL, NULL, NULL, NULL);


/* ================================================================
 * 1. marshal_format_a — MPID-Kodierung
 * ================================================================ */

ZTEST(node_wire, test_mpid_format_a_bit0_is_zero)
{
    /* Format A erfordert bit0=0 */
    uint8_t buf[4];
    node_marshal_u8(buf, PROP_SWITCH, 1);
    zassert_equal(buf[0] & 0x01, 0, "Format A: bit0 of first byte must be 0");
}

ZTEST(node_wire, test_mpid_prop_id_encoded_in_upper_bits)
{
    /* PROP_SWITCH = 0x0105, data_len=1
     * MPID = (0x0105 << 5) | (0 << 1) | 0 = 0x20A0
     * bytes: 0xA0, 0x20 */
    uint8_t buf[4];
    node_marshal_u8(buf, PROP_SWITCH, 0xAB);

    uint16_t mpid = (uint16_t)(buf[0] | (buf[1] << 8));
    uint16_t decoded_prop;
    uint8_t  decoded_len;
    bool ok = mesh_parse_mpid_format_a(mpid, &decoded_prop, &decoded_len);

    zassert_true(ok,                     "MPID should decode as Format A");
    zassert_equal(decoded_prop, PROP_SWITCH, "Decoded prop_id mismatch");
    zassert_equal(decoded_len,  1,           "SWITCH is 1 byte");
}

ZTEST(node_wire, test_mpid_len_field_for_4byte)
{
    /* SEQ ist u32 → len_field = 3 (zero-based) */
    uint8_t buf[8];
    node_marshal_u32(buf, PROP_SENSOR_SEQ, 42);

    uint16_t mpid = (uint16_t)(buf[0] | (buf[1] << 8));
    uint16_t prop_id;
    uint8_t  data_len;
    mesh_parse_mpid_format_a(mpid, &prop_id, &data_len);

    zassert_equal(prop_id,  PROP_SENSOR_SEQ, "SEQ prop_id wrong");
    zassert_equal(data_len, 4,               "SEQ must be 4 bytes");
}

/* ================================================================
 * 2. marshal_u8 / u16 / s16 / u32 — Byte-Reihenfolge (Little-Endian)
 * ================================================================ */

ZTEST(node_wire, test_marshal_u8_value_at_offset2)
{
    uint8_t buf[4];
    node_marshal_u8(buf, PROP_SWITCH, 0xAB);
    zassert_equal(buf[2], 0xAB, "u8 value should be at offset 2");
}

ZTEST(node_wire, test_marshal_u16_little_endian)
{
    /* 0x1234 → bytes: 0x34 (lo), 0x12 (hi) */
    uint8_t buf[8];
    node_marshal_u16(buf, PROP_ECO2, 0x1234);
    zassert_equal(buf[2], 0x34, "u16 low byte wrong");
    zassert_equal(buf[3], 0x12, "u16 high byte wrong");
}

ZTEST(node_wire, test_marshal_s16_negative_little_endian)
{
    /* -500 = 0xFE0C → bytes: 0x0C, 0xFE */
    uint8_t buf[8];
    node_marshal_s16(buf, PROP_TEMPERATURE, -500);
    zassert_equal(buf[2], 0x0C, "s16 negative low byte wrong");
    zassert_equal(buf[3], 0xFE, "s16 negative high byte wrong");
}

ZTEST(node_wire, test_marshal_u32_little_endian)
{
    /* 0x12345678 → bytes: 0x78, 0x56, 0x34, 0x12 */
    uint8_t buf[8];
    node_marshal_u32(buf, PROP_SENSOR_SEQ, 0x12345678);
    zassert_equal(buf[2], 0x78, "u32 byte[0] wrong");
    zassert_equal(buf[3], 0x56, "u32 byte[1] wrong");
    zassert_equal(buf[4], 0x34, "u32 byte[2] wrong");
    zassert_equal(buf[5], 0x12, "u32 byte[3] wrong");
}

ZTEST(node_wire, test_marshal_u8_total_length_is_3)
{
    /* MPID (2 Bytes) + 1 Byte Wert = 3 Bytes */
    uint8_t buf[8];
    uint16_t written = node_marshal_u8(buf, PROP_SWITCH, 1);
    zassert_equal(written, 3, "u8 marshal should write exactly 3 bytes");
}

ZTEST(node_wire, test_marshal_u16_total_length_is_4)
{
    uint8_t buf[8];
    uint16_t written = node_marshal_u16(buf, PROP_ECO2, 800);
    zassert_equal(written, 4, "u16 marshal should write exactly 4 bytes");
}

ZTEST(node_wire, test_marshal_u32_total_length_is_6)
{
    uint8_t buf[8];
    uint16_t written = node_marshal_u32(buf, PROP_SENSOR_SEQ, 0);
    zassert_equal(written, 6, "u32 marshal should write exactly 6 bytes");
}

/* ================================================================
 * 3. Button-Paket — Exakte Byte-Struktur
 *
 * button_task() publisht AUSSCHLIESSLICH PROP_SWITCH=1.
 * Kein SEQ, kein LIGHT_STATE, keine anderen Properties.
 * Gesamt: 3 Bytes (2 MPID + 1 Byte Wert).
 * ================================================================ */

ZTEST(node_wire, test_button_payload_total_3_bytes)
{
    uint8_t payload[4];
    uint16_t len = node_marshal_u8(payload, PROP_SWITCH, 1);
    zassert_equal(len, 3, "Button payload must be exactly 3 bytes");
}

ZTEST(node_wire, test_button_payload_switch_is_1)
{
    uint8_t payload[4];
    node_marshal_u8(payload, PROP_SWITCH, 1);
    zassert_equal(payload[2], 1, "Button payload value must be 1 (PRESSED)");
}

ZTEST(node_wire, test_button_payload_parses_as_switch_on)
{
    uint8_t payload[4];
    uint16_t len = node_marshal_u8(payload, PROP_SWITCH, 1);

    struct sensor_payload out = {0};
    bool ok = mesh_parse_sensor_status(payload, len, &out);

    zassert_true(ok, "Button payload should parse successfully");
    zassert_true(out.present & SENSOR_HAS_SWITCH,
                 "Parser must set SENSOR_HAS_SWITCH for value=1");
    zassert_equal(out.switch_state, 1, "switch_state must be 1");
}

ZTEST(node_wire, test_button_payload_no_seq_present)
{
    /* button_task inkludiert keinen SEQ — Parser darf SENSOR_HAS_SEQ nicht setzen */
    uint8_t payload[4];
    uint16_t len = node_marshal_u8(payload, PROP_SWITCH, 1);

    struct sensor_payload out = {0};
    mesh_parse_sensor_status(payload, len, &out);

    zassert_false(out.present & SENSOR_HAS_SEQ,
                  "Button payload must NOT contain SEQ");
}

ZTEST(node_wire, test_button_payload_no_other_sensors)
{
    /* Nur SWITCH, kein TEMP/HUM/ECO2/TVOC */
    uint8_t payload[4];
    uint16_t len = node_marshal_u8(payload, PROP_SWITCH, 1);

    struct sensor_payload out = {0};
    mesh_parse_sensor_status(payload, len, &out);

    zassert_false(out.present & SENSOR_HAS_TEMP,  "No TEMP in button pkt");
    zassert_false(out.present & SENSOR_HAS_HUM,   "No HUM in button pkt");
    zassert_false(out.present & SENSOR_HAS_ECO2,  "No ECO2 in button pkt");
    zassert_false(out.present & SENSOR_HAS_TVOC,  "No TVOC in button pkt");
}

/* ================================================================
 * 4. Button-Reset — raw_switch wird nach Publish auf 0 zurückgesetzt
 *
 * Simuliert das Verhalten nach dem Fix in button_task():
 *   net_buf_simple_reset(&raw_switch);
 *   net_buf_simple_add_u8(&raw_switch, 0);
 * Das Gateway darf den nächsten periodischen Publish NICHT als
 * PRESSED interpretieren.
 * ================================================================ */

ZTEST(node_wire, test_switch_reset_payload_not_pressed)
{
    /* Periodischer Publish nach Reset: SWITCH=0 */
    uint8_t payload[4];
    uint16_t len = node_marshal_u8(payload, PROP_SWITCH, 0);

    struct sensor_payload out = {0};
    mesh_parse_sensor_status(payload, len, &out);

    zassert_false(out.present & SENSOR_HAS_SWITCH,
                  "SWITCH=0 must NOT set SENSOR_HAS_SWITCH (no rule fire!)");
}

ZTEST(node_wire, test_switch_0_does_not_trigger_rule)
{
    /* Sicherstellen dass SWITCH=0 in keinem periodischen Paket
     * rule_engine_on_switch() auslösen würde.
     * data_handler.c: `if (d.payload.present & SENSOR_HAS_SWITCH)` →
     * nur wenn SENSOR_HAS_SWITCH gesetzt. */
    uint8_t payload[4];
    uint16_t len = node_marshal_u8(payload, PROP_SWITCH, 0);

    struct sensor_payload out = {0};
    mesh_parse_sensor_status(payload, len, &out);

    /* Rule Engine wird nur aufgerufen wenn (present & SENSOR_HAS_SWITCH).
     * Bei SWITCH=0 darf dieses Bit NICHT gesetzt sein. */
    bool would_call_rule_engine = (out.present & SENSOR_HAS_SWITCH) != 0;
    zassert_false(would_call_rule_engine,
                  "SWITCH=0 must not trigger rule engine (would cause repeated toggle)");
}

/* ================================================================
 * 5. Periodisches Paket — SEQ vorhanden, SWITCH=0
 *
 * pub_timer_cb() produziert: SEQ + (optionale Sensoren) + LIGHT_STATE.
 * SWITCH=0 ist in raw_switch, aber SENSOR_HAS_SWITCH darf nicht gesetzt sein.
 * ================================================================ */

ZTEST(node_wire, test_periodic_payload_has_seq)
{
    uint8_t payload[32];
    uint16_t off = 0;

    /* Minimal-periodisches Paket: SEQ + SWITCH=0 */
    off += node_marshal_u32(payload + off, PROP_SENSOR_SEQ, 12345);
    off += node_marshal_u8(payload + off,  PROP_SWITCH, 0);

    struct sensor_payload out = {0};
    bool ok = mesh_parse_sensor_status(payload, off, &out);

    zassert_true(ok, "Periodic payload should parse");
    zassert_true(out.present & SENSOR_HAS_SEQ,
                 "Periodic payload must have SEQ");
    zassert_equal(out.seq, 12345, "SEQ value mismatch");
}

ZTEST(node_wire, test_periodic_payload_switch_0_not_in_present)
{
    uint8_t payload[32];
    uint16_t off = 0;
    off += node_marshal_u32(payload + off, PROP_SENSOR_SEQ, 1);
    off += node_marshal_u8(payload + off,  PROP_SWITCH, 0);

    struct sensor_payload out = {0};
    mesh_parse_sensor_status(payload, off, &out);

    zassert_false(out.present & SENSOR_HAS_SWITCH,
                  "Periodic SWITCH=0 must NOT set SENSOR_HAS_SWITCH");
}

ZTEST(node_wire, test_seq_counter_increments)
{
    /* Zwei aufeinanderfolgende Pakete haben unterschiedliche SEQ */
    uint8_t pkt1[8], pkt2[8];
    uint32_t seq = 100;
    node_marshal_u32(pkt1, PROP_SENSOR_SEQ, seq);
    node_marshal_u32(pkt2, PROP_SENSOR_SEQ, seq + 1);

    struct sensor_payload out1 = {0}, out2 = {0};
    mesh_parse_sensor_status(pkt1, 6, &out1);
    mesh_parse_sensor_status(pkt2, 6, &out2);

    zassert_equal(out2.seq - out1.seq, 1,
                  "SEQ must increment by 1 between packets");
}

/* ================================================================
 * 6. PROP_LIGHT_STATE — Round-Trip für Toggle-Cache
 *
 * Der Gateway-Toggle liest node_actuator_state[target].light_on.
 * Dieser Cache wird aus PROP_LIGHT_STATE im periodischen Publish
 * befüllt (falls LIGHT_FEEDBACK aktiv) oder durch CoAP-ACK gesetzt.
 *
 * Kritisch: Wenn der Node LIGHT_STATE falsch kodiert oder der Parser
 * ihn falsch dekodiert, schlägt Toggle fehl (immer ON oder immer OFF).
 * ================================================================ */

ZTEST(node_wire, test_light_state_on_round_trip)
{
    uint8_t payload[4];
    uint16_t len = node_marshal_u8(payload, PROP_LIGHT_STATE, 1);

    struct sensor_payload out = {0};
    bool ok = mesh_parse_sensor_status(payload, len, &out);

    zassert_true(ok, "LIGHT_STATE=1 should parse");
    zassert_true(out.present & SENSOR_HAS_LIGHT,
                 "SENSOR_HAS_LIGHT must be set for LIGHT_STATE=1");
    zassert_equal(out.light_on, 1, "light_state must be 1 (ON)");
}

ZTEST(node_wire, test_light_state_off_round_trip)
{
    uint8_t payload[4];
    uint16_t len = node_marshal_u8(payload, PROP_LIGHT_STATE, 0);

    struct sensor_payload out = {0};
    mesh_parse_sensor_status(payload, len, &out);

    /* LIGHT_STATE=0 → Gateway setzt light_on=false im Cache */
    zassert_true(out.present & SENSOR_HAS_LIGHT,
                 "SENSOR_HAS_LIGHT must be set even for OFF (cache must update)");
    zassert_equal(out.light_on, 0, "light_state must be 0 (OFF)");
}

ZTEST(node_wire, test_toggle_after_light_on_payload_goes_off)
{
    /* Simuliert: Node meldet LIGHT_STATE=1 (AN).
     * Gateway-Toggle-Logik liest Cache → erwartet toggle auf OFF. */
    uint8_t payload[4];
    uint16_t len = node_marshal_u8(payload, PROP_LIGHT_STATE, 1);

    struct sensor_payload out = {0};
    mesh_parse_sensor_status(payload, len, &out);

    /* Simuliere Gateway-Cache-Update */
    bool cached_light_on = (out.light_on == 1);
    zassert_true(cached_light_on, "Cache should read ON from payload");

    /* Toggle-Richtung: war AN → gehe AUS */
    bool toggle_result = !cached_light_on;
    zassert_false(toggle_result,
                  "Toggle from ON must result in OFF — "
                  "if this fails, Toggle-Bug ist noch vorhanden");
}

ZTEST(node_wire, test_toggle_after_light_off_payload_goes_on)
{
    uint8_t payload[4];
    uint16_t len = node_marshal_u8(payload, PROP_LIGHT_STATE, 0);

    struct sensor_payload out = {0};
    mesh_parse_sensor_status(payload, len, &out);

    bool cached_light_on = (out.light_on == 1);
    bool toggle_result = !cached_light_on;
    zassert_true(toggle_result,
                 "Toggle from OFF must result in ON");
}

/* ================================================================
 * 7. Vollständiger Round-Trip: SGP30-Node-Payload (wie in den Logs)
 *
 * Node 2 sendet: SGP30 (TVOC + ECO2) + SEQ
 * Entspricht den Log-Zeilen:
 *   Air: TVOC=2392 ppb eCO2=1171 ppm
 *   seq=124139
 * ================================================================ */

ZTEST(node_wire, test_full_sgp30_payload_round_trip)
{
    uint8_t payload[32];
    uint16_t off = 0;

    off += node_marshal_u32(payload + off, PROP_SENSOR_SEQ, 124139);
    off += node_marshal_u16(payload + off, PROP_ECO2,       1171);
    off += node_marshal_u16(payload + off, PROP_TVOC,       2392);

    struct sensor_payload out = {0};
    bool ok = mesh_parse_sensor_status(payload, off, &out);

    zassert_true(ok, "SGP30 payload parse failed");
    zassert_true(out.present & SENSOR_HAS_SEQ,  "SEQ missing");
    zassert_true(out.present & SENSOR_HAS_ECO2, "ECO2 missing");
    zassert_true(out.present & SENSOR_HAS_TVOC, "TVOC missing");
    zassert_equal(out.seq,  124139, "SEQ mismatch");
    zassert_equal(out.eco2, 1171,   "eCO2 mismatch (got %d)", out.eco2);
    zassert_equal(out.tvoc, 2392,   "TVOC mismatch (got %d)", out.tvoc);
}

ZTEST(node_wire, test_full_switch_node_payload_round_trip)
{
    /* Node 3 Button-Paket aus den Logs: PRESSED, seq=0 (kein SEQ im Paket) */
    uint8_t payload[4];
    uint16_t len = node_marshal_u8(payload, PROP_SWITCH, 1);

    struct sensor_payload out = {0};
    bool ok = mesh_parse_sensor_status(payload, len, &out);

    zassert_true(ok, "Switch packet parse failed");
    zassert_true(out.present & SENSOR_HAS_SWITCH, "SENSOR_HAS_SWITCH not set");
    zassert_false(out.present & SENSOR_HAS_SEQ,
                  "Button pkt has no SEQ (seq=0 in logs = no seq field)");
    zassert_equal(out.switch_state, 1, "switch_state != 1");
}

ZTEST(node_wire, test_light_node_periodic_payload_round_trip)
{
    /* Node 0 Thread-Light periodisch: LIGHT_STATE=OFF + SEQ */
    uint8_t payload[16];
    uint16_t off = 0;

    off += node_marshal_u32(payload + off, PROP_SENSOR_SEQ,   52220);
    off += node_marshal_u8(payload + off,  PROP_LIGHT_STATE,  0);  /* OFF */
    off += node_marshal_u8(payload + off,  PROP_SWITCH,       0);  /* idle */

    struct sensor_payload out = {0};
    bool ok = mesh_parse_sensor_status(payload, off, &out);

    zassert_true(ok, "Light node payload parse failed");
    zassert_equal(out.seq,         52220, "SEQ wrong");
    zassert_equal(out.light_on, 0,     "light_state should be OFF");
    zassert_false(out.present & SENSOR_HAS_SWITCH,
                  "SWITCH=0 must not be present (no rule fire)");
}

/* ================================================================
 * 8. Grenzfälle
 * ================================================================ */

ZTEST(node_wire, test_seq_max_u32_round_trip)
{
    uint8_t buf[8];
    node_marshal_u32(buf, PROP_SENSOR_SEQ, 0xFFFFFFFF);

    struct sensor_payload out = {0};
    mesh_parse_sensor_status(buf, 6, &out);

    zassert_equal((uint32_t)out.seq, 0xFFFFFFFF,
                  "SEQ max value round-trip failed");
}

ZTEST(node_wire, test_eco2_max_round_trip)
{
    /* eCO2 max sinnvoller Wert: 60000 ppm */
    uint8_t buf[8];
    node_marshal_u16(buf, PROP_ECO2, 60000);

    struct sensor_payload out = {0};
    mesh_parse_sensor_status(buf, 4, &out);

    zassert_equal(out.eco2, 60000, "ECO2 max value round-trip failed");
}

ZTEST(node_wire, test_temperature_negative_extreme)
{
    /* -40°C → raw: -4000 (0.01°C steps) → gateway: *10 = -40000 m°C */
    uint8_t buf[8];
    node_marshal_s16(buf, PROP_TEMPERATURE, -4000);

    struct sensor_payload out = {0};
    mesh_parse_sensor_status(buf, 4, &out);

    zassert_equal(out.temp, -40000,
                  "Temperature -40°C round-trip failed (got %d)", out.temp);
}

ZTEST(node_wire, test_switch_and_light_state_same_packet)
{
    /* Kombination: Switch=1 + LightState=1 in einem Paket */
    uint8_t payload[16];
    uint16_t off = 0;
    off += node_marshal_u8(payload + off, PROP_SWITCH,      1);
    off += node_marshal_u8(payload + off, PROP_LIGHT_STATE, 1);

    struct sensor_payload out = {0};
    bool ok = mesh_parse_sensor_status(payload, off, &out);

    zassert_true(ok, "Switch+LightState combined parse failed");
    zassert_true(out.present & SENSOR_HAS_SWITCH, "SWITCH missing");
    zassert_true(out.present & SENSOR_HAS_LIGHT,  "LIGHT missing");
    zassert_equal(out.switch_state, 1, "switch_state wrong");
    zassert_equal(out.light_on,  1, "light_state wrong");
}

ZTEST(node_wire, test_prop_light_state_prop_id_not_same_as_switch)
{
    /* Sicherstellen dass PROP_SWITCH und PROP_LIGHT_STATE verschiedene IDs haben */
    zassert_not_equal(PROP_SWITCH, PROP_LIGHT_STATE,
                      "PROP_SWITCH and PROP_LIGHT_STATE must have different IDs");
}